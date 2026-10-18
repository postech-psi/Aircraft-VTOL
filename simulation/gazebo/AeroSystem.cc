// PSI all-angle aerodynamics and corrections to Gazebo's static motor model.
#include "AeroPhysics.hh"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace psi {
static Vec vector(const gz::math::Vector3d &v) { return {v.X(),v.Y(),v.Z()}; }
static gz::math::Vector3d vector(Vec v) { return {v.x,v.y,v.z}; }
template<class T> T get(const sdf::ElementPtr &e, const std::string &key, T value) {
  return e->Get<T>(key,value).first;
}
struct Rotor {
  std::string joint;
  Vec position{}, axis{};
  double maxSpeed{}, maxThrust{}, moment{}, sign{}, inflow{}, drag{}, interaction{};
  bool off{false};  // failed: the stock motor model still spins it, this system cancels its thrust
  gz::sim::Entity entity{gz::sim::kNullEntity};
};
class AeroSystem final : public gz::sim::System,
                         public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    auto e=sdf->Clone();
    gz::sim::Model model(entity);
    link=gz::sim::Link(model.LinkByName(ecm,get(e,"link_name",std::string("base_link"))));
    if (!link.Valid(ecm)) throw std::runtime_error("PSI AeroSystem: missing base link");
    link.EnableVelocityChecks(ecm);
    rho=get(e,"air_density",1.2041); weight=get(e,"weight_n",68.65);
    cdArea=vector(get(e,"body_cd_area",gz::math::Vector3d(.01,.15,.12)));
    bodyCp=vector(get(e,"body_cp",gz::math::Vector3d::Zero));
    angularDamping=vector(get(e,"angular_damping",gz::math::Vector3d(.02,.02,.02)));
    voltageEnd=get(e,"voltage_end_ratio",1.); voltageTime=get(e,"voltage_decay_s",600.);
    slowdown=get(e,"rotor_slowdown",20.);
    if (e->HasElement("surface")) for (auto s=e->GetElement("surface");s;s=s->GetNextElement("surface")) {
      Surface a;
      a.cp=vector(get(s,"cp",gz::math::Vector3d::Zero));
      a.area=get(s,"area",0.); a.chord=get(s,"chord",.25);
      a.incidence=get(s,"incidence",0.); a.alpha0=get(s,"alpha0",0.);
      a.cla=get(s,"cla",4.5); a.cd0=get(s,"cd0",.02);
      a.induced=get(s,"induced",.05); a.cm0=get(s,"cm0",0.);
      a.stall=get(s,"stall",14*pi/180); a.blend=get(s,"blend",8*pi/180);
      surfaces.push_back(a);
    }
    if (e->HasElement("rotor")) for (auto s=e->GetElement("rotor");s;s=s->GetNextElement("rotor")) {
      Rotor r;
      r.joint=get(s,"joint",std::string());
      r.position=vector(get(s,"position",gz::math::Vector3d::Zero));
      r.axis=vector(get(s,"axis",gz::math::Vector3d(0,0,1)));
      r.maxSpeed=get(s,"max_speed",1500.); r.maxThrust=get(s,"max_thrust",32.6);
      r.moment=get(s,"moment",.03); r.sign=get(s,"reaction_sign",-1.);
      r.inflow=get(s,"zero_thrust_speed",35.); r.drag=get(s,"drag_at_full_rpm",.15);
      r.interaction=get(s,"interaction",1.);
      r.entity=model.JointByName(ecm,r.joint);
      if (r.entity==gz::sim::kNullEntity) throw std::runtime_error("PSI AeroSystem: missing rotor joint");
      rotors.push_back(r);
    }
    // A rotor fails when its joint name is published here, e.g. by run_case --event pusher_off.
    const auto topic="/model/"+model.Name(ecm)+"/rotor_off";
    if (!node.Subscribe(topic,&AeroSystem::OnRotorOff,this))
      throw std::runtime_error("PSI AeroSystem: cannot subscribe to "+topic);
    const auto output=get(e,"telemetry_path",std::string());
    if (!output.empty()) {
      telemetry.open(output);
      if (!telemetry) throw std::runtime_error("PSI AeroSystem: cannot open telemetry");
      telemetry<<"sim_time_s,airspeed_mps,wing_vertical_force_n,wing_lift_fraction,voltage_ratio";
      for (size_t i=0;i<rotors.size();++i) telemetry<<",rotor"<<i<<"_thrust_n";
      telemetry<<"\n";
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const auto pose=link.WorldPose(ecm);
    const auto linear=link.WorldLinearVelocity(ecm), angular=link.WorldAngularVelocity(ecm);
    if (!pose || !linear || !angular) return;
    const double t=std::chrono::duration<double>(info.simTime).count();
    const auto rotation=pose->Rot();
    gz::math::Vector3d wind=gz::math::Vector3d::Zero;
    const auto w=ecm.EntityByComponents(gz::sim::components::Wind());
    if (auto wc=ecm.Component<gz::sim::components::WorldLinearVelocity>(w)) wind=wc->Data();
    const Vec velocity=vector(rotation.RotateVectorReverse(*linear-wind));
    const Vec rate=vector(rotation.RotateVectorReverse(*angular));
    const double voltage=1+(voltageEnd-1)*std::clamp(t/voltageTime,0.,1.);
    const Vec localBody=velocity+rate.cross(bodyCp);
    Wrench total{bodyDrag(localBody,cdArea,rho),{}};
    total.torque=bodyCp.cross(total.force)+Vec{-angularDamping.x*rate.x,
                        -angularDamping.y*rate.y,-angularDamping.z*rate.z};
    double wingVertical=0;
    for (size_t i=0;i<surfaces.size();++i) {
      const auto f=surfaceForce(velocity,rate,surfaces[i],rho);
      total.force=total.force+f.force; total.torque=total.torque+f.torque;
      if (i<2) wingVertical+=rotation.RotateVector(vector(f.force)).Z();
    }
    {
      std::lock_guard<std::mutex> lock(failedMutex);
      for (auto &r:rotors) r.off=failed.count(r.joint)>0;
    }
    std::vector<double> thrusts;
    for (const auto &r:rotors) {
      const double interaction=r.off ? 0. : r.interaction;
      auto joint=ecm.Component<gz::sim::components::JointVelocity>(r.entity);
      const double fraction=(joint && !joint->Data().empty()) ?
        std::clamp(std::abs(joint->Data()[0])*slowdown/r.maxSpeed,0.,1.) : 0.;
      const auto f=rotorCorrection(velocity,rate,r.position,r.axis,fraction,r.maxThrust,
                                  r.moment,r.sign,r.inflow,voltage,interaction,r.drag);
      total.force=total.force+f.force; total.torque=total.torque+f.torque;
      thrusts.push_back(r.maxThrust*fraction*fraction*thrustFraction(fraction,
        (velocity+rate.cross(r.position)).dot(r.axis),r.inflow,voltage,interaction));
    }
    // AddWorldWrench uses the link origin. The physics engine accounts for the inertial CG offset.
    link.AddWorldWrench(ecm,rotation.RotateVector(vector(total.force)),rotation.RotateVector(vector(total.torque)));
    if (telemetry && t>=nextLog) {
      telemetry<<std::setprecision(12)<<t<<','<<velocity.norm()<<','<<wingVertical<<','<<wingVertical/weight<<','<<voltage;
      for (auto thrust:thrusts) telemetry<<','<<thrust;
      telemetry<<'\n'; telemetry.flush(); nextLog=t+.05;
    }
  }
 private:
  void OnRotorOff(const gz::msgs::StringMsg &msg) {
    std::lock_guard<std::mutex> lock(failedMutex);
    failed.insert(msg.data());
  }
  gz::transport::Node node;
  std::mutex failedMutex;
  std::set<std::string> failed;
  gz::sim::Link link;
  std::vector<Surface> surfaces;
  std::vector<Rotor> rotors;
  Vec cdArea{},bodyCp{},angularDamping{};
  double rho{},weight{},voltageEnd{},voltageTime{},slowdown{},nextLog{};
  std::ofstream telemetry;
};
}
GZ_ADD_PLUGIN(psi::AeroSystem,gz::sim::System,psi::AeroSystem::ISystemConfigure,psi::AeroSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(psi::AeroSystem,"psi::AeroSystem")
