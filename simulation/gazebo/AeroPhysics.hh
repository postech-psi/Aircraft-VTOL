// Gazebo-independent aerodynamic/propulsion approximations, in body FLU axes.
#pragma once
#include <algorithm>
#include <cmath>

namespace psi {
constexpr double pi = 3.14159265358979323846;
struct Vec {
  double x{}, y{}, z{};
  Vec operator+(Vec b) const { return {x+b.x, y+b.y, z+b.z}; }
  Vec operator-(Vec b) const { return {x-b.x, y-b.y, z-b.z}; }
  Vec operator*(double s) const { return {x*s, y*s, z*s}; }
  double dot(Vec b) const { return x*b.x+y*b.y+z*b.z; }
  Vec cross(Vec b) const { return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x}; }
  double norm() const { return std::sqrt(dot(*this)); }
};
struct Wrench { Vec force{}, torque{}; };
struct Surface {
  Vec cp{};
  double area{}, chord{}, incidence{}, alpha0{}, cla{}, cd0{}, induced{}, cm0{};
  double stall{14*pi/180}, blend{8*pi/180}, flatPlateCd{1.8};
};
struct Polar { double cl{}, cd{}, cm{}; };

inline Polar polar(double alpha, const Surface &s) {
  // Attached flow is used only near the forward chord. Smoothly blend to a
  // bounded flat-plate approximation at stall, including reverse flow.
  const double a = std::remainder(alpha, 2*pi);
  const double distance = std::abs(a);
  const double x = std::clamp((distance-s.stall)/s.blend, 0., 1.);
  const double separated = x*x*(3-2*x);
  const double clAttached = s.cla*(a+s.alpha0);
  const double clPlate = s.flatPlateCd*std::sin(a)*std::cos(a);
  const double cdAttached = s.cd0+s.induced*clAttached*clAttached;
  const double cdPlate = s.cd0+s.flatPlateCd*std::pow(std::sin(a), 2);
  return {(1-separated)*clAttached+separated*clPlate,
          (1-separated)*cdAttached+separated*cdPlate,
          (1-separated)*s.cm0};
}

inline Wrench surfaceForce(Vec velocity, Vec angular, const Surface &s, double rho) {
  const Vec local = velocity + angular.cross(s.cp);
  const double c=std::cos(s.incidence), sn=std::sin(s.incidence);
  // Positive incidence raises the leading edge. Surface forward is (c,0,sn).
  const double u=c*local.x+sn*local.z, w=-sn*local.x+c*local.z;
  const double speed=std::hypot(u,w);
  if (speed < 1e-6) return {};
  const auto p=polar(std::atan2(-w,u),s);
  const double qS=.5*rho*speed*speed*s.area;
  const Vec along{local.x/speed,0,local.z/speed};
  const Vec lift{-along.z,0,along.x};
  const Vec force=lift*(qS*p.cl)-along*(qS*p.cd);
  // Positive Cm is nose-up; in FLU that is negative body y torque.
  return {force, s.cp.cross(force)+Vec{0,-qS*s.chord*p.cm,0}};
}

inline Vec bodyDrag(Vec velocity, Vec cdArea, double rho) {
  return {-.5*rho*cdArea.x*velocity.x*std::abs(velocity.x),
          -.5*rho*cdArea.y*velocity.y*std::abs(velocity.y),
          -.5*rho*cdArea.z*velocity.z*std::abs(velocity.z)};
}

inline double thrustFraction(double rpmFraction, double axialSpeed, double zeroThrustSpeed,
                             double voltage, double interaction) {
  if (rpmFraction <= 1e-6) return 0;
  // Linear CT(J) surrogate, bounded: no unsupported tailwind thrust boost.
  const double inflow=std::clamp(1-std::max(0.,axialSpeed)/(zeroThrustSpeed*rpmFraction),0.,1.);
  return inflow*voltage*voltage*interaction;
}

inline Wrench rotorCorrection(Vec velocity, Vec angular, Vec position, Vec axis,
                             double rpmFraction, double staticMax, double momentRatio,
                             double reactionSign, double zeroThrustSpeed, double voltage,
                             double interaction, double dragAtFullRpm) {
  const Vec local=velocity+angular.cross(position);
  const double staticThrust=staticMax*rpmFraction*rpmFraction;
  const double gain=thrustFraction(rpmFraction,local.dot(axis),zeroThrustSpeed,voltage,interaction);
  // Stock Gazebo motor supplies static thrust and reaction torque. Add only
  // their loss here; rotor drag is owned by this system, not counted twice.
  const Vec delta=axis*(staticThrust*(gain-1));
  const Vec drag=(local-axis*local.dot(axis))*(-dragAtFullRpm*rpmFraction);
  return {delta+drag, position.cross(delta+drag)+axis*(reactionSign*momentRatio*staticThrust*(gain-1))};
}
}  // namespace psi
