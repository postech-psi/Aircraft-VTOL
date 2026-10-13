#include "AeroPhysics.hh"
#include <cassert>
#include <cmath>

int main() {
  psi::Surface wing;
  wing.area=.425; wing.chord=.25; wing.alpha0=4*psi::pi/180;
  wing.cla=4.5; wing.cd0=.025; wing.induced=.055; wing.cm0=-.1;
  auto zeroLift=psi::polar(-wing.alpha0,wing);
  assert(std::abs(zeroLift.cl)<1e-12 && zeroLift.cd>0 && zeroLift.cm<0);
  for (int i=-1800;i<=1800;++i) {
    double a=i*psi::pi/1800;
    psi::Vec v{10*std::cos(a),0,-10*std::sin(a)};
    auto f=psi::surfaceForce(v,{},wing,1.2);
    assert(std::isfinite(f.force.norm()));
    assert(f.force.dot(v)<0); // aerodynamic lift does no work; drag always dissipates
    assert(std::abs(psi::polar(a,wing).cl)<2.5);
  }
  auto reverse=psi::surfaceForce({-10,0,0},{},wing,1.2);
  assert(reverse.force.x>0);
  auto crosswind=psi::bodyDrag({0,8,0},{.02,.15,.12},1.2);
  assert(crosswind.y<0 && crosswind.x==0);
  assert(psi::thrustFraction(.5,0,35,1,1)==1);
  assert(std::abs(psi::thrustFraction(.5,0,35,.8,1)-.64)<1e-12);
  assert(psi::thrustFraction(.5,20,35,1,1)==0);
  assert(psi::thrustFraction(.5,-20,35,1,1)==1);
  assert(psi::thrustFraction(.5,0,35,1,.65)==.65);
  auto correction=psi::rotorCorrection({10,0,0},{},{0,0,0},{1,0,0},.5,25,.015,1,35,1,1,0);
  assert(correction.force.x<0 && correction.torque.x<0);
  assert(25*.25+correction.force.x>=0);
  auto noseDown=psi::surfaceForce({10,0,0},{},wing,1.2);
  assert(noseDown.torque.y>0); // FLU: positive y torque is nose down
}
