#include <algorithm>
#include <cmath>
extern "C" {
struct PhysicsState{double x,z,heading,speed,vx,vz,damage,tyres,fuel,temperature;};static PhysicsState s{};
__attribute__((used)) void physics_reset(){s={0,104,1.57079632679,0,0,0,0,100,100,65};}
__attribute__((used)) PhysicsState* physics_state(){return &s;}
__attribute__((used)) int physics_step(double dt,double throttle,double brake,double steer){double drag=.00265*s.speed*s.speed+.42;s.speed=std::clamp(s.speed+(18*throttle-29*brake-drag)*dt,-3.0,80.0);s.heading+=steer*(1.16-std::min(1.0,std::abs(s.speed)/48)*.38)*s.speed*.019*dt;double tx=sin(s.heading)*s.speed,tz=cos(s.heading)*s.speed,sm=1-pow(.17,dt*60);s.vx+=(tx-s.vx)*sm;s.vz+=(tz-s.vz)*sm;s.x+=s.vx*dt;s.z+=s.vz*dt;s.fuel=std::max(0.0,s.fuel-(.018+.046*throttle)*dt);double nx=s.x/75,nz=s.z/150,d=hypot(nx,nz);if(d>1||d<.67){double t=d>1?.988:.682;s.x=nx/d*t*75;s.z=nz/d*t*150;s.vx*=-.18;s.vz*=-.18;s.speed*=.38;s.damage=std::min(100.0,s.damage+2);return 1;}return 0;}}
