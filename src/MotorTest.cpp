/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gain.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "MotorTest.hpp"

using namespace std;
using namespace RTT;
using namespace CUSTOM;

MotorTest::MotorTest(const string& name) : TaskContext(name, PreOperational)
{

  // Adding properties
  addProperty( "max_velocities", max_vels);
  // Adding ports
  addEventPort( "in", inport_motors );
  addPort( "enc", inport_encoders );
  addPort( "out", outport_doubles );
  addPort( "enable1", outport_enable1);
  addPort( "enable2", outport_enable2);
  addPort( "brake1", outport_brake1);
  addPort( "brake2", outport_brake2);

}
MotorTest::~MotorTest(){}

bool MotorTest::configureHook()
{
  // configure output paramters
  encs_prev.assign(2,0.0);
  output.assign(2,0.0);
  out_enable1 = false;
  out_enable2 = false;
  out_brake1 = false;
  out_brake2 = false;
  vel_limit_exceeded = false;
  print_once = true;

  return true;
}

bool MotorTest::startHook()
{
  // Check validity of Ports:
  if ( !inport_motors.connected() )
  {
    log(Error)<<"MotorTest::inputport not connected"<<endlog();
    return false;
  }
  if ( !outport_doubles.connected() ) {
    log(Warning)<<"MotorTest::Outputport pwm's not connected!"<<endlog();
  }
  if ( !outport_enable1.connected() ) {
    log(Warning)<<"MotorTest::Outputport enable1 not connected!"<<endlog();
  }
  if ( !outport_enable2.connected() ) {
    log(Warning)<<"MotorTest::Outputport enable2 not connected!"<<endlog();
  }
  if ( !outport_brake1.connected() ) {
    log(Warning)<<"MotorTest::Outputport brake1 not connected!"<<endlog();
  }
  if ( !outport_brake2.connected() ) {
    log(Warning)<<"MotorTest::Outputport brake2 not connected!"<<endlog();
  }
  if ( !inport_encoders.connected() ) {
    log(Warning) << "MotorTest::Inport encoders not connected!"<<endlog();
  }

  // Check if property is set
  if (max_vels.size() != 2 )
  {
    log(Error)<<"MotorTest:: max_velocities not correctly specified" << endlog();
  }

  log(Warning)<<"MOTORTEST:: started"<<endlog();
  return true;
}

void MotorTest::updateHook()
{
  
  double vel = 0.0;
  doubles encs(2,0.0);
  if ( inport_encoders.read(encs) == NewData ) {
    // Determine timestamp:
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    //log(Debug)<<"new_time: "<<new_time<<endlog();
    double dt = new_time - old_time;
    old_time = new_time;

    // check velocity constraints
    //double vel = 0.0;
    for (uint i=0; i<=1; i++ )
    {
      vel = (encs[i]-encs_prev[i])/dt;
      if ( vel > max_vels[i] || -vel > max_vels[i])
      {
        vel_limit_exceeded = true;
      }
    }
    encs_prev = encs;
  }
  
  
  vel_limit_exceeded = false;
  if (vel_limit_exceeded)
  {
    out_brake1 = false;
    out_brake2 = false;
    out_enable1 = false;
    out_enable2 = false;
    output.assign(2,0.0);
    if (print_once) {
      log(Error) << "MotorTest:: Velocity limit exceeded, vel = "<< vel << endlog();
      print_once = false;
    }
  }
  else
  {
    doubles input(2,0.0);
    // Read the inputports and set the outputs
    if ( inport_motors.read(input) == NewData ) {
        output[0] = -input[0];
        output[1] = -input[1];
      if (input[0]==0.0) {
          out_brake1 = false;
          out_enable1 = false;
      } else {
          out_brake1 = true;
          out_enable1 = true;
      }
      if (input[1]==0.0) {
          out_brake2 = false;
          out_enable2 = false;
      } else {
          out_brake2 = true;
          out_enable2 = true;
      }
    }
  }

  // Write the outputs
  outport_doubles.write( output );
  outport_brake1.write( out_brake1 );
  outport_brake2.write( out_brake2 );
  outport_enable1.write( out_enable1 );
  outport_enable2.write( out_enable2 );

}

void MotorTest::stopHook()
{
    output.assign(2,0.0);
    out_brake1 = false;
    out_brake2 = false;
    out_enable1 = false;
    out_enable2 = false;

    // Write the outputs
    outport_doubles.write( output );
    outport_brake1.write( out_brake1 );
    outport_brake2.write( out_brake2 );
    outport_enable1.write( out_enable1 );
    outport_enable2.write( out_enable2 );

}

ORO_CREATE_COMPONENT(CUSTOM::MotorTest)
