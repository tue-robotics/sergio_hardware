#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "RefGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

RefGenerator::RefGenerator(const string& name) : TaskContext(name, PreOperational)
{
    // Adding Properties:
    addProperty( "velocity_x",velocity_x).doc("");
    addProperty( "velocity_y",velocity_y).doc("");
    addProperty( "velocity_phi",velocity_phi).doc("");
    addProperty( "duration",duration).doc("");
    addProperty( "Ts",Ts).doc("");

    // Creating ports:
    addPort( "cmd_vel", outport );
}
RefGenerator::~RefGenerator(){}

bool RefGenerator::configureHook()
{
    // initialize twist message
    cmd_veldata.angular.x = 0.0;
    cmd_veldata.angular.y = 0.0;
    cmd_veldata.angular.z = 0.0;
    cmd_veldata.linear.x = 0.0;
    cmd_veldata.linear.y = 0.0;
    cmd_veldata.linear.z = 0.0;

    // check settings
    N = velocity_x.size();
    if ( N<1 || velocity_y.size()!=N || velocity_phi.size()!=N || duration.size()!=N || Ts<=0.0){
        log(Error)<<"RefGenerator: Wrong declaration of parameters"<< endlog();
        return false;
    }

    return true;
}

bool RefGenerator::startHook()
{
    Logger::In in("RefGenerator::Start");

    // Check validity of Ports:
    if ( !outport.connected() ) {
        log(Error)<<"RefGenerator::outport not connected!"<<endlog();
        return false;
    }

    time = 0.0;
    switch_time = duration[0];
    step = 0;

    return true;
}


void RefGenerator::updateHook()
{
    if ( time >= switch_time ){
        if ( step < N-1 ){
            step ++;
            switch_time += duration[step];
            cmd_veldata.linear.x = velocity_x[step];
            cmd_veldata.linear.y = velocity_y[step];
            cmd_veldata.angular.z = velocity_phi[step];
        } else {
            cmd_veldata.linear.x = 0.0;
            cmd_veldata.linear.y = 0.0;
            cmd_veldata.angular.z = 0.0;
        }
    }

    outport.write(cmd_veldata);
    time += Ts;
}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::RefGenerator)
