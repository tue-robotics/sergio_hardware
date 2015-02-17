#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "TorsoFeedForward.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

TorsoFeedForward::TorsoFeedForward(const string& name) : TaskContext(name, PreOperational),
    m_arms(0.0)
{
    // Adding Properties:
    addProperty( "mass_arms",m_arms).doc("");

    // Creating ports:
    addEventPort( "ref_in", inport );
    addPort( "vel_in", inport_vel );
    addPort( "ffw_out", outport );
}
TorsoFeedForward::~TorsoFeedForward(){}

bool TorsoFeedForward::configureHook()
{

    // initialize variables
    X1 = 0.0;
    X1_sq = 0.0;
    X2 = 0.0;
    X3 = 0.0;

    // initialize constants
    C1 = 0.150052;
    C2 = -0.115090;
    C3 = 0.614223;
    C4 = 0.174737;
    C5 = -0.064849;
    C6 = 0.205912;
    C7 = 0.171033;
    C8 = 0.107369;
    C9 = 0.756629;
    C10 = -0.133132;
    C11 = -0.779956;
    C12 = 0.132722;
    C13 = -0.155586;
    C14 = 2.913677;

    return true;
}

bool TorsoFeedForward::startHook()
{
    // initialize messages
    input.assign(2,0.0);
    input_vel.assign(2,0.0);
    output.assign(2,0.0);
    angles.assign(2,0.0);
    forces.assign(2,0.0);

    // Check validity of Ports:
    if ( !outport.connected() ) {
        log(Warning)<<"TorsoFeedForward::outport ffw_out not connected!"<<endlog();
    }
    if ( !inport.connected() ) {
        log(Warning)<<"TorsoFeedForward::inport ref_in not connected!"<<endlog();
    }

    return true;
}


void TorsoFeedForward::updateHook()
{
    if (inport.read(input)==NewData){
        // calculate joint angles
        angles[0] = acos((input[0]*input[0]-C1)/C2)-C3;
        X1_sq = C7-C8*cos(angles[0]+C9);
        X1 = sqrt(X1_sq);
        X2 = acos((C10-X1_sq)/(C11*X1));
        X3 = acos((C12-X1_sq)/(C13*X1));
        angles[1] = C14-X2-X3;
        angles[2] = acos((input[1]*input[1]-C4)/C5)+C6;

        // angle at which the force is applied to the trunk
        angleForce = acos((-0.1685+0.0062+input[1]*input[1])/(2*0.079*input[1]));

        // Calculate gravity compensation force needed
        forces[1] = (2.2819+0.47*m_arms)*124.19*cos(angles[2]+angles[0]-angles[1])/sin(angleForce);

        // Add friction force needed
        forces[1] = forces[1]+128.6;            // direction independant friction term
        inport_vel.read(input_vel);
        if (input_vel[1]>0){                    // direction dependant friction term
            forces[1] = forces[1]+393.8;
        } else if (input_vel[1]<0){
            forces[1] = forces[1]-393.8;
        }

        // convert to output voltage
        output[1] = forces[1]*0.00025156;
        output[0] = 0.0;

    }

    // write output to port
    outport.write(output);
}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::TorsoFeedForward)
