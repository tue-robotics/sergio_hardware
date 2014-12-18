#include "ConversionSpindleAngle.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

ConversionSpindleAngle::ConversionSpindleAngle(const string& name) :
    TaskContext(name, PreOperational),
    spindle_to_angle(true),
    output_joint_states(false)
{
    // Ports
    addPort( "in", inport );
    addPort( "out", outport );

    // Properties
    addProperty( "spindle_to_angle", spindle_to_angle).doc("bool, true for spindle input and angle output, false means the other way around.");
    addProperty("output_joint_states", output_joint_states).doc("Set to true ");
    addProperty( "JointNames", out_msg.name );
}

ConversionSpindleAngle::~ConversionSpindleAngle(){}

bool ConversionSpindleAngle::configureHook()
{

    // initialize outputs
    output.assign(2,0.0);

    if (output_joint_states){
        if ( out_msg.name.size() != 3 ) {
            log(Error)<<"ConversionSpindleAngle: variable JointNames not correctly specified, should contain 3 joint names!"<<endlog();
            return false;
        }

        out_msg.position.assign(3,0.0);
        out_msg.velocity.assign(3,0.0);
        out_msg.effort.assign(3,0.0);
        // add extra output port to publish joint states
        addPort( "out_joints", outport_joints );

    }



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

bool ConversionSpindleAngle::startHook()
{
    return true;
}


void ConversionSpindleAngle::updateHook()
{
    doubles input;

    // Read the inputports
    if ( inport.read(input) == NewData ){
        if (spindle_to_angle && input.size() == 2){
            // calculate ankle and hip angles
            output[0] = acos((input[0]*input[0]-C1)/C2)-C3;
            output[1] = acos((input[1]*input[1]-C4)/C5)-C6;

            if( output_joint_states ){
                // set the ankle and hip angles
                out_msg.position[0] = output[0];
                out_msg.position[2] = output[1];
                // caluclate knee angle
                X1_sq = C7-C8*cos(output[0]+C9);
                X1 = sqrt(X1_sq);
                X2 = acos((C10-X1_sq)/(C11*X1));
                X3 = acos((C12-X1_sq)/(C13*X1));
                out_msg.position[1] = C14-X2-X3;

                out_msg.header.stamp = ros::Time::now();
                // write output all angles
                outport_joints.write(out_msg);
            }
        } else if ( input.size() == 3 ){
            // calculate the 2 spindle lengths
            output[0] = sqrt(C1+C2*cos(input[0]+C3));
            output[1] = sqrt(C4+C5*cos(input[2]-C6));
        } else {
            log(Error)<< "ConversionSpindleAngle: Wrong size of vector in input, size should be 2!"<< endlog();
        }

        // write output
        outport.write(output);

    } else {
        log(Debug)<<"ConversionSpindleAngle: No new data recieved"<<endlog();
    }

}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::ConversionSpindleAngle)
