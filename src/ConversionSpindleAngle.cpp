#include "ConversionSpindleAngle.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

ConversionSpindleAngle::ConversionSpindleAngle(const string& name) :
    TaskContext(name, PreOperational),
    spindle_to_angle(true),
    output_knee_angle(false)
{
    // Ports
    addPort( "in", inport );
    addPort( "out", outport );

    // Properties
    addProperty( "spindle_to_angle", spindle_to_angle).doc("bool, true for spindle input and angle output, false means the other way around.");
    addProperty("output_knee_angle", output_knee_angle).doc("Set to true ");
}

ConversionSpindleAngle::~ConversionSpindleAngle(){}

bool ConversionSpindleAngle::configureHook()
{
    // initialize outputs
    output.assign(2,0.0);

    if (output_knee_angle){
        output_angles.assign(3,0.0);
        // add extra output port to publish all angles
        addPort( "out_angles", outport_angles );
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
        if (input.size() == 2){
            if (spindle_to_angle){
                // calculate ankle and hip angles
                output[0] = acos((input[0]*input[0]-C1)/C2)-C3;
                output[1] = acos((input[1]*input[1]-C4)/C5)-C6;

                if( output_knee_angle ){
                    // set the ankle and hip angles
                    output_angles[0] = output[0];
                    output_angles[2] = output[1];
                    // caluclate knee angle
                    X1_sq = C7-C8*cos(output[0]+C9);
                    X1 = sqrt(X1_sq);
                    X2 = acos((C10-X1_sq)/(C11*X1));
                    X3 = acos((C12-X1_sq)/(C13*X1));
                    output_angles[1] = C14-X2-X3;

                    // write output all angles
                    outport_angles.write(output_angles);
                }
            } else {
                // caluulate the 2 spindle lengths
                output[0] = sqrt(C1+C2*cos(input[0]+C3));
                output[1] = sqrt(C4+C5*cos(input[1]-C6));
            }

            // write output
            outport.write(output);

        } else {
            log(Error)<< "ConversionSpindleAngle: Wrong size of vector in input, size should be 2!"<< endlog();
        }
    } else {
        log(Debug)<<"ConversionSpindleAngle: No new data recieved"<<endlog();
    }

}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::ConversionSpindleAngle)
