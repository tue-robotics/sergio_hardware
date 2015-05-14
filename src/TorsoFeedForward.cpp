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
    double tmp = 0.0;

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
    C24 = 0.162258;
    C25 = -0.157983;
    C26 = -0.096279;
    C27 = -0.701898;
	C28 = 1075.352975;
	C29 = 2756.347553;
    g    = 9.81;
    gravity_offset = 0.0349;
    l1 = 0.389978;
    l2 = 0.411498;
    l3 = 0.469972;

    // system estimates
//    P1 = 2.7955;
    P1 = 6.58;
    P2 = 4.32;
    P3 = 2.47;
    Kmu1 = 0.288;
    Kc1 = 0.0075;
    Kc2 = 0.029;
    Kd2 = 0.0097;

    // polynomial values
    params_q0q1.assign(5,0.0);
    params_q0q1[4] = 11.68;
    params_q0q1[3] = -24.33;
    params_q0q1[2] = 20.18;
    params_q0q1[1] = -7.847;
    params_q0q1[0] = 2.957;

    return true;
}

bool TorsoFeedForward::startHook()
{
    // initialize messages
    input.assign(2,0.0);
    input_vel.assign(2,0.0);
    output.assign(2,0.0);
    angles.assign(3,0.0);
    forces.assign(2,0.0);
    angleForce.assign(2,0.0);

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
		
        //log(Error)<< "Angle 0 = "<<angles[0]<< ", Angle 1 = "<<angles[1]<<'Angle 2 '<< angles[2]<<endlog();
        // angle at which the force is applied to the trunk
        angleForce[1] = acos((C24-input[1]*input[1])/(C25*input[1]));
        angleForce[0] = acos((C26-input[0]*input[0])/(C27*input[0]));
        

        // current gear ratio between angle0 and angle1;
        dq1_dq0 = change_q1_q0(angles[0]);
        

        // Calculate gravity compensation force needed
//        forces[1] = (P3+0.47*m_arms)*124.19*cos(angles[2]+angles[0]-angles[1])/sin(angleForce);
        forces[1] = g*(P3+l3*m_arms)*cos(angles[2]+angles[0]-angles[1]+gravity_offset)/(sin(angleForce[1])*C28);
        forces[0] = g*(cos(angles[0])*(P1+l1*m_arms)+(dq1_dq0-1)*cos(angles[1]-angles[0])*(P2+l2*m_arms)+(1-dq1_dq0)*cos(angles[2]-angles[1]+angles[0]+gravity_offset)*(P3+l3*m_arms))/(sin(angleForce[0])*C29);

        
        // TODO. add spring forces

		
        // Add friction forces
//        forces[1] = forces[1]+91.9194;            // direction independant friction term
        forces[1] = forces[1]+Kd2;            // direction independent friction term
        inport_vel.read(input_vel);
        if (input_vel[1]>0){                    // Coulomb friction term
            forces[1] = forces[1]+Kc2;
        } else if (input_vel[1]<0){
            forces[1] = forces[1]-Kc2;
        }
        if (input_vel[0]>0){
            if (forces[0]>0){
                forces[0] = forces[0]*(1+Kmu1);
            } else {
                forces[0] = forces[0]*(1-Kmu1);
            }
            forces[0] += Kc1;
        } else if (input_vel[0] <0 ) {
            if (forces[0]>0){
                forces[0] = forces[0]*(1-Kmu1);
            } else {
                forces[0] = forces[0]*(1+Kmu1);
            }
            forces[0] -= Kc1;
        }

        // convert to output motor torque
//        output[1] = forces[1]*0.00025156;
        output[1] = forces[1];
        output[0] = forces[0];
        log(Debug)<< "Torque 1 = "<<output[0]<< ", Torque 2 = "<<output[1]<<endlog();

    }

    // write output to port
    outport.write(output);
}

double TorsoFeedForward::change_q1_q0(double q0)
{
    double out = params_q0q1[0];
    double q0_pow = q0;
    for (int i=1; i<params_q0q1.size(); i++){
         out += params_q0q1[i]*q0_pow;
         q0_pow = q0_pow*q0;
    }
    return out;
}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::TorsoFeedForward)
