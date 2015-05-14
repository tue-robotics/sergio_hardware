#ifndef TORSOFEEDFORWARD_HPP
#define TORSOFEEDFORWARD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>


using namespace std;
using namespace RTT;

namespace SERGIOCUSTOM
{
  typedef vector<double> doubles;
  typedef vector<string> strings;
  typedef vector<bool>   bools;

  class TorsoFeedForward
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    OutputPort<doubles> outport;
    InputPort<doubles> inport;
    InputPort<doubles> inport_vel;

    // Declaring messages
    doubles output;
    doubles input;
    doubles input_vel;

    // Declaring global variables
    doubles angles;
    doubles forces;
    doubles angleForce;

    // Declaring global variables
    double X1; // BG(q0)
    double X1_sq; // BG(q0)^2
    double X2; // an(A,G,B)
    double X3; // an(B,G,D)
    double dq1_dq0; // Partial derivative dq1/dq0

    // Declaring global constants
    double C1; // AE^2+AC^2
    double C2; // -2*AE*AC
    double C3; // an(C,A,zero)
    double C4; // HJ^2+JK^2
    double C5; // -2*HJ*JK
    double C6; // an(G,J,H)+an(K,J,L)
    double C7; // AB^2+AG^2
    double C8; // 2*AB*AG
    double C9; // an(B,A,zero)
    double C10; // AB^2-AG^2
    double C11; // -2*AG
    double C12; // BD^2-GD^2
    double C13; // -2*GD
    double C14; // an(J,G,D)

    double C24; // HJ^2-JK^2
    double C25; // -2*JK
    double C26; // AC^2-AE^2
    double C27; // -2*AE
    double C28; // JK*r_sp*r_gear1
    double C29; // AE*r_sp*r_gear2

    double g;   // gravity constant
    double gravity_offset;
    double l1; // length link 1
    double l2; // length link 2
    double l3; // length link 3

    double P1; // estimate (m1*lcm1+m2*l1+m3*l1)
    double P2; // estimate (m2*lcm2+m3*l2)
    double P3; // estimate (m3*lcm3)
    double Kmu1; // estimate proportional friction leg
    double Kc1; // estimate coulomb friction leg
    double Kc2; // estimate coulomb friction trunk
    double Kd2; // estimate direction independent friction trunk

    doubles params_q0q1; // array containing parameters of the polynomial estimate



    // variables set by properties
    double m_arms;

    public:

    TorsoFeedForward(const string& name);
    ~TorsoFeedForward();

    bool configureHook();
    bool startHook();
    void updateHook();

    private:

    double change_q1_q0( double q0 );
    };
}
#endif
