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
    double angleForce;

    // Declaring global variables
    double X1; // BG(q0)
    double X1_sq; // BG(q0)^2
    double X2; // an(A,G,B)
    double X3; // an(B,G,D)

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


    // variables set by properties
    double m_arms;

    public:

    TorsoFeedForward(const string& name);
    ~TorsoFeedForward();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
