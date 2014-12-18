#ifndef CONVERSIONSPINDLEANGLE_HPP
#define CONVERSIONSPINDLEANGLE_HPP

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sensor_msgs/JointState.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
    stringstream ss;
    ss << t;
    return ss.str();
};

using namespace RTT;

namespace SERGIOCUSTOM
{
    typedef vector<double> doubles;

/**
   * @brief A Component that converts spindle lenth to joint angles or vice versa.
   *
   * The component converts 2 spindle length inputs to 2 angle outputs and
   * optional an extra 3 angle outputs. The component can also be used to
   * convert 3 angle inputs to 2 spindle lengt outputs.
   *
   * @param * spindle_to_angle [true] - input length to output angle
   *        * output_knee_angle [false] - use additional output with 3 joint angles
   */

    class ConversionSpindleAngle
            : public RTT::TaskContext
    {
    private:

        // Declaring input- and output_ports
        InputPort<doubles> inport;
        OutputPort<doubles> outport;
        OutputPort<sensor_msgs::JointState> outport_joints;

        // Declaring properties
        bool spindle_to_angle;
        bool output_joint_states;

        // Declaring messages
        doubles output;
        sensor_msgs::JointState out_msg;

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

    public:

        ConversionSpindleAngle(const string& name);
        ~ConversionSpindleAngle();

        bool configureHook();
        bool startHook();
        void updateHook();

    };
}
#endif
