#ifndef INITCALIPHERTOSPINDLE_HPP
#define INITCALIPHERTOSPINDLE_HPP

#define maxN 10 //Maximum matrix size

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>

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
   * @brief A Component that initializes the encoder component to the initial
   *    spindle length
   *
   * The component has an input from the caliphers and translate this to an
   * spindle length output. The safe output is used to block the system until
   * the encoder is initialized.
   *
   * @param * offset_caliphers [0.0, 0.0] - offset caliphers to spring length.
   */

    class InitCalipherToSpindle
            : public RTT::TaskContext
    {
    private:

        // Declaring input- and output_ports
        OutputPort<bool> out_safe;
        OutputPort<doubles> out_reset;
        InputPort<soem_beckhoff_drivers::AnalogMsg>  inport_caliphers;

        // Declaring properties
        doubles offset_caliphers;

        // Declaring of messages
        soem_beckhoff_drivers::AnalogMsg caliphers_msg;
        doubles output;
        bool safe;

        // Declaring global variables
        bool output_written;    // used to wait 1 update hook to go to safety
        double X1;              // BG(q0)
        double X1_sq;           // BG(q0)^2
        double X2;              // an(A,G,B)
        double X3;              // an(B,G,D)
        double X4;              // length spring 1
        double X5;              // angle joint 0
        double X6;              // length spring 2
        double X7;              // angle joint 2

        // Declaring global constants
        //  Numbering the same as in ConversionSpindleAngle.hpp
        double C1;              // AE^2+AC^2
        double C2;              // -2*AE*AC
        double C3;              // an(C,A,zero)
        double C4;              // HJ^2+JK^2
        double C5;              // -2*HJ*JK
        double C6;              // an(G,J,H)+an(K,J,L)
        double C9;              // an(B,A,zero)
        double C15;             // AB^2+AF^2
        double C16;             // -2*AB*AF
        double C17;             // IJ^2+JK^2
        double C18;             // -2*IJ*JK
        double C19;             // an(G,J,I)+an(K,J,L)

    public:

        InitCalipherToSpindle(const string& name);
        ~InitCalipherToSpindle();

        bool configureHook();
        bool startHook();
        void updateHook();

    };
}
#endif
