/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gain.hpp
 * Last modification:    March 2011
 */

#ifndef MOTOR_TEST_HPP
#define MOTOR_TEST_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace CUSTOM // Just because it looks nice
{
  /**
   * @brief A Component that adds two vectors elementwise.
   *
   * The component has two input ports that should receive vectors
   * of the same size. The first port is an eventport which will
   * trigger the component.
   *
   * @par Configuration
   * The component is configured using only the 'vectorsize'-property.
   */

  class MotorTest
  : public RTT::TaskContext
    {
    private:

    /**
     * Define a new type doubles for easy coding.
     */
    typedef vector<double> doubles;


    /*
     * Declaring input- and output_ports
     */
    InputPort<doubles> inport_motors;
    InputPort<doubles> inport_encoders;
    OutputPort<doubles> outport_doubles;
    OutputPort<bool> outport_enable1;
    OutputPort<bool> outport_enable2;
    OutputPort<bool> outport_brake1;
    OutputPort<bool> outport_brake2;

    /*
     * Declaring variable vectorsize which is set by a property
     */
    doubles max_vels;
    /*
     * Declaring global variables
     */
    doubles encs_prev;
    doubles output;
    long double old_time;
    bool out_enable1;
    bool out_enable2;
    bool out_brake1;
    bool out_brake2;
    bool vel_limit_exceeded;
    bool print_once;


    public:
    /**
     * Set up a component for adding two vectors.
     */
    MotorTest(const string& name);
    ~MotorTest();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    };
}
#endif
