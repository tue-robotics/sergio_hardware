#ifndef REFGENERATOR_HPP
#define REFGENERATOR_HPP

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

  class RefGenerator
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    OutputPort<geometry_msgs::Twist> outport;

    // Declaring messages
    geometry_msgs::Twist cmd_veldata;

    // Declaring global variables
    uint N;
    double time;
    double switch_time;
    uint step;

    // variables set by properties
    doubles velocity_x;
    doubles velocity_y;
    doubles velocity_phi;
    doubles duration;
    double Ts;

    public:

    RefGenerator(const string& name);
    ~RefGenerator();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
