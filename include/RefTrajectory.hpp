#ifndef REFTRAJECTORY_HPP
#define REFTRAJECTORY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <fstream>


using namespace std;
using namespace RTT;

namespace SERGIOCUSTOM
{
  typedef vector<double> doubles;
  typedef vector<string> strings;
  typedef vector<bool>   bools;

  class RefTrajectory
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    OutputPort<doubles> outport;

    // Declaring messages
    doubles output;

    // Declaring global variables
    doubles ref1, ref2;
    doubles min_pos, max_pos;
    bool start_ref;
    uint n_ref;
    string file;
    uint time;

    public:

    RefTrajectory(const string& name);
    ~RefTrajectory();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
