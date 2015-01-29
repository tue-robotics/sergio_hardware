#include "InitCalipherToSpindle.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

InitCalipherToSpindle::InitCalipherToSpindle(const string& name) :
    TaskContext(name, PreOperational),
    offset_caliphers(2,0.0)
{
    // Ports
    addPort( "in_calipher", inport_caliphers ).doc("Inport to recieve the calipher data as an AnalogMsg");
    addPort( "out", out ).doc("Port that outputs the length of the spindle");
    addPort( "out_reset", out_reset ).doc("Port that sends the reset value to the spindle");
    addPort( "out_safe", out_safe ).doc("Sends bool true when reset command is send");

    // Properties
    addProperty( "offset_caliphers", offset_caliphers).doc("doubles, offset of the calipher, distance from calipher 0 to spring length 0");

}

InitCalipherToSpindle::~InitCalipherToSpindle(){}

bool InitCalipherToSpindle::configureHook()
{
    // resize input message
    caliphers_msg.values.resize(2);

    // initialize output message
    output.assign(2,0.0);
    safe = false;

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
    C9 = 0.756629;
    C15 = 0.157277;
    C16 = -0.102398;
    C17 = 0.154980;
    C18 = -0.060929;
    C19 = 0.220697;

    return true;
}

bool InitCalipherToSpindle::startHook()
{
    // initialize safe outport to false
    safe = false;
    output_written = false;
    safe_msg.data = safe;
    out_safe.write(safe_msg);

    return true;
}


void InitCalipherToSpindle::updateHook()
{
	if ( inport_caliphers.read(caliphers_msg) == NewData ){
		
		// Correct for overflow (only happens when in the upper endstop)
		if (caliphers_msg.values[0]<0.1)
			caliphers_msg.values[0] += 0.65536;
		if (caliphers_msg.values[1]<0.1)
			caliphers_msg.values[1] += 0.65536;
		
		// calculate spindle 1 length
		X4 = caliphers_msg.values[0]+offset_caliphers[0];
		X5 = acos((X4*X4-C15)/C16)-C9;
		output[0] = sqrt(C1+C2*cos(X5+C3));

		// calculate spindle 2 length
		X6 = caliphers_msg.values[1]+offset_caliphers[1];
		X7 = acos((X6*X6-C17)/C18)+C19;
		output[1] = sqrt(C4+C5*cos(X7-C6));
		
		// write output
		out.write(output);
		
		// reset the encoders
		if ( !output_written ){
			if ( out_reset.connected() ){
                // Write only once to reset the encoders
                out_reset.write(output);
                log(Debug)<<"InitCalipherToSpindle: encoders initialized on "<<output[0]<<" and "<< output[1]<<endlog();
                output_written = true;
            }
		} else {
			safe = true;
		}
	}

	safe_msg.data = safe;
    out_safe.write(safe_msg);
}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::InitCalipherToSpindle)
