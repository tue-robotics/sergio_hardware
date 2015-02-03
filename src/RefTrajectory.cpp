#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <fstream>

#include "RefTrajectory.hpp"

using namespace std;
using namespace RTT;
using namespace SERGIOCUSTOM;

RefTrajectory::RefTrajectory(const string& name) : TaskContext(name, PreOperational)
{
    // Adding Properties:
    addProperty( "ref_size",n_ref).doc("Number of reference points.");
    addProperty("file",file).doc("string conraining file name from root.");
    addProperty("min_ref",min_pos).doc("array containing minimal positions");
    addProperty("max_ref",max_pos).doc("array containing maximal positions");

    // Creating ports:
    addPort( "out", outport );
}
RefTrajectory::~RefTrajectory(){}

bool RefTrajectory::configureHook()
{


    if (n_ref<=0){
        log(Error)<<"RefTrajectory: size of reference to small: size "<<n_ref<<endlog();
        return false;
    }
    //resize vectors
    ref1.resize(n_ref,0.0);
    ref2.resize(n_ref,0.0);

    if (min_pos.size() != 2 || max_pos.size() != 2){
        log(Error)<<"RefTrajectory: size of min_ref and max_ref should be 2"<<endlog();
        return false;
    }

    // load file
    char load_file[100];
    strcpy(load_file,file.c_str());
    std::ifstream myfile(load_file,ios_base::in);

    if(myfile.fail()){
        log(Error)<<"RefTrajectory: file does not exist: "<<file<<endlog();
        return false;
    }

    // define input strings
    char in1[30], in2[30];
    // iterator
    uint i = 0;

    // read file
    while (!myfile.eof() && i<n_ref)
    {
        myfile >> in1 >> in2;
        ref1[i] = atof(in1);
        ref2[i] = atof(in2);
        i++;
    }
    n_ref = i;
    log(Warning)<<"RefTrajectory: Number of reference points is "<<n_ref<<endlog();

    return true;
}

bool RefTrajectory::startHook()
{
    time = 0;
    output.assign(2,0.0);
    output[0] = ref1[0];
    output[1] = ref2[0];

    return true;
}


void RefTrajectory::updateHook()
{
    if (time<n_ref){
        for (uint i=0; i<2; i++){
            output[i] = min(max_pos[i],max(min_pos[i],output[i]));
        }
        
        //write output
        outport.write(output);

        // update output
        output[0] = ref1[time];
        output[1] = ref2[time];

        // update iterator
        time++;
    }
}

ORO_CREATE_COMPONENT(SERGIOCUSTOM::RefTrajectory)
