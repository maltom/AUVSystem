#include <exception>
#include <fstream>
#include <memory>
#include <sstream>

#include <ros/ros.h>

#include "StateMachine.h"

int main (int argc, char** argv)
{
    ros::init(argc,argv,"StateMachine");

     std::shared_ptr<ros::NodeHandle> stateMachineNodePtr = std::make_shared <ros::NodeHandle>();

    std::fstream configFile;
    configFile.open("../../MetaParams.json", std::fstream::in);
    if(!configFile.is_open())
    {
        throw "Can't open config file!";
    }

    StateMachine stateMachine(stateMachineNodePtr, configFile);

}