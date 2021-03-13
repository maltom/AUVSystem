#include <external/jsonxx/jsonxx.h>
#include <ros/ros.h>

#include "StateMachine.h"

int main (int argc, char** argv)
{
    ros::NodeHandle stateMachineNode;

    StateMachine stateMachine(stateMachineNode);

}