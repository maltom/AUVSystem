#include "MainLogic.h"



void MainLogic::startMainLoop() const
{
    while(ros::ok())
    {
        ros::spinOnce();
        stateMachine->process();
    }
    return;
}