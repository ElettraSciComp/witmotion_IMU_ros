#include "witmotion_ros.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

void handle_shutdown(int s)
{
    ROS_INFO("Shutting down node...");
    ros::shutdown();
    QCoreApplication::exit(0);
}

int main(int argc, char** args)
{ 
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_shutdown;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    QCoreApplication app(argc, args);
    ros::init(argc, args, "witmotion_ros", ros::InitOption::NoSigintHandler);
    ROSWitmotionSensorController& controller = ROSWitmotionSensorController::Instance();
    controller.Start();
    ros::AsyncSpinner spinner(2);
    spinner.start();
    int result = app.exec();
    ros::waitForShutdown();
    return result;
}

