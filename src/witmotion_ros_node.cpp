#include "witmotion_ros.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>




void handle_shutdown(int s)
{
//    RCLCPP_INFO(->get_logger(),"Shutting down node...");
//    rclcpp::shutdown();
    QCoreApplication::exit(0);
}

int main(int argc, char * argv[])
{ 
 
    ROSWitmotionSensorController& controller = ROSWitmotionSensorController::Instance();
    controller.Start();
   
}

