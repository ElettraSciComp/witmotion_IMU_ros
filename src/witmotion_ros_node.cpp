#include "witmotion_ros.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>




void handle_shutdown(int s)
{
//    RCLCPP_INFO(->get_logger(),"Shutting down node...");
    rclcpp::shutdown();
    QCoreApplication::exit(0);
}

int main(int argc, char * argv[])
{ 
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_shutdown;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    QCoreApplication app(argc, argv);
    rclcpp::init(argc, argv);

    ROSWitmotionSensorController& controller = ROSWitmotionSensorController::Instance();
    auto node = controller.Start();

    rclcpp::executors::MultiThreadedExecutor executor;
    auto executor_spin_lambda = [&executor]() {
      executor.spin();
    };
    
    executor.add_node(node);
    {
        std::thread spin_thread(executor_spin_lambda);
        spin_thread.join();
    }
   int result = app.exec();
   return result;
}

