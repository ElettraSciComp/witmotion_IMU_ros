#include "witmotion_ros.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void handle_shutdown(int s) {

  RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"), "Shutting down node...");
  rclcpp::shutdown();
  QCoreApplication::exit(0);
}

int main(int argc, char *argv[]) {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = handle_shutdown;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  QCoreApplication app(argc, argv);

  rclcpp::init(argc, argv);

  ROSWitmotionSensorController &controller = ROSWitmotionSensorController::Instance();
  auto node = controller.Start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinThread([&executor, &app]() {
    executor.spin();
  });

  spinThread.detach();
  RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"), "QT spin !!!!!");
  int result = app.exec();

  return result;

}
