#include "witmotion_ros.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>

int main(int argc, char *argv[]) {

  QCoreApplication app(argc, argv);
  rclcpp::InitOptions options{};
  options.shutdown_on_signal = true;
  rclcpp::init(argc, argv, options);
  rclcpp::on_shutdown([]() {
    RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"), "Shutting down QT...");
    QCoreApplication::exit(0);
    QThreadPool::globalInstance()->waitForDone();
    RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"),"Shutting down node...");
    if (!rclcpp::shutdown()) {
      RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"), "Shutting down node fail...");
    }
  });

  ROSWitmotionSensorController &controller = ROSWitmotionSensorController::Instance();
  auto node = controller.Start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinThread([&executor, &app]() { executor.spin(); });

  spinThread.detach();
  RCLCPP_INFO(rclcpp::get_logger("MinimalPublisher"), "QT spin !!!!!");
  int result = app.exec();
  return result;
}
