/**
 * Entry point to the ADA Feeding System
 **/
#include <aikido/control/InstantaneousTrajectoryExecutor.hpp>
#include <aikido/robot/ros/RosRobot.hpp>
#include <aikido/rviz.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <dart/common/Uri.hpp>
#include <ros/package.h>
#include <ros/ros.h>

aikido::rviz::InteractiveMarkerViewerPtr gMarkerViewer;

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "aikido_tutorial");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2); // 2 threads
  spinner.start();

  // Create robot
  aikido::planner::WorldPtr env = aikido::planner::World::create("world");
  aikido::robot::ros::RosRobot robot(
      dart::common::Uri("package://ada_description/robots_urdf/ada.urdf"),
      dart::common::Uri("package://ada_description/robots_urdf/ada.srdf"),
      "ada", false);

  // Set world and traj executors
  robot.setWorld(env);
  auto id = robot.registerExecutor(
      std::make_shared<aikido::control::InstantaneousTrajectoryExecutor>(
          robot.getMetaSkeleton()));
  if (!robot.activateExecutor(id))
    throw std::runtime_error("Could not activate executor");

  // Start Visualization
  ROS_INFO("Starting RViz Visualization...");
  std::string vizTopic = nh.param<std::string>("visualization/topicName",
                                               "dart_markers/aikido_tutorial");
  std::string vizBaseFrame =
      nh.param<std::string>("visualization/baseFrameName", "map");
  gMarkerViewer = std::make_shared<aikido::rviz::InteractiveMarkerViewer>(
      vizTopic, vizBaseFrame, robot.getWorld());
  gMarkerViewer->setAutoUpdate(true);

  // Go back and forth between two configurations
  Eigen::VectorXd config = robot.getCurrentConfiguration();
  ros::Rate rate(100); // ROS Rate at 100Hz (10ms)
  while (ros::ok()) {
    // Change the config to go to
    config(0) = 3.14 - config(0);
    config(3) = 3.14 - config(3);
    config(4) = 3.14 - config(4);

    // Plan and execute
    ROS_INFO_STREAM("Planning to configuration " << config.transpose());
    aikido::trajectory::TrajectoryPtr traj = robot.planToConfiguration(config);
    ROS_INFO_STREAM("Executing trajectory of length " << traj->getEndTime());
    std::future<void> trajExecFut = robot.executeTrajectory(traj);

    std::chrono::milliseconds span(1);
    while (ros::ok() &&
           trajExecFut.wait_for(span) == std::future_status::timeout) {
      rate.sleep();
    }

    ros::Duration(0.1).sleep();
    if (!ros::ok())
      break;

    std::string decision = "";
    while (decision != "y" && decision != "n" && decision != "Y" &&
           decision != "N") {
      std::cout << "Plan to configuration complete. Go back (Y/n)? ";
      decision = std::cin.get();
      std::cin.ignore();
    }
    if (decision == "n" || decision == "N")
      break;
  }

  // Cleanup
  gMarkerViewer.reset();
  return 0;
}
