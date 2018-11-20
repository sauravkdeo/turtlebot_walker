/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saurav Kumar
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    walker.cpp
 *  @author  Saurav Kumar
 *  @copyright MIT License
 *
 *  @brief Assignment to implement walker behavior for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This program will run the walker node for the turtlebot
 *
 */
#include <iostream>
#include "turtlebot_walker/walker.h"

/**
 * @brief      Constructs the object.
 */
Walker::Walker() {
  ROS_INFO("Creating the walker behaviour...");
  linearVelocity = 0.2;
  angularVelocity = 1.0;
  collision = false;
  velocityPub = n.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 1000,
    &Walker::laserCallback, this);
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  velocityPub.publish(msg);
}

/**
 * @brief      Destroys the object.
 */
Walker::~Walker() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  velocityPub.publish(msg);
}

/**
 * @brief      Callback for the laser scan data
 *
 * @param      const sensor_msgs::LaserScan::ConstPtr& msg  The message
 * @return     void
 */
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.8) {
      collision = true;
      return;
    }
  }
  collision = false;
}

/**
 * @brief      Returns the collision flag
 * @param      void
 * @return     boolean value for the collision flag
 */
bool Walker::checkObstacle() {
  return collision;
}

/**
 * @brief      Runs the robot
 */
void Walker::runRobot() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (checkObstacle()) {
      ROS_INFO("Obstacle detected in path.Turning");
      msg.linear.x = 0.0;
      msg.angular.z = angularVelocity;
    } else {
      ROS_INFO("Moving Forward");
      msg.angular.z = 0.0;
      msg.linear.x = linearVelocity;
    }

    velocityPub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
