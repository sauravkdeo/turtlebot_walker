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
 *  @file    main.cpp
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

#include "turtlebot_walker/walker.h"

/**
 * @brief      main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     nothing
 */
int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "walker");
  // Create the walker object
  Walker walker;
  // Run the walker behaviour
  walker.runRobot();
  return 0;
}
