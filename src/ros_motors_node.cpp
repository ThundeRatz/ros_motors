/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <ros/ros.h>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros_motors/Motor.h>
#include "ros_motors/serial.h"
#include "ros_motors/errno_string.h"

#define INIT_VMAX 255
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(x, min , max) ((x) < (min) ? (min) : (x) > (max) ? (max) : (x))

#define SERVO_OFFSET 23

class MotorNode
{
  public:
    MotorNode();
    void spin();
    void motors_callback(const ros_motors::Motor motor_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber motors_sub;

    void init_serial();
    void drive(int servo, int speed);

    int max_speed = INIT_VMAX;
    int servo = 0, speed = 0, cur_speed = 80;

    int fd;
    const bool use_serial = true;
    const int baudrate = 9600;

    void set_max_speed(int new_max_speed);
    void reset_max_speed();
    void stop(int servo_offset, int vel);
};

MotorNode::MotorNode() : nh_()
{
  motors_sub = nh_.subscribe("motor", 1, &MotorNode::motors_callback, this);
  init_serial();
}

void MotorNode::init_serial()
{
  fd = serial_open("/dev/arduino", &baudrate, O_WRONLY);
  while (fd == -1 && ros::ok())
  {
    ROS_INFO("Serial open failed");
    fd = serial_open("/dev/arduino", &baudrate, O_WRONLY);
    ros::Duration(0.01).sleep();
  }
}

void MotorNode::motors_callback(const ros_motors::Motor motor_msg)
{
  if (motor_msg.header.stamp == ros::Time(0))
  {
    ROS_INFO("Invalid motor message.");
    return;
  }
  servo = motor_msg.servo_angle;
  speed = motor_msg.speed;
}

void MotorNode::drive(int servo, int speed)
{
  servo += SERVO_OFFSET;
  servo = constrain(servo, -255, 255);
  speed = constrain(speed, -255, 255);

  uint8_t message[6] = { 255, servo < 0, abs(servo), speed < 0, abs(speed), 254 };
  
  if (write(this->fd, message, sizeof(message)) == -1)
  {
    std::cerr << "write: " << errno_string() << std::endl;
    close(this->fd);
    init_serial();
    fsync(this->fd);
  }
  cur_speed = speed;
}

void MotorNode::set_max_speed(int new_max_speed)
{
  max_speed = new_max_speed > 255 ? 255 : new_max_speed;
}

void MotorNode::spin()
{
  int max_speed_par;
  nh_.param("max_speed", max_speed_par, INIT_VMAX);

  while (ros::ok())
  {
    ros::spinOnce();

    if (nh_.getParam("max_speed", max_speed_par) && max_speed_par != max_speed)
    {
      set_max_speed(max_speed_par);
    }

    drive(this->servo, this->speed);
    ros::Rate r(300.0);
    r.sleep();
  }
}

void MotorNode::stop(int servo_offset, int vel)
{
  cur_speed = vel;
  while (cur_speed != 0)
  {
    drive(servo_offset, cur_speed--);
    ros::Duration(0.01).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_subscriber");

  MotorNode motor;
  motor.spin();

  return 0;
}
