#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

class TeleopKeyboard
{
public:
  TeleopKeyboard()
  { 
    cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel_", walk_vel_, 0.02);
    n_private.param("run_vel_", run_vel_, 0.05);
    n_private.param("yaw_rate_", yaw_rate_, 0.0174533 / 2.);
    n_private.param("yaw_run_rate", yaw_rate_run_, 0.0174533);
  }
  
  void keyboardLoop();

private:
  double walk_vel_;
  double run_vel_;
  double yaw_rate_;
  double yaw_rate_run_;
  geometry_msgs::Twist cmd_;
  turtlesim::Pose pose_;

  ros::NodeHandle n_;
  tf::TransformBroadcaster br_;

  void updatePose();
  void publishTransform();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

void TeleopKeyboard::keyboardLoop()
{
  char c;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  int flags = fcntl(kfd, F_GETFL, 0);
  fcntl(kfd, F_SETFL, flags | O_NONBLOCK);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) > 0)
    {
      cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;

      switch(c)
      {
        // Walking
      case KEYCODE_W:
        cmd_.linear.x = walk_vel_;
        break;
      case KEYCODE_S:
        cmd_.linear.x = - walk_vel_;
        break;
      case KEYCODE_A:
        cmd_.linear.y = walk_vel_;
        break;
      case KEYCODE_D:
        cmd_.linear.y = - walk_vel_;
        break;
      case KEYCODE_Q:
        cmd_.angular.z = yaw_rate_;
        break;
      case KEYCODE_E:
        cmd_.angular.z = - yaw_rate_;
        break;

        // Running 
      case KEYCODE_W_CAP:
        cmd_.linear.x = run_vel_;
        break;
      case KEYCODE_S_CAP:
        cmd_.linear.x = - run_vel_;
        break;
      case KEYCODE_A_CAP:
        cmd_.linear.y = run_vel_;
        break;
      case KEYCODE_D_CAP:
        cmd_.linear.y = - run_vel_;
        break;
      case KEYCODE_Q_CAP:
        cmd_.angular.z = yaw_rate_run_;
        break;
      case KEYCODE_E_CAP:
        cmd_.angular.z = - yaw_rate_run_;
        break;
      }

      updatePose();
    }
    
    publishTransform();
  }
}

void TeleopKeyboard::updatePose()
{
  pose_.theta += cmd_.angular.z;
  pose_.x += cos(pose_.theta) * cmd_.linear.x - sin(pose_.theta) * cmd_.linear.y;
  pose_.y += sin(pose_.theta) * cmd_.linear.x + cos(pose_.theta) * cmd_.linear.y;
}

void TeleopKeyboard::publishTransform()
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose_.x, pose_.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, pose_.theta);
  transform.setRotation(q);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "overworld_teleop");

  TeleopKeyboard teleop;

  signal(SIGINT,quit);

  teleop.keyboardLoop();

  return 0;
}
