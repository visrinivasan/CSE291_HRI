#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <depth_image_proc/depth_traits.h>

const float min_x_ = -0.2;
const float max_x_ = 0.2;
const float min_y_ = 0.1;
const float max_y_ = 0.5;
const float max_z_ = 1.5;
const float goal_z_ = 0.1;


int state;
int msgFlag;
int bumpflag;
int prev;
int turnflag;

ros::Publisher cmdpub_;
ros::Subscriber msgsub_;
ros::Subscriber blobsub;
ros::Subscriber bumper_;
ros::Subscriber depth_;

void  bumpercb(const kobuki_msgs::BumperEvent::ConstPtr& bump){
      if(bump->state==1){
        std::cout<<"Bump sensor pressed\n";
        bumpflag=1;
      }
  }


void msgsubcb (const std_msgs::String::ConstPtr& msgIn){
      int temp=std::atoi(msgIn->data.c_str());
      if(temp==3){
        std::cout<<"Move the robot\n";
        msgFlag=1;
      }
  }




/* Blob callback*/
  void blobsCallBack (const cmvision::Blobs& blobsIn){
    if(blobsIn.blob_count >0){
	if(blobsIn.blobs[0].area > 12000){
		turnflag=1;
	}
	else if(blobsIn.blobs[0].area<12000){
		turnflag=0;
	}
        if(msgFlag==1){
                geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
		if(blobsIn.blobs[0].y<300 && blobsIn.blobs[0].x<280){
			cmd->angular.x=0.5;
			turnflag=1;
		}
		if(blobsIn.blobs[0].y<300 && blobsIn.blobs[0].x>360){
			cmd->angular.x=-0.5;
			turnflag=2;
		}

		if(turnflag==1 || turnflag==2){
			cmdpub_.publish(cmd);
	                cmd->linear.x = 0.3;
        	        cmdpub_.publish(cmd);
			turnflag=0;
		}
                if(bumpflag==1){
                        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
                        msgFlag=0;
                        bumpflag=0;
			turnflag=0;
                }

        }
    }
  else{
//        std::cout<<"No blob detected\n";
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmdpub_.publish(cmd);
	turnflag=0;

    }
  }


/* Pointcloud from follower */

/*
  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

//    std::cout<<"IMAGE CALLBACK\n";
    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    if(msgFlag==1){
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.

/*Move if ball comes closer */
/*
	//std::cout<<"z="<<z<<"\n";
      ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);
      //publishMarker(x, y, z);
      if (z<max_z_)
      {
        std::cout<<"obstacle detected\n";
	turnflag=1;
      }
    }
}
*/

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ball_kick_node");
	
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
    cmdpub_ = n.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);
    msgsub_=n.subscribe<std_msgs::String>("/message_pub",1,msgsubcb);
    blobsub = n.subscribe("/blobs", 1, blobsCallBack);
    bumper_ = n.subscribe("mobile_base/events/bumper",100,bumpercb);
  //  depth_= n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1,imagecb);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
