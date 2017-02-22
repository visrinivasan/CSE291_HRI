#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"

#include <depth_image_proc/depth_traits.h>

#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>

int state;
double min_y_; /**< The minimum y position of the points in the box. */
double max_y_; /**< The maximum y position of the points in the box. */
double min_x_; /**< The minimum x position of the points in the box. */
double  max_x_; /**< The maximum x position of the points in the box. */
double max_z_; /**< The maximum z position of the points in the box. */
double goal_z_; /**< The distance away from the robot to hold the centroid */
double z_scale_; /**< The scaling factor for translational robot speed */
double x_scale_; /**< The scaling factor for rotational robot speed */

ros::NodeHandle n;

ros::Subscriber blobsSubscriber;// = n.subscribe("/blobs", 100, blobsCallBack);
ros::Publisher  cmdpub_;// = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
ros::Subscriber sub_;//= n.subscribe<sensor_msgs::Image>("depth/image_rect", 1);
ros::Subscriber blobsub;// = n.subscribe("/blobs", 1);


  /* Blob callback*/
  void blobsCallBack (const cmvision::Blobs& blobsIn){
    if(blobsIn.blob_count >0){
	state=1;
	if(blobsIn.blobs[0].x < 300){
                        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                         cmd->angular.z = 0.4;
                        cmdpub_.publish(cmd);
                }       
                else if(blobsIn.blobs[0].x > 340){
                        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                        cmd->angular.z = -0.4;
                        cmdpub_.publish(cmd);
                }
                else if(blobsIn.blobs[0].area < 95000){
                        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                        cmd->linear.x = 1.0;
                        cmdpub_.publish(cmd);
                }
                else if(blobsIn.blobs[0].area > 95000){
                        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
                }

    }
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for depth images. It finds the centroid
   * of the points in a box in the center of the image. 
   * Publishes cmd_vel messages with the goal from the image.
   * @param cloud The point cloud message.
   */
  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
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
    if (n>4000)
    {
	state=2;
      }

  }

int main(int argc, char **argv)
{
        min_y_=0.1;
	max_y_=0.5;
        min_x_=-0.2;
	max_x_=(0.2);
        max_z_=(0.8); goal_z_=(0.4); z_scale_=(1.0); x_scale_=(5.0);

	state=0;


cmdpub_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
sub_= n.subscribe<sensor_msgs::Image>("depth/image_rect", 1,imagecb);
blobsub = n.subscribe<cmvision::Blobs>("/blobs", 1,blobsCallBack);



	ros::init(argc, argv, "statemachine");
	ros::Rate loop_rate(10);
  while (ros::ok())
  {

/*State machine goes here !!*/

	if(state==0){
		geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        	cmd->angular.z = -0.5;
        	std::cout<<"trying to find target\n";
        	cmdpub_.publish(cmd);
	}
	

	if(state==1){
		std::cout<<"blob detected\n";   
	}
	
	if(state==2){
		cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        	std::cout<<"obstacle detected\n";
	}

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;

}



