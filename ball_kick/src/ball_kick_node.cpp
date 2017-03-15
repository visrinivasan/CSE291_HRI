#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
const double PI = 3.14159265359;

int state=1;
int msgFlag;
int bumpflag;

int goalx,goaly,ballx,bally;

ros::Publisher cmdpub_;
ros::Subscriber msgsub_;
ros::Subscriber blobsub;
ros::Subscriber bumper_;

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis
   if (isForward)
	   vel_msg.linear.x =abs(speed);
   else
	   vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   do{
	   cmdpub_.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
	   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   cmdpub_.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
	   do{
		   cmdpub_.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   cmdpub_.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}



double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}



void  bumpercb(const kobuki_msgs::BumperEvent::ConstPtr& bump){
      if(bump->state==1){
        std::cout<<"Bump sensor pressed\n";
        bumpflag=1;
      }
  }


void msgsubcb (const std_msgs::String::ConstPtr& msgIn){
      int temp=std::atoi(msgIn->data.c_str());
      if(temp==2){
        std::cout<<"Move the robot\n";
        msgFlag=1;
      }
  }




/* Blob callback*/
  void blobsCallBack (const cmvision::Blobs& blobsIn){
    std::cout<<"Current State : "<<state<<"\n";	
    if(msgFlag==1){
	if(blobsIn.blob_count >0){
	for(int i=0;i<blobsIn.blob_count;i++){
                if(blobsIn.blobs[i].name=="Red"){
                        ballx=blobsIn.blobs[i].x;
                        bally=blobsIn.blobs[i].y;
                        break;
                }
        }
	for(int i=0;i<blobsIn.blob_count;i++){
		if(blobsIn.blobs[i].name=="Green"){
			goalx=blobsIn.blobs[i].x;
			goaly=blobsIn.blobs[i].y;
			break;
		}
	}
//        std::cout<<"blob detected";   
        if(state==1){
		if(ballx < 280){
//			std::cout<<":: left\n";
                	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                	cmd->angular.z = 0.4;
                	cmdpub_.publish(cmd);
        	}
        	else if(ballx > 360){
//			std::cout<<":: right\n";
                	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                	cmd->angular.z = -0.4;
                	cmdpub_.publish(cmd);
		
        	}
		else if(ballx>280 && ballx<360){
			state=2;
		}
	}

	if(state==2){
		if(goalx>280 && goalx<360){
			state=3;
		}
		else{
			state=4;
		}
	}


        if(state==3){
                geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
                cmd->linear.x = 0.6;
                cmdpub_.publish(cmd);
                if(bumpflag==1){
			cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
                        msgFlag=0;
			bumpflag=0;
			state=1;

                }
        }

	if(state==4){
		geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
		cmd->linear.x=0.0;
		cmd->angular.z=2.0;
		cmdpub_.publish(cmd);
		for(int i=0;i<1000000000;i++){
		;
		}
		cmd->linear.x=0.3;
		cmd->angular.z=0.0;
		cmdpub_.publish(cmd);
		for(int i=0;i<1000000000;i++){
		;
		}
        	cmd->linear.x=0.0;
                cmd->angular.z=-2.0;
                cmdpub_.publish(cmd);

		for(int i=0;i<1000000000;i++){
		;
		}
		state=1;
	
	}		

    }
  else{
//        std::cout<<"No blob detected\n";
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	//cmd->angular.z = -0.2;
        cmdpub_.publish(cmd);
    }
}
}



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

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
