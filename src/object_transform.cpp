#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <deque>

using namespace Eigen;

bool pose_received;
std::deque<geometry_msgs::PoseStamped> pose_msg_q;

void pose_callback(const geometry_msgs::PoseStampedConstPtr pMsg)
{
	pose_msg_q.push_back(*pMsg);
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "object_transform");
	ros::NodeHandle nh("~");
  	ros::Subscriber pose_sub = nh.subscribe("in_pose", 10, pose_callback);
  	ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

  	double publish_rate;
  	nh.param("publish_rate", publish_rate, 25.0);

  	ros::Rate r(1000.0);
  	ros::Duration pose_interval(1.0/publish_rate);
  	ros::Duration pose_delay(150.0/1000.0);
  	tf::TransformBroadcaster br;
  	tf::Transform trans;


  	ros::Time last_pose_time(0.0);
  	bool first_pose_set = false;
	
  	Matrix3d w_R_m; w_R_m << -1,0,0,0,-1,0,0,0,1;
  	Matrix3d b_R_m; b_R_m << -1,0,0,0,-1,0,0,0,1;

	Vector3d mopti_P_mk;
	Matrix3d mopti_R_mk;
	Vector3d mopti_P_m0;
	Matrix3d mopti_R_m0;
	Matrix3d w_R_bk;
	Vector3d w_P_bk;
	Quaterniond w_q_b;

	geometry_msgs::PoseStamped pose_msg;
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
		if (pose_msg_q.size())
		{
			pose_msg = pose_msg_q.front();
			if ((pose_msg_q.back().header.stamp - pose_msg_q.front().header.stamp) > pose_delay)
			{
				pose_msg_q.pop_front();
			}
			else
			{
				continue;
			}
		}
		else
		{
			continue;
		}

		if ((pose_msg.header.stamp - last_pose_time) < pose_interval)
		{
			continue;
		}
		else
		{
			last_pose_time = pose_msg.header.stamp;
		}

		mopti_P_mk.x() = pose_msg.pose.position.x;
		mopti_P_mk.y() = pose_msg.pose.position.y;
		mopti_P_mk.z() = pose_msg.pose.position.z;
		
		mopti_R_mk = Quaterniond(
			pose_msg.pose.orientation.w,
			pose_msg.pose.orientation.x,
			pose_msg.pose.orientation.y,
			pose_msg.pose.orientation.z).toRotationMatrix();

		if (!first_pose_set)
		{
			mopti_P_m0 = mopti_P_mk;
			mopti_R_m0 = mopti_R_mk;
			ROS_INFO("[OptiTransform] Initialization finished.");
			first_pose_set = true;
		}
		else
		{
			w_R_bk = w_R_m * mopti_R_m0.transpose() * mopti_R_mk * b_R_m.transpose();
			w_P_bk = w_R_m * mopti_R_m0.transpose() * (mopti_P_mk - mopti_P_m0);
			w_q_b = Quaterniond(w_R_bk);

			trans.setOrigin( tf::Vector3(w_P_bk.x(), w_P_bk.y(), w_P_bk.z() ) );
			trans.setRotation( tf::Quaternion(w_q_b.x(), w_q_b.y(), w_q_b.z(), w_q_b.w() ) );
			br.sendTransform(
				tf::StampedTransform(trans, pose_msg.header.stamp, "world", "obj"));

			// trans.setOrigin( tf::Vector3(mopti_P_mk.x(), mopti_P_mk.y(), mopti_P_mk.z() ) );
			// w_q_b = Quaterniond(mopti_R_mk);
			// trans.setRotation( tf::Quaternion(w_q_b.x(), w_q_b.y(), w_q_b.z(), w_q_b.w() ) );
			// br.sendTransform(
			// 	tf::StampedTransform(trans, pose_msg.header.stamp, "world", "obj"));

			nav_msgs::Odometry odom_msg;
			odom_msg.header = pose_msg.header;
			odom_msg.header.frame_id = "world";
			odom_msg.child_frame_id = "null";
			odom_msg.pose.pose.position.x = w_P_bk.x();
			odom_msg.pose.pose.position.y = w_P_bk.y();
			odom_msg.pose.pose.position.z = w_P_bk.z();
			odom_msg.pose.pose.orientation.x = w_q_b.x();
			odom_msg.pose.pose.orientation.y = w_q_b.y();
			odom_msg.pose.pose.orientation.z = w_q_b.z();
			odom_msg.pose.pose.orientation.w = w_q_b.w();
			pose_pub.publish(odom_msg);
		}
	}

	return 0;
}
