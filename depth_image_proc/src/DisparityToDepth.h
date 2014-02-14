#ifndef SR_DISPARITYTODEPTH_H
#define SR_DISPARITYTODEPTH_H

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include "nodelets/depth_traits.h"


class DepthNode
{
	public: 	
		// Constructor
		DepthNode();
		// Destructor
		~DepthNode();
		// The depth Message
		
		//stereo_msgs::DisparityImageConstPtr
		sensor_msgs::Image depth_img;
		stereo_msgs::DisparityImage disp_img;
  template<typename T>
  void convert(sensor_msgs::Image& depth_msg,const stereo_msgs::DisparityImage& disp_msg);
  void depthCb(const stereo_msgs::DisparityImage& disp_msg);
  void publishMessage(ros::Publisher *pub_message);
};

#endif
