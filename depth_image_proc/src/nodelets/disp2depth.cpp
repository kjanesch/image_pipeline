/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include "depth_traits.h"

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class DepthNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> left_it_;
  ros::NodeHandlePtr right_nh_;
  //image_transport::SubscriberFilter sub_depth_image_;
 //ros::Subscriber<stereo_msgs:Disparity> sub_disp_;
 ros::Subscriber sub_disp_;
 // message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  
  //typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> Sync;
  //boost::shared_ptr<Sync> sync_;
  
  boost::mutex connect_mutex_;
  //ros::Publisher pub_disparity_;
  ros::Publisher pub_depth_;
  double min_range_;
  double max_range_;
  double delta_d_;

  virtual void onInit();

  void connectCb();

  void depthCb(const stereo_msgs::DisparityImageConstPtr& disp_msg);

  template<typename T>
  void convert(sensor_msgs::ImagePtr& depth_msg,
               const stereo_msgs::DisparityImageConstPtr& disp_msg);
};

void DepthNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  ros::NodeHandle left_nh(nh, "left");
  left_it_.reset(new image_transport::ImageTransport(left_nh));
  right_nh_.reset( new ros::NodeHandle(nh, "right") );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  /*
  private_nh.param("min_range", min_range_, 0.0);
  private_nh.param("max_range", max_range_, std::numeric_limits<double>::infinity());
  private_nh.param("delta_d", delta_d_, 0.125); 
 */
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  //sync_.reset( new Sync(sub_depth_image_, sub_info_, queue_size) );
 // sync_->registerCallback(boost::bind(&DisparityNodelet::depthCb, this, _1, _2));

  // Monitor whether anyone is subscribed to the output
 // ros::SubscriberStatusCallback connect_cb = boost::bind(&DepthNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
 // boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //pub_depth_ = left_nh.advertise<sensor_msgs::Image>("depth", 1, connect_cb, connect_cb);
   pub_depth_ = left_nh.advertise<sensor_msgs::Image>("depth", 1);
   sub_disp_ = nh.subscribe("disparity",1,&DepthNodelet::depthCb,this);
}

// Handles (un)subscribing when clients (un)subscribe
void DepthNodelet::connectCb()
  
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  /*
  if (pub_depth_.getNumSubscribers() == 0)
  {
    //sub_disp_.unsubscribe();
    //sub_info_ .unsubscribe();
  }
  else if (!pub_depth_.getSubscriber())
  {
    //image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    //sub_depth_image_.subscribe(*left_it_, "image_rect", 1, hints);
    //sub_info_.subscribe(*right_nh_, "camera_info", 1);
    sub_disp_ = right_nh_.subscribe("disparity",1,depthCB);
  }
  */
  //sub_disp_ = nh.subscribe("disparity",1,depthCB);
}

//void DisparityNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                               //const sensor_msgs::CameraInfoConstPtr& info_msg)
 void DepthNodelet::depthCb(const stereo_msgs::DisparityImageConstPtr& disp_msg)
{
  // All new depth image
  sensor_msgs::ImagePtr depth_msg(new sensor_msgs::Image);
  //
  depth_msg->header = disp_msg->header;
  //
  depth_msg->encoding = enc::TYPE_16UC1;// previously enc::TYPE_32FC1;
  //
  depth_msg->height = disp_msg->image.height;
  //
  depth_msg->width = disp_msg->image.width;
  //
  depth_msg->step = disp_msg->image.width*sizeof(unsigned short);
  //printf(" step size is: %u",depth_msg->step);
  //
  depth_msg->data.resize(depth_msg->height*depth_msg->step,0.0f);
  
  /*
  // Allocate new DisparityImage message
  stereo_msgs::DisparityImagePtr disp_msg( new stereo_msgs::DisparityImage );
  // 
  disp_msg->header         = depth_msg->header;
  // 
  disp_msg->image.header   = disp_msg->header;
  // set encoding to float32
  disp_msg->image.encoding = enc::TYPE_32FC1;
  //
  disp_msg->image.height   = depth_msg->height;
  // 
  disp_msg->image.width    = depth_msg->width;
  // 
  disp_msg->image.step     = disp_msg->image.width * sizeof (float);
  */
  
  //resize image to handle 32-bit floats based on image size ( #rows * row size )
  //disp_msg->image.data.resize( disp_msg->image.height * disp_msg->image.step, 0.0f );// refactor for depth?
  // pull focal length from CameraInfo topic
  //double fx = info_msg->P[0];         //ignore
  //calculate baseline from P[3] (Tx) and focal length
 // disp_msg->T = -info_msg->P[3] / fx; //remove when reversing
  //disp_msg->f = fx;                   //remove when reversing
  // Remaining fields depend on device characteristics, so rely on user input
 // disp_msg->min_disparity = disp_msg->f * disp_msg->T / max_range_; //remove when reversing
  //disp_msg->max_disparity = disp_msg->f * disp_msg->T / min_range_; //remove when reversing
 // disp_msg->delta_d = delta_d_;

// is this based on output or input images?
// will assume output for now...
  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, disp_msg);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_msg, disp_msg);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", disp_msg->image.encoding.c_str());
    return;
  }

  pub_depth_.publish(depth_msg );
}

template<typename T>
void DepthNodelet::convert( sensor_msgs::ImagePtr& depth_msg,
                               const stereo_msgs::DisparityImageConstPtr& disp_msg)
{
  // For each depth Z, disparity d = fT / Z
  //   scales 1 to meters (see depth_traits.h), always a float.
  //     uint16 depths are in mm so unit_scaling is 0.001f
  //     float  depths are already in m, so it's 1
 // float unit_scaling = DepthTraits<T>::toMeters( T(1) );
 float unit_scaling = DepthTraits<T>::fromMeters( T(1) );
  
  // [focal length] * [baseline] / [unit scale]
  float constant = disp_msg->f * disp_msg->T / unit_scaling;

  // re-cast the DepthImage image data to one loooong array
  //const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  //
  const T* disp_row = reinterpret_cast<const T*>(&disp_msg->image.data[0]);
  
  // row step size in pixels, from row step size in bytes
  //int row_step = depth_msg->step / sizeof(T);
  int row_step = disp_msg->image.step/sizeof(T);
  
  // re-cast the DisparityImage image data to one looong array
 // float* disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);// will be unused
  //
  float* depth_data = reinterpret_cast<float*>(&depth_msg->data[0]);
  
  // use disp_msg->image.height and width for loop bounds
  for (int v = 0; v < (int)disp_msg->image.height; ++v)
  {
    for (int u = 0; u < (int)disp_msg->image.width; ++u)
    {
      //pull out a pixel
      //T depth = depth_row[u];
      T disp = disp_row[u];
      //if the depth isn't NaN (float) or 0 (int), set it
      if (DepthTraits<T>::valid(disp))
       // *disp_data = constant / depth;
       *depth_data  = constant / disp;
      ++depth_data;	//increment pointer one pixel
    }

    disp_row += row_step; //increment pointer one row's worth of pixels down the array
  }
}

} // namespace depth_image_proc

// Register as nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::DepthNodelet,nodelet::Nodelet);

