#include "DisparityToDepth.h"

DepthNode::DepthNode()
{
}


DepthNode::~DepthNode()
{
}
void DepthNode::depthCb(const stereo_msgs::DisparityImage& disp_msg)
{
namespace enc = sensor_msgs::image_encodings;
  // All new depth image
  //sensor_msgs::ImagePtr depth_msg(new sensor_msgs::Image);
  //
  //depth_msg.header = disp_msg->header;
  this->depth_img.header = disp_msg.header;
    this->disp_img.header = disp_msg.header;

  //
  //depth_msg->encoding = enc::TYPE_16UC1;// previously enc::TYPE_32FC1;
  this->depth_img.encoding = enc::TYPE_32FC1;
  this->disp_img.image.encoding = enc::TYPE_32FC1;
  //
  //depth_msg->height = disp_msg->image.height;
  this->depth_img.height = disp_msg.image.height;
  this->disp_img.image.height = disp_msg.image.height;
  //
//  depth_msg->width = disp_msg->image.width;
  this->depth_img.width = disp_msg.image.width;
  this->disp_img.image.width = disp_msg.image.width;
  //
 // depth_msg->step = disp_msg->image.width*sizeof(unsigned short);
  this->depth_img.step = disp_msg.image.width*sizeof(float);
this->disp_img.image.step = disp_msg.image.step;
this->disp_img.f = disp_msg.f;
this->disp_img.T = disp_msg.T;
this->disp_img.delta_d = disp_msg.delta_d;
this->disp_img.image.data = disp_msg.image.data;
  //printf(" step size is: %u",depth_msg->step);
  //
//  depth_msg->data.resize(depth_msg->height*depth_msg->step,0.0f);
  this->depth_img.data.resize(this->depth_img.height*this->depth_img.step,0.0f);
  
  //this->depth_img = depth_msg;
  //this->disp_img = disp_msg;
  
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
/*
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

  //pub_depth_.publish(depth_msg );
  */
}
 void DepthNode::publishMessage(ros::Publisher *pub_message)
 {
 namespace enc = sensor_msgs::image_encodings;
 
 if (this->depth_img.encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(this->depth_img, this->disp_img);
  }
  else if (this->depth_img.encoding== enc::TYPE_32FC1)
  {
    convert<float>(this->depth_img, this->disp_img);
  }
 }
template<typename T>
void DepthNode::convert( sensor_msgs::Image& depth_msg,
                               const stereo_msgs::DisparityImage& disp_msg)
{
  // For each depth Z, disparity d = fT / Z
  //   scales 1 to meters (see depth_traits.h), always a float.
  //     uint16 depths are in mm so unit_scaling is 0.001f
  //     float  depths are already in m, so it's 1
 // float unit_scaling = DepthTraits<T>::toMeters( T(1) );
 float unit_scaling = depth_image_proc::DepthTraits<T>::fromMeters( T(1) );
  
  // [focal length] * [baseline] / [unit scale]
  float constant = disp_msg.f * disp_msg.T / unit_scaling;

  // re-cast the DepthImage image data to one loooong array
  //const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  //
  const T* disp_row = reinterpret_cast<const T*>(&disp_msg.image.data[0]);
  
  // row step size in pixels, from row step size in bytes
  //int row_step = depth_msg->step / sizeof(T);
  int row_step = disp_msg.image.step/sizeof(T);
  
  // re-cast the DisparityImage image data to one looong array
 // float* disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);// will be unused
  //
  float* depth_data = reinterpret_cast<float*>(&depth_msg.data[0]);
  
  // use disp_msg->image.height and width for loop bounds
  for (int v = 0; v < (int)disp_msg.image.height; ++v)
  {
    for (int u = 0; u < (int)disp_msg.image.width; ++u)
    {
      //pull out a pixel
      //T depth = depth_row[u];
      T disp = disp_row[u];
      //if the depth isn't NaN (float) or 0 (int), set it
      if (depth_image_proc::DepthTraits<T>::valid(disp))
       // *disp_data = constant / depth;
       *depth_data  = constant / disp;
      ++depth_data;	//increment pointer one pixel
    }

    disp_row += row_step; //increment pointer one row's worth of pixels down the array
  }
}
