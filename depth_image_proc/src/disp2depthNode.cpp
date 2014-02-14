#include "DisparityToDepth.h"

int main(int argc, char **argv)
{

ros::init(argc,argv,"disp2depth");
ros::NodeHandle n;

DepthNode *depth_node =  new DepthNode();

ros::Publisher pub_message = n.advertise<sensor_msgs::Image>("rgbd/depth", 1);
ros::Subscriber sub_message = n.subscribe("stereo/disparity",5,&DepthNode::depthCb,depth_node);
//ros::Subscriber rgb_message  = n.subscribe("stereo/left_image_rect",5);
//ros::Subscriber _message  = n.subscribe("stereo/left_image_rect",5);
ros::Rate r(40);

    while(n.ok())
	{
        depth_node->publishMessage(&pub_message);
        pub_message.publish(depth_node->depth_img);
		ros::spinOnce();
		r.sleep();
	}
return 0;
} // end main
