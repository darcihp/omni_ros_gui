#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


std::string path = ros::package::getPath("omni_ros_gui");

int main(int argc, char **argv)
{
	ros::init(argc, argv, "n_view_img_raw");
	ros::NodeHandle n;

	cv::Mat image;
	image = cv::imread(path + "/src/oi.png", CV_LOAD_IMAGE_COLOR);

	if(! image.data)
	{
		ROS_INFO("Not OK");
		return -1;
	}

	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
	cv::imshow( "Display window", image );
	cv::waitKey(0);

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ROS_INFO("Olá mundo!");
		ros::spinOnce();
		loop_rate.sleep();
	}

	return(0);

}
