#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string BRG_IMG = "BRG_IMG";
//static const std::string GRAY_IMG = "GRAY_IMG";

sensor_msgs::ImagePtr img_msg;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat img_gray;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);

	cv::circle(cv_ptr->image, cv::Point(50, 50), 50, CV_RGB(255,0,0));

	cv::blur(img_gray, img_gray, cv::Size(3,3));

	cv::Canny(img_gray, img_gray, 10, 30, 3);

	//cv::imshow(GRAY_IMG, img_gray);
	//cv::waitKey(3);

	img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gray).toImageMsg();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;

	image_transport::ImageTransport it_(nh);
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &imageCb);
	image_pub_ = it_.advertise("/image_converter/output_video", 1);

	//cv::namedWindow(GRAY_IMG);

	ros::Rate loop_rate(5);
	while(nh.ok())
	{
		image_pub_.publish(img_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	//cv::destroyWindow(GRAY_IMG);
	return 0;
}

