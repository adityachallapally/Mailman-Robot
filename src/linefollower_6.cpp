/**
* Group number : 6
* Student 1: 
* Bryan van Wijk 4363329
* Student 2:
* Dorian de Koning 4348737
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <vector>

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher cmd_vel_pub;
  
public:
  ImageConverter()
    : it_(nh)
  {
     // subscribe to the camera
    image_sub_ = it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this, 
		image_transport::TransportHints("compressed"));
    image_pub_ = it_.advertise("/image_converter/output_video", 1);


    //publish geometry twist messages on the cmd_vel topic
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg){	
//SOME CONSTANTS
    // width of ROI in px
    int regionwidth = 100;
    int regionYOffset = 1200;
    // 1/2 height of region line is allowed to be in
    const float lineregion = 2/10;
	  //convert the incoming msg to a cv mat.
	  cv_bridge::CvImagePtr cv_ptr;
	  cv::Mat img_rgb; 
	  try{
	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img_rgb = cv_ptr->image;
	  }catch (cv_bridge::Exception& e){
    	 ROS_ERROR("cv_bridge exception: %s", e.what());
     	 return;
    }
    cv::Mat dst, cropped, gray,binary, canned;
    // Converting rgb to black/white
    std::vector<cv::Vec4i> lines;
    // Region of interest
    cv::Rect ROI =  cv::Rect(regionYOffset, 0,regionwidth, img_rgb.size().height);
    cropped =  img_rgb(ROI);
    // Getting contrast lines
		cvtColor(cropped,gray,CV_RGB2GRAY);
		threshold( gray, binary, 180 , 255,1);
		cv::Canny(binary, canned, 50, 200, 3);
    cv::HoughLinesP(canned, lines, 1, 0.01, 10, 50, 100);
    geometry_msgs::Twist cmd_vel_msg;
    if(lines.size()>1){
      cv::Vec4i l = lines[0];
      cv::line(cv_ptr->image, cv::Point(l[0], l[1])+ROI.tl(), cv::Point(l[2], l[3])+ROI.tl(), cv::Scalar(0, 255, 255), 3, CV_AA);
      l = lines[1];
      cv::line(cv_ptr->image, cv::Point(l[0], l[1])+ROI.tl(), cv::Point(l[2], l[3])+ROI.tl(), cv::Scalar(0, 255, 255), 3, CV_AA);
      //Drawing contrast lines
      for(int i = 2; i<lines.size(); i++){
        l = lines[i];
       // cv::line(cv_ptr->image, cv::Point(l[0], l[1])+ROI.tl(), cv::Point(l[2], l[3])+ROI.tl(), cv::Scalar(0, 0, 255), 3, CV_AA);
      }
      // Determining which is the higher and wich is the lower line
      int highLineY = lines[0][1] + (0.5 * (lines[0][3]-lines[0][1]));
      int lowLineY = lines[1][1] + (0.5 * (lines[1][3]-lines[1][1]));
      if(lines[0][1] + 0.5 * (lines[0][3]-lines[0][1])<lines[1][1] + (0.5*lines[1][3]-lines[1][1])){
        highLineY = lines[1][1] + (0.5 * lines[1][3]-lines[1][1]);
        lowLineY = lines[0][1] + (0.5 * lines[0][3]-lines[1][1]);
      }
      // Choosing message based on high and low line position
			if(lowLineY<((img_rgb.size().height*3/10)+ROI.tl().y)&&highLineY>((img_rgb.size().height*6/10)+ROI.tl().y)){
				cmd_vel_msg.linear.x = 0.2;
			}else if(lowLineY<((img_rgb.size().height*3/10)+ROI.tl().y)){
        cmd_vel_msg.angular.z = -0.35;
      }else if(highLineY>((img_rgb.size().height*6/10)+ROI.tl().y)){
        cmd_vel_msg.angular.z = 0.35;
      }else{
        cmd_vel_msg.linear.x = 0.2;
      }
      cv::line(cv_ptr->image, ROI.tl() + cv::Point(ROI.size().width/2,highLineY), ROI.tl() +  cv::Point(ROI.size().width/2, lowLineY), cv::Scalar(255, 255, 0), 3, CV_AA);
    }
    cmd_vel_pub.publish(cmd_vel_msg);
    cv::line(cv_ptr->image, cv::Point(0, ROI.size().height*2/10)+ROI.tl(), cv::Point(ROI.size().width, ROI.size().height*2/10)+ROI.tl(), cv::Scalar(255, 0, 0), 3, CV_AA);
    cv::line(cv_ptr->image, cv::Point(0, ROI.size().height/2)+ROI.tl(), cv::Point(ROI.size().width, ROI.size().height/2)+ROI.tl(), cv::Scalar(255, 0, 0), 3, CV_AA);
    cv::line(cv_ptr->image, cv::Point(0, ROI.size().height*8/10)+ROI.tl(), cv::Point(ROI.size().width, ROI.size().height*8/10)+ROI.tl(), cv::Scalar(255, 0, 0), 3, CV_AA);
    cv::rectangle(cv_ptr->image, ROI, cv::Scalar(0, 255, 0), 3, CV_AA, 0);

    image_pub_.publish(cv_ptr->toImageMsg());
	  //Send the geometry twist message on the cmd_vel topic
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}