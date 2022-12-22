#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <stdlib.h>
#include <functional>
#include <vector>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using std::placeholders::_1;

class ColorDetection : public rclcpp::Node 
  {
  
    public:
	 ColorDetection() : Node("image_converter") {
	 
	 this->declare_params();
	 
	 
	  // Create a parameter subscriber that can be used to monitor parameter changes
	 param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	 
	 // Set a callback for each integer parameter
	 	 
	 auto cbLH1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbLH1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  LowerH = p.as_int();
	      };
      
	auto cbLS1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbLS1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  LowerS = p.as_int();
	      };
      
	auto cbLV1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbLV1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  LowerV = p.as_int();
	      };
            
	auto cbUH1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbUH1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  UpperH = p.as_int();
	      };
		    
	auto cbUS1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbUS1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  UpperS = p.as_int();
	      };
      
            
	auto cbUV1 = [this](const rclcpp::Parameter & p) {
		RCLCPP_INFO(
		  this->get_logger(), "cbUV1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
		  p.get_name().c_str(),
		  p.get_type_name().c_str(),
		  p.as_int());
		  UpperV = p.as_int();
	      };     
	      
	//save the handle that is returned by add_parameter_callback; 
	//otherwise, the callback will not be properly 	registered.    
	                     
	cb_handle_1 = param_subscriber_->add_parameter_callback("LowerH", cbLH1);
	cb_handle_2 = param_subscriber_->add_parameter_callback("LowerS", cbLS1);
	cb_handle_3 = param_subscriber_->add_parameter_callback("LowerV", cbLV1);
	cb_handle_4 = param_subscriber_->add_parameter_callback("UpperH", cbUH1);
	cb_handle_5 = param_subscriber_->add_parameter_callback("UpperS", cbUS1);
	cb_handle_6 = param_subscriber_->add_parameter_callback("UpperV", cbUV1);

	 sub = this->create_subscription<sensor_msgs::msg::Image>(
	      "image_raw", 10, std::bind(&ColorDetection::colorDetectionCallback, this, _1));
	      

	}
	~ColorDetection() {
	    
		cv::destroyWindow("Image Processed");
	    }
 
	 void declare_params() {
		 // declares 6 integer parameter for HSV range values
		 LowerH = this->declare_parameter("LowerH", 0);
		 LowerS = this->declare_parameter("LowerS", 70);
		 LowerV = this->declare_parameter("LowerV", 50);
		 UpperH = this->declare_parameter("UpperH", 10);
		 UpperS = this->declare_parameter("UpperS", 255);
		 UpperV = this->declare_parameter("UpperV", 255);
 
  	}
  
  
  
  
   private:


	int LowerH;
	int LowerS;
	int LowerV;
	int UpperH;
	int UpperS;
	int UpperV;
		

         cv::Mat imgOriginal; 
         
         // Declare subscriber
	 rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
	 
	 //ParameterEventHandler to monitor parameter changes
	 std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
	 
	 //callback handle
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_1;
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_2;
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_3;
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_4;
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_5;
	 std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_6;
  
  	// callback for colorDetection
  
	void colorDetectionCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
	
	 cv_bridge::CvImagePtr cv_ptr;
	 
	 
	  try {
	  
		    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		    cv::waitKey(10);
	    
	  } 
	  catch (const cv_bridge::Exception & e) {
	  
		    auto logger = rclcpp::get_logger("Image_converter");
		    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
	  
        
		cv::Mat img_mask1,img_hsv,img_final;
		
		//convert the image to HSV colorspace
		cv::cvtColor(cv_ptr->image,img_hsv,CV_RGB2HSV);
		
		// Detect the object based on HSV Range Values
		cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask1); 
		
				
		cv::RNG rng(12345);
		
		//Create new Mat of unsigned 8-bit chars, filled with zeros
		img_final = cv::Mat::zeros( cv_ptr->image.size(), CV_8UC3 );;
		
		//Finds contours and saves them to the vectors contour and hierarchy
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		
		// detect the contours using cv2.CHAIN_APPROX_SIMPLE
		cv::findContours( img_mask1, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
		cv::imshow("Contours_Image", img_mask1);
		
		cv::Mat image_copy = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
		
	        std::vector<cv::Rect> boundRect( contours.size() );
		std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
		
		//Create new Mat of unsigned 8-bit chars, filled with zeros
		cv::Mat drawing = cv::Mat::zeros( image_copy.size(), CV_8UC3 );
		
		
		for( size_t i = 0; i < contours.size(); i++ ) { // iterate through each contour.
		
			approxPolyDP( contours[i], contours_poly[i], 3, true );
			
			//we find a bounding rect for every polygon and save it to boundRect
			boundRect[i] = cv::boundingRect( contours_poly[i] );
			
		// Find the bounding rectangle for biggest contour	
		 if (cv::contourArea(contours[i]) > 8000) {
		 
				
			cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
			cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
			
		}
		
		}
			
			
			img_final = drawing + image_copy;
			// draw contours on the original image
			cv::imshow("Final_Image", img_final);
			//Add some delay in miliseconds
       	 		cv::waitKey(3);
              
       }
       
         
  };



  int main(int argc, char** argv)
  {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<ColorDetection>());
      rclcpp::shutdown();
      return 0;
  }

