// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>		// Make sure CMakeLists.txt and package.xml are
						// updated for OpenCV and cv_bridge


static sensor_msgs::Image::ConstPtr dnn_msg;

void dnnCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if (dnn_msg != nullptr)
        return;
    dnn_msg = msg;
}

// Taken from cv_brdige tutorial
static cv_bridge::CvImagePtr cv_img;

void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv_img = cv_ptr;
    
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting redtail_debug ROS node...\n");

    ros::init(argc, argv, "redtail_debug");
    ros::NodeHandle nh("~");

    std::string caffe_ros_topic;
    float       pub_rate;
    nh.param<std::string>("caffe_ros_topic", caffe_ros_topic, "/trails_dnn/network/output");
    nh.param("pub_rate",  pub_rate, 30.0f);

    ROS_INFO("Topic : %s", caffe_ros_topic.c_str());
    ROS_INFO("Rate  : %.1f", pub_rate);

    const int queue_size = 10;
    ros::Subscriber dnn_sub;
    ros::Subscriber img_sub;
    ros::Publisher  debug_output_pub;
    ros::Publisher  image_output_pub;
	
	// This is for the TrailNet overlay for the RTSP stream
	// The subscriber topic can be changed if the Darknet-YOLO topic is desired
	// If you decide to change the published topic name, make sure to update the
	// RTSP .yaml file to match, found at ~/ws/src/ros_rtsp/config/stream_setup.yaml
    img_sub = nh.subscribe<sensor_msgs::Image>("/zed2/zed_node/left/image_rect_color", 1, cameraCallback);
    image_output_pub = nh.advertise<sensor_msgs::Image>("network/image_with_arrow", queue_size);
   

    dnn_sub = nh.subscribe<sensor_msgs::Image>(caffe_ros_topic, queue_size, dnnCallback);
    debug_output_pub = nh.advertise<geometry_msgs::PoseStamped>("network/output_debug", queue_size);



    ros::Rate rate(pub_rate);
    while (ros::ok())
    {
        if (dnn_msg != nullptr)
        {
            auto pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();
            pose_msg->header = dnn_msg->header;

            size_t dnn_out_size = dnn_msg->data.size() / sizeof(float);
            ROS_ASSERT(dnn_out_size * sizeof(float) == dnn_msg->data.size());
            ROS_ASSERT(dnn_out_size == 3 || dnn_out_size == 6 || dnn_out_size == 12);

            const float* probs = (const float*)(dnn_msg->data.data());

            // Orientation head.
            // Scale from -1..1 to -pi/2..pi/2. probs[0] is left turn, probs[2] - right.
            const float pi = std::acos(-1);
            float angle    = 0.5 * pi * (probs[0] - probs[2]);
            Eigen::Vector3d target_dir(std::cos(angle), std::sin(angle), 0);
            Eigen::Vector3d center_dir(1, 0, 0);
            geometry_msgs::Quaternion rotq_msg;
            tf::quaternionEigenToMsg(Eigen::Quaterniond::FromTwoVectors(center_dir, target_dir),
                                     rotq_msg);
            pose_msg->pose.orientation = rotq_msg;

	    float shift_by = probs[3] - probs[5];

            // Translation head.
            if (dnn_out_size >= 6)
                pose_msg->pose.position.y = shift_by;

            debug_output_pub.publish(pose_msg);
            dnn_msg = nullptr;
	    
	    	// Draw arrow on image given angle from trails_dnn
	    int line_thickness = 3;
	    int line_length = 100;
	    float shift_multiplier = 50.0; // shift_by is a -1.0 to 1.0 float

		// Cast shift_by to int so the arrow can be shifted. Change shift_multiplier to
		// change the range, ie, 50x means -50 to 50 pixel shift
	    int arrow_shift = static_cast<int>(shift_by * shift_multiplier); 

		// Create first point of the arrow halfway across screen (+/- shift amount), 3/4 down 
	    cv::Point point1 = cv::Point(cv_img->image.cols/2 - arrow_shift, 3 * cv_img->image.rows/4);
		// end point of arrow depends on point1 and trig based on angle previously determined
	    cv::Point point2 = cv::Point(point1.x + (line_length*std::sin(angle)), point1.y - (line_length*std::cos(angle)));

		// Target point1
	    cv::Point point3 = cv::Point(cv_img->image.cols/2, 3 * cv_img->image.rows/4);
		// Target point2
	    cv::Point point4 = cv::Point(cv_img->image.cols/2, point1.y - line_length);

		// Target line
	    cv::arrowedLine(cv_img->image, point3, point4, cv::Scalar(225, 225, 0), line_thickness);

		// Current line
	    cv::arrowedLine(cv_img->image, point1, point2, cv::Scalar(0, 0, 255), line_thickness);

	    	// DEBUG Text - Offset
	    //std::string offset_str = std::to_string(shift_by);
	    //cv::putText(cv_img->image, offset_str, point4, 1, 1, cv::Scalar(225, 225, 0), 2, 0);
 	    
	    // Publish modified image
	    image_output_pub.publish(cv_img->toImageMsg());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
