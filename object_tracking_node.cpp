//
// Created by davinci on 21/08/17.
//

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static const std::string OPENCV_WINDOW = "Image window";
cv::Mat right_image;
cv::Mat left_image;

// For the initial selection of the bounding box
using namespace cv;
using namespace std;
Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
Rect rectL; /* bounding box */
Rect rectR;
Rect rect;

Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;

// Callback function for dragging the bounding box
void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        /* left button clicked. ROI selection begins */
        point1 = Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        /* mouse dragged. ROI being selected */
        Mat img1 = img.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("image", img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        point2 = Point(x, y);
        rect = Rect(point1.x,point1.y,x-point1.x,y-point1.y);
        drag = 0;
        roiImg = img(rect);
    }

    if (event == CV_EVENT_LBUTTONUP)
    {
        /* ROI selected */
        select_flag = 1;
        drag = 0;
    }
}

void imageCallbackL(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptrL;
    try
    {
        cv_ptrL = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        left_image = cv_ptrL->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void imageCallbackR(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptrR;
    try
    {
        cv_ptrR = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        right_image = cv_ptrR->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}


int main(int argc, char **argv)
{
    // Start up ros node
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle node_handle;

    // Get ROS params
    std::string camera_left_topic;
    node_handle.param<std::string>("left_camera_topic", camera_left_topic, "image_output/left");
    std::string camera_right_topic;
    node_handle.param<std::string>("right_camera_topic", camera_right_topic, "image_output/right");

    std::string position_left_topic;
    node_handle.param<std::string>("positionL", position_left_topic, "position_output/left");
    std::string position_right_topic;
    node_handle.param<std::string>("positionR", position_right_topic, "position_output/right");

    // Defining centroid variables
    cv::Point centreL;
    cv::Point centreR;


    // Set up publishers and subscribers
    ros::Publisher needleL_pub = node_handle.advertise<geometry_msgs::Point>(position_left_topic, 1000);
    ros::Publisher needleR_pub = node_handle.advertise<geometry_msgs::Point>(position_right_topic, 1000);
    image_transport::ImageTransport it(node_handle);
    image_transport::Subscriber subL = it.subscribe(camera_left_topic, 1, imageCallbackL);
    image_transport::Subscriber subR = it.subscribe(camera_right_topic, 1, imageCallbackR);

    // Start the loop
    ros::Rate loop_rate(20);
    geometry_msgs::Point left_point_msg, right_point_msg;

    // Initialise trackers
    ros::spinOnce();

    // Open up gui to select initial point-> i have to find a way of opening a gui for selecting the first roi
    cv::imshow(OPENCV_WINDOW, left_image);
    cv::imshow(OPENCV_WINDOW, right_image);

    // Selecting for the LEFT camera
    int k;
    //VideoCapture cap = VideoCapture(0); /* Start webcam */
    //cap >> img; /* get image(Mat) */
    imshow("image", left_image);
    while(1)
    {
        // cap >> img;
        cvSetMouseCallback("image", mouseHandler, NULL);
        if (select_flag == 1)
        {
            imshow("ROI", roiImg); /* show the image bounded by the box */
        }
        rectangle(left_image, rect, CV_RGB(255, 0, 0), 3, 8, 0);
        rectL = rect;
        imshow("image", left_image);
        k = waitKey(10);
        if (k == 27)
        {
            break;
        }
    }

    // Selecting for the RIGHT camera
    imshow("image", right_image);
    while(1)
    {
        // cap >> img;
        cvSetMouseCallback("image", mouseHandler, NULL);
        if (select_flag == 1)
        {
            imshow("ROI", roiImg); /* show the image bounded by the box */
        }
        rectangle(right_image, rectR, CV_RGB(255, 0, 0), 3, 8, 0);
        rectR = rect;
        imshow("image", right_image);
        k = waitKey(10);
        if (k == 27)
        {
            break;
        }
    }


    while (ros::ok())
    {
        // Current image is got from the subscriber (no locks)
        ros::spinOnce();

        // Parse mat files into tracker
        // right_image, left_image, rectL, rectR
        // Xiaofei PART ??



        // Get tracker points: from the detected bounding box, defining the centroids
        //Bounding Box Centroid
        centreL.x=boundRectL.x + boundRectL.width/2;
        centreL.y=boundRectL.y + boundRectL.height/2;

        centreR.x=boundRectR.x + boundRectR.width/2;
        centreR.y=boundRectR.y + boundRectR.height/2;

        // Fill in the message
        left_point_msg.x=centreL.x;
        left_point_msg.y=centreL.y;
        left_point_msg.z=0;

        right_point_msg.x=centreR.x;
        right_point_msg.y=centreR.y;
        right_point_msg.z=0;

        // Sending the message -> is publishing in positionL and positionR?
        needleL_pub.publish(left_point_msg);
        needleR_pub.publish(right_point_msg);


        loop_rate.sleep();
    }


    return 0;
}
