#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

struct mouseParam {
    int x;
    int y;
    int event;
    int flags;
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  mouseParam mouseEvent;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

    cv::setMouseCallback(OPENCV_WINDOW, CallBackFunc, &mouseEvent);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  static void CallBackFunc(int eventType, int x, int y, int flags, void* userdata)
  {
      mouseParam *ptr = static_cast<mouseParam*> (userdata);

      ptr->x = x;
      ptr->y = y;
      ptr->event = eventType;
      ptr->flags = flags;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(100);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
      ros::spinOnce();
      if (ic.mouseEvent.event == cv::EVENT_LBUTTONDOWN) {
          //クリック後のマウスの座標を出力
          std::cout << ic.mouseEvent.x << " , " << ic.mouseEvent.y << std::endl;
      }
      //右クリックがあったら終了
      else if (ic.mouseEvent.event == cv::EVENT_RBUTTONDOWN) {
          break;
      }
      loop_rate.sleep();
  }
  return 0;
}
