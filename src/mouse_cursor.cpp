#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

static const std::string OPENCV_WINDOW = "Image window";

//マウス入力用のパラメータ
struct mouseParam {
    int x;
    int y;
    int event;
    int flags;
};

//コールバック関数
void CallBackFunc(int eventType, int x, int y, int flags, void* userdata)
{
    mouseParam *ptr = static_cast<mouseParam*> (userdata);

    ptr->x = x;
    ptr->y = y;
    ptr->event = eventType;
    ptr->flags = flags;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mouse_cursor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);


    mouseParam mouseEvent;

    //入力画像

    cv::Mat input_img = cv::imread("/home/hirota/catkin_ws/src/for_blog/mouse_cursor/lena.jpg");

    //表示するウィンドウ名
    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);



    //コールバックの設定
    cv::setMouseCallback(OPENCV_WINDOW, CallBackFunc, &mouseEvent);

    while (ros::ok()) {

	   //画像の表示
    cv::imshow(OPENCV_WINDOW, input_img);
	cv::waitKey(100);

        //左クリックがあったら表示
        if (mouseEvent.event == cv::EVENT_LBUTTONDOWN) {
            //クリック後のマウスの座標を出力
            std::cout << mouseEvent.x << " , " << mouseEvent.y << std::endl;
        }
        //右クリックがあったら終了
        else if (mouseEvent.event == cv::EVENT_RBUTTONDOWN) {
            break;
        }

    loop_rate.sleep();
    }
    return 0;
}
