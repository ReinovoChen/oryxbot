#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/MultiFormatReader.h>
#include <zxing-opencv/MatSource.h>
#include "oryxbot_msgs/qr_info.h"
#include "oryxbot_msgs/pose_err.h"


using namespace zxing;
using namespace zxing::qrcode;

#define MARK_WIDTH (210)

ros::NodeHandle* nh;
ros::Publisher pub;
cv::Point toCvPoint(Ref<ResultPoint> resultPoint)
{
    return cv::Point(resultPoint->getX(), resultPoint->getY());
}

std::vector<cv::Point> get_pattern(cv::Mat& grey, std::string& info)
{
    std::vector<cv::Point> point_vec;
    try {
        // Create luminance  source
        Ref<LuminanceSource> source = MatSource::create(grey);
        // Search for QR code
        Ref<Reader> reader;
        reader.reset(new QRCodeReader);
        Ref<Binarizer> binarizer(new GlobalHistogramBinarizer(source));
        Ref<BinaryBitmap> bitmap(new BinaryBitmap(binarizer));
        Ref<Result> result(reader->decode(bitmap, DecodeHints(DecodeHints::TRYHARDER_HINT)));
        int resultPointCount = result->getResultPoints()->size();
        //ROS_INFO("%s\n", result->getText()->getText().c_str());
        info = result->getText()->getText();
        if (resultPointCount != 3) {
            ROS_WARN("qrcode format isn't support");
            return point_vec;
        } else {
            for(int i=0; i < 3; i++) {
                point_vec.push_back(toCvPoint(result->getResultPoints()[i]));
            }
        }
        return point_vec;
    } catch (const ReaderException& e) {
        std::cerr << e.what() << " (ignoring)" << std::endl;
    } catch (const zxing::IllegalArgumentException& e) {
        std::cerr << e.what() << " (ignoring)" << std::endl;
    } catch (const zxing::Exception& e) {
        std::cerr << e.what() << " (ignoring)" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << " (ignoring)" << std::endl;
    }
}


void img_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat gray_img;
    cv::cvtColor(img_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
    std::string info;
    std::vector<cv::Point> p_vec = get_pattern(gray_img, info);
    if(p_vec.size() == 3){
            oryxbot_msgs::qr_info qr_info_msg;
            qr_info_msg.header.stamp = ros::Time::now();
            qr_info_msg.header.frame_id = "qr_frame";
            qr_info_msg.qr_info = info;
            qr_info_msg.err.x_err = 0.5*(p_vec[0].x+p_vec[2].x)-gray_img.cols/2;
            qr_info_msg.err.y_err = 0.5*(p_vec[0].y+p_vec[2].y)-gray_img.rows/2;
            qr_info_msg.err.k_err = ((float)p_vec[2].y-(float)p_vec[0].y)/((float)p_vec[2].x-(float)p_vec[0].x)-1.0;
            pub.publish(qr_info_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qrcode_reader");
    nh = new ros::NodeHandle;
    cv::namedWindow("sub_img");
    cv::startWindowThread();
    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, img_cb);
    pub = nh->advertise<oryxbot_msgs::qr_info>("/oryxbot/qr_info", 10);
    ros::spin();
    cv::destroyWindow("sub_img");
    delete nh;
    return 0;
}
