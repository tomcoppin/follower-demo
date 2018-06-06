#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Convert Image
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
     // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    Mat hsv, hue;
    //int thresh = 100;
    // cvtColor(cv_ptr->image, imghsv, cv::COLOR_BGR2HSV);
    cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    /// Use only the Hue value
    hue.create( hsv.size(), hsv.depth() );
    int ch[] = { 0, 0 };
    mixChannels( &hsv, 1, &hue, 1, ch, 1 );

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    inRange(hue, Scalar(60), Scalar(80), threshold_output);
    /// Find contours
    findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }

    ///  Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    /// Draw contours
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
      if (mu[i].m00 > 100)
        { Scalar color = Scalar( 0,255,0 ); //green
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 ); }
    }

    /// Approximate contours to polygons + get bounding rects and circles
    // vector<vector<Point> > contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    // vector<Point2f>center( contours.size() );
    // vector<float>radius( contours.size() );

    // for( int i = 0; i < contours.size(); i++ )
    // { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
    //   boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    //   minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    // }


  // /// Draw polygonal contour + bonding rects + circles
  //   Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  //   for( int i = 0; i< contours.size(); i++ )
  //   {
  //     Scalar color = Scalar( 0, 255, 255 ); //yellow
  //     drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
  //     rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
  //     circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
  //   }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, drawing);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

