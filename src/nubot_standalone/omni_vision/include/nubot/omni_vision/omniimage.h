#ifndef __NUBOT_VISION_OMNIIMAGE_H_
#define __NUBOT_VISION_OMNIIMAGE_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"

namespace nubot
{ 

using std::vector;
using cv::Mat;
using cv::Scalar;
using cv::Vec3b;

enum 
{	
	SELECTION_RGB=1,
	SELECTION_YUV=2,
	SELECTION_HSI=3,
};

class Omni_Image
{

public:
	
    Omni_Image(const char* infopath);
    ~Omni_Image();
    bool set_bgr_image(const Mat & img);
	bool set_hsv_image(const Mat & img);
	bool set_yuv_image(const Mat & img);

    Mat  get_bgr_image();
	Mat  get_hsv_image();
	Mat  get_yuv_image();

    void bgr2hsv();
    void bgr2yuv();
    void yuv2bgr();
	void yuv2hsv();
	void hsv2yuv();
    void hsv2bgr();

    /*! get the code from opencv*/
    cv::Vec3b bgr2hsv(cv::Vec3b bgr);

	void get_color_value(DPoint2i & pts,uchar & color_value,int channels,int flags);
	void get_color_values(std::vector<DPoint2i> & pts,std::vector<uchar> & color_value,int channels,int flags);

    void set_small_roi(Circle _circle);
    void set_big_roi(Circle _circle);
    void set_width(int _width);
    void set_height(int _height);

    Circle get_small_roi();
    Circle get_big_roi();
    int get_width();
    int get_height();

public:
    /*! @brief the bgr image.*/
    Mat bgr_image_;
    /*! @brief the hsv image.*/
	Mat hsv_image_; 
    /*! @brief the yuv image.*/
	Mat yuv_image_; 

private:
    /*! @brief the big ROI area which should be processed.*/
	Circle img_ROI_;
    /*! @brief the small ROI area which should be the robot itself*/
	Circle small_ROI_;
    /*! @brief the width(cols) of the image*/
    int width_;
    /*! @brief the height(rows) of the image.*/
	int height_;
};

}

#endif  //!__NUBOT_VISION_OMNIIMAGE_H_

