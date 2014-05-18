#ifndef __NUBOT_VISION_COLORSEGMENT_H_
#define __NUBOT_VISION_COLORSEGMENT_H_

//Macro definition for color segmentation
#define VISION_COLORSEGMENT_YELLOW	0	//Yellow represent for football
#define VISION_COLORSEGMENT_BLACK	1	//green represent for ground
#define VISION_COLORSEGMENT_GREEN	2	//green represent for ground
#define VISION_COLORSEGMENT_UNKNOWCOLOR	3	//Unknown color

#include <opencv2/opencv.hpp>
#include <string>
namespace nubot
{
  class ColorSegment
  {
  private:
    //int width_;   //The width of the picture, as well as the "Color Segment Result"
	//int height_;  //The height of the picture, as well as the "Color Segment Result"
    unsigned char table_[64*64*64];  // the color table for segmentation
    cv::Mat check_flag_;  //when you need to do image area process, this mat is used to flag the points temporarily
    std::deque<cv::Point> grow_queue_;//the point queue when region grow
  public:
    cv::Mat segment_result_;//the result of segmentation;
    struct ImageArea
    {
      cv::Point area_center_;
      int area_size_;
      cv::Rect area_rect_;//ex-rectangle
      std::vector<cv::Point> edge_points_;//the edge points(eight neighbors)
    };
    ColorSegment(const char*  _table_name);
    //the color of the image should be organized just the "Inverse" of the table. e.g. if the image is BGR image, the table index must be RGB.
    bool Segment(cv::Mat &_image, const cv::Rect &_ROI=cv::Rect(0,0,INT_MAX,INT_MAX));
    bool RegionSearch(std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas=1, const cv::Rect &_ROI=cv::Rect(0,0,INT_MAX,INT_MAX), const int &_threshold_size=5, const int &_threshold_combination=1, unsigned char _target_color=VISION_COLORSEGMENT_YELLOW);  
    bool RegionSearch(ImageArea &_target_area);
  };
}

#endif//!__NUBOT_VISION_COLORSEGMENT_H_
