#include<opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <stdio.h>
#include<vector>
using namespace cv;
using namespace std;

typedef struct {
    short int x_translation=0;
	float x_rotation=0.0;
	short int z_translation=0;
	//mzy 2019/10/18
	unsigned short int detect_flag = 0;
}posture_result;
class infrared_solvePnP_parameter {
public:
	int gray_to_binary_low_threshold = 150;
	int gray_to_binary_high_threshold = 255;
	int minsize_between_each_contour_centriod =80;
	int min_size_of_contour = 25;
	//canny 阈值说明：
	//1.低于low的值被认为不是边缘
	//2.高于high的值被认为是边缘
	//3.在low和high之间的点如果与2的点相邻，则被认为是边缘
	int canny_low_size = 200;
	int canny_high_size = 250;
	//std::vector<cv::Point3d> four_world_location = { cv::Point3d(0,0,0),cv::Point3d(193,0,0),cv::Point3d(0,145,0),cv::Point3d(193,151,0) };
	std::vector<cv::Point3d> four_world_location = { cv::Point3d(0,0,0), cv::Point3d(200,0,0),cv::Point3d(0,170, 0), cv::Point3d(200,170,0) };
	std::vector<cv::Point3d> five_world_location = { cv::Point3d(0,0,0),cv::Point3d(200,0,0),cv::Point3d(0,170,0),cv::Point3d(200,170,0),cv::Point3d{100,85,0} };
	//std::vector<cv::Point3d> nine_world_location = { cv::Point3d(0,0,0), cv::Point3d(260,0,0), cv::Point3d(0,200,0), cv::Point3d(260,200,0), cv::Point3d(0,100,0),cv::Point3d(260,100,0),cv::Point3d(130,0,0),cv::Point3d(130,200,0),cv::Point3d(130,100,0) };
	//改变九个灯的顺序
	std::vector<cv::Point3d> nine_world_location = { cv::Point3d(0,0,0),cv::Point3d(130,0,0), cv::Point3d(260,0,0), cv::Point3d(0,100,0),cv::Point3d(130,100,0),cv::Point3d(260,100,0),cv::Point3d(0,200,0),cv::Point3d(130,200,0), cv::Point3d(260,200,0) };
	//上下平行线的距离差值倍数
	float min_distance_difference_between_01_23 = 0.3;
	//可偏离90度的夹角限定
	int inclination_deviation_90 = 40;
	//聚类紧密度的倍数差值，以此来决定要聚成几类；
	int compactness_magnification = 8;
	//5个点的时候确定的中心点离实际的中心点的距离不能超过实际长宽比的多少倍
	float five_medium_point_deviation = 0.3;
	//9个点的时候确定的中心点离实际的中心点的距离不能超过多少像素值
	float nine_medium_point_deviation = 0.25;
	//中间九个灯的contour与外部灯的contour的差值大小
	float out_inner_contour_ratio = 0.75;
};

posture_result infrared_solvePnP(cv::Mat scene_image, infrared_solvePnP_parameter parameter);