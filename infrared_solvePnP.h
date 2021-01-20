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
	//canny ��ֵ˵����
	//1.����low��ֵ����Ϊ���Ǳ�Ե
	//2.����high��ֵ����Ϊ�Ǳ�Ե
	//3.��low��high֮��ĵ������2�ĵ����ڣ�����Ϊ�Ǳ�Ե
	int canny_low_size = 200;
	int canny_high_size = 250;
	//std::vector<cv::Point3d> four_world_location = { cv::Point3d(0,0,0),cv::Point3d(193,0,0),cv::Point3d(0,145,0),cv::Point3d(193,151,0) };
	std::vector<cv::Point3d> four_world_location = { cv::Point3d(0,0,0), cv::Point3d(200,0,0),cv::Point3d(0,170, 0), cv::Point3d(200,170,0) };
	std::vector<cv::Point3d> five_world_location = { cv::Point3d(0,0,0),cv::Point3d(200,0,0),cv::Point3d(0,170,0),cv::Point3d(200,170,0),cv::Point3d{100,85,0} };
	//std::vector<cv::Point3d> nine_world_location = { cv::Point3d(0,0,0), cv::Point3d(260,0,0), cv::Point3d(0,200,0), cv::Point3d(260,200,0), cv::Point3d(0,100,0),cv::Point3d(260,100,0),cv::Point3d(130,0,0),cv::Point3d(130,200,0),cv::Point3d(130,100,0) };
	//�ı�Ÿ��Ƶ�˳��
	std::vector<cv::Point3d> nine_world_location = { cv::Point3d(0,0,0),cv::Point3d(130,0,0), cv::Point3d(260,0,0), cv::Point3d(0,100,0),cv::Point3d(130,100,0),cv::Point3d(260,100,0),cv::Point3d(0,200,0),cv::Point3d(130,200,0), cv::Point3d(260,200,0) };
	//����ƽ���ߵľ����ֵ����
	float min_distance_difference_between_01_23 = 0.3;
	//��ƫ��90�ȵļн��޶�
	int inclination_deviation_90 = 40;
	//������ܶȵı�����ֵ���Դ�������Ҫ�۳ɼ��ࣻ
	int compactness_magnification = 8;
	//5�����ʱ��ȷ�������ĵ���ʵ�ʵ����ĵ�ľ��벻�ܳ���ʵ�ʳ���ȵĶ��ٱ�
	float five_medium_point_deviation = 0.3;
	//9�����ʱ��ȷ�������ĵ���ʵ�ʵ����ĵ�ľ��벻�ܳ�����������ֵ
	float nine_medium_point_deviation = 0.25;
	//�м�Ÿ��Ƶ�contour���ⲿ�Ƶ�contour�Ĳ�ֵ��С
	float out_inner_contour_ratio = 0.75;
};

posture_result infrared_solvePnP(cv::Mat scene_image, infrared_solvePnP_parameter parameter);