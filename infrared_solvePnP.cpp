#include"infrared_solvePnP.h"
#include"own_solvePnP.h"
void show_the_result(cv::Mat imageContours, cv::Mat image, cv::Mat scene_image, vector<cv::Point2f>contour_area_big, vector<cv::Point2f>contour_area_small, vector<Point2f>final_result, vector<Point2f>final_final_result);
Mat convertTo3Channels(const Mat& binImg)
{
	Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
	vector<Mat> channels;
	for (int i = 0; i < 3; i++)
	{
		channels.push_back(binImg);
	}
	merge(channels, three_channel);
	return three_channel;
}
double centerPoints(vector<Point>contour)
{
	double factor = (contourArea(contour) * 4 * CV_PI) /
		(pow(arcLength(contour, true), 2));
	return factor;
	//cout << "factor:" << factor << endl;  //计算出圆形度factor
}
void RotVec2Angle_test(cv::Mat rvec, cv::Mat& angle) {
	rvec.convertTo(rvec, CV_64F);
	cv::Mat R;
	cv::Rodrigues(rvec, R);

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}

	angle = (cv::Mat_<double>(3, 1) << x, y, z);
	angle *= 180 / CV_PI;
}

vector<Point2f>find_infrared(cv::Mat scene_image, infrared_solvePnP_parameter parameter);
bool check_is_rectangle(vector<cv::Point2d>four_point, infrared_solvePnP_parameter parameter);
std::vector<cv::Point2f>cluster_result(std::vector<cv::Point2f>result,infrared_solvePnP_parameter parameter);

//mzy 2019/10/18
posture_result pose_result;

posture_result infrared_solvePnP(cv::Mat scene_image,infrared_solvePnP_parameter parameter)
{
	pose_result.detect_flag = 0;
/*
	//相机参数部分
	cv::Mat cameraMatrix(3, 3, CV_32F);
	//cameraMatrix.at<float>(0, 0) = 3328.57616;
	cameraMatrix.at<float>(0, 0) = 3426.37193;
	cameraMatrix.at<float>(0, 1) = 0;
	//cameraMatrix.at<float>(0, 2) = 1516.08609;
	cameraMatrix.at<float>(0, 2) = 1547.56292;
	cameraMatrix.at<float>(1, 0) = 0;
	//cameraMatrix.at<float>(1, 1) = 3328.83601;
	cameraMatrix.at<float>(1, 1) = 3427.52138;
	//cameraMatrix.at<float>(1, 2) = 1056.16850;
	cameraMatrix.at<float>(1, 2) = 1066.43477;
	cameraMatrix.at<float>(2, 0) = 0;
	cameraMatrix.at<float>(2, 1) = 0;
	cameraMatrix.at<float>(2, 2) = 1;
	cv::Mat rotation_Matrix, translation_Matrix;
	cv::Mat distCoeffs(1, 5, CV_32F);
	/*distCoeffs.at<float>(0) = -0.39208;
	distCoeffs.at<float>(1) = 0.45432;
	distCoeffs.at<float>(2) = 0.00154;
	distCoeffs.at<float>(3) = 0.00081;
	distCoeffs.at<float>(4) = 0.0;
	distCoeffs.at<float>(0) = -0.04657;
	distCoeffs.at<float>(1) = 0.15827;
	distCoeffs.at<float>(2) = 0.00045;
	distCoeffs.at<float>(3) = 0.0004;
	distCoeffs.at<float>(4) = 0.0;
*/
	//730老相机新镜头
	cv::Mat cameraMatrix(3, 3, CV_32F);
	cameraMatrix.at<float>(0, 0) = 3420.51053;

	cameraMatrix.at<float>(0, 1) = 0;
	cameraMatrix.at<float>(0, 2) = 1575.40085;
	//cameraMatrix.at<float>(0, 2) = 1547.56292;
	cameraMatrix.at<float>(1, 0) = 0;
	cameraMatrix.at<float>(1, 1) = 3421.5417;
	//cameraMatrix.at<float>(1, 1) = 3427.52138;
	cameraMatrix.at<float>(1, 2) = 1054.75924;
	//cameraMatrix.at<float>(1, 2) = 1066.43477;
	cameraMatrix.at<float>(2, 0) = 0;
	cameraMatrix.at<float>(2, 1) = 0;
	cameraMatrix.at<float>(2, 2) = 1;
	cv::Mat rotation_Matrix, translation_Matrix;
	cv::Mat distCoeffs(1, 5, CV_32F);
	distCoeffs.at<float>(0) = 0.00084;
	distCoeffs.at<float>(1) = 0.00314;
	distCoeffs.at<float>(2) = 0.00016;
	distCoeffs.at<float>(3) = 0.00013;
	distCoeffs.at<float>(4) = 0.0;

	std::vector<cv::Point2f>result;
	result = find_infrared(scene_image,parameter);
	
	if (result.size() == 4)
	{
		std::vector<cv::Point3d> world_location = parameter.four_world_location;
		std::cout << "zuhe is 44444444" << std::endl;
		cv::solvePnP(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix);
		
		cv::Mat rot, output_a, output_b;
		cv::Vec3d  rvec_euler;
		cv::Rodrigues(rotation_Matrix, rot);
		rvec_euler = cv::RQDecomp3x3(rot, output_a, output_b);
		rot = rot.t();  // rotation of inverse
		translation_Matrix = -rot * translation_Matrix; // translation of inverse
		pose_result.x_rotation = rvec_euler[1];
		pose_result.x_translation = translation_Matrix.at<double>(0)-650;
		pose_result.z_translation = translation_Matrix.at<double>(2);
		//mzy 2019/10/18
		pose_result.detect_flag = 1;
		return pose_result;
	}
	else if (result.size() == 5)
	{
		std::vector<cv::Point3d> world_location = parameter.five_world_location;
		std::cout << "zuhe is 555555555" << std::endl;
		//使用ransac五个灯的估计会不准
		//cv::solvePnPRansac(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix,false,300,30.0);
		cv::solvePnP(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix);
		cv::Mat rot, output_a, output_b;
		cv::Vec3d  rvec_euler;
		cv::Rodrigues(rotation_Matrix, rot);
		rvec_euler = cv::RQDecomp3x3(rot, output_a, output_b);
		rot = rot.t();  // rotation of inverse
		translation_Matrix = -rot * translation_Matrix; // translation of inverse
		pose_result.x_rotation = rvec_euler[1];
		double translation_1= translation_Matrix.at<double>(1);
		pose_result.x_translation = translation_Matrix.at<double>(0)+421;
		//pose_result.x_translation = translation_Matrix.at<double>(0)-650;
		pose_result.z_translation = translation_Matrix.at<double>(2);
		//mzy 2019/10/18
		pose_result.detect_flag = 1;
		return pose_result;
	}
	else if (result.size() == 9)
	{
		std::vector<cv::Point3d> world_location = parameter.nine_world_location;
		//mzy 2019/10/14
		//cv::solvePnPRansac(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix, 0, 200, 8.0, 0.99, noArray(), CV_EPNP);
		cv::solvePnPRansac(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix, 0,200,8.0,0.99,noArray(), SOLVEPNP_EPNP);
		cv::Mat rot, output_a, output_b;
		cv::Vec3d  rvec_euler;
		cv::Rodrigues(rotation_Matrix, rot);
		rvec_euler = cv::RQDecomp3x3(rot, output_a, output_b);
		rot = rot.t();  // rotation of inverse
		translation_Matrix = -rot * translation_Matrix; // translation of inverse
		//if (abs(translation_Matrix.at<double>(2)) > 1400)
		//	return pose_result;	
		pose_result.x_rotation = rvec_euler[1];
		pose_result.x_translation = translation_Matrix.at<double>(0)-130;
		pose_result.z_translation = translation_Matrix.at<double>(2)+1435 ;
		//mzy 2019/10/18
		pose_result.detect_flag = 1;
		//int font_face = cv::FONT_HERSHEY_COMPLEX;
		//double font_scale = 2;
		//int thickness = 2;
		//for (int i = 0; i < scene_location.size(); i++)
	 //    	cv::putText(scene_image, to_string(i), cv::Point2f(scene_location[i].x, scene_location[i].y), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
		//cv::resize(scene_image,scene_image,cv::Size(scene_image.cols/2,scene_image.rows/2));
		//cv::imshow("排列组合",scene_image);
		//cv::waitKey(30);
		return pose_result;

		/*
		std::vector<cv::Point3d> world_location = parameter.nine_world_location;
		std::cout << "zuhe is 99999999" << std::endl;
		//rotation_Matrix = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		//translation_Matrix = (cv::Mat_<double>(3, 1) << 0, -15, 2000);
		
		//bool ret = solvePnP(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix, false, CV_ITERATIVE);
		//vector<int> inliers;
		cv::solvePnPRansac(world_location, result, cameraMatrix, distCoeffs, rotation_Matrix, translation_Matrix, 0,200,8.0,100,noArray(),CV_EPNP);
		cv::Mat angle;
		RotVec2Angle_test(rotation_Matrix, angle);

		std::cout << "rotation_Matrix" << angle << endl;
		std::cout << "translation_Matrix" << translation_Matrix<<endl;

		vector<cv::Point2d> projPts;
		cv::projectPoints(world_location, rotation_Matrix, translation_Matrix, cameraMatrix, distCoeffs, projPts);
		double sumErr = 0.0;
		for (size_t k = 0; k < result.size(); k++) {
			sumErr += cv::norm(result[k] - cv::Point2f(projPts[k].x, projPts[k].y));
		}
		sumErr /= result.size();
		cout << "repoject error: " << sumErr << endl;

		cv::Mat rot, output_a, output_b;
		cv::Vec3d  rvec_euler;
		cv::Rodrigues(rotation_Matrix, rot);
		rvec_euler = cv::RQDecomp3x3(rot, output_a, output_b);
		rot = rot.inv();  // rotation of inverse
		translation_Matrix = -rot * translation_Matrix; // translation of inverse
		//if (abs(translation_Matrix.at<double>(2)) > 1400)
		//	return pose_result;	
		pose_result.x_rotation = rvec_euler[1];
		pose_result.x_translation = translation_Matrix.at<double>(0)-130;
		pose_result.z_translation = translation_Matrix.at<double>(2)+1435+15 ;
		pose_result.detect_flag = true;
	/*	int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 2;
		int thickness = 2;
		for (int i = 0; i < scene_location.size(); i++)
	     	cv::putText(scene_image, to_string(i), cv::Point2f(scene_location[i].x, scene_location[i].y), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
		cv::resize(scene_image,scene_image,cv::Size(scene_image.cols/2,scene_image.rows/2));
		cv::imshow("排列组合",scene_image);
		cv::waitKey(30);*/
		//return pose_result;
	}
	return pose_result;
}
bool check_four_point(std::vector<cv::Point2f>&result,infrared_solvePnP_parameter parameter)
{
	std::vector<cv::Point2d>scene_location = { cv::Point2d(0, 0),cv::Point2d(0, 0) ,cv::Point2d(0, 0) ,cv::Point2d(0, 0) };
	double medium_point_x = (result[0].x + result[1].x + result[2].x + result[3].x) / 4.0;
	double medium_point_y = (result[0].y + result[1].y + result[2].y + result[3].y) / 4.0;
	for (int i = 0; i < 4; i++)
	{
		
		if (result[i].x < medium_point_x)
			if (result[i].y < medium_point_y)
				scene_location[0] = (cv::Point2d(result[i].x, result[i].y));
			else
				scene_location[2] = (cv::Point2d(result[i].x, result[i].y));
		else
			if (result[i].y < medium_point_y)
				scene_location[1] = (cv::Point2d(result[i].x, result[i].y));
			else
				scene_location[3] = (cv::Point2d(result[i].x, result[i].y));

	}
	if (scene_location[0].y > 1000)
		return false;
	if (check_is_rectangle(scene_location, parameter) == false)
		return false;
	//if(sqrt(pow(scene_location[0].x- scene_location[3].x,2)+ pow(scene_location[0].x - scene_location[3].x,2))<320)
	//	return false;
		for (int i = 0; i < scene_location.size(); i++)
			result[i] = scene_location[i];
		return true;
}
bool check_five_point(std::vector<cv::Point2f>&result, infrared_solvePnP_parameter parameter)
{
	std::vector<cv::Point2d>scene_location = { cv::Point2d(0, 0),cv::Point2d(0, 0) ,cv::Point2d(0, 0) ,cv::Point2d(0, 0),cv::Point2d(0, 0) };
	double medium_point_x = (result[0].x + result[1].x + result[2].x + result[3].x + result[4].x) / 5.0;
	double medium_point_y = (result[0].y + result[1].y + result[2].y + result[3].y + result[4].y) / 5.0;
	double min_distance = 10000;
	int min_record = -1;
	for (int i = 0; i < 5; i++)
	{
		double cunrrent_distance = abs(result[i].x - medium_point_x) + abs(result[i].y - medium_point_y);
		if (cunrrent_distance < min_distance)
		{
			min_distance = cunrrent_distance;
			min_record = i;
		}
	}
	scene_location[4] = result[min_record];
	for (int i = 0; i < 5; i++)
	{
		if (i == min_record)
			continue;
		if (result[i].x < medium_point_x)
			if (result[i].y < medium_point_y)
				scene_location[0] = (cv::Point2d(result[i].x, result[i].y));
			else
				scene_location[2] = (cv::Point2d(result[i].x, result[i].y));
		else
			if (result[i].y < medium_point_y)
				scene_location[1] = (cv::Point2d(result[i].x, result[i].y));
			else
				scene_location[3] = (cv::Point2d(result[i].x, result[i].y));

	}
	double distance_0_4 = sqrt(pow(scene_location[0].x - scene_location[4].x, 2) + pow(scene_location[0].y - scene_location[4].y, 2));
	if (abs(scene_location[4].x - medium_point_x) + abs(scene_location[4].y - medium_point_y) > parameter.five_medium_point_deviation*distance_0_4)
		return false;
	vector<cv::Point2d>check_four_point = { scene_location[0],scene_location[1] ,scene_location[2] ,scene_location[3] };
	if (check_is_rectangle(check_four_point, parameter) == false)
		return false;
	for (int i = 0; i < scene_location.size(); i++)
		result[i] = scene_location[i];
	return true;
}
bool check_nine_point(std::vector<cv::Point2f>&result, infrared_solvePnP_parameter parameter)
{
	std::vector<cv::Point2d>scene_location = { cv::Point2d(0, 0),cv::Point2d(0, 0) ,cv::Point2d(0, 0) ,cv::Point2d(0, 0),cv::Point2d(0, 0),cv::Point2d(0, 0),cv::Point2d(0, 0),cv::Point2d(0, 0),cv::Point2d(0, 0) };
	double medium_point_x = (result[0].x + result[1].x + result[2].x + result[3].x + result[4].x + result[5].x + result[6].x + result[7].x + result[8].x) / 9.0;
	double medium_point_y = (result[0].y + result[1].y + result[2].y + result[3].y + result[4].y + result[5].y + result[6].y + result[7].y + result[8].y) / 9.0;
	//先找出离中心点最近的点
	double min_distance = 10000;
	int min_record = -1;
	for (int i = 0; i < 9; i++)
	{
		//if(result[i].x<medium_point_x)

		double cunrrent_distance = abs(result[i].x - medium_point_x) + abs(result[i].y - medium_point_y);
		if (cunrrent_distance < min_distance)
		{
			min_distance = cunrrent_distance;
			min_record = i;
		}
	}
	scene_location[4] = result[min_record];

	double min_left_distance_away_center = 10000;
	double min_right_distance_away_center = 10000;
	double min_up_distance_away_center = 10000;
	double min_down_distance_away_center = 10000;
	int left_min = -1, left_medium = -1, left_max = -1, up_medium = -1;
	int right_min = -1, right_medium = -1, right_max = -1, down_medium = -1;
	for (int i = 0; i < 9; i++)
	{
		if (i == min_record)
			continue;
		double current_y_distance = abs(result[i].y - medium_point_y);
		double current_x_distance = abs(result[i].x - medium_point_x);
		if (result[i].x < medium_point_x)
		{
			if (current_y_distance < min_left_distance_away_center)
			{
				min_left_distance_away_center = current_y_distance;
				left_medium = i;
			}
		}
		else
		{
			if (current_y_distance < min_right_distance_away_center)
			{
				min_right_distance_away_center = current_y_distance;
				right_medium = i;
			}

		}
		if (result[i].y < medium_point_y)
		{
			if (current_x_distance < min_up_distance_away_center)
			{
				min_up_distance_away_center = current_x_distance;
				up_medium = i;
			}
		}
		else
		{
			if (current_x_distance < min_down_distance_away_center)
			{
				min_down_distance_away_center = current_x_distance;
				down_medium = i;
			}

		}
	}
	scene_location[3] = result[left_medium];
	scene_location[5] = result[right_medium];
	scene_location[1] = result[up_medium];
	scene_location[7] = result[down_medium];
	for (int i = 0; i < 9; i++)
	{
		if (i == left_medium || i == right_medium || i == min_record || i == up_medium || i == down_medium)
			continue;
		if (result[i].x < medium_point_x)
		{
			if (result[i].y < medium_point_y)
				scene_location[0] = result[i];
			else
				scene_location[6] = result[i];
		}
		else
		{
			if (result[i].y < medium_point_y)
				scene_location[2] = result[i];
			else
				scene_location[8] = result[i];
		}
	}
	float distance_0_1 = sqrt(pow(scene_location[0].x - scene_location[2].x, 2) + pow(scene_location[0].y - scene_location[2].y, 2));
	if (abs(scene_location[4].x - medium_point_x) + abs(scene_location[4].y - medium_point_y) > parameter.nine_medium_point_deviation*distance_0_1)
		return false;
	vector<cv::Point2d>check_four_point = { scene_location[0],scene_location[2] ,scene_location[6] ,scene_location[8] };
	if (check_is_rectangle(check_four_point, parameter) == false)
		return false;
	for (int i = 0; i < scene_location.size(); i++)
		result[i] = scene_location[i];
	return true;
}
vector<Point2f>find_infrared(cv::Mat scene_image, infrared_solvePnP_parameter parameter)
{
	Mat image;
	vector<Point2d>result;
	//scene_image为灰度图，result为二值图像
   //100~255为阈值，可以根据情况设定
   //在阈值中的像素点将变为0（白色部分），阈值之外的像素将变为1（黑色部分）
//	parameter.gray_to_binary_low_threshold = 80;

	//mzy  2019/10/14去掉二值化
	threshold(scene_image, image, parameter.gray_to_binary_low_threshold, parameter.gray_to_binary_high_threshold, THRESH_BINARY);
	//image = scene_image;
	//GaussianBlur(image, image, Size(3, 3), 0);
	//cv::Mat threshold_image;
	//cv::resize(image,threshold_image,cv::Size(image.cols/4, image.rows/4));
	//cv::imshow("threshold_image", threshold_image);
	//Canny(image, image, parameter.canny_low_size, parameter.canny_high_size);
	//cv::Mat canny_image;
	//cv::resize(image, canny_image, cv::Size(image.cols / 4, image.rows / 4));
	//cv::imshow("canny_image", canny_image);
	vector<vector<Point>> contours;
	//2019/09/25
	vector<vector<Point>>final_contours;
	vector<vector<Point>>contours_temp;
	vector<Vec4i> hierarchy;
	cv::Mat huabu_test = image.clone();
	cv::Mat binMat = image.clone();
	findContours(binMat, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE, Point());
	Mat imageContours = Mat::zeros(image.size(), CV_8UC1);
	Mat Contours = Mat::zeros(image.size(), CV_8UC1);  //绘制
	vector<Point2d> mc(contours.size());
    vector<float>contours_temp_record;
	vector<double>contour_area_temp_record;
	//cv::resize(scene_image,scene_image,cv::Size(scene_image.cols/4,scene_image.rows/4));
	int font_face = cv::FONT_HERSHEY_COMPLEX;
	double font_scale = 0.5;
	int thickness = 0.5;
	//先通过轮廓判断
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > parameter.min_size_of_contour)
		{
			Moments mu = moments(contours[i], false);
			if (abs(mu.m00) < 0.0001)
				mc[i] = Point2d(0, 0);
			else
			    mc[i] = Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
			result.push_back(mc[i]);
			contours_temp_record.push_back(contours[i].size());
			contour_area_temp_record.push_back(contourArea(contours[i]));
			contours_temp.push_back(contours[i]);
			//绘制轮廓
		//	drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
		}
	}
	//极大值抑制
	vector<Point2f>final_result;
	vector<float>final_contours_temp_record;
	vector<double>final_contour_area_temp_record;
	vector<double>final_second_contour_area_temp_record;
	vector<int>contours_number(result.size(),1);
	for (int i = 0; i < result.size(); i++)
	{
		int flag = 0;
		for (int j = 0; j < final_result.size(); j++)
		{

			if (sqrt(pow(final_result[j].x - result[i].x, 2) + pow(final_result[j].y - result[i].y, 2)) < parameter.minsize_between_each_contour_centriod)
			{
				flag = 1;
				if (contour_area_temp_record[i] > final_contour_area_temp_record[j])
				{
					final_contour_area_temp_record[j] = contour_area_temp_record[i];
					final_result[j] = result[i];
					//2019/09/25
					final_contours[j] = contours_temp[i];
				}
			}
		}
		if (flag == 0)
		{
			final_result.push_back(result[i]);
			final_contour_area_temp_record.push_back(contour_area_temp_record[i]);
			//2019/09/25
			final_contours.push_back(contours_temp[i]);
		}
	}
	//mzy 2019/10/18
	if (final_result.size() > 3)
		pose_result.detect_flag = 2;
	//剔除位置低、不够圆的斑点
	//2019/9/25
	vector<Point2f>final_second_results;
	vector<Point>circle_contour;
	double pi = 3.1415926;
	for (double angle = 0; angle < 2 * pi; angle=angle+0.2*pi)
	{
		circle_contour.push_back(cv::Point2f(20*cos(angle),-20*sin(angle)));
	}
	huabu_test=convertTo3Channels(huabu_test);
	for (int i = 0; i < final_contours.size(); i++)
	{
		//double factor = centerPoints(final_contours[i]);
		double factor_new=cv::matchShapes(circle_contour, final_contours[i],1,0);
	//	cv::putText(huabu_test,to_string(i),cv::Point(final_result[i].x+10, final_result[i].y + 10), font_face, font_scale, cv::Scalar(255,0,255), thickness, 8, 0);
		double circle_thresh = 0.1;
		//drawContours(huabu_test, final_contours, i,Scalar(255,255,0), 1, 8);
		//std::cout <<"factor"<< factor << std::endl;
		//std::cout << "factor_new" << factor_new << std::endl;
		//如果大于阈值且y值大于一定数值代表不匹配，否则添加
		if (factor_new > circle_thresh&&final_result[i].y>1300)
		{
			continue;
		}
		//mzy 2019/10/18特别不圆的有可能是窗口，直接去掉
		if (factor_new > 1.0)
		{
			continue;
		}
        if (final_result[i].y > 2000)
			continue;
		final_second_results.push_back(final_result[i]);
		final_second_contour_area_temp_record.push_back(final_contour_area_temp_record[i]);
	}
	//cvtColor(scene_image, scene_image,COLOR_GRAY2BGR);
	//for (int i = 0; i < final_result.size(); i++)
	//	cv::circle(scene_image, final_result[i], 1, cv::Scalar(255, 255, 0));
	vector<cv::Point2f>final_final_result;
	vector<cv::Point2f>contour_area_small;
	vector<cv::Point2f>contour_area_big;
	
	if (final_second_results.size() == 4)
		if (check_four_point(final_second_results, parameter))
		{
			final_final_result = final_second_results;
			//show_the_result(imageContours, image, scene_image, contour_area_big, contour_area_small, final_second_results, final_final_result);
			return final_second_results;
		}
	if (final_second_results.size() == 5)
		if (check_five_point(final_second_results, parameter))
		{
			final_final_result = final_second_results;
		//	show_the_result(imageContours, image, scene_image, contour_area_big, contour_area_small, final_second_results, final_final_result);
			return final_second_results;
		}
	if (final_second_results.size() == 9)
		if (check_nine_point(final_second_results, parameter))
		{
			cv::Mat showImg = scene_image.clone();
			if (showImg.channels() == 1) {
				cv::cvtColor(showImg, showImg, cv::COLOR_GRAY2BGR);
			}
	//		for (int i = 0; i < final_second_results.size(); i++)
	//		{
	//			cv::putText(showImg, to_string(i), cv::Point2d(final_second_results[i].x, final_second_results[i].y), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
	//			cv::circle(showImg, cv::Point2f(final_second_results[i].x, final_second_results[i].y),1,(0, 0, 255));
	//		}
		//	drawContours(showImg, contours, -1, Scalar(0, 255, 0), 1, 8);
			//cv::imshow("scene_image", scene_image);
			//cv::waitKey(30);
			final_final_result = final_second_results;
//			show_the_result(imageContours, image, scene_image, contour_area_big, contour_area_small, final_second_results, final_final_result);
			return final_second_results;
		}
	
	if (final_second_results.size() >= 4)
		final_final_result = cluster_result(final_second_results, parameter);
	if (final_final_result.size() != 0)
	{
		//8_03
		//bool flag=check_nine_point(final_final_result, parameter);

		/*cv::Mat showImg = scene_image.clone();
		if (showImg.channels() == 1) {
			cv::cvtColor(showImg, showImg, cv::COLOR_GRAY2BGR);
		}
		for (int i = 0; i < final_final_result.size(); i++)
		{
			cv::putText(showImg, to_string(i), cv::Point2d(final_final_result[i].x, final_final_result[i].y), font_face, font_scale, cv::Scalar(255), 1, 8, 0);
			cv::circle(showImg, cv::Point2f(final_final_result[i].x, final_final_result[i].y), 1, cv::Scalar(0, 0, 255));
		}
		drawContours(showImg, contours, -1, Scalar(0, 255, 0), 1, 8);*/
		//cv::imshow("scene_image", scene_image);
		//cv::waitKey(30);
	//	show_the_result(imageContours, image, scene_image, contour_area_big, contour_area_small, final_second_results, final_final_result);
		return final_final_result;
	}
	float max_contour_area_size = 0.0;

	float max_contour_area_y = 0.0;
	int max = -1;
	for (int i = 0; i < final_second_contour_area_temp_record.size(); i++)
		if (final_second_contour_area_temp_record[i] > max_contour_area_size)
		{
			max_contour_area_size = final_second_contour_area_temp_record[i];
			max_contour_area_y = final_second_results[i].y;
			max = i;
		}
	//contour较小的点同时y值也是很小的
	for (int i = 0; i < final_second_contour_area_temp_record.size(); i++)
		if (final_second_contour_area_temp_record[i] < max_contour_area_size&&final_second_results[i].y>max_contour_area_y+20&&(!(final_second_contour_area_temp_record[i]>0.5*max_contour_area_size)))
			contour_area_small.push_back(final_second_results[i]);
		else
			contour_area_big.push_back(final_second_results[i]);
	
	if (contour_area_big.size() >= 4)
	final_final_result = cluster_result(contour_area_big, parameter);
    if(final_final_result.size()==0)
		if(contour_area_small.size()>=4)
			final_final_result = cluster_result(contour_area_small, parameter);

	

  
//	show_the_result(imageContours, image, scene_image, contour_area_big, contour_area_small, final_second_results, final_final_result);
	return final_final_result;
}   
void show_the_result(cv::Mat imageContours,cv::Mat image,cv::Mat scene_image, vector<cv::Point2f>contour_area_big, vector<cv::Point2f>contour_area_small, vector<Point2f>final_result, vector<Point2f>final_final_result)
{
	int font_face = cv::FONT_HERSHEY_COMPLEX;
double font_scale = 2;
int thickness = 2;
	cv::Mat image_copy;
	cv::resize(imageContours, imageContours, cv::Size(image.cols / 4, image.rows / 4));
     cv::Mat image_contour_copy = imageContours.clone();

	for (int i = 0; i < contour_area_big.size(); i++)
		cv::putText(imageContours, "big", cv::Point2d(contour_area_big[i].x / 4, contour_area_big[i].y / 4), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
	for (int i = 0; i < contour_area_small.size(); i++)
		cv::putText(imageContours, "small", cv::Point2d(contour_area_small[i].x / 4, contour_area_small[i].y / 4), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
	for (int i = 0; i < final_result.size(); i++)
		cv::putText(imageContours, to_string(i), cv::Point2d(final_result[i].x / 4, final_result[i].y / 4), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
	for (int i = 0; i < final_final_result.size(); i++)
		cv::putText(image_contour_copy, to_string(i), cv::Point2d(final_final_result[i].x / 4, final_final_result[i].y / 4), font_face, font_scale, cv::Scalar(255), thickness, 8, 0);
	cv::imshow("image_final_result", imageContours);
	cv::imshow("final_final", image_contour_copy);
	cv::resize(scene_image, scene_image, cv::Size(scene_image.cols / 4, scene_image.rows / 4));
	cv::imshow("scene_image", scene_image);
	cv::waitKey(30);
}
bool check_is_rectangle(vector<cv::Point2d>four_point, infrared_solvePnP_parameter parameter)
{
	if (four_point.size() != 4)
		return false;
	float distance_0_1 = sqrt(pow((four_point[0].x - four_point[1].x), 2) + pow((four_point[0].y - four_point[1].y), 2));
	float distance_2_3= sqrt(pow((four_point[2].x - four_point[3].x), 2) + pow((four_point[2].y - four_point[3].y), 2));
	if (abs(distance_0_1 - distance_2_3) > parameter.min_distance_difference_between_01_23*distance_0_1)
		return false;
	float slope_01, slope_02, slope_23, slope_13;
	if (four_point[0].x - four_point[1].x != 0)
		slope_01 = (four_point[0].y - four_point[1].y) / (four_point[0].x - four_point[1].x);
	else
		slope_01 = four_point[0].y - four_point[1].y;

	if (four_point[0].x - four_point[2].x != 0)
		slope_02 = (four_point[0].y - four_point[2].y) / (four_point[0].x - four_point[2].x);
	else
		slope_02 = four_point[0].y - four_point[2].y;

	if (four_point[1].x - four_point[3].x != 0)
		slope_13 = (four_point[1].y - four_point[3].y) / (four_point[1].x - four_point[3].x);
	else
		slope_13 = four_point[1].y - four_point[3].y;

	if (four_point[2].x - four_point[3].x != 0)
		slope_23 = (four_point[2].y - four_point[3].y) / (four_point[2].x - four_point[3].x);
	else
		slope_23 = four_point[2].y - four_point[3].y;


	float angle_01_02, angle_01_13, angle_02_23, angle_13_23;
	if (slope_01*slope_02 == -1)
		angle_01_02 = 90;
	else
        angle_01_02 = std::atan(std::abs((slope_02 - slope_01) / (1 + slope_01 * slope_02)))*180/3.1415;
	if (std::abs(angle_01_02 - 90) > parameter.inclination_deviation_90)
		return false;

	if (slope_01*slope_13 == -1)
		angle_01_13 = 90;
	else
		angle_01_13 = std::atan(std::abs((slope_01 - slope_13) / (1 + slope_01 * slope_13))) * 180 / 3.1415;
	if (std::abs(angle_01_13 - 90) > parameter.inclination_deviation_90)
		return false;

	if (slope_02*slope_23 == -1)
		angle_02_23 = 90;
	else
		angle_02_23 = std::atan(std::abs((slope_02 - slope_23) / (1 + slope_02 * slope_23))) * 180 / 3.1415;
	if (std::abs(angle_02_23 - 90) > parameter.inclination_deviation_90)
		return false;

	if (slope_13*slope_23 == -1)
		angle_13_23 = 90;
	else
		angle_13_23 = std::atan(std::abs((slope_13 - slope_23) / (1 + slope_13 * slope_23))) * 180 / 3.1415;
	if (std::abs(angle_13_23 - 90) > parameter.inclination_deviation_90)
		return false;
	return true;
}
std::vector<cv::Point2f>cluster_result(std::vector<cv::Point2f>result,infrared_solvePnP_parameter parameter)
{
	cv::Mat_<cv::Point2f> src_points(result);
	vector<cv::Point2f> false_result;
	vector<int>ClusterNum = { 1,2,3 };
	vector<double> compactness(3);
	vector<Mat> center(3);
	vector<Mat> clusters(3);// (result.size(), 1, CV_32SC1);
	//compactness to measure the clustering center dist sum by different flag
	for (int i = 0; i < ClusterNum.size(); i++)
	{
		//clusters[i] = cv::Mat(result.size(),1, CV_32SC1);
		//center[i] = cv::Mat(ClusterNum[i],2,CV_32FC1);
		//criteria : It is the iteration termination criteria. When this criteria is satisfied, algorithm iteration stops. Actually, it should be a tuple of 3 parameters. They are ( type, max_iter, epsilon epsilon - Required accuracy)
		//mzy 2019/10/14
		//compactness[i] = kmeans(src_points, ClusterNum[i], clusters[i], cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01), 1,KMEANS_PP_CENTERS, center[i]);
		compactness[i] = kmeans(src_points, ClusterNum[i], clusters[i], TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 100, 0.01), 1, KMEANS_PP_CENTERS, center[i]);
	}
	std::cout << "compactness-------------" << std::endl;
	std::cout << compactness[0] << std::endl;
	std::cout << compactness[1] << std::endl;
	std::cout << compactness[2] << std::endl;
	std::cout << "0/1"<<compactness[0]/ compactness[1] << std::endl;
	std::cout << "1/2" << compactness[1] / compactness[2] << std::endl;
	std::cout << "0/2" << compactness[0] / compactness[2] << std::endl;

	//当分两类的时候，如果类间距离《类外距离的五倍，一定不能分一类
	//分两类时的类间距离
	float two_division_distance=sqrt(pow(center[1].at<float>(0, 0) - center[1].at<float>(1, 0), 2) + pow(center[1].at<float>(0, 1) - center[1].at<float>(1, 1), 2));
	bool must_division = false;
	if ((result.size() == 9 && !check_nine_point(result, parameter)) || (result.size() == 5 && !check_five_point(result, parameter)))
		must_division = true;
    if(compactness[0]>compactness[1]*parameter.compactness_magnification || result.size() > 9||result.size()==6||result.size()==7||result.size()==8||must_division)
		if (compactness[1] > compactness[2] * parameter.compactness_magnification|| compactness[0]/ compactness[2]> compactness[0] / compactness[1]*3)
		{
			//此处要分三类
			vector<vector<cv::Point2f>>result_012(3);
			for (int i = 0; i < result.size(); i++)
				if (clusters[2].at<int>(i) == 0)
					result_012[0].push_back(result[i]);
				else if (clusters[2].at<int>(i) == 1)
					result_012[1].push_back(result[i]);
				else
					result_012[2].push_back(result[i]);

			for (int i = 0; i < result_012.size(); i++)
				if (result_012[i].size() == 4)
				{
					if (check_four_point(result_012[i], parameter))
						return result_012[i];
				}
				else if (result_012[i].size() == 5)
				{
					if (check_five_point(result_012[i], parameter))
						return result_012[i];
				}
			for (int i = 0; i < result_012.size(); i++)
				if (result_012[i].size() == 9)
					if (check_nine_point(result_012[i], parameter))
					return result_012[i];
			for (int i = 0; i < result_012.size(); i++)
				if (result_012[i].size() >=5)
					return cluster_result(result_012[i], parameter);

		}
		else
		{
			//此处要分两类
			vector<vector<cv::Point2f>>result_01(2);
			for (int i = 0; i < result.size(); i++)
				if (clusters[1].at<int>(i) == 0)
					result_01[0].push_back(result[i]);
				else
					result_01[1].push_back(result[i]);
			for (int i = 0; i < result_01.size(); i++)
				if (result_01[i].size() == 4)
				{
					if (check_four_point(result_01[i], parameter))
						return result_01[i];
				}
				else if(result_01[i].size() == 5)
				{
					if (check_five_point(result_01[i], parameter))
						return result_01[i];
				}
			for (int i = 0; i < result_01.size(); i++)
				if (result_01[i].size() == 9)
					if (check_nine_point(result_01[i], parameter))
					    return result_01[i];	
			for (int i = 0; i < result_01.size(); i++)
				if (result_01[i].size()>=5)
					return cluster_result(result_01[i], parameter);

		}
	if (result.size() == 4)
		if (check_four_point(result, parameter))
			return result;
	if (result.size() == 5)
		if (check_five_point(result, parameter))
			return result;
	if (result.size() == 9)
		if (check_nine_point(result, parameter))
		return result;
		return false_result;
}
