#pragma once

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace dnn;

class CCamera
{
public:
	CCamera();
	~CCamera();
	bool track_board = false;

private:
	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	void calculate_intrinsic();
	void calculate_extrinsic();

	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;

	//Lab4
	// Real webcam
	Mat _cam_real_intrinsic;
	Mat _cam_real_extrinsic;
	Mat _cam_real_dist_coeff;

	Mat cameraMatrix, distCoeffs;
	VideoCapture inputVideo;
	Mat _canvas;
	int dictionary_id;
	Size board_size;
	float size_aruco_square;
	float size_aruco_mark;
	Mat image;
	Mat imageCopy;
	string filename;
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	vector<Point2f> charucoCorners;
	vector<int> charucoIds;
	Scalar color = Scalar(255, 0, 0);
	Vec3d rvec, tvec;

	//LAB5

	//LAB6
	float _link1, _link2; //note: in private


public:
	void init(Size image_size);

	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	void createChArUcoBoard();
	void calibrate_board(int cam_id);

	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);


	//LAB 4
	void update_settings(Mat &im);
	void update_real(Mat& im, Vec3d& tvec, Vec3d& rvec);
	void transform_to_image_augmented(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);
	void transform_to_image_augmented(Mat pt3d_mat, std::vector<Point2f>& pt);
	void initialize_camera(int camid);
	void frameTocanvas(Mat& frame);

	//LAB 5

	//LAB 6
	void ikine_pose(float x, float y, int &s1, int &s2);

};

