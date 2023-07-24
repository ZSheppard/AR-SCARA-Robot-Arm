#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas, _canvas2;

	double timeNew, timeOld, timeChange, velocity, position;
	vector<vector<Mat>> _simple_robot;
	vector<vector<Mat>> _augmented_robot;
	vector<vector<Mat>> _robot;
	vector<vector<Mat>> _robot_coords;

	Vec3d rvec_robot, tvec_robot;
	int _j0x, _j0y, _j0z;
	int _j1x, _j1y, _j1z;
	int _j2x, _j2y, _j2z;
	int _j3d;
	int s;

	//LAB6
	int EE_x, EE_y;
	int _link1_angle, _link2_angle;
	float link1, link2;

	CCamera _virtualcam, _realcam;

	//CuArm uarm;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void draw_augmented_box(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);
	

	////////////////////////////////////
	// LAB 5
	int _do_animate, flag; // Animation state machine
	void init();
	void update_settings(Mat& im);
	int _j0_link_twist;
	int linkLength;
	float _j0_link_offset, _j0_link_length;
	void _coords5();

	////////////////////////////////////
	// LAB 6
	void _coords6();

public:
	Mat createHT(Vec3d t, Vec3d r);

	/////////////////////////////
	// Lab 3
	void create_simple_robot();
	void draw_simple_robot();

	/////////////////////////////
	// Lab 4
	void draw_coord_aruco();
	void create_augmented_robot();
	void draw_augmented_robot();
	Mat createHT_augmented(Vec3d t, Vec3d r);


	////////////////////////////////////
	// Lab 5
	void create_scara_robot();
	void draw_scara_robot();
	void draw();
	void drawCoord_augmented(Mat& im, std::vector<Mat> coord3d);
	void fkine(); // Input joint variables, output end effector pose
	Mat DH_transformation(int joint_angleX, int joint_angleY, int joint_angleZ, float link_offset, float link_length, int link_twist);
	Mat rot(int angleX, int angleY, int angleZ);
	Mat translX(float positionX);
	Mat translY(float positionY);
	Mat translZ(float positionZ);
	vector<Mat> createBox_robot(float w, float h, float d);

	////////////////////////////////////
	// Lab 6
	void draw_inverse();
	void create_scara_robot6();
	bool joint_control = false;
	
};

