#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"
#define PI 3.14159265358979323846

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1100, 700);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	_canvas2 = cv::Mat::zeros(_image_size, CV_8UC3);
	//Lab3 canvas initialization
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	//Lab4 canvas initialization
	_realcam.initialize_camera(0);
	_realcam.frameTocanvas(_canvas);

	//Lab5 canvas2 initialization
	_realcam.frameTocanvas(_canvas2);

	timeOld = getTickCount();
	timeChange = 0;
	velocity = 50;
	position = 0;
	linkLength = 0.15;
	init();

  ///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	//LAB 5
	// reset variables
	_do_animate = 0;
	_j0x = 0;//start off angle for first link at zero degrees
	_j0y = 0;
	_j0z = 0;

	_j1x = 0;//start off angle for first link at zero degrees
	_j1y = 0;
	_j1z = 0;

	_j2x = 0;//start off angle for first link at zero degrees
	_j2y = 0;
	_j2z = 0;

	_j3d = 0;

	//LAB6

	EE_x = 20;
	EE_y = 1;

	flag = 0;
}

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	//return (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

	// t = x, y,z
	// r = roll, pitch, yaw

	float c_alpha = cos(((int)r[2] * PI) / 180);
	float c_beta = cos((r[1] * PI) / 180);
	float c_gamma = cos((r[0] * PI) / 180);
	float s_alpha = sin(((int)r[2] * PI) / 180);
	float s_beta = sin((r[1] * PI) / 180);
	float s_gamma = sin((r[0] * PI) / 180);

	 return (Mat1f(4, 4) << (c_alpha * c_beta), ((c_alpha * s_beta * s_gamma) - (s_alpha * c_gamma)), ((c_alpha * s_beta * c_gamma) + (s_alpha * s_gamma)), ((float)t[0] / 1000),
					    	(s_alpha * c_beta), ((s_alpha * s_beta * s_gamma) + (c_alpha * c_gamma)), (s_alpha * s_beta * c_gamma) - (c_alpha * s_gamma), ((float)t[1] / 1000),
	    					 -s_beta, (c_beta * s_gamma), (c_beta * c_gamma), ((float)t[2] / 1000),
							 0, 0, 0, 1);
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;

	float axis_length = 0.05;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, -axis_length, 1)); // Z

	return coord;
}

void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::create_simple_robot()
{
	//create a new vanilla box for each individual box
	std::vector<Mat> box  = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box2 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box3 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box4 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box5 = createBox(0.05, 0.05, 0.05);

	//Create HT matrix for respective box
	Mat box_2_transform = createHT(Vec3d(0,50,0),Vec3d(0,0,0));	//Translate box 50mm upwards along z axis
	Mat box_3_transform = createHT(Vec3d(0, 150, 0), Vec3d(0, 0, 0));
	Mat box_4_transform = createHT(Vec3d(-50, 100, 0), Vec3d(0, 0, 0));
	Mat box_5_transform = createHT(Vec3d(50, 100, 0), Vec3d(0, 0, 0));

	//fill _simple_robot with points for each box
	_simple_robot.push_back(box);	//Use private member (_simple_robot to act as global variable and use in all functions	//ZS

	//Transform box matrix using transformation matrix, updating "box#" with 2D points
	transformPoints(box2, box_2_transform);
	transformPoints(box3, box_3_transform);
	transformPoints(box4, box_4_transform);
	transformPoints(box5, box_5_transform);
	
	_simple_robot.push_back(box2);	//ZS
	_simple_robot.push_back(box3);	//ZS
	_simple_robot.push_back(box4);	//ZS
	_simple_robot.push_back(box5);	//ZS
	
}	

void CRobot::draw_simple_robot()
{
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	std::vector<Mat> O = createCoord();
	drawCoord(_canvas, O);	//ZS

	drawBox(_canvas, _simple_robot.at(0), CV_RGB(255, 0, 0));	//ZS
	drawBox(_canvas, _simple_robot.at(1), CV_RGB(255, 0, 0));	//ZS
	drawBox(_canvas, _simple_robot.at(2), CV_RGB(255, 0, 0));	//ZS
	drawBox(_canvas, _simple_robot.at(3), CV_RGB(0, 0, 255));	//ZS
	drawBox(_canvas, _simple_robot.at(4), CV_RGB(0, 255, 0));	//ZS

	_virtualcam.update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////
//LAB4
////////////////////////////////////
void CRobot::draw_augmented_box(Mat& im, std::vector<Mat> box3d, Scalar colour){

	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_realcam.transform_to_image_augmented(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::create_augmented_robot() {

	std::vector<Point2f> box2d;
	//create a new vanilla box for each individual box
	std::vector<Mat> box = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box2 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box3 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box4 = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> box5 = createBox(0.05, 0.05, 0.05);

	Mat box_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, position));
	Mat box_2_transform = createHT(Vec3d(0, 0, 50), Vec3d(0, 0, position));
	Mat box_3_transform = createHT(Vec3d(0, 0, 150), Vec3d(0, 0, position));

	Mat box_4_transform = createHT(Vec3d(-50, 0, 100), Vec3d(0, 0, 0));
	Mat box_4_W_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, position));
	
	Mat box_5_transform = createHT(Vec3d(50, 0, 100), Vec3d(0, 0, 0));
	Mat box_5_W_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, position));

	transformPoints(box, box_transform);
	transformPoints(box2, box_2_transform);
	transformPoints(box3, box_3_transform);
	
	transformPoints(box4, box_4_W_transform *  box_4_transform);

	transformPoints(box5, box_5_W_transform * box_5_transform);


	_augmented_robot.push_back(box);
	_augmented_robot.push_back(box2);
	_augmented_robot.push_back(box3);	//ZS
	_augmented_robot.push_back(box4);	//ZS
	_augmented_robot.push_back(box5);	//ZS
	//_realcam.transform_to_image_augmented();

}

void CRobot::draw_augmented_robot() {

	create_augmented_robot();
	_realcam.update_real(_canvas, tvec_robot, rvec_robot);	//Captures frames of webcam and detects markers/pose
	_realcam.update_settings(_canvas);	//Tracks XYZ value of pose on board

	//Code to control rotation of robot
	timeNew = getTickCount();
	timeChange = (timeNew - timeOld) / getTickFrequency();
	timeOld = timeNew;
	position += (velocity * timeChange);

	//draw boxes of robot
	draw_augmented_box(_canvas, _augmented_robot.at(0), CV_RGB(255, 0, 0));	//ZS
	draw_augmented_box(_canvas, _augmented_robot.at(1), CV_RGB(255, 0, 0));	//ZS
	draw_augmented_box(_canvas, _augmented_robot.at(2), CV_RGB(255, 0, 0));	//ZS
	draw_augmented_box(_canvas, _augmented_robot.at(3), CV_RGB(0, 0, 255));	//ZS
	draw_augmented_box(_canvas, _augmented_robot.at(4), CV_RGB(0, 255, 0));	//ZS


	cv::imshow("out", _canvas);

	_augmented_robot.clear();
}

////////////////////////////////////
//LAB5
////////////////////////////////////
std::vector<Mat> CRobot::createBox_robot(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << 0, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << 0, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << 0, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << 0, h / 2, d / 2, 1)));

	return box;
}

void CRobot::drawCoord_augmented(Mat& im, std::vector<Mat> coord3d)
{
	vector<Point2f> O, X, Y, Z;
	vector<Point2f> coord;

	//_realcam.transform_to_image_augmented(coord3d.at(0), coord);

	//line(im, coord.at(0), coord.at(1), CV_RGB(255, 0, 0), 1); // X=RED
	//line(im, coord.at(0), coord.at(2), CV_RGB(0, 255, 0), 1); // Y=GREEN
	//line(im, coord.at(0), coord.at(3), CV_RGB(0, 0, 255), 1); // Z=BLUE

	_realcam.transform_to_image_augmented(coord3d.at(0), O);
	_realcam.transform_to_image_augmented(coord3d.at(1), X);
	_realcam.transform_to_image_augmented(coord3d.at(2), Y);
	_realcam.transform_to_image_augmented(coord3d.at(3), Z);
	
	line(im, O.at(0), X.at(0), CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O.at(0), Y.at(0), CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O.at(0), Z.at(0), CV_RGB(0, 0, 255), 1); // Z=BLUE
}

Mat CRobot::rot(int angleX, int angleY, int angleZ) {

	float c_alpha = cos(((int)angleZ * PI) / 180);
	float c_beta = cos(((int)angleY * PI) / 180);
	float c_gamma = cos(((int)angleX * PI) / 180);
	float s_alpha = sin(((int)angleZ * PI) / 180);
	float s_beta = sin(((int)angleY * PI) / 180);
	float s_gamma = sin(((int)angleX * PI) / 180);

	Mat Rz = (Mat1f(4, 4) << (c_alpha * c_beta), ((c_alpha * s_beta * s_gamma) - (s_alpha * c_gamma)), ((c_alpha * s_beta * c_gamma) + (s_alpha * s_gamma)), 0,
							 (s_alpha * c_beta), ((s_alpha * s_beta * s_gamma) + (c_alpha * c_gamma)), (s_alpha * s_beta * c_gamma) - (c_alpha * s_gamma), 0,
							 -s_beta, (c_beta * s_gamma), (c_beta * c_gamma), 0,
							  0, 0, 0, 1);
	return Rz;
}

Mat CRobot::translX(float positionX) {

	Mat Tz = (Mat1f(4, 4) << 1, 0, 0,positionX,
							 0, 1, 0, 0,
							 0, 0, 1, 0,
							 0, 0, 0, 1);

		return Tz;
}

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 475, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j0y, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j0");

	//_setting_window.x += 5;
	_setting_window.y += 45;

	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j1y, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j1");

	//_setting_window.x += 5;
	_setting_window.y += 45;

	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j2x, 0, 360);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j2");

	//_setting_window.x += 5;
	_setting_window.y += 45;

	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j3d, 0, 15);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j3");

	
	_setting_window.y += 45;
	if (cvui::button(im, _setting_window.x, _setting_window.y + 10, 100, 30, "Reset"))
	{
		init();
	}
	
	if (cvui::button(im, _setting_window.x + 100, _setting_window.y + 10, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	//LAB6 INVERSE KINEMATICS TRACKBARS & BUTTONS

	//If pose is invalid display "Invalid Pose"
	if ((EE_x == 0 && EE_y == 0) || (EE_x == 30) || (EE_x == -30) || (sqrt(pow(EE_x, 2) + pow(EE_y, 2)) > 30)) {
		cvui::text(im, 20, 650, "Invalid Pose", 1, 0xff0000);
	}
	
		_setting_window.y += 45;
		
		cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &EE_x, -30, 30);
		cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "X");

		_setting_window.y += 45;

		cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &EE_y, 0, 15);
		cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Y");

		_setting_window.y += 45;

		cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j3d, 0, 15);
		cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Z");

		_setting_window.y += 45;

		cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_j2x, -180, 180);
		cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Theta");

		_setting_window.y += 45;
	

		if (cvui::checkbox(im, _setting_window.x, _setting_window.y + 10, "Joint Control", &joint_control))
		{
			//init();

		}
	

	if (cvui::button(im, _setting_window.x + 110, _setting_window.y + 10, 80, 30, "Track Mark"))
	{
		init();
	}

	if (_do_animate != 0)
	{
		int step_size = 5;
		int step_sizet = 1;
			
			//STATE 1
		if (_do_animate == 1)
		{
			if (flag == 0)
			{
				if (_j0y <= 180) {
					_j0y += step_size;
				}
				else
					flag = 1;
			}
			else if (flag == 1)
			{
				if (_j0y >= -180)
					_j0y -= step_size;
				else
					flag = 2;
			}
			else if (flag == 2)
			{
				if (_j0y <= 0)
					_j0y += step_size;
				else
					flag = 3;
			}
			else
			{
				flag = 0;
				_do_animate = 2;
			}
		}
			
			//STATE 2
		if (_do_animate == 2)
		{
			if (flag == 0)
			{
				if (_j1y <= 180) {
					_j1y += step_size;
				}
				else
					flag = 1;
			}
			else if (flag == 1)
			{
				if (_j1y >= -180)
					_j1y -= step_size;
				else
					flag = 2;
			}
			else if (flag == 2)
			{
				if (_j1y <= 0)
					_j1y += step_size;
				else
					flag = 3;
			}
			else
			{
				flag = 0;
				_do_animate = 3;
			}
		}

			//STATE 3
		if (_do_animate == 3)
		{
			if (flag == 0)
			{
				if (_j2x<= 360) {
					_j2x += step_size;
				}
				else
					flag = 1;
			}

			else
			{
				flag = 0;
				_do_animate = 4;
			}
		}

			//STATE 4
		if (_do_animate == 4)
			{
				if (flag == 0)
				{
					if (_j3d < 15) {
						_j3d += step_sizet;
					}
					else
						flag = 1;
				}
				else if (flag == 1)
				{
					if (_j3d > 0) {
						_j3d -= step_sizet;
					}
					else
						flag = 2;
				}
				else
				{
					flag = 0;
					_do_animate = 5;
				}
			}
		
		//STATE 5 - RESET
		else if (_do_animate == 5) {
			//state 5
			if (1) { _do_animate = 0; flag = 0; init(); }
		}	
	}
	cvui::update(); //If I comment this out and leave cvui::update(); in camera.cpp reset in CRobot doesnt work. Why?
}

void CRobot::create_scara_robot() {

	//create box
	std::vector<Mat> base = createBox(0.125, 0.05, 0.05);
	std::vector<Mat> box = createBox_robot(0.15, 0.05, 0.05);
	std::vector<Mat> box2 = createBox_robot(0.15, 0.05, 0.05);
	std::vector<Mat> box3 = createBox_robot(0.15, 0.05, 0.05);
	
	cout << "\n_j3d = " << _j3d << endl;
	//Create HT matrices for boxes
	Mat joint_base = rot(0, 0, 90) * translX(0.0625);	//Augmented settings: rot(0, -90, 180)		VirtualCam settings: rot(0, 0, 90)
	Mat joint0 = translX(0.0875) * rot(0, _j0y,-90) ;
	Mat joint1 = translX(0.15) * rot(0, _j1y, 0);
	Mat joint2 = translX(0.175) * rot(_j2x, 0, -90);
	Mat joint3 = translX((float((float)_j3d-15) / 100));
	
	//Transform box matrix using transformation matrix, updating "box#" with 2D points
	transformPoints(base, joint_base);
	transformPoints(box, joint_base * joint0);
	transformPoints(box2, joint_base * joint0 * joint1);
	transformPoints(box3, joint_base * joint0 * joint1 * joint2 * joint3);

	//private member adds boxes to a vector
	_robot.push_back(base);
	_robot.push_back(box);
	_robot.push_back(box2);
	_robot.push_back(box3);
}

void CRobot::_coords5() {

	//Creating all 5 coordinates
	//Mat box_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));	//Translate box 75mm upwards along z axis
	Mat joint_base = rot(0, 0, 90) * translX(0.0625);	//Augmented settings: rot(0, -90, 180)			VirtualCam settings: rot(0, 0, 90)
	Mat joint0 = translX(0.0875) * rot(0, _j0y, -90);
	Mat joint1 = translX(0.15) * rot(0, _j1y, 0);
	Mat joint2 = translX(0.175) * rot(_j2x, 0, -90);
	Mat joint3 = translX((((float((float)_j3d) - 15)) / 100) + 0.15);

	std::vector<Mat> base_coord = createCoord();
	std::vector<Mat> O = createCoord();
	std::vector<Mat> O2 = createCoord();
	std::vector<Mat> O3 = createCoord();
	std::vector<Mat> O4 = createCoord();

	transformPoints(O, joint_base * joint0);
	transformPoints(O2, joint_base * joint0 * joint1);
	transformPoints(O3, joint_base * joint0 * joint1 * joint2);
	transformPoints(O4, joint_base * joint0 * joint1 * joint2 * joint3);

	//private member adds boxes to a vector
	_robot_coords.push_back(base_coord);
	_robot_coords.push_back(O);
	_robot_coords.push_back(O2);
	_robot_coords.push_back(O3);
	_robot_coords.push_back(O4);
}

void CRobot::draw()
{
	create_scara_robot();

	_realcam.update_real(_canvas, tvec_robot, rvec_robot);	//Captures frames of webcam and detects markers/pose
	_realcam.update_settings(_canvas);	//Tracks XYZ value of pose on board
	//_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//Creating all 5 coordinates
	//Mat box_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));	//Translate box 75mm upwards along z axis
	Mat joint_base = rot(0, -90, 180) * translX(0.0625);
	Mat joint0 = translX(0.0875) * rot(0, _j0y, -90) ;
	Mat joint1 = translX(0.15) * rot(0, _j1y, 0);
	Mat joint2 = translX(0.175) * rot(_j2x, 0, -90);
	Mat joint3 = translX((((float((float)_j3d) - 15))/100) + 0.15);
	
	std::vector<Mat> base_coord = createCoord();
	std::vector<Mat> O = createCoord();
	std::vector<Mat> O2 = createCoord();
	std::vector<Mat> O3 = createCoord();
	std::vector<Mat> O4 = createCoord();

	transformPoints(O, joint_base * joint0);
	transformPoints(O2, joint_base * joint0 * joint1);
	transformPoints(O3, joint_base * joint0 * joint1 * joint2);
	transformPoints(O4, joint_base * joint0 * joint1 * joint2 * joint3);

	drawCoord_augmented(_canvas, base_coord);
	drawCoord_augmented(_canvas, O);	//ZS
	drawCoord_augmented(_canvas, O2);	//ZS
	drawCoord_augmented(_canvas, O3);	//ZS
	drawCoord_augmented(_canvas, O4);	//ZS

	draw_augmented_box(_canvas, _robot.at(0), CV_RGB(255, 255, 255));	//ZS
	draw_augmented_box(_canvas, _robot.at(1), CV_RGB(255, 0, 0));		//ZS
	draw_augmented_box(_canvas, _robot.at(2), CV_RGB(0, 255, 0));		//ZS
	draw_augmented_box(_canvas, _robot.at(3), CV_RGB(0, 0, 255));		//ZS

	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
	//cv::imshow("out", _canvas);

	_robot.clear();
}

////////////////////////////////////
//LAB6A
////////////////////////////////////
void CRobot::create_scara_robot6() {

	//create box
	std::vector<Mat> base = createBox(0.125, 0.05, 0.05);
	std::vector<Mat> box = createBox_robot(0.15, 0.05, 0.05);
	std::vector<Mat> box2 = createBox_robot(0.15, 0.05, 0.05);
	std::vector<Mat> box3 = createBox_robot(0.15, 0.05, 0.05);

	cout << "\n_j3d = " << _j3d << endl;
	//Create HT matrices for boxes
	Mat joint_base = rot(0, 0, 90) * translX(0.0625);	//Augmented settings: rot(0, -90, 180)		VirtualCam settings: rot(0, 0, 90)
	Mat joint0 = translX(0.0875) * rot(0, _link1_angle, -90);
	Mat joint1 = translX(0.15) * rot(0, _link2_angle, 0);
	Mat joint2 = translX(0.175) * rot(_j2x + _link1_angle + _link2_angle, 0, -90);
	Mat joint3 = translX((float((float)_j3d - 15) / 100));

	//Transform box matrix using transformation matrix, updating "box#" with 2D points
	transformPoints(base, joint_base);
	transformPoints(box, joint_base * joint0);
	transformPoints(box2, joint_base * joint0 * joint1);
	transformPoints(box3, joint_base * joint0 * joint1 * joint2 * joint3);

	//private member adds boxes to a vector
	_robot.push_back(base);
	_robot.push_back(box);
	_robot.push_back(box2);
	_robot.push_back(box3);
}

void CRobot::_coords6() {

	//Creating all 5 coordinates
	//Mat box_transform = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));	//Translate box 75mm upwards along z axis
	Mat joint_base = rot(0, 0, 90) * translX(0.0625);	//Augmented settings: rot(0, -90, 180)			VirtualCam settings: rot(0, 0, 90)
	Mat joint0 = translX(0.0875) * rot(0, _link1_angle, -90);
	Mat joint1 = translX(0.15) * rot(0, _link2_angle, 0);
	Mat joint2 = translX(0.175) * rot(_j2x + _link1_angle + _link2_angle, 0, -90);
	Mat joint3 = translX((((float((float)_j3d) - 15)) / 100) + 0.15);

	std::vector<Mat> base_coord = createCoord();
	std::vector<Mat> O = createCoord();
	std::vector<Mat> O2 = createCoord();
	std::vector<Mat> O3 = createCoord();
	std::vector<Mat> O4 = createCoord();

	transformPoints(O, joint_base * joint0);
	transformPoints(O2, joint_base * joint0 * joint1);
	transformPoints(O3, joint_base * joint0 * joint1 * joint2);
	transformPoints(O4, joint_base * joint0 * joint1 * joint2 * joint3);

	//private member adds boxes to a vector
	_robot_coords.push_back(base_coord);
	_robot_coords.push_back(O);
	_robot_coords.push_back(O2);
	_robot_coords.push_back(O3);
	_robot_coords.push_back(O4);

}

void CRobot::draw_inverse() {

	create_scara_robot6();
	_coords6();

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	if ((EE_x != 0 || EE_y != 0) && (sqrt(pow(EE_x, 2) + pow(EE_y, 2)) < 30)) {

		if (!joint_control) {			
			_virtualcam.ikine_pose(EE_x, EE_y, _link1_angle, _link2_angle);
		}
		else if (joint_control) {
			_robot.clear();
			_robot_coords.clear();
			create_scara_robot();
			_coords5();
		}
	}

	drawCoord(_canvas, _robot_coords.at(0));	//ZS
	drawCoord(_canvas, _robot_coords.at(1));	//ZS
	drawCoord(_canvas, _robot_coords.at(2));	//ZS
	drawCoord(_canvas, _robot_coords.at(3));	//ZS
	drawCoord(_canvas, _robot_coords.at(4));	//ZS

	drawBox(_canvas, _robot.at(0), CV_RGB(255, 255, 255));	//ZS
	drawBox(_canvas, _robot.at(1), CV_RGB(255, 0, 0));		//ZS
	drawBox(_canvas, _robot.at(2), CV_RGB(0, 255, 0));		//ZS
	drawBox(_canvas, _robot.at(3), CV_RGB(0, 0, 255));		//ZS

	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
	//cv::imshow("out", _canvas);

	_robot.clear();
	_robot_coords.clear();
}

////////////////////////////////////
//LAB6B
////////////////////////////////////
