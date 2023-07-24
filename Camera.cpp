#include "stdafx.h"

#include "Camera.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"
#define PI 3.14159265358979323846

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1000, 600));

	
	
	dictionary_id = aruco::DICT_6X6_250;
	board_size = Size(5, 7);
	size_aruco_square = 0.0355; // MEASURE THESE (unit is in meters)
	size_aruco_mark = 0.0175; // MEASURE THESE (unit is in meters)
	filename = "cam_param.xml";
}

CCamera::~CCamera()
{
}

void CCamera::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 2;

	_cam_setting_x = 111; // units in mm
	_cam_setting_y = -65; // units in mm
	_cam_setting_z = -600; // units in mm
	_cam_setting_roll = -28; // units in degrees
	_cam_setting_pitch = -180; // units in degrees
	_cam_setting_yaw = 0; // units in degrees

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);
	
	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();
}

void CCamera::calculate_intrinsic()
{
	Mat1f pixel_matrix = (Mat1f(3, 3) << 1 / (float)_pixel_size, 0, _principal_point.x,
										 0, 1 / (float)_pixel_size, _principal_point.y,
										 0, 0, 1);

	Mat1f focal_matrix = (Mat1f(3, 4) << (float)_cam_setting_f / 1000, 0, 0, 0,
										 0, (float)_cam_setting_f / 1000, 0, 0,
										 0, 0, 1, 0);

	Mat1f intrinsic_matrix = pixel_matrix * focal_matrix;

	_cam_virtual_intrinsic = intrinsic_matrix;
}

void CCamera::calculate_extrinsic()
{
	_cam_virtual_extrinsic = (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

	//Defining 3D trig values for 3D homogeneous transformation
	float c_alpha = cos((_cam_setting_yaw * PI) / 180);
	float c_beta = cos((_cam_setting_pitch * PI) / 180);
	float c_gamma = cos((_cam_setting_roll * PI) / 180);
	float s_alpha = sin((_cam_setting_yaw * PI) / 180);
	float s_beta = sin((_cam_setting_pitch * PI) / 180);
	float s_gamma = sin((_cam_setting_roll * PI) / 180);


	Mat1f extrinsic_matrix = (Mat1f(4, 4) << (c_alpha * c_beta), ((c_alpha * s_beta * s_gamma) - (s_alpha * c_gamma)), ((c_alpha * s_beta * c_gamma) + (s_alpha * s_gamma)), ((float)_cam_setting_x / 1000),
											 (s_alpha * c_beta), ((s_alpha * s_beta * s_gamma) + (c_alpha * c_gamma)), (s_alpha * s_beta * c_gamma) - (c_alpha * s_gamma), ((float)_cam_setting_y / 1000),
											  -s_beta, (c_beta * s_gamma), (c_beta * c_gamma), ((float)_cam_setting_z / 1000),
											  0, 0, 0, 1);

	_cam_virtual_extrinsic = extrinsic_matrix;
}

bool CCamera::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

bool CCamera::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);
	
	if (!fs.isOpened())
	{
		return false;
	}
	
	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

void CCamera::createChArUcoBoard()
{
	Mat im;
	float size_square = 0.04; // user specified
	float size_mark = 0.02; // user specified
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_id);
	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
	
	board->draw(cv::Size(1600, 1500), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCamera::calibrate_board(int cam_id)
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;
  
	float size_aruco_square = 0.0355; // MEASURE THESE (unit is in meters)
	float size_aruco_mark = 0.0175; // MEASURE THESE (unit is in meters)

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	VideoCapture inputVideo;
	inputVideo.open(cam_id);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	// Collect data from live video 
	while (inputVideo.grab()) 
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(draw_im);
		
		// First pass detect markers
		aruco::detectMarkers(im, dictionary, corners, corner_ids, detectorParams, rejected_corners);
		// Second pass detect markers
		aruco::refineDetectedMarkers(im, board, corners, corner_ids, rejected_corners);

		// Refine charuco corners
		if (corner_ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0)
		{
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0)
		{
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}
		
		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0) 
		{
			cout << "Frame captured" << endl;
			calib_corner.push_back(corners);
			calib_id.push_back(corner_ids);
			calib_im.push_back(im);
			calib_im_size = im.size();
		}
	}

	if (calib_id.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(calib_corner.size());
	for (unsigned int i = 0; i < calib_corner.size(); i++) {
		markerCounterPerFrame.push_back((int)calib_corner[i].size());
		for (unsigned int j = 0; j < calib_corner[i].size(); j++) {
			allCornersConcatenated.push_back(calib_corner[i][j]);
			allIdsConcatenated.push_back(calib_id[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, calib_im_size, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

	// prepare data for charuco calibration
	int nFrames = (int)calib_corner.size();
	vector< Mat > allCharucoCorners;
	vector< Mat > allCharucoIds;
	vector< Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for (int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		Mat currentCharucoCorners, currentCharucoIds;
		aruco::interpolateCornersCharuco(calib_corner[i], calib_id[i], calib_im[i], charucoboard,
			currentCharucoCorners, currentCharucoIds, cameraMatrix,
			distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(calib_im[i]);
	}

	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	// calibrate camera using charuco
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, calib_im_size, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = save_camparam("cam_param.xml", cameraMatrix, distCoeffs);
	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Rep Error Aruco: " << arucoRepErr << endl;
	cout << "Calibration saved to " << "cam_param.xml" << endl;

	// show interpolated charuco corners for debugging
		for (unsigned int frame = 0; frame < filteredImages.size(); frame++) 
		{
			Mat imageCopy = filteredImages[frame].clone();
			
			if (calib_id[frame].size() > 0) {

				if (allCharucoCorners[frame].total() > 0) 
				{
					aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],allCharucoIds[frame]);
				}
			}

			imshow("out", imageCopy);
			char key = (char)waitKey(0);
			if (key == 27) break;
	}
}

void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	Mat1f pt1 = _cam_virtual_intrinsic * _cam_virtual_extrinsic * pt3d_mat;
	pt.x = (pt1.at<float>(0, 0) / pt1.at<float>(2, 0));	//learn notation
	pt.y = (pt1.at<float>(1, 0) / pt1.at<float>(2, 0));
}

void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	std::vector<Mat> new_pts3d_mat;	//ZS

	for (auto it = begin(pts3d_mat); it != end(pts3d_mat); ++it) {
		new_pts3d_mat.push_back(_cam_virtual_intrinsic * _cam_virtual_extrinsic * *it);  //Dereference iterator??
	}

	for (auto it2 = begin(new_pts3d_mat); it2 != end(new_pts3d_mat); ++it2) {

		pts2d.push_back(Point2f(it2->at<float>(0, 0) / it2->at<float>(2, 0), it2->at<float>(1, 0) / it2->at<float>(2, 0)));
	}


}

void CCamera::update_settings(Mat& im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 400, "Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -750, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -360, 360);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 50;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board);

	if (track_board) {
		_camera_setting_window.y = 90;
		_camera_setting_window.x = 1095;

		cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);	//Cannot cast tvec to trackbar
		cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

		_camera_setting_window.y += 45;
		cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
		cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

		_camera_setting_window.y += 45;
		cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
		cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");
	}

	_camera_setting_window.y += 25;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	//cvui::update(); 

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}

////////////////////////////////////
//LAB4
////////////////////////////////////

//tranforms 3D points to 2D image for webcam using projectPoints and calculated frame variables from update_real
void CCamera::transform_to_image_augmented(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d) {

	vector<cv::Point3f> pts3d;
	for (int j = 0; j < pts3d_mat.size(); ++j) {

			Mat dummy = pts3d_mat.at(j);
			pts3d.push_back(cv::Point3f(dummy.at<float>(0, 0), dummy.at<float>(1, 0), dummy.at<float>(2, 0)));
	}

	projectPoints(pts3d, rvec, tvec, cameraMatrix, distCoeffs, pts2d);

}

void CCamera::transform_to_image_augmented(Mat pt3d_mat, std::vector<Point2f>& pt) {

	vector<cv::Point3f> pt3d;
	vector<Point2f> pt2d;


	pt3d.push_back(cv::Point3f(pt3d_mat.at<float>(0, 0), pt3d_mat.at<float>(1, 0), pt3d_mat.at<float>(2, 0)));
	pt3d.push_back(cv::Point3f(pt3d_mat.at<float>(0, 0), pt3d_mat.at<float>(1, 0), pt3d_mat.at<float>(2, 0)));
	cout << "pt3d = " << pt3d << "\n" << endl;

	projectPoints(pt3d, rvec, tvec, cameraMatrix, distCoeffs, pt);
	
}

void CCamera::initialize_camera(int camid) {
	inputVideo.open(0);
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280); //MOD
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720); //MOD
}

void CCamera::frameTocanvas(Mat& frame) {
	inputVideo.retrieve(image);
	image.copyTo(frame);
}

void CCamera::update_real(Mat &im, Vec3d &tvec_robot, Vec3d &rvec_robot)
{
	//////////////////////////////////////////////////////////////////////////////////
	//Start Webcam, load cam_param files information into cameraMatrix and distCoeffs
	bool readOk = load_camparam(filename, cameraMatrix, distCoeffs);

	////////////////////////////////////////////////////
	//Check if cam_param was loaded successfully
	if (!readOk) {
		std::cerr << "Invalid camera file" << std::endl;
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	//Instructions if camera was loaded successfully (ChArUco detection with pose estimation)
	else {
		std::cerr << "VALID camera file" << std::endl;
		// Defining grid size of board and size of squares/marks; Choosing dictionary for ChAruco markers
		Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
		Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
		Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();	//WHAT DOES THIS DO!
			////////////////////////////////////////////////////////
			//check to see if frame can be captured then capture frame in order to detect markers
		   if(inputVideo.grab()){
			
			inputVideo.retrieve(image);
			image.copyTo(imageCopy);
			aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

			//If at least one marker is detected
			//If the webcam detects a marker it will draw it on imageCopy
			if (markerIds.size() > 0) {
				aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
				aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);

				/////////////////////////////////////////////
				//if at least one charuco corner detected
				if (charucoIds.size() > 0) {
					aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
					bool valid = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);

					//if charuco pose is valid
					//draw coordinate axes at origin
					if (valid)
						drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
				}
			}
		}
		   _cam_setting_y = int(tvec[0] * 1000);
		   _cam_setting_x = int(tvec[1] * 1000);
		   _cam_setting_z = int(tvec[2] * 1000);

		   _cam_setting_roll = int(rvec[0] * 50);
		   _cam_setting_pitch = int(rvec[1] * 50);
		   _cam_setting_yaw = int(rvec[2] * 50);



		   tvec_robot = tvec;
		   rvec_robot = rvec;
		   im = imageCopy;
		
	}
}

////////////////////////////////////
//LAB5
////////////////////////////////////


////////////////////////////////////
//LAB6
////////////////////////////////////

void CCamera::ikine_pose(float x, float y, int &s1, int &s2) {
	//s1 = angle of first arm
	//s2 = angle of second arm
	//pow(3,9) = 19683 

	//convert to meters (15cm -> 0.15m)
	x = x / 100;
	y = y / 100;

	//a1 & a2 = 0.15
	_link1 = 2 * atan((3 * y + sqrt(-100 * pow(x,4) - 200 * pow(x,2) * pow(y,2) + 9 * pow(x,2) - 100 * pow(y,4) + 9 * pow(y,2))) / (10 * pow(x,2) + 3 * x + 10 * pow(y,2)));
	_link2 = -2 * atan(sqrt(-100 * pow(x,2) - 100 * pow(y,2) + 9) / (10 * sqrt(pow(x,2) + pow(y,2))));

	int dummy_angle = (int)((-1.6799) * 180) / PI;
	s1 = (int)((_link1 * 180) / PI);
	s2 = (int)((_link2 * 180) / PI);
}