//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// Copyright (c) 2013, Aldebaran Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

// ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
// ----------------------------------------------------------------------------

#define GL_GLEXT_PROTOTYPES

#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if USE_RENDERER_GLUT
#include <object_recognition_renderer/renderer_glut.h>
#else
#include <object_recognition_renderer/renderer_osmesa.h>
#endif
#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer2d.h>

using namespace std;

void render3d(std::string file_name, size_t width, size_t height) {
#if USE_RENDERER_GLUT
	RendererGlut renderer = RendererGlut(file_name);
#else
	RendererOSMesa renderer = RendererOSMesa(file_name);
#endif

	double near = 0.1, far = 1000;
	double focal_length_x = 525, focal_length_y = 525;

	renderer.set_parameters(width, height, focal_length_x, focal_length_y, near,
			far);

	RendererIterator renderer_iterator = RendererIterator(&renderer, 150);

	cv::Mat image, depth, mask;
	for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator) {
		try {
			renderer_iterator.render(image, depth, mask);
			cv::imwrite(boost::str(boost::format("depth_%05d.png") % (i)),
					depth);
			cv::imwrite(boost::str(boost::format("image_%05d.png") % (i)),
					image);
			cv::imwrite(boost::str(boost::format("mask_%05d.png") % (i)), mask);
		} catch (...) {

		}
	}
}
void render3d_2Ros(std::string file_name, size_t width, size_t height,
		double r_in, int angle_in, int step_in, bool isLoop) {

	ros::NodeHandle n;
	ros::Rate r(2);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>(
			"visualization_marker_array", 1);
	ros::Publisher sphere_pub = n.advertise<visualization_msgs::Marker>(
			"sphere_marker", 10);
	ros::Publisher cam_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
			"cam_pose_marker", 1);
	ros::Publisher obj_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
			"obj_pose_marker", 1);
	ros::Publisher origin_pose_pub = n.advertise<geometry_msgs::PoseStamped>(
			"origin_pose_marker", 1);

	uint32_t shape = visualization_msgs::Marker::ARROW;

#if USE_RENDERER_GLUT
	RendererGlut renderer = RendererGlut(file_name);
#else
	RendererOSMesa renderer = RendererOSMesa(file_name);
#endif

	double near = 0.1, far = 1000;
	double focal_length_x = 525, focal_length_y = 525;

	renderer.set_parameters(width, height, focal_length_x, focal_length_y, near,
			far);

	RendererIterator renderer_iterator = RendererIterator(&renderer, 50);

	cv::Mat image, depth, mask;
	cv::Matx33f R, R_obj, R_cam;
	cv::Vec3f T;

	//Marker for the object in the center of the world
	visualization_msgs::Marker obj_marker, check_marker;
	int id = 0;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.

	obj_marker.header.frame_id = "/camera_rgb_optical_frame";
	obj_marker.header.stamp = ros::Time::now();

	obj_marker.ns = "basic_shapes";
	obj_marker.id = id;
	id++;
	obj_marker.type = shape;
	obj_marker.action = visualization_msgs::Marker::ADD;
	cv::Vec3f t_obj_up(1,0,0);
	Eigen::Matrix3f origin_rotation;
	origin_rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Quaternionf origin_quaternion(origin_rotation);
	obj_marker.pose.orientation.w = origin_quaternion.w();
	obj_marker.pose.orientation.x = origin_quaternion.x();
	obj_marker.pose.orientation.y = origin_quaternion.y();
	obj_marker.pose.orientation.z = origin_quaternion.z();

	obj_marker.pose.orientation.x = 0;
	obj_marker.pose.orientation.y = 0;
	obj_marker.pose.orientation.z = 1;
	obj_marker.pose.orientation.w = 1.0;

	obj_marker.scale.x = 0.02;
	obj_marker.scale.y = 0.02;
	obj_marker.scale.z = 0.02;

	obj_marker.color.r = 1.0f;
	obj_marker.color.g = 0.0f;
	obj_marker.color.b = 0.0f;
	obj_marker.color.a = 1.0;

	obj_marker.lifetime = ros::Duration();

	check_marker = obj_marker;
	check_marker.id = id;
	id++;
	check_marker.color.r = 0.0f;
	check_marker.color.g = 1.0f;
	check_marker.color.b = 0.0f;
	check_marker.color.a = 1.0f;

	renderer_iterator.angle_ = angle_in;
	renderer_iterator.radius_ = r_in;
	vector<cv::Vec3f> Ts;
	for (int my_index = 0; my_index < 50; my_index++) {
		renderer_iterator.index_ = my_index;
		cv::Vec3f t = renderer_iterator.T();
		Ts.push_back(t);
	}

	string mystr;
	bool angle_looping = false;
	int min_angle = -80, max_angle = 80, step_angle = 5;
	//Matx to switch between X and Z
	//	cv::Mat R_xz =
	//	(cv::Mat_<float>(3, 3) << 0, 0, -1, 1, 0, 0, 0, 1, 0);
	while (ros::ok()) {
		if (angle_looping && (angle_in < max_angle)) {
			angle_in += step_angle;
		} else if (!angle_looping
				|| (angle_looping && (angle_in >= max_angle))) {
			cout << "Do you want to loop with angle? ";

			getline(cin, mystr);
			if (mystr[0] == 'n') {
				angle_looping = false;
			} else {
				angle_looping = true;
				angle_in = min_angle;
			}
		}
		if (!angle_looping) {
			cout << "Enter angle (default is " << angle_in << "): ";
			getline(cin, mystr);
			if (mystr.length() > 0)
				stringstream(mystr) >> angle_in;
			cout << "Enter index_ (default is " << step_in << "): ";
			getline(cin, mystr);
			if (mystr.length() > 0)
				stringstream(mystr) >> step_in;
		}
		renderer_iterator.angle_ = angle_in;
		renderer_iterator.radius_ = r_in;
		renderer_iterator.index_ = step_in;

		renderer_iterator.render(image, depth, mask);
		cv::Mat R_in = cv::Mat(renderer_iterator.R());
		cv::Mat T_in = cv::Mat(renderer_iterator.T());
		T_in.convertTo(T, CV_32F);
		R_in.convertTo(R, CV_32F);
//std::cout << "R:\n" << R << "\nT:\n\n" << T << std::endl;

		R_cam = cv::Mat(R.t()).clone();
		R_obj = cv::Mat(R).clone();


		t_obj_up = cv::Vec3f(R(0, 0), R(0, 1), R(0, 2));
		float x_angle = acos(cv::Vec3f(R(0, 0), R(0, 1), R(0, 2)).dot( cv::Vec3f(1,0,0)));
		float y_angle = acos(cv::Vec3f(R(1, 0), R(1, 1), R(1, 2)).dot( cv::Vec3f(0,1,0)));
		float z_angle = acos(cv::Vec3f(R(2, 0), R(2, 1), R(2, 2)).dot( cv::Vec3f(0,0,1)));
		cout << "obj_up :\t"<<t_obj_up<<"\n"<<x_angle * 180 / 3.14<<"\t"<<y_angle * 180 / 3.14<<"\t"<<z_angle * 180 / 3.14<<"\n";

		cv::Vec3f T_up = cv::Vec3f(R(2, 0), R(2, 1), R(2, 2));
		cout<<"t_up: :\t"<<T_up<<"\n";
//R_cam = R_cam.t();

		visualization_msgs::MarkerArray marker_array;
		marker_array.markers.resize(5);
		visualization_msgs::Marker marker;
// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/camera_rgb_optical_frame";
		marker.header.stamp = ros::Time::now();
//
		obj_marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";

		marker.id = id;
		marker.type = shape;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = T(0);
		marker.pose.position.y = T(1);
		marker.pose.position.z = T(2);

		check_marker.pose.position.x = 0;
		check_marker.pose.position.y = 0;
		check_marker.pose.position.z = 0;

		obj_marker.pose.position.x = 0;
		obj_marker.pose.position.y = 0;
		obj_marker.pose.position.z = 0;

		Eigen::Matrix3f obj_rotation;
		obj_rotation << R_obj(0, 0), R_obj(0, 1), R_obj(0, 2), R_obj(1, 0), R_obj(
				1, 1), R_obj(1, 2), R_obj(2, 0), R_obj(2, 1), R_obj(2, 2);
		Eigen::Quaternionf obj_quaternion(obj_rotation);
		obj_marker.pose.orientation.w = obj_quaternion.w();
		obj_marker.pose.orientation.x = obj_quaternion.x();
		obj_marker.pose.orientation.y = obj_quaternion.y();
		obj_marker.pose.orientation.z = obj_quaternion.z();
		Eigen::Matrix3f cam_rotation;
		cam_rotation << R_cam(0, 0), R_cam(0, 1), R_cam(0, 2), R_cam(1, 0), R_cam(
				1, 1), R_cam(1, 2), R_cam(2, 0), R_cam(2, 1), R_cam(2, 2);
		Eigen::Quaternionf cam_quaternion(cam_rotation);
		marker.pose.orientation.w = cam_quaternion.w();
		marker.pose.orientation.x = cam_quaternion.x();
		marker.pose.orientation.y = cam_quaternion.y();
		marker.pose.orientation.z = cam_quaternion.z();

		R = R_cam * R_obj;
		Eigen::Matrix3f check_rotation;
		check_rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(
				2, 0), R(2, 1), R(2, 2);
		Eigen::Quaternionf check_quaternion(check_rotation);
		check_marker.pose.orientation.w = 1; //check_quaternion.w();
		check_marker.pose.orientation.x = 0; //check_quaternion.x();
		check_marker.pose.orientation.y = 1; //check_quaternion.y();
		check_marker.pose.orientation.z = 0; //check_quaternion.z();

		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;

		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

//Pose message
		geometry_msgs::PoseArray pose_array;
		geometry_msgs::PoseStamped cam_pose, obj_pose, origin_pose;
		cam_pose.pose.position = marker.pose.position;
		cam_pose.pose.orientation = marker.pose.orientation;
		obj_pose.pose.position = obj_marker.pose.position;
		obj_pose.pose.orientation = obj_marker.pose.orientation;
		origin_pose.pose.position = check_marker.pose.position;
		origin_pose.pose.orientation = check_marker.pose.orientation;
		cam_pose.header.frame_id = obj_pose.header.frame_id =
				origin_pose.header.frame_id = "/camera_rgb_optical_frame";
		cam_pose.header.stamp = ros::Time::now();
		obj_pose.header.stamp = origin_pose.header.stamp = ros::Time::now();
		cam_pose_pub.publish(cam_pose);
		obj_pose_pub.publish(obj_pose);
		origin_pose_pub.publish(origin_pose);

		if (!image.empty()) {
			cv::imshow("Rendering", image);
			//std::cout << "Rin:\n" << R_in << "\nT_in:\n" << T_in << std::endl;
			//std::cout << "R:\n" << R << "\nT:\n" << T << std::endl;
			cv::waitKey(50);
		}

//the up vector
		visualization_msgs::Marker up_cam_marker, up_obj_marker;
		up_cam_marker.header.frame_id = "/camera_rgb_optical_frame";
		up_cam_marker.header.stamp = ros::Time::now();
		up_cam_marker.ns = "basic_shapes";
		up_cam_marker.action = visualization_msgs::Marker::MODIFY;
		up_cam_marker.id = 8;
		up_cam_marker.type = visualization_msgs::Marker::ARROW;
		up_cam_marker.scale.x = 0.01;
		up_cam_marker.scale.y = 0.02;
		up_cam_marker.scale.z = 0.01;
		up_cam_marker.color.r = 1.0f;
		up_cam_marker.color.g = 1.0f;
		up_cam_marker.color.b = 1.0f;
		up_cam_marker.color.a = 1.0;
		up_obj_marker = up_cam_marker;
		up_obj_marker.id = 9;

		geometry_msgs::Point up_cam_start, up_cam_end;
		up_cam_start.x = T(0);
		up_cam_start.y = T(1);
		up_cam_start.z = T(2);
		up_cam_end.x = up_cam_start.x + T_up(0);
		up_cam_end.y = up_cam_start.y + T_up(1);
		up_cam_end.z = up_cam_start.z + T_up(2);
		up_cam_marker.points.push_back(up_cam_start);
		up_cam_marker.points.push_back(up_cam_end);

		geometry_msgs::Point up_obj_start, up_obj_end;
		up_obj_start.x = 0;
		up_obj_start.y = 0;
		up_obj_start.z = 0;
		up_obj_end.x = up_obj_start.x + t_obj_up(0);
		up_obj_end.y = up_obj_start.y + t_obj_up(1);
		up_obj_end.z = up_obj_start.z + t_obj_up(2);
		up_obj_marker.points.push_back(up_obj_start);
		up_obj_marker.points.push_back(up_obj_end);

		marker.lifetime = ros::Duration();
		marker_array.markers[0] = obj_marker;
		marker_array.markers[1] = check_marker;
		marker_array.markers[2] = marker;
		marker_array.markers[3] = up_cam_marker;
		marker_array.markers[4] = up_obj_marker;
		marker_pub.publish(marker_array);

		visualization_msgs::Marker points;
		points.header.frame_id = "/camera_rgb_optical_frame";
		points.header.stamp = ros::Time::now();
		points.ns = "basic_shapes";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;
		points.id = 10;

		points.type = visualization_msgs::Marker::LINE_STRIP;
// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.01;
		points.scale.y = 0.01;
		points.scale.z = 0.01;
// Points are pink
		points.color.r = 1.0f;
		points.color.b = 1.0f;
		points.color.a = 0.4;

// Create the vertices for the points and lines
		for (uint32_t i = 0; i < Ts.size(); ++i) {

			geometry_msgs::Point p;
			p.x = Ts.at(i)(0);
			p.y = Ts.at(i)(1);
			p.z = Ts.at(i)(2);

			points.points.push_back(p);
		}

		sphere_pub.publish(points);

		r.sleep();
		if (angle_looping && (angle_in < max_angle)) {
			continue;
		}
		cout << "Press 'q' to quit: ";
		getline(cin, mystr);
		if (mystr[0] == 'q')
			break;

//++renderer_iterator;

	}			//end of while
}

void render3d_withFixeParams(std::string file_name, size_t width, size_t height,
		double r_in, int angle_in, int step_in, bool isLoop) {
#if USE_RENDERER_GLUT
	RendererGlut renderer = RendererGlut(file_name);
#else
	RendererOSMesa renderer = RendererOSMesa(file_name);
#endif

	double near = 0.1, far = 1000;
	double focal_length_x = 525, focal_length_y = 525;

	renderer.set_parameters(width, height, focal_length_x, focal_length_y, near,
			far);

	int max_index = 1500;
	RendererIterator renderer_iterator = RendererIterator(&renderer, 50);

	cv::Mat image, depth, mask;
	cv::Matx33d R, R_in, r;
	cv::Vec3d T, T_in;
//Matx to switch between Y and Z
	cv::Mat R_yz = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);
//Setup the view_params for RendererIterator, including angle_, radius_, index_

	renderer_iterator.angle_ = angle_in;
	renderer_iterator.radius_ = r_in;
	renderer_iterator.index_ = step_in;
	/*
	 renderer_iterator.render(image, depth, mask);
	 R_in = renderer_iterator.R();
	 T_in = renderer_iterator.T();
	 std::cout << "Rendering params:\nRadius:\t" << renderer_iterator.radius_
	 << "\nAngle:\t" << renderer_iterator.angle_ << "\nIndex:\t"
	 << renderer_iterator.index_ << std::endl;
	 */
// Display the rendered image
//cv::namedWindow("Rendering");
//R_in = R_in.t() * cv::Matx33d(R_yz);
	if (isLoop) {
//for (int my_angle = -90; my_angle < 90; my_angle += 10)
		for (int my_index = 0; my_index < 50; my_index++) {
			//
			//for (double my_r = 0.4; my_r < 1.0; my_r+=0.05) {
			try {
				renderer_iterator.index_ = my_index;
				//renderer_iterator.angle_ = my_angle;
				//renderer_iterator.radius_ = my_r;
				renderer_iterator.render(image, depth, mask);
				R = renderer_iterator.R();
				T = renderer_iterator.T();
				//R = R.t() * cv::Matx33d(R_yz);
				//compare the R with R_in and show it if possible
				// Display the rendered image
				//cv::namedWindow("Rendering");
				//std::cout<<"i: "<<i<<std::endl;

				if (!image.empty()) {
					cv::imshow("Rendering", image);
					//std::cout << "Rin:\n" << R_in << "\nT_in:\n" << T_in << std::endl;
					std::cout << "R:\n" << R << "\nT:\n" << T << std::endl;
					cv::waitKey(500);
				}

			} catch (...) {
				std::cout << "Something wrong in the input params ;-)\n";
			}
		}
		std::cout << "Press any key to stop" << std::endl;
		cv::waitKey(0);
	} else {
		try {
			//renderer_iterator.angle_ = my_angle;
			//renderer_iterator.radius_ = my_r;
			renderer_iterator.render(image, depth, mask);

			R = renderer_iterator.R();
			T = renderer_iterator.T();
			std::cout << "Rendering params:\nRadius:\t"
					<< renderer_iterator.radius_ << "\nAngle:\t"
					<< renderer_iterator.angle_ << "\nIndex:\t"
					<< renderer_iterator.index_ << std::endl;

			// Display the rendered image
			cv::namedWindow("Rendering");
			//R = R.t() * cv::Matx33d(R_yz);
			if (!image.empty()) {
				cv::imshow("Rendering", image);
				std::cout << "R:\n" << R << "\nT:\n" << T << std::endl;
				cv::waitKey(0);
			}

		} catch (...) {
			std::cout << "Something wrong in the input params ;-)\n";
		}
	}

}

void render2d(std::string file_name, size_t width, size_t height) {
	Renderer2d render(file_name, 0.2);
	double focal_length_x = 525, focal_length_y = 525;
	render.set_parameters(width, height, focal_length_x, focal_length_y);
	float y = 0., z = 1;
	cv::Vec2f up(z, -y);
	up = up / norm(up);
	render.lookAt(0.5, y, z, 0, up(0), up(1));
	cv::Mat img, depth, mask;
	render.render(img, depth, mask);
	cv::imshow("img", img);
	cv::imshow("depth", depth);
	cv::imshow("mask", mask);
	cv::waitKey(0);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "basic_shapes");
// Define the display
	size_t width = 640, height = 480;

// the model name can be specified on the command line.
//std::string file_name(argv[1]), file_ext = file_name.substr(file_name.size() - 3, file_name.npos);
//filename of the coke mesh: /home/hdang/ork/src/ork_tutorials/data/coke.stl
	std::string file_name("/home/hdang/ork/src/ork_tutorials/data/coke.stl"),
			file_ext = "stl";
	double r = atof(argv[1]);  //input radius
	int angle = atoi(argv[2]);  //input angle
	int index = atoi(argv[3]);  //input index
	int isLoop = atoi(argv[4]);  //input index
	if (file_ext == "png")
		render2d(file_name, width, height);
	else if (isLoop == 1)
		render3d_withFixeParams(file_name, width, height, r, angle, index,
				isLoop);
	if (isLoop == 2) {

		render3d_2Ros(file_name, width, height, r, angle, index, isLoop);
	}

	return 0;
}
