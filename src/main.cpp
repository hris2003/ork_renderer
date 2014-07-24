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
#include <stdlib.h>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#if USE_RENDERER_GLUT
#include <object_recognition_renderer/renderer_glut.h>
#else
#include <object_recognition_renderer/renderer_osmesa.h>
#endif
#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer2d.h>

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
	RendererIterator renderer_iterator = RendererIterator(&renderer, 100);

	cv::Mat image, depth, mask;
	cv::Matx33d R;
	cv::Vec3d T;
	//Setup the view_params for RendererIterator, including angle_, radius_, index_

	renderer_iterator.angle_ = angle_in;
	renderer_iterator.radius_ = r_in;
	renderer_iterator.index_ = step_in;
	if (isLoop) {
		for (int my_angle = -90; my_angle < 90; my_angle += 30)
			for (int my_index = 0; my_index < 100; my_index++) {
				//
				//for (double my_r = 0.4; my_r < 1.0; my_r+=0.05) {
				try {
					renderer_iterator.index_ = my_index;
					renderer_iterator.angle_ = my_angle;
					//renderer_iterator.radius_ = my_r;
					renderer_iterator.render(image, depth, mask);

					// Display the rendered image
					cv::namedWindow("Rendering");
					if (!image.empty()) {
						cv::imshow("Rendering", image);
						//std::cout << "R:\n" << R << "\nT:\n" << T << std::endl;
						if (cv::waitKey(50)==27) break;
					}

				} catch (...) {
					std::cout << "Something wrong in the input params ;-)\n";
				}
			}
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
	bool isLoop = atoi(argv[4]) > 0;  //input index
	if (file_ext == "png")
		render2d(file_name, width, height);
	else
		render3d_withFixeParams(file_name, width, height, r, angle, index,
				isLoop);

	return 0;
}
