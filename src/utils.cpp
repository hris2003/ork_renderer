/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2013, Vincent Rabaud
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <object_recognition_renderer/renderer.h>
#include <object_recognition_renderer/utils.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RendererIterator::RendererIterator(Renderer *renderer, size_t n_points) :
		n_points_(n_points), index_(0), renderer_(renderer), angle_min_(-90), angle_max_(
				90), angle_step_(30), angle_(angle_min_), radius_min_(0.4), radius_max_(
				0.8), radius_step_(0.2), radius_(radius_min_) {
}

RendererIterator &
RendererIterator::operator++() {
	angle_ += angle_step_;
	if (angle_ > angle_max_) {
		angle_ = angle_min_;
		radius_ += radius_step_;
		if (radius_ > radius_max_) {
			radius_ = radius_min_;
			++index_;
		}
	}

	return *this;
}

void RendererIterator::render(cv::Mat &image_out, cv::Mat &depth_out,
		cv::Mat &mask_out) {
	if (isDone())
		return;

	cv::Vec3d t, up;
	view_params(t, up);

	renderer_->lookAt(t(0), t(1), t(2), up(0), up(1), up(2));
	renderer_->render(image_out, depth_out, mask_out);
}

/**
 * @return the rotation of the mesh with respect to the current view point
 */
cv::Matx33d RendererIterator::R() const {
	cv::Vec3d t, up;
	view_params(t, up);

	cv::Vec3d y = t.cross(up);
	cv::Mat R_full = (cv::Mat_<double>(3, 3) << t(0), t(1), t(2), y(0), y(1), y(
			2), up(0), up(1), up(2));
	cv::Matx33d R = R_full;

	return R;
}

/**
 * @return the translation of the mesh with respect to the current view point
 */
cv::Vec3d RendererIterator::T() const {
	cv::Vec3d t, _up;
	view_params(t, _up);

	return -t;
}

/**
 * @return the total number of templates that will be computed
 */
size_t RendererIterator::n_templates() const {
	return ((angle_max_ - angle_min_) / angle_step_ + 1) * n_points_
			* ((radius_max_ - radius_min_) / radius_step_ + 1);
}

/**
 * @param T the translation vector
 * @param up the up vector of the view point
 */
void RendererIterator::view_params(cv::Vec3d &T, cv::Vec3d &up) const {
	// Calculate Point(x, y ,z) on the sphere based on index_ and radius_ using Golden Spiral technique
	static float inc = CV_PI * (3 - sqrt(5));
	static float off = 2.0 / float(n_points_);
	float z = index_ * off - 1 + (off / 2);
	float r = sqrt(1 - z * z);
	float phi = index_ * inc;
	float y = cos(phi) * r;
	float x = sin(phi) * r;

	float lat = acos(z), lon;
	if ((fabs(sin(lat)) < 1e-5) || (fabs(y / sin(lat)) > 1))
		lon = 0;
	else
		lon = asin(y / sin(lat));
	x *= radius_; // * cos(lon) * sin(lat);
	y *= radius_; //float y = radius * sin(lon) * sin(lat);
	z *= radius_; //float z = radius * cos(lat);

	// Figure out the up vector
	//try getting up_vec from forward vector and the global up_vector(0,0,1) (i.e. coincide with the Z axis)
	cv::Vec3f right = cv::Vec3f(x, y, z).cross(cv::Vec3f(0, 0, 1));
	cv::Vec3f T_up = right.cross(cv::Vec3f(x, y, z));
	normalize_vector(T_up(0), T_up(1), T_up(2));
	normalize_vector(right(0), right(1), right(2));

	// Rotate the up vector in that basis
	float angle_rad = angle_ * CV_PI / 180.;
	up = T_up * cos(angle_rad) + right * sin(angle_rad);
	T = cv::Vec3d(x, y, z);
}
