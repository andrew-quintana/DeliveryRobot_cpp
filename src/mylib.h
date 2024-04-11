#ifndef MYLIB_H
#define MYLIB_H

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include "opencv2/opencv.hpp"

#ifdef __cplusplus
extern "C" {
#endif




/*
#include <eigen3/Eigen/Core>

#include <boost/algorithm/string.hpp>

#include <gtest/gtest.h>
*/

void testApriltag();

void testOpencv( const std::string& DIR);

bool testEigen();

bool testBoost();

// calibration setup
cv::Mat camera_matrix;
cv::Mat distance_coefficients;
cv::Mat rotation_vectors;
cv::Mat translation_vectors;
cv::Size frame_size;

#ifdef __cplusplus
}
#endif

#endif // MYLIB_H
