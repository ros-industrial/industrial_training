/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DETECT_CALIBRATION_PATTERN_
#define _DETECT_CALIBRATION_PATTERN_

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>

#include <iostream>
#include <stdexcept>

using namespace std;

enum Pattern
{
  CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
};

typedef std::vector<cv::Point3f> object_pts_t;
typedef std::vector<cv::Point2f> observation_pts_t;

void convertCVtoEigen(cv::Mat& tvec, cv::Mat& R, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation);

class PatternDetector
{
  public:
    PatternDetector() { }
  
    static object_pts_t calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType = CHESSBOARD,
                                          cv::Point3f offset = cv::Point3f());
                                          
    int detectPattern(cv::Mat& image_in, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation, cv::Mat& image_out);
    
    void setCameraMatrices(cv::Mat K_, cv::Mat D_);
    
    void setPattern(cv::Size grid_size_, float square_size_, 
          Pattern pattern_type_, cv::Point3f offset_ = cv::Point3f());

  public:
    cv::Mat K;
    cv::Mat D;
    cv::Mat rvec, tvec, R;
    
    Pattern pattern_type;
    cv::Size grid_size;
    float square_size;
    object_pts_t ideal_points;
};

#endif
