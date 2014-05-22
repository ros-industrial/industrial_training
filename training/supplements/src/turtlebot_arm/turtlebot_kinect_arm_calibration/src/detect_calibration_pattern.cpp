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

#include <turtlebot_kinect_arm_calibration/detect_calibration_pattern.h>

void PatternDetector::setCameraMatrices(cv::Mat K_, cv::Mat D_)
{
  K = K_;
  D = D_; 
}

void PatternDetector::setPattern(cv::Size grid_size_, float square_size_, 
      Pattern pattern_type_, cv::Point3f offset_)
{
  ideal_points = calcChessboardCorners(grid_size_, square_size_, pattern_type_, offset_);
  pattern_type = pattern_type_;
  grid_size = grid_size_;
  square_size = square_size_;
}

object_pts_t PatternDetector::calcChessboardCorners(cv::Size boardSize,
                                          float squareSize,
                                          Pattern patternType,
                                          cv::Point3f offset)
{
  object_pts_t corners;
  switch (patternType)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(j * squareSize),
                                        float(i * squareSize), 0) + offset);
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(
                            cv::Point3f(float(i * squareSize),
                                        float((2 * j + i % 2) * squareSize), 0) + offset);
      break;
    default:
      std::logic_error("Unknown pattern type.");
  }
  return corners;
}



int PatternDetector::detectPattern(cv::Mat& image_in, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation, cv::Mat& image_out)
{
  translation.setZero();
  orientation.setIdentity();
  
  bool found = false;
  
  observation_pts_t observation_points;
  
  switch (pattern_type)
  {
    case ASYMMETRIC_CIRCLES_GRID:
      found = cv::findCirclesGrid(image_in, grid_size, observation_points,
                                cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      break;
    case CHESSBOARD:
      found = cv::findChessboardCorners(image_in, grid_size, observation_points, cv::CALIB_CB_ADAPTIVE_THRESH);
      break;
    case CIRCLES_GRID:
      found = cv::findCirclesGrid(image_in, grid_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID);
      break;
  }

  if(found)
  {
    // Do subpixel ONLY IF THE PATTERN IS A CHESSBOARD
    if (pattern_type == CHESSBOARD)
    {
      cv::cornerSubPix(image_in, observation_points, cv::Size(5,5), cv::Size(-1,-1), 
      cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 0.01));
    }
  
    cv::solvePnP(cv::Mat(ideal_points), cv::Mat(observation_points), K, D,
                 rvec, tvec, false);
    cv::Rodrigues(rvec, R); //take the 3x1 rotation representation to a 3x3 rotation matrix.
    
    cv::drawChessboardCorners(image_out, grid_size, cv::Mat(observation_points), found);
    
    convertCVtoEigen(tvec, R, translation, orientation);
  }
  
  return found;
}

void convertCVtoEigen(cv::Mat& tvec, cv::Mat& R, Eigen::Vector3f& translation, Eigen::Quaternionf& orientation)
{
  // This assumes that cv::Mats are stored as doubles. Is there a way to check this?
  // Since it's templated...
  translation = Eigen::Vector3f(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0, 2));
  
  Eigen::Matrix3f Rmat;
  Rmat << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
                                          
  orientation = Eigen::Quaternionf(Rmat);

}

