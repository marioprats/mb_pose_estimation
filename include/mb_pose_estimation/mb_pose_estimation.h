/*
 * Copyright (c) 2013 Willow Garage, Inc.
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
 *
 * Author: Mario Prats
 */

#ifndef MBPOSEESTIMATION_H
#define MBPOSEESTIMATION_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <lima/VVSPoseEstimator.h>
#include <lima/Object.h>
#include <lima/Camera.h>
#include <lima/CameraObjectPose.h>
#include <lima/tfHomogeneousMatrix.h>

#include <visp/vpDisplayX.h>

#include <boost/shared_ptr.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

class CameraParams : public Camera
{
public:
  CameraParams(vpCameraParameters params) : Camera(params) {}

  void acquire(vpImage<unsigned char> &I) {}
  void acquire(vpImage<vpRGBa> &I) {}

  void close() {};
};

/** Implements pose estimation and tracking based on object models
 */
class MBPoseEstimation
{
public:
  static const double CAMERA_MOTION_TOLERANCE = 0.01;
  static const double CAMERA_FAST_MOTION_TOLERANCE = 0.2;
  static const unsigned int DEFAULT_SEARCH_INTERVAL = 9;
  static const unsigned int DEFAULT_SAMPLING_DISTANCE = 8;
  static const double DEFAULT_RECOVER_TIMEOUT = 0.5;

  MBPoseEstimation(ros::NodeHandle &nh,
           sensor_msgs::ImageConstPtr &image,
           sensor_msgs::CameraInfoConstPtr &info,
           const std::string &world_frame);

  MBPoseEstimation(ros::NodeHandle &nh,
           vpImage<vpRGBa> &image,
           const vpCameraParameters &info,
           const std::string &camera_frame,
           const std::string &world_frame);

  /** Sets the distance (in pixels) between two consecutive sampled points */
  void setSamplingDistance(unsigned int sampling_distance)
  {
    vvs_->setDistance(sampling_distance);
  }

  /** Sets an interval (in pixels) where to perform a linear search around  the sampled point */
  void setSearchInterval(unsigned int search_interval)
  {
    vvs_->setIval(search_interval);
  }

  /** Sets the degrees of freedom on which to track the object (in the object frame) */
  void setTrackDOF(bool x, bool y, bool z, bool rx, bool ry, bool rz)
  {
    vpMatrix S(6,6);
    S[0][0] = x;
    S[1][1] = y;
    S[2][2] = z;
    S[3][3] = rx;
    S[4][4] = ry;
    S[5][5] = rz;

    vvs_->setS(S);
  }

  /** Set to true if tracking a fixed object */
  void setMotionlessObject(bool flag)
  {
    motionless_object_ = flag;
  }

  /** Sets the name of the object to track and its estimated pose wrt the camera frame */
  void setObjectID(const std::string &object_id, const tf::Transform &cMo);
  void setObjectID(const std::vector<std::string> &object_ids, const std::vector<tf::Transform> &cMo);

  /** Sets the world frame.
   * It is used for camera motion estimation. At each iteration, the pose of the camera frame wrt the world
   * is obtained and used for feedforwarding the camera motion into the minimization algorithm, or temporarily
   * stopping tracking under fast camera motion. Normally set to the odom frame of the vehicle
   */
  void setWorldFrame(const std::string &world_frame);

  /* If true, a window of the tracker is displayed */
  void setDebugMode(bool flag);

  /** Runs one iteration of the minimization algorithm
   * @returns the object pose after the iteration
   */
  bool track(sensor_msgs::ImageConstPtr &image, tf::Transform &object_pose);
  bool track(const vpImage<vpRGBa> &image, tf::Transform &object_pose);

  /** Runs multiple iterations of the minimization algorithm, until the timeout is reached or the error is below tolerance
   * @returns the object pose after all the iterations
   */
  bool minimizePose(sensor_msgs::ImageConstPtr &image, tf::Transform &object_pose, ros::Duration timeout = ros::Duration(1.0), double tolerance = 0.01);
  bool minimizePose(const vpImage<vpRGBa> &image, tf::Transform &object_pose, ros::Duration timeout = ros::Duration(1.0), double tolerance = 0.01);

  bool projectModel(const vpImage<vpRGBa> &image);

  /** Runs multiple iterations of the minimization algorithm, until the timeout is reached or the error is below tolerance
   * Uses feature descriptors already stored
   * @returns the object pose after all the iterations
   */
  bool minimizePoseFromDescriptors(sensor_msgs::ImageConstPtr &image, tf::Transform &object_pose, ros::Duration timeout = ros::Duration(1.0), double tolerance = 0.01);
  bool minimizePoseFromDescriptors(const vpImage<vpRGBa> &image, tf::Transform &object_pose, ros::Duration timeout = ros::Duration(1.0), double tolerance = 0.01);

  /** Returns the euclidean norm of the VVS velocity vector
   * Can be used to estimate if the algorithm has converged (error close to zero).
   */
  double getError()
  {
    return vvs_->getError();
  }

  ~MBPoseEstimation() {}

private:
  void init(ros::NodeHandle &nh);
  Object *createModelFromCAO(const std::string &file);
  Object *createModelFromCAO(const std::vector<std::string> &files, const std::vector<tf::Transform> &cMo);
  void tfTransformToVisp(const tf::Transform &tf, vpHomogeneousMatrix &vptf);
  void vpHomogeneousMatrixToTFTransform(const vpHomogeneousMatrix &vptf, tf::Transform &tf);

  boost::shared_ptr<tf::TransformListener> listener_;
  tf::TransformBroadcaster br_;
  ros::Publisher marker_pub_;

  boost::shared_ptr<Object> object_;
  boost::shared_ptr<Camera> video_;
  boost::shared_ptr<CameraObjectPose<vpRGBa> > cop_;
  boost::shared_ptr<vpDisplayX> window_;
  boost::shared_ptr<tfHomogeneousMatrix> wMc_;
  boost::shared_ptr<VVSPoseEstimator<vpRGBa> > vvs_;
  boost::shared_ptr<visualization_msgs::MarkerArray> marker_;
  boost::shared_ptr<CameraParams> camera_;

  ros::NodeHandle nh_;
  vpHomogeneousMatrix wMc0_;
  ros::Time last_motion_stamp_;

  bool debug_mode_, motionless_object_;
  std::string camera_frame_;
  std::string world_frame_;
  std::string object_id_;
  std::vector<vpColVector> descriptors_;
};

#endif
