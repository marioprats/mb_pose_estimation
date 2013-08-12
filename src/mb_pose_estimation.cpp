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

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <mb_pose_estimation/mb_pose_estimation.h>
#include <mb_pose_estimation/utils.h>
#include <lima/CAOEdgesModel.h>
#include <lima/FJoint.h>

#include <visp/vpThetaUVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpPose.h>

#include <iostream>
#include <sys/signal.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/tokenizer.hpp>

MBPoseEstimation::MBPoseEstimation(ros::NodeHandle &nh,
                   sensor_msgs::ImageConstPtr &image,
                   sensor_msgs::CameraInfoConstPtr &info,
                   const std::string &world_frame) :
                   nh_(nh),
                   debug_mode_(false),
                   world_frame_(world_frame)

{
  //Instantiate camera-object pose
  int binx = (info->binning_x == 0) ? 1 : info->binning_x;
  int biny = (info->binning_y ==0) ? 1 : info->binning_y;
  camera_.reset(new CameraParams(vpCameraParameters(info->P[0]/binx, info->P[5]/biny, info->P[2]/binx, info->P[6]/biny)));

  cop_.reset(new CameraObjectPose<vpRGBa>(camera_.get(), NULL));
  cop_->view = toVispImageRGBa(*image);

  camera_frame_ = image->header.frame_id;
  init(nh);
}

MBPoseEstimation::MBPoseEstimation(ros::NodeHandle &nh,
                   vpImage<vpRGBa> &image,
                   const vpCameraParameters &info,
                   const std::string &camera_frame,
                   const std::string &world_frame) :
                   nh_(nh),
                   debug_mode_(false),
                   world_frame_(world_frame)
{
  //Instantiate camera-object pose
  camera_.reset(new CameraParams(info));
  cop_.reset(new CameraObjectPose<vpRGBa>(camera_.get(), NULL));
  cop_->view = image;
  camera_frame_ = camera_frame;
  init(nh);
}

void MBPoseEstimation::init(ros::NodeHandle &nh)
{
  listener_.reset(new tf::TransformListener());

  //Display window if debug mode
  if (debug_mode_ && ! window_)
  {
    window_.reset(new vpDisplayX());
    window_->init(cop_->getView(),0,0,"VVS Pose");
    vpDisplay::display(cop_->getView());
    vpDisplay::flush(cop_->getView());
  }

  setWorldFrame(world_frame_);

  //Create and setup VVS tracker
  vvs_.reset(new VVSPoseEstimator<vpRGBa>(cop_.get()));
  vvs_->setIval(DEFAULT_SEARCH_INTERVAL);
  vvs_->setDistance(DEFAULT_SAMPLING_DISTANCE);

  vpMatrix S(6,6);
  S.setIdentity();
  vvs_->setS(S);
}

void MBPoseEstimation::setWorldFrame(const std::string &world_frame)
{
  //get the camera pose wrt world if the frame names have been specified
  wMc_.reset(new tfHomogeneousMatrix(listener_));
  if ( ! world_frame.empty() )
  {
    ROS_INFO_STREAM("Waiting for tf between " << world_frame << " and " << camera_frame_);
    wMc_->buildFrom(world_frame, camera_frame_);
    ros::Rate r(20);
    while ( ! wMc_->update() && ros::ok() )
    {
      r.sleep();
    }
    ROS_INFO_STREAM("Got tf between " << world_frame << " and " << camera_frame_);
    world_frame_ = world_frame;
    wMc0_ = *wMc_;
  }
}

void MBPoseEstimation::setDebugMode(bool flag)
{
  debug_mode_ = flag;
  if (debug_mode_ && ! window_)
  {
    window_.reset(new vpDisplayX());
    window_->init(cop_->getView(),0,0,"VVS Pose");
    vpDisplay::display(cop_->getView());
    vpDisplay::flush(cop_->getView());
  }
  else if (! debug_mode_ && window_)
  {
    window_->close(cop_->getView());
    window_.reset();
  }
}

Object *MBPoseEstimation::createModelFromCAO(const std::string &file) {
  CAOEdgesModel *o1_gmodel=new CAOEdgesModel(file);
  ObjectModel *o1_model=new ObjectModel(o1_gmodel);
  Object *o1=new Object("Object", o1_model);

  return o1;
}

Object *MBPoseEstimation::createModelFromCAO(const std::vector<std::string> &files, const std::vector<tf::Transform> &cMo)
{
  if (files.size() > 0)
  {
    Object *objects[files.size()];
    objects[0] = createModelFromCAO(files[0]);
    vpHomogeneousMatrix vpcMo[files.size()];
    tfTransformToVisp(cMo[0], vpcMo[0]);

    for (std::size_t i = 1; i < files.size(); ++i)
    {
      objects[i] = createModelFromCAO(files[i]);

      FJoint *fjoint=new FJoint();

      tfTransformToVisp(cMo[i], vpcMo[i]);
      vpHomogeneousMatrix o0Mj = vpcMo[0].inverse() * vpcMo[i];
      vpHomogeneousMatrix oiMj(0,0,0,0,0,0);

      objects[0]->linkTo(o0Mj, fjoint, oiMj, objects[i]);
    }
    return objects[0];
  }
  return NULL;
}

void MBPoseEstimation::setObjectID(const std::vector<std::string> &object_ids, const std::vector<tf::Transform> &cMo)
{
  if (object_ids.size() > 0 && object_ids.size() == cMo.size())
  {
    //Load the object model.
    object_id_ = object_ids[0];
    std::vector<std::string> cao_files(object_ids.size());
    for (std::size_t i = 0; i < object_ids.size(); ++i)
    {
      cao_files[i] = ros::package::getPath("mb_pose_estimation") + "/data/" + object_ids[i] + ".cao";
      if (! boost::filesystem::exists(cao_files[i]))
      {
        ROS_ERROR_STREAM("Cannot locate model " << cao_files[i]);
        //TODO return false instead of exit
        exit(0);
      }
    }

    object_.reset(createModelFromCAO(cao_files, cMo));
    cop_->init(camera_.get(), object_.get());

    tfTransformToVisp(cMo[0], cop_->cMo);

    //Load descriptors if available
    std::ifstream filein(std::string(ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".descriptor").c_str());
    if (filein)
    {
      descriptors_.clear();
      for (std::string line; std::getline(filein, line); )
      {
        vpColVector descriptor;
        boost::char_separator<char> sep(" ");
        boost::tokenizer< boost::char_separator<char> > tokens(line, sep);

        for (boost::tokenizer<boost::char_separator<char> >::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
        {
          descriptor.resize(descriptor.size() + 1, false);
          descriptor[descriptor.size() - 1] = boost::lexical_cast<double>(*tok_iter);
        }
        descriptors_.push_back(descriptor);
      }
    }

    //Prepare the marker array that will be published
    marker_.reset(new visualization_msgs::MarkerArray());
    visualization_msgs::Marker moutline, mtext;
    moutline.header.frame_id = object_id_ + "_pose";
    moutline.id = 1;
    moutline.type = moutline.LINE_LIST;
    moutline.action = 0;
    moutline.color.r = 0.0;
    moutline.color.g = 1.0;
    moutline.color.b = 0.0;
    moutline.color.a = 1.0;
    moutline.scale.x = 0.02;
    moutline.lifetime = ros::Duration(20.0);
    EdgesModel *m = dynamic_cast<EdgesModel*>(object_->model->geometry);
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      geometry_msgs::Point p0, p1;
      p0.x = m->edges_list[i]->v1->v.get_oX();
      p0.y = m->edges_list[i]->v1->v.get_oY();
      p0.z = m->edges_list[i]->v1->v.get_oZ();
      p1.x = m->edges_list[i]->v2->v.get_oX();
      p1.y = m->edges_list[i]->v2->v.get_oY();
      p1.z = m->edges_list[i]->v2->v.get_oZ();
      moutline.points.push_back(p0);
      moutline.points.push_back(p1);
    }

    mtext.header.frame_id = object_id_ + "_pose";
    mtext.id = 2;
    mtext.type = mtext.TEXT_VIEW_FACING;
    mtext.action = 0;
    mtext.color.r = 1.0;
    mtext.color.g = 0.0;
    mtext.color.b = 0.0;
    mtext.color.a = 1.0;
    mtext.scale.x = 0.1;
    mtext.scale.y = 0.1;
    mtext.scale.z = 0.1;
    mtext.text = object_id_;
    mtext.lifetime = ros::Duration(20.0);

    marker_->markers.push_back(moutline);
    marker_->markers.push_back(mtext);

    //Create the publisher
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_" + object_id_, 1);
  }
  else
  {
    ROS_ERROR("MBPoseEstimation::setObjectID: object_ids and cMo vectors must be of the same dimension");
    //TODO: return false instead of exit
    exit(0);
  }
}

void MBPoseEstimation::setObjectID(const std::string &object_id, const tf::Transform &cMo)
{
  //Load the object model.
  object_id_ = object_id;
  std::string cao_file(ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".cao");
  if (! boost::filesystem::exists(cao_file))
  {
    ROS_ERROR_STREAM("Cannot locate model " << cao_file);
  }
  else
  {
    object_.reset(createModelFromCAO(cao_file));
    cop_->init(camera_.get(), object_.get());

    tfTransformToVisp(cMo, cop_->cMo);

    //Load descriptors if available
    std::ifstream filein(std::string(ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".descriptor").c_str());
    if (filein)
    {
      descriptors_.clear();
      for (std::string line; std::getline(filein, line); )
      {
        vpColVector descriptor;
        boost::char_separator<char> sep(" ");
        boost::tokenizer< boost::char_separator<char> > tokens(line, sep);

        for (boost::tokenizer<boost::char_separator<char> >::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
        {
          descriptor.resize(descriptor.size() + 1, false);
          descriptor[descriptor.size() - 1] = boost::lexical_cast<double>(*tok_iter);
        }
        descriptors_.push_back(descriptor);
      }
    }

    //Prepare the marker array that will be published
    marker_.reset(new visualization_msgs::MarkerArray());
    visualization_msgs::Marker moutline, mtext;
    moutline.header.frame_id = object_id_ + "_pose";
    moutline.id = 1;
    moutline.type = moutline.LINE_LIST;
    moutline.action = 0;
    moutline.color.r = 0.0;
    moutline.color.g = 1.0;
    moutline.color.b = 0.0;
    moutline.color.a = 1.0;
    moutline.scale.x = 0.02;
    moutline.lifetime = ros::Duration(20.0);
    EdgesModel *m = dynamic_cast<EdgesModel*>(object_->model->geometry);
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      geometry_msgs::Point p0, p1;
      p0.x = m->edges_list[i]->v1->v.get_oX();
      p0.y = m->edges_list[i]->v1->v.get_oY();
      p0.z = m->edges_list[i]->v1->v.get_oZ();
      p1.x = m->edges_list[i]->v2->v.get_oX();
      p1.y = m->edges_list[i]->v2->v.get_oY();
      p1.z = m->edges_list[i]->v2->v.get_oZ();
      moutline.points.push_back(p0);
      moutline.points.push_back(p1);
    }

    mtext.header.frame_id = object_id_ + "_pose";
    mtext.id = 2;
    mtext.type = mtext.TEXT_VIEW_FACING;
    mtext.action = 0;
    mtext.color.r = 1.0;
    mtext.color.g = 0.0;
    mtext.color.b = 0.0;
    mtext.color.a = 1.0;
    mtext.scale.x = 0.1;
    mtext.scale.y = 0.1;
    mtext.scale.z = 0.1;
    mtext.text = object_id_;
    mtext.lifetime = ros::Duration(20.0);

    marker_->markers.push_back(moutline);
    marker_->markers.push_back(mtext);

    //Create the publisher
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_" + object_id, 1);
  }
}

bool MBPoseEstimation::track(sensor_msgs::ImageConstPtr &image, tf::Transform &object_pose)
{
  cop_->view = toVispImageRGBa(*image);
  bool result = track(cop_->view, object_pose);
  vpDisplay::getClick(cop_->getView());
}

bool MBPoseEstimation::track(const vpImage<vpRGBa> &image, tf::Transform &object_pose)
{
  static vpTime time;
  double t_start = 0.0, t_end = 0.0;

  cop_->view = image;
  if (debug_mode_)
  {
    vpDisplay::display(cop_->getView());
  }

  //Get current camera pose with respect to a fixed frame
  wMc_->update();
  vpMatrix cam_motion(*wMc_ - wMc0_);
  if (cam_motion.euclideanNorm() > CAMERA_MOTION_TOLERANCE)
  {
    //ROS_INFO("feedforward");
    //if camera pose is changing, feedforward to vvs
    vpHomogeneousMatrix cMc0(wMc_->inverse() * wMc0_);
    wMc0_ = *wMc_;
    cop_->cMo = cMc0 * cop_->cMo;
  }
  else
  {
    //ROS_INFO("not feedforwarding");
  }

  if (cam_motion.euclideanNorm() > CAMERA_FAST_MOTION_TOLERANCE)
  {
    //if fast camera motion, stop tracking, use odometry
    //ROS_INFO("Fast motion");
    last_motion_stamp_ = wMc_->getStamp();
  }
  else if (wMc_->getStamp() - last_motion_stamp_ > ros::Duration(DEFAULT_RECOVER_TIMEOUT))
  {
    //ROS_INFO("Recover");
    //If camera not moving or moving slow, apply VVS
    t_start = time.measureTimeMs();
    vvs_->computePose();
    t_end = time.measureTimeMs();

    if (debug_mode_)
    {
      vvs_->drawModel(object_.get(), cop_->getView());
      vvs_->drawSearchPixels(object_.get(), cop_->getView());
      vvs_->drawMatchedPixels(object_.get(), cop_->getView());
    }
  }

  //Publish tf
  object_pose.setOrigin(tf::Vector3(cop_->cMo[0][3], cop_->cMo[1][3], cop_->cMo[2][3]));
  vpThetaUVector utheta(vpRotationMatrix(cop_->cMo));
  double theta;
  vpColVector u;
  utheta.extract(theta, u);
  tf::Quaternion tracked_rotation(tf::Vector3(u[0], u[1], u[2]), theta);
  object_pose.setRotation(tracked_rotation);
  br_.sendTransform(tf::StampedTransform(object_pose, wMc_->getStamp(), camera_frame_, object_id_ + "_pose"));

  //Publish a marker
  marker_->markers[0].header.stamp = wMc_->getStamp();
  marker_->markers[1].header.stamp = wMc_->getStamp();
  marker_pub_.publish(marker_);

  //Draw results if in debug mode
  if (debug_mode_)
  {
    char st[128];
    sprintf(st,"%.2f       Right click cancels. Left click stores pose", t_end - t_start);
    vpDisplay::displayCharString(cop_->getView(),20,20,st,vpColor::red);
    vpDisplay::flush(cop_->getView());

    vpMouseButton::vpMouseButtonType button;
    vpImagePoint ip;
    bool click = vpDisplay::getClick(cop_->getView(), ip, button, false);
    if (click && button == vpMouseButton::button1)
    {
      //save object pose
      vpHomogeneousMatrix wMo = *wMc_ * cop_->cMo;
      std::string wMo_file(ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".wMo");
      vpMatrix::saveMatrix(wMo_file, wMo);
      ROS_INFO_STREAM("Object pose wrt world saved into " << wMo_file);

      //store descriptors
      ofstream descriptor_file((ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".descriptor").c_str());
      EdgesModel *emodel = dynamic_cast<EdgesModel*>(object_->model->geometry);
      for (std::size_t i = 0; i < emodel->edges_list.size(); ++i)
      {
        if (emodel->edges_list[i]->active)
        {
          vpColVector normal(2);
          double theta = emodel->edges_list[i]->line.getTheta();
          normal[0] = cos(theta);
          normal[1] = sin(theta);

          for (std::size_t j = 0; j < emodel->edges_list[i]->points.size(); ++j)
          {
            vpColVector descriptor;
            descriptor = emodel->edges_list[i]->points[j].compute_edge_descriptor(cop_->getView(), normal, 10);
            descriptor_file << i << " " << j << " " << descriptor.t() << std::endl;
          }
        }
      }

    } else if (click && button == vpMouseButton::button3)
    {
      return false;
    }
  }
  return true;
}

bool MBPoseEstimation::minimizePose(sensor_msgs::ImageConstPtr &image, tf::Transform &object_pose, ros::Duration timeout, double tolerance)
{
  double error = 0.0;
  bool run = true;
  ros::Time start = ros::Time::now();
  bool run_timeout = false;
  do
  {
    run = track(image, object_pose);
    error = vvs_->getError();
    run_timeout = ! (ros::Time::now() - start < timeout);
  } while (error > tolerance && run && ! run_timeout);

  return (run && ! run_timeout);
}

bool MBPoseEstimation::minimizePose(const vpImage<vpRGBa> &image, tf::Transform &object_pose, ros::Duration timeout, double tolerance)
{
  double error = 0.0;
  bool run = true;
  ros::Time start = ros::Time::now();
  bool run_timeout = false;
  do
  {
    run = track(image, object_pose);
    error = vvs_->getError();
    run_timeout = ! (ros::Time::now() - start < timeout);
  } while (error > tolerance && run && ! run_timeout);

  return (run && ! run_timeout);
}

bool MBPoseEstimation::minimizePoseFromDescriptors(const vpImage<vpRGBa> &image, tf::Transform &object_pose, ros::Duration timeout, double tolerance)
{
  if (descriptors_.size() == 0)
  {
    ROS_WARN_STREAM("No descriptors available for object " << object_->getName());
    return false;
  }

  double error = 0.0;
  bool run = true;
  ros::Time start = ros::Time::now();
  bool run_timeout = false;
  do
  {
    cop_->view = image;
    if (debug_mode_)
    {
      vpDisplay::display(cop_->getView());
      vvs_->drawModel(object_.get(), cop_->getView());
      vvs_->drawSearchPixels(object_.get(), cop_->getView());
      vvs_->drawMatchedPixels(object_.get(), cop_->getView());

      char st[128];
      sprintf(st, "       Left click validates. Right click cancels.");
      vpDisplay::displayCharString(cop_->getView(),20,20,st,vpColor::red);
      vpDisplay::flush(cop_->getView());

      vpMouseButton::vpMouseButtonType button;
      vpImagePoint ip;
      bool click = vpDisplay::getClick(cop_->getView(), ip, button, false);
      if (click && button == vpMouseButton::button1)
      {
        return true;
      }
      else if (click)
      {
        return false;
      }
    }
    vvs_->computePoseFromDescriptors(descriptors_);
    vpHomogeneousMatrixToTFTransform(cop_->cMo, object_pose);

    error = vvs_->getError();
    run_timeout = ! (ros::Time::now() - start < timeout);
  } while ( ros::ok() && (debug_mode_ || (error > tolerance && run && ! run_timeout)));

  return (run && ! run_timeout);
}

bool MBPoseEstimation::projectModel(const vpImage<vpRGBa> &image)
{
  cop_->view = image;

  vvs_->project_model(object_.get(), cop_->cMo);

  if (debug_mode_)
  {
    vpDisplay::display(cop_->getView());
    vvs_->drawModel(object_.get(), cop_->getView());
    vvs_->drawSearchPixels(object_.get(), cop_->getView());
    vvs_->drawMatchedPixels(object_.get(), cop_->getView());

    char st[128];
    sprintf(st, "       Right click cancels. Left click stores pose");
    vpDisplay::displayCharString(cop_->getView(),20,20,st,vpColor::red);
    vpDisplay::flush(cop_->getView());

    vpMouseButton::vpMouseButtonType button;
    vpImagePoint ip;
    bool click = vpDisplay::getClick(cop_->getView(), ip, button, false);
    if (click && button == vpMouseButton::button1)
    {
      //save object pose
      vpHomogeneousMatrix wMo = *wMc_ * cop_->cMo;
      std::string wMo_file(ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".wMo");
      vpMatrix::saveMatrix(wMo_file, wMo);
      ROS_INFO_STREAM("Object pose wrt world saved into " << wMo_file);

      //store descriptors
      ofstream descriptor_file((ros::package::getPath("mb_pose_estimation") + "/data/" + object_id_ + ".descriptor").c_str());
      EdgesModel *emodel = dynamic_cast<EdgesModel*>(object_->model->geometry);
      for (std::size_t i = 0; i < emodel->edges_list.size(); ++i)
      {
        if (emodel->edges_list[i]->active)
        {
          vpColVector normal(2);
          double theta = emodel->edges_list[i]->line.getTheta();
          normal[0] = cos(theta);
          normal[1] = sin(theta);

          for (std::size_t j = 0; j < emodel->edges_list[i]->points.size(); ++j)
          {
            vpColVector descriptor;
            descriptor = emodel->edges_list[i]->points[j].compute_edge_descriptor(cop_->getView(), normal, 10);
            descriptor_file << i << " " << j << " " << descriptor.t() << std::endl;
          }
        }
      }

    } else if (click && button == vpMouseButton::button3)
    {
      return false;
    }
  }
  return true;
}

void MBPoseEstimation::tfTransformToVisp(const tf::Transform &tf, vpHomogeneousMatrix &vptf)
{
  tf::Matrix3x3 R;
  R = tf.getBasis();
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      vptf[i][j] = R[i][j];
  vptf[0][3] = tf.getOrigin().x();
  vptf[1][3] = tf.getOrigin().y();
  vptf[2][3] = tf.getOrigin().z();
}

void MBPoseEstimation::vpHomogeneousMatrixToTFTransform(const vpHomogeneousMatrix &vptf, tf::Transform &tf)
{
  tf::Quaternion object_rotation;
  tf.setOrigin(tf::Vector3(vptf[0][3], vptf[1][3], vptf[2][3]));
  double theta;
  vpColVector u;
  vpThetaUVector utheta_vector;
  utheta_vector.buildFrom(vpRotationMatrix(vptf));
  utheta_vector.extract(theta, u);
  object_rotation.setRotation(tf::Vector3(u[0], u[1], u[2]), theta);
  tf.setRotation(object_rotation);
}
