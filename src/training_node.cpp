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
#include <tf/transform_datatypes.h>
#include <mb_pose_estimation/mb_pose_estimation.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>

#include <lima/CAOEdgesModel.h>
#include <lima/Object.h>
#include <lima/Camera.h>
#include <lima/CameraObjectPose.h>
#include <lima/ROSGrabber.h>

#include <tf/transform_listener.h>

namespace po = boost::program_options;

Object *createObjectFromCAO(const std::string &file) {
        CAOEdgesModel *o1_gmodel=new CAOEdgesModel(file);
        ObjectModel *o1_model=new ObjectModel(o1_gmodel);
        Object *o1=new Object("Object", o1_model);

        return o1;
}


int main (int argc, char **argv)
{
  po::options_description desc("Allowed options");
  std::string id;
  std::string frame_id;
  desc.add_options()
               ("help", "produce help message")
               ("id", po::value<std::string>(), "Object to train")
               ("camera", po::value<std::string>(), "listen to a ROS camera topic")
               ("camera-info", po::value<std::string>(), "listen to a ROS camera info topic")
               ("world-frame", po::value<std::string>(), "The name of the tf frame to be used as world");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    exit(0);
  }

  if (! vm.count("id"))
  {
    ROS_ERROR("Missing --id argument. Need to specify at least one object to track");
    std::cout << desc << "\n";
    exit(0);
  }
  id = vm["id"].as<std::string>();

  ros::init(argc, argv, "training_node");
  ros::NodeHandle nh;


  //Get an image and estimate the object pose by clicking on known points in the image.
  ROSGrabber *ros_grabber = NULL;
  if (vm.count("camera") && vm.count("camera-info"))
  {
    ROS_INFO_STREAM("Listening to topic " << vm["camera"].as<std::string>());
    ros_grabber = new ROSGrabber(nh, vm["camera"].as<std::string>(), vm["camera-info"].as<std::string>());
    ros::Rate r(5);
    while ( ! ros_grabber->ready() && ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }
  else
  {
    ROS_INFO_STREAM("Missing parameters. Please make sure you have specified --camera, --camera-info and --init-click or --init-tf");
    exit(0);
  }

  vpHomogeneousMatrix wMo_init;

  std::string cao_file = ros::package::getPath("mb_pose_estimation") + "/data/" + id + ".cao";
  if (! boost::filesystem::exists(cao_file))
  {
    ROS_ERROR_STREAM("Cannot locate model " << cao_file);
    exit(0);
  }

  Object *object = createObjectFromCAO(cao_file);
  CameraObjectPose<vpRGBa> cop(ros_grabber, object);
  ros_grabber->open(cop.getView());
  cop.captureView();

  //Display window if debug mode
  vpDisplayX window;
  window.init(cop.getView(),0,0,"MBPoseEstimation training");
  vpDisplay::display(cop.getView());
  vpDisplay::flush(cop.getView());


  std::string init_file(ros::package::getPath("mb_pose_estimation") + "/data/" + id + ".init");
  if (! boost::filesystem::exists(init_file))
  {
    ROS_ERROR_STREAM("Cannot locate init file " << init_file);
    exit(0);
  }
  cop.initClick(cop.getView(), init_file);

  boost::shared_ptr<MBPoseEstimation> track_;
  track_.reset(new MBPoseEstimation(nh, ros_grabber->image, ros_grabber->cparams, ros_grabber->frame_id, vm["world-frame"].as<std::string>()));
  track_->setDebugMode(true);

  tf::Transform object_pose;
  tf::Quaternion object_rotation;
  object_pose.setOrigin(tf::Vector3(cop.cMo[0][3], cop.cMo[1][3], cop.cMo[2][3]));
  double theta;
  vpColVector u;
  vpThetaUVector utheta_vector;
  utheta_vector.buildFrom(vpRotationMatrix(cop.cMo));
  utheta_vector.extract(theta, u);
  object_rotation.setRotation(tf::Vector3(u[0], u[1], u[2]), theta);
  object_pose.setRotation(object_rotation);
  geometry_msgs::TransformStamped object_pose_msg;
  tf::transformTFToMsg(object_pose, object_pose_msg.transform);
  object_pose_msg.header.frame_id = ros_grabber->frame_id;

  tf::StampedTransform cMo_init;
  tf::transformStampedMsgToTF(object_pose_msg, cMo_init);

  track_->setObjectID(id, cMo_init);

  static const double sampling_distance = 10;
  static const double search_interval = 30;
  track_->setSamplingDistance(sampling_distance);
  track_->setSearchInterval(search_interval);

  track_->setTrackDOF(true, true, true, true, true, true);

  tf::Transform cMo;

  while (ros::ok() && track_->projectModel(ros_grabber->image))
  {
    ros::spinOnce();
  }

  ROS_INFO("Finished");
  return 0;
}
