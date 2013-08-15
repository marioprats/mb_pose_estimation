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

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpThetaUVector.h>

#include <lima/ROSGrabber.h>
#include <mb_pose_estimation/mb_pose_estimation.h>

#include <iostream>
#include <sys/signal.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <mb_pose_estimation/PoseEstimation.h>

using namespace std;
namespace po = boost::program_options;

/** An action server for binfrastructure tracking
 */
class SGPoseEstimation
{
public:

  SGPoseEstimation(int argc, char **argv, std::string name) :
    argc_(argc),
    argv_(argv)
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
     desc.add_options()
         ("help", "produce help message")
         ("camera", po::value<std::string>(), "listen to a ROS camera topic")
         ("camera-info", po::value<std::string>(), "listen to a ROS camera info topic")
         ("debug", "display a window with the tracking")
         ("world-frame", po::value<std::string>(), "The name of the tf frame to be used as world");

     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);

     if (vm.count("help"))
     {
         cout << desc << "\n";
         exit(0);
     }

     service_ = nh_.advertiseService("pose_estimation", &SGPoseEstimation::poseEstimationCallback, this);
     ROS_INFO("Service adverstised");
  }

  ~SGPoseEstimation(void)
  {
  }

  bool poseEstimationCallback(mb_pose_estimation::PoseEstimation::Request& request, mb_pose_estimation::PoseEstimation::Response& response)
  {
    //start tf listener
    listener_.reset(new tf::TransformListener());


    //subscribe to the image topic
    if (vm.count("camera") && vm.count("camera-info"))
    {
      ROS_INFO_STREAM("Listening to topic " << vm["camera"].as<std::string>());
      ros_grabber_.reset(new ROSGrabber(nh_, vm["camera"].as<std::string>(), vm["camera-info"].as<std::string>()));
      ros::Rate r(5);
      while ( ! ros_grabber_->ready() && ros::ok())
      {
        ros::spinOnce();
        r.sleep();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Missing parameters. Please make sure you have specified --camera, --camera-info");
      return false;
    }

    tf::Transform cMo_init;

    track_.reset(new MBPoseEstimation(nh_, ros_grabber_->image, ros_grabber_->cparams, ros_grabber_->frame_id, vm["world-frame"].as<std::string>()));

    if (vm.count("debug"))
    {
      track_->setDebugMode(true);
    }
    else
    {
      track_->setDebugMode(false);
    }

    //Initialize object pose wMo file
    vpHomogeneousMatrix wMo_vp;
    vpMatrix::loadMatrix(std::string(ros::package::getPath("mb_pose_estimation") + "/data/" + request.object_id + ".wMo").c_str(), wMo_vp);

    tf::Transform object_pose;
    tf::Quaternion object_rotation;
    object_pose.setOrigin(tf::Vector3(wMo_vp[0][3], wMo_vp[1][3], wMo_vp[2][3]));
    double theta;
    vpColVector u;
    vpThetaUVector utheta_vector;
    utheta_vector.buildFrom(vpRotationMatrix(wMo_vp));
    utheta_vector.extract(theta, u);
    object_rotation.setRotation(tf::Vector3(u[0], u[1], u[2]), theta);
    object_pose.setRotation(object_rotation);
    geometry_msgs::TransformStamped object_pose_msg;
    tf::transformTFToMsg(object_pose, object_pose_msg.transform);
    object_pose_msg.header.frame_id = "map";

    tf::StampedTransform wMo;
    tf::transformStampedMsgToTF(object_pose_msg, wMo);

    tf::StampedTransform wMc;
    try {
      if (listener_->waitForTransform( vm["world-frame"].as<std::string>(), ros_grabber_->frame_id, ros::Time(0), ros::Duration(5.0)))
      {
        listener_->lookupTransform( vm["world-frame"].as<std::string>(), ros_grabber_->frame_id, ros::Time(0), wMc);
      }
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    cMo_init = wMc.inverse() * wMo;


    track_->setObjectID(request.object_id, cMo_init);

    static const double sampling_distance = 10;
    double search_interval = -15 * cMo_init.getOrigin().z() + 70.0; //makes search interval dependent on the distance
    ROS_INFO_STREAM("Distance to target is " << cMo_init.getOrigin().z() << ". Search interval is " << search_interval);
    track_->setSamplingDistance(sampling_distance);
    track_->setSearchInterval(search_interval);

    track_->setTrackDOF(true, true, true, true, true, true);

    tf::Transform cMo;

    response.is_valid = track_->minimizePoseFromDescriptors(ros_grabber_->image, cMo);

    tf::transformTFToMsg(wMc * cMo, response.estimated_pose.transform);
    tf::transformTFToMsg(cMo, response.camera_estimated_pose.transform);
    response.estimated_pose.child_frame_id = "object_pose";
    response.estimated_pose.header.frame_id =  vm["world-frame"].as<std::string>();
    response.camera_estimated_pose.child_frame_id = "object_pose";
    response.camera_estimated_pose.header.frame_id =  ros_grabber_->frame_id;

    track_.reset();
    ros_grabber_.reset();
    listener_.reset();
    ROS_INFO("Service finished");
    return true;
  }


protected:
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> listener_;

  int argc_;
  char **argv_;
  ros::ServiceServer service_;
  std::string service_name_;

  po::variables_map vm;
  boost::shared_ptr<MBPoseEstimation> track_;
  boost::shared_ptr<ROSGrabber> ros_grabber_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sg_pose_estimation");

  SGPoseEstimation pose_estimation(argc, argv, ros::this_node::getName());
  ros::spin();

  return 0;
}

