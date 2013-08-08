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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mb_pose_estimation/MBTrackAction.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp_bridge/conversions/3dpose.h>

#include <lima/CAOEdgesModel.h>
#include <lima/Object.h>
#include <lima/Camera.h>
#include <lima/CameraObjectPose.h>
#include <lima/ROSGrabber.h>

#include <tf/transform_listener.h>

namespace po = boost::program_options;

#define DEFAULT_SEARCH_INTERVAL 11
#define DEFAULT_SAMPLING_DISTANCE 8

Object *createObjectFromCAO(const std::string &file) {
        CAOEdgesModel *o1_gmodel=new CAOEdgesModel(file);
        ObjectModel *o1_model=new ObjectModel(o1_gmodel);
        Object *o1=new Object("Object", o1_model);

        return o1;
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const mb_pose_estimation::MBTrackResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const mb_pose_estimation::MBTrackFeedbackConstPtr& feedback)
{
}

int main (int argc, char **argv)
{
  po::options_description desc("Allowed options");
  std::vector<std::string> ids;
  std::vector<std::string> frame_ids;
  desc.add_options()
               ("help", "produce help message")
               ("id", po::value<std::vector<std::string> >(&ids)->multitoken(), "Objects to track")
               ("camera", po::value<std::string>(), "listen to a ROS camera topic")
               ("camera-info", po::value<std::string>(), "listen to a ROS camera info topic")
               ("init-click", "initialize by clicking on the corners specified in a file")
               ("init-file", "initialize from a pose stored in a file")
               ("init-tf", po::value<std::vector<std::string> >(&frame_ids)->multitoken(), "initialize from tf transforms")
               ("sampling-distance", po::value<int>(), "distance in pixels between consecutive sampling points")
               ("search-interval", po::value<int>(), "interval in pixels for searching for image edges");

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

  ros::init(argc, argv, "test_track");
  ros::NodeHandle nh;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<mb_pose_estimation::MBTrackAction> ac("mb_pose_estimation_action_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time


  //Get an image and estimate the object pose by clicking on known points in the image.
  Camera *ros_grabber = NULL;
  if (vm.count("camera") && vm.count("camera-info") &&
      (vm.count("init-click") || vm.count("init-tf") || vm.count("init-file") ))
  {
    ROS_INFO_STREAM("Listening to topic " << vm["camera"].as<std::string>());
    ros_grabber = new ROSGrabber(nh, vm["camera"].as<std::string>(), vm["camera-info"].as<std::string>());
    ros::Rate r(5);
    while ( ! ((ROSGrabber*)ros_grabber)->ready() && ros::ok())
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

  vpHomogeneousMatrix wMo_init[ids.size()];

  for (std::size_t i = 0; i< ids.size(); ++i)
  {
    std::string cao_file = ros::package::getPath("mb_pose_estimation") + "/data/" + ids[i] + ".cao";
    if (! boost::filesystem::exists(cao_file))
    {
      ROS_ERROR_STREAM("Cannot locate model " << cao_file);
      exit(0);
    }

    if (! vm.count("init-tf"))
    {
      Object *object = createObjectFromCAO(cao_file);
      CameraObjectPose<vpRGBa> cop(ros_grabber, object);
      ros_grabber->open(cop.getView());
      cop.captureView();

      //Display window if debug mode
      vpDisplayX window;
      window.init(cop.getView(),0,0,"MBTrack client");
      vpDisplay::display(cop.getView());
      vpDisplay::flush(cop.getView());

      //Initialize object pose by clicking on the image
      if (vm.count("init-click"))
      {
        std::string init_file(ros::package::getPath("mb_pose_estimation") + "/data/" + ids[i] + ".init");
        if (! boost::filesystem::exists(init_file))
        {
          ROS_ERROR_STREAM("Cannot locate init file " << init_file);
          exit(0);
        }
        cop.initClick(cop.getView(), init_file);
      }

      //TODO: Convert cop.cMo into wMo
      tf::TransformListener listener;
      tf::StampedTransform wMc_tf;

      try {
        if (listener.waitForTransform("/map", ((ROSGrabber*)ros_grabber)->frame_id, ros::Time(0), ros::Duration(5.0)))
        {
          listener.lookupTransform("/map", ((ROSGrabber*)ros_grabber)->frame_id, ros::Time(0), wMc_tf);
        }
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
      }

      vpHomogeneousMatrix wMc;
      for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
          wMc[r][c] = wMc_tf.getBasis()[r][c];
      wMc[0][3] = wMc_tf.getOrigin().x();
      wMc[1][3] = wMc_tf.getOrigin().y();
      wMc[2][3] = wMc_tf.getOrigin().z();

      wMo_init[i] = wMc * cop.cMo;
    }

    if (vm.count("init-file"))
    {
      vpHomogeneousMatrix wMo;
      vpMatrix::loadMatrix(std::string(ros::package::getPath("mb_pose_estimation") + "/data/" + ids[i] + ".wMo").c_str(), wMo);

      wMo_init[i] = wMo;
    }
  }

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  mb_pose_estimation::MBTrackGoal goal;
  for (std::size_t i = 0; i< ids.size(); ++i)
  {
    goal.object_id.push_back(ids[i]);
    if (vm.count("init-tf"))
    {
      goal.target_frame_id.push_back(frame_ids[i]);
    }
    else
    {
      tf::Transform object_pose;
      tf::Quaternion object_rotation;
      object_pose.setOrigin(tf::Vector3(wMo_init[i][0][3], wMo_init[i][1][3], wMo_init[i][2][3]));
      vpThetaUVector utheta(vpRotationMatrix(wMo_init[i]));
      double theta;
      vpColVector u;
      utheta.extract(theta, u);
      object_rotation.setRotation(tf::Vector3(u[0], u[1], u[2]), theta);
      object_pose.setRotation(object_rotation);
      geometry_msgs::TransformStamped object_pose_msg;
      tf::transformTFToMsg(object_pose, object_pose_msg.transform);
      object_pose_msg.header.frame_id = "map";

      goal.wMo.push_back(object_pose_msg);
    }

    goal.samples_distance.push_back((vm.count("sampling-distance") ? vm["sampling-distance"].as<int>() : DEFAULT_SAMPLING_DISTANCE));
    goal.search_interval.push_back((vm.count("search-interval") ? vm["search-interval"].as<int>() : DEFAULT_SEARCH_INTERVAL));
  }

  goal.dof.push_back(1);
  goal.dof.push_back(1);
  goal.dof.push_back(1);
  goal.dof.push_back(1);
  goal.dof.push_back(1);
  goal.dof.push_back(1);
  goal.do_initial_minimization = true;
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();

  ROS_INFO("Finished");
  return 0;
}
