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

#include <lima/ROSGrabber.h>
#include <mb_pose_estimation/mb_pose_estimation.h>

#include <iostream>
#include <sys/signal.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <mb_pose_estimation/MBTrackAction.h>

using namespace std;
namespace po = boost::program_options;

/** An action server for binfrastructure tracking
 */
class MBTrackActionServer
{
public:

  MBTrackActionServer(int argc, char **argv, std::string name) :
    argc_(argc),
    argv_(argv),
    as_(nh_, name, false),
    action_name_(name)
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
     desc.add_options()
         ("help", "produce help message")
         ("camera", po::value<std::string>(), "listen to a ROS camera topic")
         ("camera-info", po::value<std::string>(), "listen to a ROS camera info topic")
         ("debug", "display a window with the tracking")
         ("train", "just train the system for a specific object")
         ("world-frame", po::value<std::string>(), "The name of the tf frame to be used as world");

     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);

     if (vm.count("help"))
     {
         cout << desc << "\n";
         exit(0);
     }

     //register the goal and feeback callbacks
     as_.registerGoalCallback(boost::bind(&MBTrackActionServer::goalCB, this));
     as_.registerPreemptCallback(boost::bind(&MBTrackActionServer::preemptCB, this));

     //subscribe to the data topic of interest
     as_.start();
  }

  ~MBTrackActionServer(void)
  {
  }

  void goalCB()
  {
    //start tf listener
    listener_.reset(new tf::TransformListener());

    // accept the new goal
    mb_pose_estimation::MBTrackGoalConstPtr goal = as_.acceptNewGoal();

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
      as_.setAborted(result_);
    }

    std::vector<tf::Transform> cMo_init(goal->object_id.size());
    for (std::size_t id = 0; id < goal->object_id.size(); ++id)
    {
      boost::shared_ptr<MBTrack> track_i(new MBTrack(nh_, ros_grabber_->image, ros_grabber_->cparams, ros_grabber_->frame_id, vm["world-frame"].as<std::string>()));

      if (vm.count("debug"))
      {
        track_i->setDebugMode(true);
      }
      else
      {
        track_i->setDebugMode(false);
      }

      //Initialize object pose: either wait for a tf frame, or use the action goal cMo field
      if (goal->target_frame_id.size() > id && ! goal->target_frame_id[id].empty())
      {
        tf::StampedTransform cMo;
        bool init = false;
        while (! init && ros::ok())
        {
          try
          {
            listener_->lookupTransform(ros_grabber_->frame_id, goal->target_frame_id[id], ros_grabber_->stamp, cMo);
            cMo_init[id] = cMo;
            init = true;
          }
          catch (tf::TransformException ex) {
            ROS_INFO_THROTTLE(2.0, "Waiting for AR detection");
          }
        }
      }
      else
      {
        tf::StampedTransform wMo;
        //Initialize object pose from action goal
        tf::transformStampedMsgToTF(goal->wMo[id], wMo);

        tf::StampedTransform wMc;
        try {
          if (listener_->waitForTransform("/map", ros_grabber_->frame_id, ros::Time(0), ros::Duration(5.0)))
          {
            listener_->lookupTransform("/map", ros_grabber_->frame_id, ros::Time(0), wMc);
          }
        } catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
        }

        cMo_init[id] = wMc.inverse() * wMo;
      }

      track_i->setObjectID(goal->object_id[id], cMo_init[id]);

      track_i->setSamplingDistance(goal->samples_distance[id]);
      track_i->setSearchInterval(goal->search_interval[id]);

      track_i->setTrackDOF(true, true, true, true, true, true);
      if (! goal->do_initial_minimization && goal->dof.size() == 6)
      {
        track_i->setTrackDOF(goal->dof[0], goal->dof[1], goal->dof[2], goal->dof[3], goal->dof[4], goal->dof[5]);
      }
      track_.push_back(track_i);
    }

    tf::Transform cMo[goal->object_id.size()];
    bool tracking = true;
    ros::Time start = ros::Time::now();
    ros::Time current;
    bool initial_minimization = goal->do_initial_minimization;
    feedback_.estimated_pose.resize(goal->object_id.size());
    while (tracking && ros::ok() && ! as_.isPreemptRequested())
    {
      ros::spinOnce();
      current = ros::Time::now();
      
      if (initial_minimization && (current - start) > ros::Duration(INITIAL_MINIMIZATION_TIMEOUT))
      {
        initial_minimization = false;
	if (goal->dof.size() == 6)
	{
	  for (std::size_t i = 0; i < track_.size(); ++i)
	  {
	    track_[i]->setTrackDOF(goal->dof[0], goal->dof[1], goal->dof[2], goal->dof[3], goal->dof[4], goal->dof[5]);
	  }
	}
      }

      for (std::size_t i = 0; i < track_.size(); ++i)
      {
	if (vm.count("train"))
        {
          tracking = track_[i]->projectModel(ros_grabber_->image);
        }
        else
        {
          track_[i]->minimizePoseFromDescriptors(ros_grabber_->image, cMo[i] );
        }
        tf::transformTFToMsg(cMo[i], feedback_.estimated_pose[i]);
      }

      as_.publishFeedback(feedback_);
    }

    track_.clear();
    ros_grabber_.reset();
    listener_.reset();
    ROS_INFO("Action finished");
    as_.setSucceeded(result_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());

    as_.setPreempted();
  }

  void analysisCB()
  {
    if (!as_.isActive())
      return;
  }

protected:
  static const double INITIAL_MINIMIZATION_TIMEOUT = 0.0;

  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> listener_;

  int argc_;
  char **argv_;
  actionlib::SimpleActionServer<mb_pose_estimation::MBTrackAction> as_;
  std::string action_name_;
  int goal_;
  mb_pose_estimation::MBTrackFeedback feedback_;
  mb_pose_estimation::MBTrackResult result_;

  po::variables_map vm;
  std::vector<boost::shared_ptr<MBTrack> > track_;
  boost::shared_ptr<ROSGrabber> ros_grabber_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mb_pose_estimation_action_server");

  MBTrackActionServer track_server(argc, argv, ros::this_node::getName());
  ros::spin();

  return 0;
}

