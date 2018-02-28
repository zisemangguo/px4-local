/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_OPTICAL_FLOW_PLUGIN_HH_
#define _GAZEBO_OPTICAL_FLOW_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "opticalFlow.pb.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/timer/timer.hpp>

#include "flow_opencv.hpp"
#include "flow_px4.hpp"

#define DEFAULT_RATE 20

using namespace cv;
using namespace std;

namespace gazebo
{
  class GAZEBO_VISIBLE OpticalFlowPlugin : public SensorPlugin
  {
    public:
      OpticalFlowPlugin();
      virtual ~OpticalFlowPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected:
      unsigned int width, height, depth;
      std::string format;
      sensors::CameraSensorPtr parentSensor;
      rendering::CameraPtr camera;

    private:
      event::ConnectionPtr newFrameConnection;
      transport::PublisherPtr opticalFlow_pub_;
      transport::NodePtr node_handle_;
      opticalFlow_msgs::msgs::opticalFlow opticalFlow_message;
      std::string namespace_;
      boost::timer::cpu_timer timer_;
      OpticalFlowOpenCV *optical_flow_;
      // OpticalFlowPX4 *optical_flow_;

      float hfov_;
      int dt_us_;
      int output_rate_;
      float focal_length_;
      double first_frame_time_;
      uint32_t frame_time_us_;
  };
}
#endif
