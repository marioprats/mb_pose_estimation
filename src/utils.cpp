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

#include <mb_pose_estimation/utils.h>
#include <sensor_msgs/image_encodings.h>

/** Adapted from visp_bridge package conversions/image.h */
vpImage<vpRGBa> toVispImageRGBa(const sensor_msgs::Image& src)
{
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::RGBA8;
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::BGRA8;


  vpImage<vpRGBa> dst(src.height, src.width);

  if (src.encoding == MONO8)
    for (unsigned i = 0; i < dst.getWidth(); ++i){
      for (unsigned j = 0; j < dst.getHeight(); ++j){

        dst[j][i] = vpRGBa(src.data[j * src.step + i],src.data[j * src.step + i],src.data[j * src.step + i]);
      }
    }
  else{
    unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);

    for (unsigned i = 0; i < dst.getWidth(); ++i){
      for (unsigned j = 0; j < dst.getHeight(); ++j){
        dst[j][i] = vpRGBa(src.data[j * src.step + i * nc + 0],src.data[j * src.step + i * nc + 1],src.data[j * src.step + i * nc + 2]);
      }
    }
  }
  return dst;
}

