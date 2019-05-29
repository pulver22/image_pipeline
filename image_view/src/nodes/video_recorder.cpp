/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <fstream>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
bool do_timestamp_srt;
std::timed_mutex image_mutex;
cv::Mat last_image;
cv::Mat last_saved_image;
ros::Time first_time;
ros::Time last_time;
ros::Time last_saved_time;
std::ofstream subfile;
std::string subfilename;

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    if (!outputVideo.isOpened()) {

        cv::Size size(image_msg->width, image_msg->height);

        outputVideo.open(filename,
#if CV_MAJOR_VERSION == 3
                cv::VideoWriter::fourcc(codec.c_str()[0],
#else
                CV_FOURCC(codec.c_str()[0],
#endif
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]),
                fps,
                size,
                true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );
//        if (do_timestamp_srt)
//            first_time = image_msg->header.stamp;
    }

    // if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1.0 / fps))
    // {
    //   // Skip to get video with correct fps
    //   return;
    // }

    try
    {
      cv_bridge::CvtColorForDisplayOptions options;
      options.do_dynamic_scaling = use_dynamic_range;
      options.min_image_value = min_depth_range;
      options.max_image_value = max_depth_range;
      options.colormap = colormap;
      const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
      // if (!image.empty()) {
      //   outputVideo << image;
      //   ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
      //   g_count++;
      //   g_last_wrote_time = image_msg->header.stamp;
      // } else {
      //     ROS_WARN("Frame skipped, no data!");
      // }
      if (!image.empty()) {
        image_mutex.lock();
        last_image = image;
        if (do_timestamp_srt) {
            //std::cout << image_msg->header.stamp << std::endl;
            last_time = image_msg->header.stamp;
            //last_duration = last_time - first_time;
        }
        image_mutex.unlock();
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
        g_last_wrote_time = image_msg->header.stamp;
      } else {
        ROS_WARN("Frame skipped, no data!");
      }
    } catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

std::string duration_to_strformat(const ros::Duration duration)
{
    int t_minutes;
    std::string hours, minutes, seconds, mseconds;
    t_minutes = duration.sec / 60;
    seconds = std::to_string(int(duration.sec % 60));
    seconds = ((seconds.length() == 1) ? "0"+seconds : seconds);
    hours = std::to_string(int(t_minutes / 60));
    hours = ((hours.length() == 1) ? "0"+hours : hours);
    minutes = std::to_string(int(t_minutes % 60));
    minutes = ((minutes.length() == 1) ? "0"+minutes : minutes);
    std::string tosec = std::to_string(duration.toSec());
    mseconds = tosec.substr(tosec.find_last_of(".") + 1, 3);
    return hours + ":" + minutes + ":" + seconds + "," + mseconds;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    local_nh.param("filename", filename, std::string("output.avi"));
    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);
    local_nh.param("do_timestamp_srt", do_timestamp_srt, false);

    if (stamped_filename) {
      std::size_t found = filename.find_last_of("/\\");
      std::string path = filename.substr(0, found + 1);
      std::string basename = filename.substr(found + 1);
      std::stringstream ss;
      ss << ros::Time::now().toNSec() << basename;
      filename = path + ss.str();
      ROS_INFO("Video recording to %s", filename.c_str());
    }

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName("image");
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);

    // use async spinner
    ros::AsyncSpinner spinner(0);

    if (do_timestamp_srt) {
        std::size_t found_d= filename.find_last_of(".");
        subfilename = filename.substr(0, found_d) + ".srt";
        subfile.open (subfilename);
    }

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");

    spinner.start();

    std::chrono::milliseconds max_wait_ms(int(std::floor(1000.0 / fps)));

    // waiting to receive the first image
    while (ros::ok() && last_image.empty())
       std::this_thread::sleep_for(max_wait_ms);

    int n_frame = 0;
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::milliseconds elapsed_ms;
    std::chrono::milliseconds ms_left;

    // take the first image
//    image_mutex.lock();
//   last_saved_image = last_image.clone();
//    last_saved_duration = last_duration;
//    last_saved_time = last_time;
//    image_mutex.unlock();

    while (ros::ok()) {
        // time left for staying in the framerate
        elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time);
        ms_left = max_wait_ms - elapsed_ms;

        // continue take the last possible image 
        do {
             if (image_mutex.try_lock_for(ms_left)) {
                 last_saved_image = last_image.clone();
                 last_saved_time = last_time;
                 image_mutex.unlock();
             }
             elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time);
             ms_left = max_wait_ms - elapsed_ms;
        } while (ms_left > std::chrono::milliseconds(0));

        start_time = std::chrono::system_clock::now();

        // save timestamp in subtitles
        if (do_timestamp_srt) {
           if (first_time.isZero())
               first_time = ros::Time::now();

           std::string time_str = duration_to_strformat(ros::Time::now() - first_time);
           subfile << n_frame++ << "\n";
           subfile << time_str << " --> " << time_str << "\n";
           subfile << last_saved_time << "\n\n";
        }
        // save frame
        outputVideo << last_saved_image;
    }

    spinner.stop();

    outputVideo.release();
    std::cout << "\nVideo saved as " << filename << std::endl;

    if (do_timestamp_srt) {
        subfile.close();
        std::cout << "Subs saved as " << subfilename << std::endl;
    }

    return 0;
}
