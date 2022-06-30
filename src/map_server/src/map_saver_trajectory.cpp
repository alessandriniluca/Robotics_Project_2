/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <cstdio>
#include <cmath>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "map_server/SaveTrajectory.h"

using namespace std;

/**
 * @brief Map generation node.
 */
class MapTrajectory
{
  private:
    nav_msgs::OccupancyGrid map_saved;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> trajectory;
    ros::ServiceServer service;
    std::vector<int> x_interpolation;
    std::vector<int> y_interpolation;

    std::vector<int> arange(int start, int stop, int step = 1) {
      std::vector<int> values;
      for (int value = start; value < stop; value += step)
          values.push_back(value);
      return values;
    }

    void interpolate(int x0, int y0, int x1, int y1){
      x_interpolation.clear();
      y_interpolation.clear();
      bool transpose = false;
      if (abs(x1 - x0) < abs(y1 - y0)){
          transpose = true;
          swap(x0, y0);
          swap(x1, y1);
      }

      if (x0>x1){
          swap(x0, x1);
          swap(y0, y1);
      }

      x_interpolation = arange(x0 + 1, x1);

      for (int i = 0; i<x_interpolation.size(); i++){
          y_interpolation.push_back(round((static_cast< float >(y1 - y0) / static_cast< float >(x1 - x0)) * static_cast< float >(x_interpolation.at(i) - x0) + y0));
      }

      if (transpose){
          swap(x_interpolation, y_interpolation);
      }

      return;
    }


  public:
    MapTrajectory(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      //map_sub_ = n.subscribe("map", 1, &MapTrajectory::mapCallback, this); //to delete
      traj_sub = n.subscribe("/amcl_pose", 1000, &MapTrajectory::mapCallback, this);
      // here subscribe to pose topic
      ROS_INFO("Map received!");
      service=n.advertiseService("SaveTrajectory", &MapTrajectory::traj_funct, this);
    }

    void mapCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
      geometry_msgs::PoseWithCovarianceStamped temp;
      temp = *msg;
      trajectory.push_back(temp);
    }

    bool traj_funct(map_server::SaveTrajectory::Request &req, map_server::SaveTrajectory::Response &res){
      // save the trajectory onto the map
      boost::shared_ptr<nav_msgs::OccupancyGrid const> sharedPtr(new nav_msgs::OccupancyGrid); //non risolve
			sharedPtr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(10));
      map_saved = *sharedPtr;
      std::string mapdatafile = req.filename + ".pgm";
      ROS_INFO("Writing trajectory %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save trajectory to %s", mapdatafile.c_str());
        return false;
      }

      ROS_INFO("Computing trajectory with interpolations ...");
      int y_init = 0;
      for(unsigned int i = 0; i<trajectory.size();i++){
        int pixel_position = static_cast<int>(trajectory[i].pose.pose.position.x/map_saved.info.resolution + map_saved.info.width/2) + (map_saved.info.height - static_cast<int>(trajectory[i].pose.pose.position.y/map_saved.info.resolution + map_saved.info.height/2) - 1) * map_saved.info.width;
        map_saved.data[pixel_position] = -2;
        if(i!=0){
          int pixel_position_prev_i = static_cast<int>(trajectory[i-1].pose.pose.position.x/map_saved.info.resolution + map_saved.info.width/2) + (map_saved.info.height - static_cast<int>(trajectory[i-1].pose.pose.position.y/map_saved.info.resolution + map_saved.info.height/2) - 1) * map_saved.info.width;
          int x0 = pixel_position_prev_i % map_saved.info.width;
          int y0 = pixel_position_prev_i / map_saved.info.width;
          int x1 = pixel_position % map_saved.info.width;
          int y1 = pixel_position / map_saved.info.width;
          interpolate(x0, y0, x1, y1);
          for (int i = 0; i<x_interpolation.size(); i++){
            if (y_init == 0){
              y_init = y_interpolation[0];
            }
            int pixel_position_temp = x_interpolation[i] + (map_saved.info.height - y_interpolation[i] + 2*(y_interpolation[i]-y_init) - 2) * map_saved.info.width;
            map_saved.data[pixel_position_temp] = -3;
          }
        }
      }
      ROS_INFO("Generating map file ...")
      fprintf(out, "P5\n# CREATOR: map_saver_trajectory.cpp %.3f m/pix\n%d %d\n255\n",
              map_saved.info.resolution, map_saved.info.width, map_saved.info.height);
      for(unsigned int y = 0; y < map_saved.info.height; y++) {
        for(unsigned int x = 0; x < map_saved.info.width; x++) {
          unsigned int i = x + (map_saved.info.height - y - 1) * map_saved.info.width;
          if (map_saved.data[i] == -2){ // trajectory pixel
            fputc(70, out);
            ROS_INFO("i=%d", i);
          }else if (map_saved.data[i] == -3){
            fputc(150, out);
            ROS_ERROR("interpolazione=%d", i);
          }else if (map_saved.data[i] >= 0 && map_saved.data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map_saved.data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }
      

      fclose(out);


      std::string mapmetadatafile = req.filename + ".yaml";
      ROS_INFO("Writing map occupancy data and trajectory to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      geometry_msgs::Quaternion orientation = map_saved.info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map_saved.info.resolution, map_saved.info.origin.position.x, map_saved.info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
      return true;
    }

    std::string mapname_;
    ros::Subscriber traj_sub;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--occ"))
    {
      if (++i < argc)
      {
        threshold_occupied = std::atoi(argv[i]);
        if (threshold_occupied < 1 || threshold_occupied > 100)
        {
          ROS_ERROR("threshold_occupied must be between 1 and 100");
          return 1;
        }

      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--free"))
    {
      if (++i < argc)
      {
        threshold_free = std::atoi(argv[i]);
        if (threshold_free < 0 || threshold_free > 100)
        {
          ROS_ERROR("threshold_free must be between 0 and 100");
          return 1;
        }

      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  if (threshold_occupied <= threshold_free)
  {
    ROS_ERROR("threshold_free must be smaller than threshold_occupied");
    return 1;
  }

  MapTrajectory mg(mapname, threshold_occupied, threshold_free);

  while(!mg.saved_map_ && ros::ok())
    ros::spin();

  return 0;
}