/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu), Michal
 * WEBSITE: https://www.BrucebotStudio.com/
 */

#pragma once 

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_space.h>
//#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <stdio.h>
#include <numeric>
#include <vector>
#include <deque>
#include <stack>
#include <tuple>
#include <cmath>
//reconfigure
#include <dynamic_reconfigure/server.h>
#include <osm_localization/osm_localizationConfig.h>


//ros and tf
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigen>

//messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
//my own
#include <osm_planner/osmWays.h>


#define METERS_PER_LAT 111078.9281974937 
#define METERS_PER_LON  82472.616449

namespace maplite {

    //! OSM Localization class
/*!
 * The localization on the osm map.
 * It's create a union of geographic (GPS) pose, cartesian (XY) pose and nearest point on the map
*/
    typedef struct coord{
       double y;
       double x;

    } COORD;
    

    typedef std::vector<COORD> OSM_WAY;



    class MapliteClass {

    public:

        MapliteClass(ros::NodeHandle& n);

        ros::NodeHandle& n;

        ros::Subscriber odo_sub;

        ros::Publisher marker_pub;

        tf::TransformBroadcaster mcn_broadcaster;

        void update(geometry_msgs::PoseWithCovarianceStamped input);
        
        void publishResult();

        void publishtf();


        void localize(void);

        //from [start, end]
        OSM_WAY applyDouglasPeuker(uint way_id, double epsilon, uint start, uint end);

        double perpendicularDist(COORD p, COORD start, COORD end);
        
        void printWays(std::vector<OSM_WAY> & vec);

    private:

        double lat_zero_;
        double lon_zero_;

        double odox;
        double odoy;
        double odoz;
        double ox;
        double oy;
        double oz;
        double ow;

        ros::ServiceClient osm_node_client_;

        std::vector<OSM_WAY> ways_;
        std::vector<OSM_WAY> filtered_ways_;

    };
}

