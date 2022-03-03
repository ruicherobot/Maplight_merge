#include <maplite.h>
#define THETA -2.225
namespace maplite{


    MapliteClass::MapliteClass(ros::NodeHandle& n): n(n){

        odo_sub = n.subscribe("/cassie/pose", 10, &MapliteClass::update, this);
    
        std::cout<<"HERE!"<<std::endl;
        
    }

    void MapliteClass::update(geometry_msgs::PoseWithCovarianceStamped input)
    {   
        
        this->odox = input.pose.pose.position.x;
        this->odoy = input.pose.pose.position.y;
        this->odoz = input.pose.pose.position.z;
        std::cout<<"odox: "<<this->odox<<"  odoy: "<<this->odoy<<"  odoz: "<<this->odoz<<" \n";
        this->ox = input.pose.pose.orientation.x;
        this->oy = input.pose.pose.orientation.y;
        this->oz = input.pose.pose.orientation.z;
        this->ow = input.pose.pose.orientation.w;
        this->publishResult();
    }

    void MapliteClass::publishtf()
    {
        // tf
        geometry_msgs::TransformStamped mcn_trans;
        mcn_trans.header.stamp = ros::Time::now();
        mcn_trans.header.frame_id = "mcn_base";
        mcn_trans.child_frame_id = "mcn";

        mcn_trans.transform.translation.x = this->odox;
        mcn_trans.transform.translation.y = this->odoy;
        mcn_trans.transform.translation.z = this->odoz;
        mcn_trans.transform.rotation.w = this->ow;
        mcn_trans.transform.rotation.x = this->ox;
        mcn_trans.transform.rotation.y = this->oy;
        mcn_trans.transform.rotation.z = this->oz;

        mcn_broadcaster.sendTransform(mcn_trans);
    }

    void MapliteClass::publishResult(){
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        uint32_t shape = visualization_msgs::Marker::ARROW;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "maplite";
        marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = cos(THETA) * this->odox - sin(THETA) * this->odoy + 39;
        marker.pose.position.y = sin(THETA) * this->odox + cos(THETA) * this->odoy + 63;
        marker.pose.position.z = this->odoz;
        marker.pose.orientation.x = this->ow;
        marker.pose.orientation.y = this->ox;
        marker.pose.orientation.z = this->oy;
        marker.pose.orientation.w = this->oz;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 5;
        marker.scale.y = 5;
        marker.scale.z = 5;

    // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

    //marker.lifetime = ros::Duration();

        marker_pub.publish(marker);
        std::cout<<"PUBLISHED!!\n";

    }

    void MapliteClass::printWays(std::vector<OSM_WAY> & vec){

        std::cout << "Way count : " << vec.size() << std::endl;
        int i = 0;
        for(OSM_WAY w :vec){

            std::cout << "  Way [" << i++ << "] with size ["<< 
                    w.size()  << "] : " << std::endl;

            for(COORD c : w){
                std::cout << "      (" << std::setprecision(9)<<
                        c.y << ","   << std::setprecision(9) <<
                        c.x << ")\n";
            }

            std::cout<< "\n";
            return;
        }
    }

    void MapliteClass::localize(void){

        printWays(ways_);
        double epsilon = 5;

        for(int way_id = 0; way_id < ways_.size(); ++way_id){
            filtered_ways_.push_back
                (applyDouglasPeuker(way_id, epsilon, 0, ways_[way_id].size() -1));
                break;
        }

        printWays(filtered_ways_);
    }

    OSM_WAY MapliteClass::applyDouglasPeuker(uint way_id, double epsilon, uint start, uint end){

        double dmax = 0;
        uint index = 0;
        COORD st_coord  = ways_[way_id][start];
        COORD end_coord = ways_[way_id][end];

        //0. Find maximum distance point 
        for(int i = start+1; i < end; ++i){

            double d = perpendicularDist(ways_[way_id][i], st_coord, end_coord);

            if(d>dmax){
                dmax = d;
                index = i;
            }
        }
        //1. apply logic of epsilon 
        OSM_WAY    ans;

        //1(a) Distance is too far to create one line. Recurse
        if(dmax>epsilon){
            OSM_WAY recRes1 = applyDouglasPeuker(way_id, epsilon, start, index);
            OSM_WAY recRes2 = applyDouglasPeuker(way_id, epsilon, index, end);

            ans.insert(ans.end(), recRes1.begin(), recRes1.end());
            ans.pop_back();
            ans.insert(ans.end(), recRes2.begin(), recRes2.end());
        } 
        //2(b) epsilon is never met. Drop all in-between points
        else{

            ans = {st_coord, end_coord};
        }

        return ans;
    }

    double MapliteClass::perpendicularDist(COORD p, COORD start, COORD end){

        double m = ( end.y - start.y)/ (end.x - start.x);
        double b = end.y - m*start.x;

        // |Ax + By + c|/sqrt(A^2 + B^2)
        return abs(-p.y + m*p.x + b)/sqrt(1 + m*m);
    }

}