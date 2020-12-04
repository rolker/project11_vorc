#include "ros/ros.h"

#include "project11/gz4d_geo.h"
#include "dp_hover/dp_hoverAction.h"
#include "actionlib/server/simple_action_server.h"
#include "marine_msgs/NavEulerStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "nav_msgs/Odometry.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "std_msgs/String.h"
#include "dynamic_reconfigure/server.h"
#include "dp_hover/dp_hoverConfig.h"
#include "tf/tf.h"
#include "math.h"

#define PI 3.14159

class DP_Hover
{
public:
    DP_Hover(std::string const &name):
        m_action_server(m_node_handle, name, false),m_autonomous_state(false),lastOdomTime(0.0)
    {
        m_desired_heading_pub = m_node_handle.advertise<marine_msgs::NavEulerStamped>("/project11/desired_heading",1);
        m_desired_speed_pub = m_node_handle.advertise<geometry_msgs::TwistStamped>("/project11/desired_speed",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",5);
        m_desiredTwistCmd_pub = m_node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",1);

        /*
        m_position_sub = m_node_handle.subscribe("/position", 10, &DP_Hover::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading",10, &DP_Hover::headingCallback, this);
        */

        m_navstate_sub = m_node_handle.subscribe("/cora/robot_localization/odometry/filtered",10, &DP_Hover::navStateCallback, this);
        m_state_sub = m_node_handle.subscribe("/project11/piloting_mode", 10, &DP_Hover::stateCallback, this);

        dynamic_reconfigure::Server<dp_hover::dp_hoverConfig>::CallbackType f;
        f = boost::bind(&DP_Hover::reconfigureCallback, this,  _1, _2);
        m_config_server.setCallback(f);

        
        m_action_server.registerGoalCallback(boost::bind(&DP_Hover::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&DP_Hover::preemptCallback, this));
        m_action_server.start();
    }
    
    ~DP_Hover()
    {
    }

    void navStateCallback(const nav_msgs::Odometry::ConstPtr &inmsg) {


        //if(m_action_server.isActive() && m_autonomous_state) {
        if(m_action_server.isActive()) {


            double now = ros::Time::now().toSec();
            double timedelta = now - lastOdomTime;

            // Calculate relation of vehicle to hover point goal:
            double roll, pitch, yaw, yawerror, range, yaw_to_dp_point;
            double dx, dy;
            // Range:
            dx = m_target_position[0] - inmsg->pose.pose.position.x;
            dy = m_target_position[1] - inmsg->pose.pose.position.y;
            range = sqrt((dx * dx) + (dy * dy));

            // Yaw to hover point:
            yaw_to_dp_point = atan2(dy, dx);
            ROS_DEBUG("yaw to hover point: %0.9f",yaw_to_dp_point);

            // Yaw error
            tf::Quaternion q(
                    inmsg->pose.pose.orientation.x,
                    inmsg->pose.pose.orientation.y,
                    inmsg->pose.pose.orientation.z,
                    inmsg->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);

            m.getRPY(roll, pitch, yaw);

            yawerror = m_target_yaw-yaw;
            float yaw_to_dp_point_difference = yaw_to_dp_point - yaw;

            // Calculate commands to achive dp_hover.
            geometry_msgs::Twist cmd;

            // Set the time.
            //ros::Time now = ros::Time::now();
            //cmd.header.stamp = now;

            // Implements linear variation of speed between min and max distance.
            if (range >= m_maximum_distance)
            {
                cmd.linear.x = m_maximum_speed;
                cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
            }
            else if (range > m_minimum_distance)
            {
                float p = (range - m_minimum_distance)/(m_maximum_distance - m_minimum_distance);
                cmd.linear.x = p*m_maximum_speed;
                cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
            }
            if(fabs(yaw_to_dp_point_difference) > 2.5) // are we pointng backwards?
                cmd.linear.x = -cmd.linear.x;
            else if(fabs(yaw_to_dp_point_difference) > 0.2) // don't go if we are not pointing the right way
                cmd.linear.x = 0;
            
            // If we can get within some minimum distance, try to sit still and just adjust our heading
            if (range < m_minimum_distance) {
                cmd.linear.x = 0.0;
                cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed,float(yawerror)));
            }

            // Rate limit commands to keep Gazebo from crashing.
            if (timedelta > 0.2) {
                m_desiredTwistCmd_pub.publish(cmd);
                lastOdomTime = now;
                ROS_DEBUG("Sending Twist on /cmd_vel!");
            }
            // Send feedback.
            dp_hover::dp_hoverFeedback feedback;
            feedback.range = range;
            feedback.yawerror = yawerror;
            feedback.speed = cmd.linear.x;
            m_action_server.publishFeedback(feedback);
        }

    }
    /*
    void GeoPoseTORPHeading(geographic_msgs::GeoPoseStamped Gmsg, double *roll, double *pitch, double *heading) {

        tf::Quaternion q(
                Gmsg.pose.orientation.x,
                Gmsg.pose.orientation.y,
                Gmsg.pose.orientation.z,
                Gmsg.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double yaw;
        m.getRPY(*roll,*pitch,yaw);
        *heading =  fmod((PI/2.0 - yaw * PI/180.0) + 360.0, 360.0);
    }
     */



    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();
        m_target_position[0] = goal->target.pose.position.x;
        m_target_position[1] = goal->target.pose.position.y;

        tf::Quaternion q(
                goal->target.pose.orientation.x,
                goal->target.pose.orientation.y,
                goal->target.pose.orientation.z,
                goal->target.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);

        m_target_yaw = yaw;

        ROS_INFO("DP_HOVER GOAL: %0.9f,%0.9f  %0.3f", m_target_position[0],m_target_position[1],m_target_yaw);

        sendDisplay();
    }
    
    void sendDisplay()
    {
        geographic_visualization_msgs::GeoVizItem vizItem;
        vizItem.id = "DP_hover";
        if(m_action_server.isActive())
        {
            geographic_visualization_msgs::GeoVizPointList plist;
            geographic_msgs::GeoPoint gp;
            gp.latitude = m_target_position[0];
            gp.longitude = m_target_position[1];
            plist.points.push_back(gp);
            plist.size = 10;
            if (m_autonomous_state)
            {
                plist.color.r = .5;
                plist.color.g = .8;
                plist.color.b = .5;
                plist.color.a = 1.0;
            }
            else
            {
                plist.color.r = .2;
                plist.color.g = .3;
                plist.color.b = .2;
                plist.color.a = .5;
            }    
            vizItem.point_groups.push_back(plist);
            
            geographic_visualization_msgs::GeoVizPolygon polygon;
            // exterior ring is counter-clockwise
            for (double azimuth = 360.0; azimuth >= 0.0;  azimuth -= 10.0)
            {
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target_position,azimuth,m_maximum_distance);
                geographic_msgs::GeoPoint gp;
                gp.latitude = p[0];
                gp.longitude = p[1];
                polygon.outer.points.push_back(gp);
            }
            
            geographic_visualization_msgs::GeoVizSimplePolygon inner;
            // inner ring is clockwise
            for (double azimuth = 0.0; azimuth <= 360.0;  azimuth += 10.0)
            {
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target_position,azimuth,m_minimum_distance);
                geographic_msgs::GeoPoint gp;
                gp.latitude = p[0];
                gp.longitude = p[1];
                inner.points.push_back(gp);
            }
            polygon.inner.push_back(inner);
            
            if (m_autonomous_state)
            {
                polygon.fill_color.r = 0.0;
                polygon.fill_color.g = 1.0;
                polygon.fill_color.b = 0.0;
                polygon.fill_color.a = 0.5;
            }
            else
            {
                polygon.fill_color.r = 0.0;
                polygon.fill_color.g = 0.5;
                polygon.fill_color.b = 0.0;
                polygon.fill_color.a = 0.25;
            }
            
            polygon.edge_size = 2.0;
            
            if (m_autonomous_state)
            {
                polygon.edge_color.r = 0.0;
                polygon.edge_color.g = 0.0;
                polygon.edge_color.b = 1.0;
                polygon.edge_color.a = 0.75;
            }
            else
            {
                polygon.edge_color.r = 0.0;
                polygon.edge_color.g = 0.0;
                polygon.edge_color.b = 0.5;
                polygon.edge_color.a = 0.375;
            }
            
            vizItem.polygons.push_back(polygon);
        }
        m_display_pub.publish(vizItem);
    }
    
    void preemptCallback()
    {
        m_action_server.setPreempted();
        sendDisplay();
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& msg)
    {
        m_heading = msg->orientation.heading;
    }

    /*
    void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
    {
        if(m_action_server.isActive() && m_autonomous_state)
        {
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> vehicle_position(inmsg->position.latitude, inmsg->position.longitude,0.0);
            std::pair<double,double> azimuth_distance_to_target = gz4d::geo::WGS84::Ellipsoid::inverse(vehicle_position, m_target_position);
            
            float target_speed = 0.0;
            float range = azimuth_distance_to_target.second;
            if (range >= m_maximum_distance)
                target_speed = m_maximum_speed;
            else if (range > m_minimum_distance)
            {
                float p = (range-m_minimum_distance)/(m_maximum_distance-m_minimum_distance);
                target_speed = p*m_maximum_speed;
            }
            
            dp_hover::dp_hoverFeedback feedback;
            feedback.range = azimuth_distance_to_target.second;
            feedback.bearing = azimuth_distance_to_target.first;
            feedback.speed = target_speed;
            
            m_action_server.publishFeedback(feedback);
            
            ros::Time now = ros::Time::now();
            
            marine_msgs::NavEulerStamped desired_heading;
            desired_heading.header.stamp = now;
            desired_heading.orientation.heading = azimuth_distance_to_target.first;
            m_desired_heading_pub.publish(desired_heading);
            
            geometry_msgs::TwistStamped desired_speed;
            desired_speed.header.stamp = now;
            desired_speed.twist.linear.x = target_speed;
            m_desired_speed_pub.publish(desired_speed);
        }
    }
     */
    
    void stateCallback(const std_msgs::String::ConstPtr &inmsg)
    {
        m_autonomous_state = inmsg->data == "autonomous";
        sendDisplay();
    }

    void reconfigureCallback(dp_hover::dp_hoverConfig &config, uint32_t level)
    {
        m_minimum_distance = config.minimum_distance;
        m_maximum_distance = config.maximum_distance;
        m_maximum_speed = config.maximum_speed;
        m_maximum_angular_speed = config.maximum_angular_speed;
        sendDisplay();
    }
    
private:
    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<dp_hover::dp_hoverAction> m_action_server;

    
    ros::Publisher m_desired_speed_pub;
    ros::Publisher m_desired_heading_pub;
    ros::Publisher m_desiredTwistCmd_pub;
    ros::Publisher m_display_pub;
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_navstate_sub;

    double lastOdomTime;
    
    dynamic_reconfigure::Server<dp_hover::dp_hoverConfig> m_config_server;

    // goal variables
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> m_target_position;
    double m_target_yaw;
    
    float m_minimum_distance; // meters
    float m_maximum_distance; // meters
    float m_maximum_speed;    // m/s
    float m_maximum_angular_speed; // rad/s
    bool m_autonomous_state;

    double m_heading;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DP_hover");

    DP_Hover h("DP_hover_action");
    
    ros::spin();
    
    return 0;
}
