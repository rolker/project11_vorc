#include "ros/ros.h"

#include "project11/gz4d_geo.h"
#include "hover/hoverAction.h"
#include "actionlib/server/simple_action_server.h"
#include "marine_msgs/NavEulerStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "std_msgs/String.h"
#include "dynamic_reconfigure/server.h"
#include "hover/hoverConfig.h"

class Hover
{
public:
    Hover(std::string const &name):
        m_action_server(m_node_handle, name, false),m_autonomous_state(false)
    {
        m_desired_heading_pub = m_node_handle.advertise<marine_msgs::NavEulerStamped>("/project11/desired_heading",1);
        m_desired_speed_pub = m_node_handle.advertise<geometry_msgs::TwistStamped>("/project11/desired_speed",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",5);
        
        m_position_sub = m_node_handle.subscribe("/position", 10, &Hover::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading",10, &Hover::headingCallback, this);

        m_state_sub = m_node_handle.subscribe("/project11/piloting_mode", 10, &Hover::stateCallback, this);

        dynamic_reconfigure::Server<hover::hoverConfig>::CallbackType f;
        f = boost::bind(&Hover::reconfigureCallback, this,  _1, _2);
        m_config_server.setCallback(f);

        
        m_action_server.registerGoalCallback(boost::bind(&Hover::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&Hover::preemptCallback, this));
        m_action_server.start();
    }
    
    ~Hover()
    {
    }
    
    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();
        
        m_target[0] = goal->target.latitude;
        m_target[1] = goal->target.longitude;
        sendDisplay();
    }
    
    void sendDisplay()
    {
        geographic_visualization_msgs::GeoVizItem vizItem;
        vizItem.id = "hover";
        if(m_action_server.isActive())
        {
            geographic_visualization_msgs::GeoVizPointList plist;
            geographic_msgs::GeoPoint gp;
            gp.latitude = m_target[0];
            gp.longitude = m_target[1];
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
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target,azimuth,m_maximum_distance);
                geographic_msgs::GeoPoint gp;
                gp.latitude = p[0];
                gp.longitude = p[1];
                polygon.outer.points.push_back(gp);
            }
            
            geographic_visualization_msgs::GeoVizSimplePolygon inner;
            // inner ring is clockwise
            for (double azimuth = 0.0; azimuth <= 360.0;  azimuth += 10.0)
            {
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target,azimuth,m_minimum_distance);
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

    void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
    {
        if(m_action_server.isActive() && m_autonomous_state)
        {
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> vehicle_position(inmsg->position.latitude, inmsg->position.longitude,0.0);
            std::pair<double,double> azimuth_distance_to_target = gz4d::geo::WGS84::Ellipsoid::inverse(vehicle_position, m_target);
            
            float target_speed = 0.0;
            float range = azimuth_distance_to_target.second;
            if (range >= m_maximum_distance)
                target_speed = m_maximum_speed;
            else if (range > m_minimum_distance)
            {
                float p = (range-m_minimum_distance)/(m_maximum_distance-m_minimum_distance);
                target_speed = p*m_maximum_speed;
            }

            hover::hoverFeedback feedback;
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
    
    void stateCallback(const std_msgs::String::ConstPtr &inmsg)
    {
        m_autonomous_state = inmsg->data == "autonomous";
        sendDisplay();
    }

    void reconfigureCallback(hover::hoverConfig &config, uint32_t level)
    {
        m_minimum_distance = config.minimum_distance;
        m_maximum_distance = config.maximum_distance;
        m_maximum_speed = config.maximum_speed;
        sendDisplay();
    }
    
private:
    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<hover::hoverAction> m_action_server;

    
    ros::Publisher m_desired_speed_pub;
    ros::Publisher m_desired_heading_pub;
    ros::Publisher m_display_pub;
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_state_sub;
    
    dynamic_reconfigure::Server<hover::hoverConfig> m_config_server;

    // goal variables
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> m_target;
    
    float m_minimum_distance; // meters
    float m_maximum_distance; // meters
    float m_maximum_speed;    // m/s
    bool m_autonomous_state;
    double m_heading;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover");

    Hover h("hover_action");
    
    ros::spin();
    
    return 0;
}
