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
#include "robot_localization/ToLL.h"

#define PI 3.14159
#define FALSE 0
#define TRUE 1

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

        m_maptoLLSrvClient = m_node_handle.serviceClient<robot_localization::ToLL>("ToLL");

        
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


            ROS_DEBUG("yaw control: %d", m_yaw_control);
            
            double now = ros::Time::now().toSec();
            double timedelta = now - lastOdomTime;
            
            // Calculate relation of vehicle to hover point goal:
            double roll, pitch, yaw; 
            float yawerror, range, yaw_to_dp_point;
            float dx, dy, dR, dRdt, dxdt, yawrate;
            // Range:
            dx = m_target_position[0] - inmsg->pose.pose.position.x;
            dy = m_target_position[1] - inmsg->pose.pose.position.y;
            range = sqrt((dx * dx) + (dy * dy));
            if (m_lastRange != 0.0)
            {
                dR = range - m_lastRange;
            }
            else
            {
                dR = 0.0;
            }
            

            dRdt = dR / timedelta;
            dxdt = dx / timedelta;
                
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
            
            /*    def normalize_angle(self, x):
            ''' From Kalman and Baysian Filters in Python'''
            x = x % (2 * np.pi)    # force in range [0, 2 pi)
            x[x > np.pi] -= 2 * np.pi        # move to [-pi, pi)
            return x
            */
            
            
            yawerror = m_target_yaw-yaw;
            //yawerror = fmod(yawerror,2*PI);
            //if (yawerror > PI){yawerror -= (2*PI);}
                
            if (yawerror > PI) { yawerror = yawerror - 2*PI;}
            if (yawerror < -PI) { yawerror = yawerror + 2*PI;}
                
            float yaw_to_dp_point_difference = yaw_to_dp_point - yaw;
            if (yaw_to_dp_point_difference > PI) { yaw_to_dp_point_difference = yaw_to_dp_point_difference - 2*PI;}
            if (yaw_to_dp_point_difference < -PI) { yaw_to_dp_point_difference = yaw_to_dp_point_difference + 2*PI;}
            ROS_WARN("%0.3f,%0.3f",yaw_to_dp_point,yaw);
            // Normalize angle to +/- pi
            //yaw_to_dp_point_difference = fmod(yaw_to_dp_point_difference,2*PI);
            //if (yaw_to_dp_point_difference > PI){yaw_to_dp_point_difference -= (2*PI);}
                
            // Calculate commands to achive dp_hover.
            geometry_msgs::Twist cmd;

            // Set the time.
            //ros::Time now = ros::Time::now();
            //cmd.header.stamp = now;

            // Implements a range and time based strategy.
            // If you're far away...
            if (range >= m_maximum_distance)
            {
                m_node_handle.setParam("/simple_differential_controller/enableAngularPID",1);
                // ...First, take a few seconds to redirect the vehicle to point to the goal.
                // The proceed at maximum speed. 
                m_navTimer0 += timedelta;
                if (abs(yaw_to_dp_point_difference) > 0.1)
                {
                    cmd.linear.x = -0.3;
                    cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
                    ROS_WARN("DP Hover Start - Twist to Point R:%0.1f, dth: %0.2f", range,yaw_to_dp_point_difference);

                }else{
                    ROS_WARN("Go To Point at Speed R:%0.1f, dth: %0.2f", range,yaw_to_dp_point_difference);

                cmd.linear.x = m_maximum_speed;
                cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
                }
            }
            // Once inside the maximum_distnace but outside the minimum_distance, decrease
            // the speed of the vehicle linearlly with range to the minimum_distance. 
            // Don't drop speed to 0, as we want to continue our progress to the deisred 
            // hover point, proceeding slowly. 
            else if (range > m_minimum_distance)
            {
                ROS_WARN("Middle Circle Slowing. R:%0.1f, dth: %0.2f", range,yaw_to_dp_point_difference);
                if (abs(yaw_to_dp_point_difference) > 0.03)
                {
                    ROS_WARN("Twist to Point, x=0 R:%0.1f", range);
                    cmd.linear.x = .05;
                    //cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
                    cmd.angular.z = std::max(-float(1),std::min(float(1), yaw_to_dp_point_difference));

                    

                }else
                {
                    
                m_navTimer0 = 0.0;
                float p = (range - m_minimum_distance)/(m_maximum_distance - m_minimum_distance);
                cmd.linear.x = p*m_maximum_speed + .1;
                cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, 2*yaw_to_dp_point_difference));
            
                }
            } else if (range< m_minimum_distance & range > (m_minimum_distance / 4.0)) 
            
            
            // If we can get within 1/4 of the minimum_distance, we want our movements to be more 
            // precise. So first, slow, or stop the boat, and try to sit still and allow the heading
            // controller to adjust our heading alone. 
            // Note angular PID will be changed below for this same condition.
            
            {
                
                m_node_handle.setParam("/simple_differential_controller/enableAngularPID",0);

                // Setup for when we first enter this circle.
                if (m_lastRange > m_minimum_distance | m_lastRange < m_minimum_distance / 5.0){     
                    ROS_WARN("Reset Conditions...");
                    m_stopAndTwistConditionNotMet = TRUE;
                    m_twistToPointConditionNotMet = TRUE;
                }
                
                /// Here I'm trying to use the yaw rate to steer a bit more agressively mitigate the bearing rate.
                // this is not used at the moment.
                if (m_lastYawerror != 0.0) {
                     yawrate = (yaw_to_dp_point_difference - m_lastYawerror) / timedelta;
                }
                else
                {
                    yawrate = 0.0;
                }
                
                //ROS_DEBUG("%0.2f",delaytime);
                
                
                // The idea here is to momentarily set the speed to 0 when we get inside the inner most circle. 
                // Followed by a short burst of backward thrust, which has been shown to twist the boat in place.
                // This should focus all the control effort on adjusting the heading to twist the boat 
                // toward the point before continuing our approach.
                m_navTimer1 += timedelta;

                
                
                
                /*if (m_stopAndTwistConditionNotMet == TRUE){
                    ROS_WARN("Inner Circle Stop R:%0.1f", range);
                    cmd.linear.x = -.1;
                    m_node_handle.setParam("/simple_differential_controller/enableAngularPID",0);
                    //cmd.angular.z = 0.0;
                    // When these condtions met, re-enable yaw control...
                    if(inmsg->twist.twist.linear.x < 0 & inmsg->twist.twist.angular.z < .02)
                    {
                        m_stopAndTwistConditionNotMet = FALSE;
                        m_node_handle.setParam("/simple_differential_controller/enableAngularPID",1);

                    }
                }
                if (m_stopAndTwistConditionNotMet == FALSE & m_twistToPointConditionNotMet == TRUE)
                    */
                //if (m_twistToPointConditionNotMet == TRUE)
                if (abs(yaw_to_dp_point_difference) > .03)
                {
                    ROS_WARN("Inner Circle Twist R:%0.1f, dth: %0.2f", range,yaw_to_dp_point_difference); 

                    cmd.linear.x = -0.025; 
                    cmd.angular.z = std::max(-float(1),std::min(float(1), 2*yaw_to_dp_point_difference));
                    //cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, 2*yaw_to_dp_point_difference));

                    //if (abs(yaw_to_dp_point_difference) < 0.05) 
                    //{
                    //    m_twistToPointConditionNotMet = FALSE;
                    //}
                }else
                //if (m_stopAndTwistConditionNotMet == TRUE & m_twistToPointConditionNotMet == FALSE) 
                {
                    ROS_WARN("Inner Circle Proceed to Point R:%0.1f, dth: %0.2f", range,yaw_to_dp_point_difference);

                    //if (m_navTimer1 < 3){cmd.linear.x = 0;}else{cmd.linear.x =0.7;}
                    // Try to adjust the linear velocity down as you approach....
                    cmd.linear.x = std::max(float(0.1),float(range * 2./10.));
                    cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed, yaw_to_dp_point_difference));
                }
                
                
                
                m_lastYawerror = yaw_to_dp_point_difference;
            }else{
                m_navTimer1 = 0.0;
            }
            
            if (range < (m_minimum_distance/ 4.0)) 
            {
                
                ROS_WARN("HOVERING R:%0.1f, yawerr: %0.2f", range,yawerror);
                //m_node_handle.setParam("/simple_differential_controller/enableAngularPID",0);

                cmd.linear.x = 0;
                if(fabs(yaw_to_dp_point_difference) > 2.0 and range > 2.0) // are we pointng backwards?
                {
                    ROS_WARN("We're backwards, backing up! Honk Honk!");
                    cmd.linear.x = -.02;
                }
                else if(fabs(yaw_to_dp_point_difference) < 0.2 & range > 2.0) 
                {
                    cmd.linear.x = .02;
                }
            
                cmd.angular.z = std::max(float(-.01),std::min(float(0.01),yawerror));
                /*
                if (yawerror < 0){
                    cmd.angular.z = -.005;
                }else{
                    cmd.angular.z = 0.005;
                }
                */
            }
                /*
                // Setup for when we first enter this circle.
                if (m_lastRange > m_minimum_distance){                    
                    m_stopAndTwistConditionNotMet = TRUE;
                    m_twistToPointConditionNotMet = TRUE;
                }
                // Experimental. Try to estimate our drift and compensate for it. 
                //auto xDrift = dRdt * dx / (dR * dR);  // dRdt cos()
                //auto yDrift = dRdt * dy / (dR * dR);  // dRdt sin()
                
                //if (dxdt >= 0.0 and dRdt >= 0.0)
                //{
                //    cmd.linear.x = -dxdt;
                //}
                m_navTimer2 += timedelta;
                

                if(m_stopAndTwistConditionNotMet) {
                    cmd.linear.x = -0.2;
                    ROS_DEBUG("At Point. Remove Weigh.");
                    m_node_handle.setParam("/simple_differential_controller/enableAngularPID",0);

                    //cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed,float(0)));
                    if(inmsg->twist.twist.linear.x < 0 & inmsg->twist.twist.angular.z < .02)
                    {
                        m_stopAndTwistConditionNotMet = FALSE;
                        m_node_handle.setParam("/simple_differential_controller/enableAngularPID",1);

                    }  
                }else{
                    if (m_yaw_control == TRUE) {
                        ROS_DEBUG("Twist to Desired Heading");
                        if (abs(m_lastYawerror) != yawerror) { cmd.linear.x=-0.3;}
                        cmd.linear.x = 0.0;
                    }
                    else{
                        cmd.linear.x = -.1;   
                    }
                    cmd.linear.x = -3.0;
                }
                //cmd.angular.z = std::max(-m_maximum_angular_speed,std::min(m_maximum_angular_speed,float(yawerror)));
                cmd.angular.z = - (inmsg->twist.twist.linear.x - 1);

            }else{
                m_navTimer2 = 0.0;   
            }

            */
            
                /*
            if(fabs(yaw_to_dp_point_difference) > 2.5 & range < 2.0) // are we pointng backwards?
            {
                ROS_WARN("We're backwards, backing up!");
                cmd.linear.x = -cmd.linear.x;
            }
            else if(fabs(yaw_to_dp_point_difference) > 0.2 & range < m_minimum_distance) // don't go if we are not pointing the right way
            {
                cmd.linear.x = 0;
            }
            */
            
            // Rate limit commands to keep Gazebo from crashing.
            if (timedelta > 0.1) {
                m_desiredTwistCmd_pub.publish(cmd);
                lastOdomTime = now;
                ROS_DEBUG("Sending Twist on /cmd_vel!");
                //ROS_DEBUG("%0.3f, %0.3f", dRdt, yaw_to_dp_point);


                if (range < m_minimum_distance) {
                    //m_node_handle.setParam("/simple_differential_controller/angularPID_Kp", 100.0);
                } else {
                    //m_node_handle.setParam("/simple_differential_controller/angularPID_Kp",m_defaultAngularPID_Kp);
                }

            }
            
            m_lastRange = range;
            
            // Send feedback.
            dp_hover::dp_hoverFeedback feedback;
            feedback.range = range;
            feedback.yawerror = yawerror;
            feedback.speed = cmd.linear.x;
            //feedback.rangerate = dRdt;
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
        m_yaw_control = goal->yaw_control;
        
        // Get the default Kp PID value for heading so we can change it back later.
        m_node_handle.getParam("/simple_differential_controller/angularPID_Kp", m_defaultAngularPID_Kp);
        
        ROS_INFO("DP_HOVER GOAL: %0.2f,%0.2f  %0.2f", m_target_position[0],m_target_position[1],m_target_yaw);

        sendDisplay();
    }
    
    void sendDisplay()
    {
        geographic_visualization_msgs::GeoVizItem vizItem;
        vizItem.id = "DP_hover";
        
        //if (0){
        if(m_action_server.isActive())
        {
            geographic_visualization_msgs::GeoVizPointList plist;
            geographic_msgs::GeoPoint gp;
            
            robot_localization::ToLL maptoLL;
            maptoLL.request.map_point.x = m_target_position[0];
            maptoLL.request.map_point.y = m_target_position[1];
            maptoLL.request.map_point.z = 0.0;
            
            if (m_maptoLLSrvClient.call(maptoLL))
            {
                ROS_WARN("HERE.");
            }
            else
            {
             ROS_ERROR("Failed service call to robot_localization::ToLL.");   
            }
            
            
            //gp.latitude = m_target_position[0];
            //gp.longitude = m_target_position[1];
            plist.points.push_back(maptoLL.response.ll_point);
            plist.size = 10;
            //if (m_autonomous_state)
            //{
                plist.color.r = .5;
                plist.color.g = .8;
                plist.color.b = .5;
                plist.color.a = 1.0;
            /*}
            else
            {
                plist.color.r = .2;
                plist.color.g = .3;
                plist.color.b = .2;
                plist.color.a = .5;
            } */
            
            vizItem.point_groups.push_back(plist);
            
            geographic_visualization_msgs::GeoVizPolygon polygon;
            // exterior ring is counter-clockwise
            for (double azimuth = 360.0; azimuth >= 0.0;  azimuth -= 10.0)
            {
                
                maptoLL.request.map_point.x = m_maximum_distance * cos(azimuth * PI/180.0);
                maptoLL.request.map_point.y = m_maximum_distance * sin(azimuth * PI/180.0);
                maptoLL.request.map_point.z = 0.0;
                if (m_maptoLLSrvClient.call(maptoLL))
                {
                    polygon.outer.points.push_back(maptoLL.response.ll_point);
                }
                else
                {
                    ROS_ERROR("Failed service call to robot_localization::ToLL.");   
                }
                /*
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target_position,azimuth,m_maximum_distance);
                geographic_msgs::GeoPoint gp;
                gp.latitude = p[0];
                gp.longitude = p[1];
                polygon.outer.points.push_back(gp);
                */
            }
            
            geographic_visualization_msgs::GeoVizSimplePolygon inner;
            // inner ring is clockwise
            for (double azimuth = 0.0; azimuth <= 360.0;  azimuth += 10.0)
            {
                
                maptoLL.request.map_point.x = m_maximum_distance * cos(azimuth * PI/180.0);
                maptoLL.request.map_point.y = m_maximum_distance * sin(azimuth * PI/180.0);
                maptoLL.request.map_point.z = 0.0;
                
                if (m_maptoLLSrvClient.call(maptoLL))
                {
                    inner.points.push_back(maptoLL.response.ll_point);
                }else
                {
                    ROS_ERROR("Failed service call to robot_localization::ToLL.");   
                }
                /*
                auto p = gz4d::geo::WGS84::Ellipsoid::direct(m_target_position,azimuth,m_minimum_distance);
                geographic_msgs::GeoPoint gp;
                gp.latitude = p[0];
                gp.longitude = p[1];
                inner.points.push_back(gp);
                */
            }
            polygon.inner.push_back(inner);
            
            //if (m_autonomous_state)
            //{
                polygon.fill_color.r = 0.0;
                polygon.fill_color.g = 1.0;
                polygon.fill_color.b = 0.0;
                polygon.fill_color.a = 0.5;
            //}
            /*else
            {
                polygon.fill_color.r = 0.0;
                polygon.fill_color.g = 0.5;
                polygon.fill_color.b = 0.0;
                polygon.fill_color.a = 0.25;
            }
            */
            
            polygon.edge_size = 2.0;
            
            //if (m_autonomous_state)
            //{
                polygon.edge_color.r = 0.0;
                polygon.edge_color.g = 0.0;
                polygon.edge_color.b = 1.0;
                polygon.edge_color.a = 0.75;
            /*}
            else
            {
                polygon.edge_color.r = 0.0;
                polygon.edge_color.g = 0.0;
                polygon.edge_color.b = 0.5;
                polygon.edge_color.a = 0.375;
            }*/
            
            vizItem.polygons.push_back(polygon);
        }
        m_display_pub.publish(vizItem);
    }
    //}
    
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
    
    ros::ServiceClient m_maptoLLSrvClient;

    double lastOdomTime;
    
    dynamic_reconfigure::Server<dp_hover::dp_hoverConfig> m_config_server;

    // goal variables
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> m_target_position;
    double m_target_yaw;
    bool m_yaw_control = TRUE;   // Attempt to control yaw if set.
    
    float m_minimum_distance; // meters
    float m_maximum_distance; // meters
    float m_maximum_speed;    // m/s
    float m_maximum_angular_speed; // rad/s
    bool m_autonomous_state;

    double m_heading;
    float m_lastRange = 0.0;  // meter
    float m_lastYawerror = 0.0; // radians
    double m_navTimer0 = 0.0;
    double m_navTimer1 = 0.0;
    double m_navTimer2 = 0.0;
    bool m_stopAndTwistConditionNotMet = TRUE;
    bool m_twistToPointConditionNotMet = TRUE;

    double m_defaultAngularPID_Kp;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DP_hover");

    DP_Hover h("DP_hover_action");
    
    ros::spin();
    
    return 0;
}
