#include "planner_library/planner.h"

#include <iostream>

#define PI 3.14

using namespace std;
const double Ts = 0.01;

double influence_radius = 2.0;
bool local_minima = false;
Vector3d randomic_vec;
bool first_read = false;

PLAN::PLAN()
{
    // pubblishers and subscribers inizializzation
    // SUB
    _lidar_sub = _nh.subscribe("/laser/scan", 1, &PLAN::laser_callback, this);
    _odom_sub = _nh.subscribe("/iris/odometry", 1, &PLAN::odom_callback, this);
    _marker_sub = _nh.subscribe("/aruco_single/pose", 1, &PLAN::marker_cb, this);

    // PUB
    _desider_pos_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/desideratapos", 0);
    _desider_yaw_pub = _nh.advertise<std_msgs::Float64>("/iris/desideratayaw", 0);
    _desider_vel_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/desideratavel", 0);
    _desider_acc_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/desiderataacc", 0);
    _position_marker_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/position_marker", 0);
    _request_pub = _nh.advertise<std_msgs::String>("/iris/request", 0);

    if (_nh.hasParam("g_p")){_nh.getParam("g_p", gain_p);ROS_INFO_STREAM("g_p\t" << gain_p);}
    if (_nh.hasParam("g_d")){_nh.getParam("g_d", gain_d);ROS_INFO_STREAM("g_d\t" << gain_d);}

    // insert waypoints in a vector
    via_points.push_back(Vector3d(15.5, 0, -1.5));
    via_points.push_back(Vector3d(16, 5, -1));
    via_points.push_back(Vector3d(0.0, 4.7, -2));
    via_points.push_back(Vector3d(0.0, 11, -2));
    via_points.push_back(Vector3d(15.5, 9.0, -3));

    // landing point
    via_points.push_back(Vector3d(16, 10, -0.25));
    curr_via_point_index = 0;
}

// This function take the odometry from the subscriber (Gazebo)
// and do the necessary operations
void PLAN::odom_callback(nav_msgs::OdometryConstPtr odom)
{
    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q); // make rotation matrix from the quaternion

    Rb.getRPY(roll, pitch, yaw); // transform rotation matrix in roll, pitch and yaw angles

    current_pose.x = odom->pose.pose.position.x;
    current_pose.y = odom->pose.pose.position.y;
    current_pose.z = odom->pose.pose.position.z;

    current_velocity.x = odom->twist.twist.linear.x;
    current_velocity.y = odom->twist.twist.linear.y;
    current_velocity.z = odom->twist.twist.linear.z;
    if (!takeoff_started)
        first_read = true;
}

void PLAN::run()
{

    boost::thread planning(&PLAN::planning, this);

    ros::spin();
}

void PLAN::planning()
{
    ros::Rate r(100);
    TAKE_OFF = false;
    while (ros::ok())
    {
        if (!TAKE_OFF) // if not take-off is finished
        {
            takeoff();
            if (sqrt(pow(current_pose.x - pose_des.x, 2) + pow(current_pose.y - pose_des.y, 2) + pow(current_pose.z - pose_des.z, 2)) < 0.05 && takeoff_started)
                TAKE_OFF = true;
            via_point.x = via_points.at(curr_via_point_index)(0);
            via_point.y = via_points.at(curr_via_point_index)(1);
            via_point.z = via_points.at(curr_via_point_index)(2);
        }
        else
        {
            float distanza = sqrt(pow(via_point.x - current_pose.x, 2) + pow(via_point.y - current_pose.y, 2) + pow(via_point.z - current_pose.z, 2));
            
            if (!win_obstacle_pas && win_obstacle_appr) // if an empty obstacle was seen but the drone isn't in good position to pass and is approaching the window
            {
                // chek if the drone is close (less than 0.15m) the good point before the pass through the window and if the position of window was just been updated
                if (distanza < 0.15 && !next_aggiornamento) 
                {
                    win_obstacle_pas = true;
                    via_point.x = via_point_post_passaggio(0);
                    via_point.y = via_point_post_passaggio(1);
                    via_point.z = via_point_post_passaggio(2);
                }
            }
            // if the drone is good to pass in the window
            if (win_obstacle_appr && win_obstacle_pas) 
            {

                if (distanza < 0.05)
                {
                    via_point.x = via_points.at(curr_via_point_index)(0);
                    via_point.y = via_points.at(curr_via_point_index)(1);
                    via_point.z = via_points.at(curr_via_point_index)(2);
                    win_obstacle_pas = false;
                    win_obstacle_appr = false;
                    win_obstacle_appr_agg = false;
                }
            }
            go_to_point(via_point);
        }

        r.sleep();
    }
}

void PLAN::takeoff()
{
    if (first_read)
    {
        cout<< "TAKE OFF STARTED"<<endl;
        pose_des.x = current_pose.x;
        pose_des.y = current_pose.y;
        pose_des.z = -2.5;
        velocity_des.x = 0.0;
        velocity_des.y = 0.0;
        velocity_des.z = 0.0;
        acceleration_des.x = 0.0;
        acceleration_des.y = 0.0;
        acceleration_des.z = 0.0;
        yaw_des.data = 0.0;
        takeoff_started = true;
        first_read = false;
    }
    _desider_pos_pub.publish(pose_des);
    _desider_yaw_pub.publish(yaw_des);
    _desider_vel_pub.publish(velocity_des);
    _desider_acc_pub.publish(acceleration_des);
}

void PLAN::marker_cb(const geometry_msgs::PoseStamped &marker)
{
    Eigen::Matrix3d R_al2cl; // rotation matrix from camera_link and effective camera_link
    Eigen::Matrix3d _Rb;
    
    R_al2cl.row(0) << 0, 0, 1;
    R_al2cl.row(1) << -1, 0, 0;
    R_al2cl.row(2) << 0, -1, 0;

    Eigen::Vector3d al_pose; // position marker in a aruco's frame
    Eigen::Vector3d cl_pose; // position marker in camera's frame
    Eigen::Vector3d window_center_cl;
    Eigen::Vector3d via_point_pre_cl;
    Eigen::Vector3d via_point_post_cl;
    Eigen::Vector3d via_point_agg_cl;
    double roll_mar, pitch_mar, yaw_mar;
    float abs_distance;

    Eigen::Matrix3d R_marker = Eigen::Quaterniond(marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z).toRotationMatrix(); // costruisce matrice di rotazione a partire dal quaternione
    Eigen::Vector3d marker_rot_vect;

    tf::Quaternion q_mar(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
    tf::Matrix3x3 Rb_mar(q_mar); // make matrix rotation from quaternion

    Rb_mar.getRPY(roll_mar, pitch_mar, yaw_mar); // take marker roll, pitch and yaw

    al_pose(0) = marker.pose.position.x;
    al_pose(1) = marker.pose.position.y;
    al_pose(2) = marker.pose.position.z;
    cl_pose = R_al2cl * al_pose;

    _Rb << cos(pitch) * cos(yaw), sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw), cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw),
        cos(pitch) * sin(yaw), sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw), cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw),
        -sin(pitch), sin(roll) * cos(pitch), cos(roll) * cos(pitch);

    cl_pose(0) = cl_pose(0) / 1.1;       // calibration
    cl_pose(1) = cl_pose(1) / 1.1 + 0.1; // calibration
    cl_pose(2) = cl_pose(2);             // calibration

    window_center_cl(0) = cl_pose(0) + 0.1;
    window_center_cl(1) = cl_pose(1) - 0.6;
    window_center_cl(2) = cl_pose(2) + 0.6;

    via_point_agg_cl(0) = cl_pose(0) - 0.5;
    via_point_agg_cl(1) = cl_pose(1);
    via_point_agg_cl(2) = cl_pose(2) - 0.1;

    via_point_pre_cl(0) = window_center_cl(0) - 0.7;
    via_point_pre_cl(1) = window_center_cl(1);
    via_point_pre_cl(2) = window_center_cl(2);

    via_point_post_cl(0) = window_center_cl(0) + 0.4;
    via_point_post_cl(1) = window_center_cl(1);
    via_point_post_cl(2) = window_center_cl(2);

    // transform the points from camera's frame in a drone's frame
    cl_pose = _Rb.transpose() * cl_pose;
    via_point_agg_cl = _Rb.transpose() * via_point_agg_cl;
    window_center_cl = _Rb.transpose() * window_center_cl; 
    via_point_pre_cl = _Rb.transpose() * via_point_pre_cl;
    via_point_post_cl = _Rb.transpose() * via_point_post_cl;

    // transform the points from drone's frame in a world frame
    window_center(0) = current_pose.x + window_center_cl(0);
    window_center(1) = current_pose.y - window_center_cl(1);
    window_center(2) = current_pose.z - window_center_cl(2);

    via_point_pre_passaggio(0) = current_pose.x + via_point_pre_cl(0);
    via_point_pre_passaggio(1) = current_pose.y - via_point_pre_cl(1);
    via_point_pre_passaggio(2) = current_pose.z - via_point_pre_cl(2);

    via_point_aggiornamento(0) = current_pose.x + via_point_agg_cl(0);
    via_point_aggiornamento(1) = current_pose.y - via_point_agg_cl(1);
    via_point_aggiornamento(2) = current_pose.z - via_point_agg_cl(2);

    abs_distance = sqrt(pow(window_center(0) - current_pose.x, 2) + pow(window_center(1) - current_pose.y, 2)); // calcolo la distanza dal centro della finestra
    if (win_obstacle_appr && abs_distance < 1.3)                                                              // ho dovuto inserire questo if per fare in modo che il punto di passaggio dopo la finestra sia bloccato e non vede già un altro marker lontanno
    {
        via_point_post_passaggio(0) = current_pose.x + via_point_post_cl(0);
        via_point_post_passaggio(1) = current_pose.y - via_point_post_cl(1);
        via_point_post_passaggio(2) = current_pose.z - via_point_post_cl(2);
    }

    marker_rot = -roll_mar + yaw;

    if (marker_rot > M_PI)
    {
        marker_rot -= 2 * M_PI;
    }
    else if (marker_rot < -M_PI)
    {
        marker_rot += 2 * M_PI;
    }

    marker_pose.x = current_pose.x + cl_pose(0); // da sommare
    marker_pose.y = current_pose.y - cl_pose(1); // da sottrarre alla posizione attuale
    marker_pose.z = current_pose.z - cl_pose(2); // da sottrarre
    win_obstacle();

    _position_marker_pub.publish(marker_pose);
}

void PLAN::win_obstacle() // entro in questa funzione solo se sono entrato nella callback di aruco e quindi ho visto un marker
{
    float abs_distance;
    abs_distance = sqrt(pow(window_center(0) - current_pose.x, 2) + pow(window_center(1) - current_pose.y, 2) + pow(window_center(2) - current_pose.z, 2)); // calcolo la distanza dal centro della finestra
    if (abs_distance < 2.5 && !win_obstacle_appr)
    {

        if (!win_obstacle_appr_agg || (win_obstacle_appr_agg && sqrt(pow(via_point.x - via_point_aggiornamento(0), 2) + pow(via_point.y - via_point_aggiornamento(1), 2) + pow(via_point.z - via_point_aggiornamento(2), 2)) > 0.2)) // se la distanza dal marker è minore di 2.5 metri e non ho la variabile di approccio alla finestra vera allora è la prima volta che vedo la finestra e sono a meno di 3 metri
        {
            win_obstacle_appr_agg = true; // metto vera la variabile di approccio all'aggiornamento
            via_point.x = via_point_aggiornamento(0);
            via_point.y = via_point_aggiornamento(1);
            via_point.z = via_point_aggiornamento(2);
            // impose the desired yaw to make the drone parallel to the marker
            if (marker_rot < 0)
            {
                yaw_des.data = marker_rot + M_PI;
            }
            else
            {
                yaw_des.data = marker_rot - M_PI;
            } 
            // the next if is insert to solve the problem that occurs in some cases when the aruco package misread the marker's orientation
            if (abs(yaw_des.data - yaw) > M_PI_2 && !((yaw_des.data * yaw < 0) && (abs(yaw_des.data - yaw) > 3 * M_PI_2)))
            {
                yaw_des.data = yaw;
                win_obstacle_appr_agg = false;
            }

            next_aggiornamento = true; 
        }
    }
    abs_distance = sqrt(pow(via_point.x - current_pose.x, 2) + pow(via_point.y - current_pose.y, 2) + pow(via_point.z - current_pose.z, 2)); 
    if (abs_distance < 0.4 && !win_obstacle_pas && win_obstacle_appr_agg)                                                                 
        {
        win_obstacle_appr = true;
        next_aggiornamento = false;
        via_point.x = via_point_pre_passaggio(0);
        via_point.y = via_point_pre_passaggio(1);
        via_point.z = via_point_pre_passaggio(2);
        if (marker_rot < 0)
        {
            yaw_des.data = marker_rot + M_PI;
        }
        else
        {
            yaw_des.data = marker_rot - M_PI;
        }
    }
}

void PLAN::go_to_point(geometry_msgs::Vector3 goal_point)
{
    Eigen::Vector3d total_force, att_force, rep_force, q_goal, q_curr, q_dot_curr;
    geometry_msgs::Vector3 pose_des;
    float velocity_abs;
    float distanza = sqrt(pow(via_point.x - current_pose.x, 2) + pow(via_point.y - current_pose.y, 2) + pow(via_point.z - current_pose.z, 2));
    q_goal(0) = goal_point.x;
    q_goal(1) = goal_point.y;
    q_goal(2) = goal_point.z;

    q_curr(0) = current_pose.x;
    q_curr(1) = current_pose.y;
    q_curr(2) = current_pose.z;

    q_dot_curr(0) = current_velocity.x;
    q_dot_curr(1) = current_velocity.y;
    q_dot_curr(2) = current_velocity.z;

    att_force = attractive_force(q_goal, q_curr, q_dot_curr); // calculate attractive force

    if (win_obstacle_appr) // if the drone doing the approach to the window the repulsive force must be zero
    {
        rep_force = Eigen::MatrixXd::Zero(3, 1);
    }
    else if (distanza < 1 && min_dist > drone_radius) // if the drone is close to the target point but out of range of the drone, the repulsive force is reduced
    {
        rep_force = repulsive_force() * distanza;
    }
    else
    {
        rep_force = repulsive_force();
    }

    total_force = att_force - rep_force;
   
    if (!local_minima && total_force.norm() < 0.05 && distanza > 1) // if the drone is far to the goal position and the total force norm is small
    {
        
        randomic_vec = Vector3d::Random();
        total_force = randomic_vec;
        total_force(2) = 0;
        cout << "HO RISCONTRATO UN MINIMO LOCALE" << endl;
        local_minima = true;
    }
    else if (local_minima && total_force.norm() < 1)
    {
        total_force = randomic_vec;
    }
    else if (local_minima)
    {
        local_minima = false;
    }

    acceleration_des.x = total_force(0);
    acceleration_des.y = total_force(1);
    acceleration_des.z = total_force(2);

    velocity_des.x = current_velocity.x + Ts * acceleration_des.x;
    velocity_des.y = current_velocity.y + Ts * acceleration_des.y;
    velocity_des.z = current_velocity.z + Ts * acceleration_des.z;
 
    pose_des.x = current_pose.x + Ts * velocity_des.x;
    pose_des.y = current_pose.y + Ts * velocity_des.y;
    pose_des.z = current_pose.z + Ts * velocity_des.z;
    calcolo_yaw_des();

    if (_dest_near == true) // if the drone is close to target position the planner is turned off
    {
        acceleration_des.x = 0;
        acceleration_des.y = 0;
        acceleration_des.z = 0;
        velocity_des.x = 0;
        velocity_des.y = 0;
        velocity_des.z = 0;
        pose_des = goal_point;
    }

    if (distanza < 0.05 && !win_obstacle_appr) // quando arriva più vicino di 5 cm
    {
        cout << "Way point "<<curr_via_point_index + 1<<" raggiunto "<< endl;
        curr_via_point_index++;
        if (curr_via_point_index == via_points.size())
        {
            std_msgs::String msg;
            msg.data = "land";
            _request_pub.publish(msg);
        }
        else
        {
            via_point.x = via_points.at(curr_via_point_index)(0);
            via_point.y = via_points.at(curr_via_point_index)(1);
            via_point.z = via_points.at(curr_via_point_index)(2);
        }
    }

    _desider_pos_pub.publish(pose_des);
    _desider_yaw_pub.publish(yaw_des);
    _desider_vel_pub.publish(velocity_des);
    _desider_acc_pub.publish(acceleration_des);
}

// This function compute the attractive force
Eigen::Vector3d PLAN::attractive_force(const Eigen::Vector3d &q_goal, const Eigen::Vector3d &q_curr, const Eigen::Vector3d &q_dot_curr)
{
    Eigen::Vector3d ATT_F = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Vector3d velocity_error = Eigen::MatrixXd::Zero(3, 1);
    double yaw_err = 0;
    double dyaw_err = 0;

    position_error = q_goal - q_curr;
    velocity_error = -q_dot_curr;

    if (position_error.norm() <= 0.05)
    {
        _dest_near = true;
    }
    else
    {
        _dest_near = false;
    }

    if (position_error.norm() <= 1)
    {
        ATT_F(0) = gain_p * position_error(0) + gain_d * velocity_error(0);
        ATT_F(1) = gain_p * position_error(1) + gain_d * velocity_error(1);
        ATT_F(2) = gain_p * position_error(2) + gain_d * velocity_error(2);
    }
    else
    {
        ATT_F(0) = (double(gain_p) / position_error.norm()) * position_error(0) + gain_d * velocity_error(0);
        ATT_F(1) = (double(gain_p) / position_error.norm()) * position_error(1) + gain_d * velocity_error(1);
        ATT_F(2) = (double(gain_p) / position_error.norm()) * position_error(2) + gain_d * velocity_error(2);
    }

    return ATT_F;
}

// This function compute the repulsive force
Eigen::Vector3d PLAN::repulsive_force()
{
    float k_rep = 1;
    int gamma = 2;

    Eigen::Vector3d REP_F = Eigen::MatrixXd::Zero(3, 1);

    if (min_dist < influence_radius)
    {
        REP_F(0) = k_rep / pow(min_dist, 2) * pow((1 / min_dist - 1 / influence_radius), gamma - 1) * x_dist_near / (sqrt(pow(x_dist_near, 2) + pow(y_dist_near, 2)));
        REP_F(1) = k_rep / pow(min_dist, 2) * pow((1 / min_dist - 1 / influence_radius), gamma - 1) * y_dist_near / (sqrt(pow(x_dist_near, 2) + pow(y_dist_near, 2)));
    }

    return REP_F;
}

// This function is called when the laser scan acquires samples and searches for the closest obstacle point 
void PLAN::laser_callback(sensor_msgs::LaserScan laser)
{

    const int end_ind = int(floor(float(PI) / double(laser.angle_increment)));

    double angle;
    double x_dist;
    double y_dist;
    double dist;
    bool first_min = true;

    for (int i = 0; i < end_ind; i++)
    {
        angle = laser.angle_max - i * laser.angle_increment;
        x_dist = laser.ranges[i] * cos(angle + yaw);
        y_dist = laser.ranges[i] * sin(angle + yaw);
        dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
        if (first_min && dist > drone_radius)
        {
            min_dist = dist;
            first_min = false;
        }
        if (dist < min_dist && dist > drone_radius)
        {
            min_dist = dist;
            x_dist_near = x_dist;
            y_dist_near = y_dist;
        }
    }
}

// This function compute the desired yaw
void PLAN::calcolo_yaw_des()
{
    float temp;
    if (!win_obstacle_appr_agg && sqrt(pow(via_point.x - current_pose.x, 2) + pow(via_point.y - current_pose.y, 2)) > 0.15)
    {
        temp = atan2((via_point.y - current_pose.y), (via_point.x - current_pose.x));
        if (yaw_des.data > PI)
        {
            yaw_des.data -= 2 * PI;
        }
        else if (yaw_des.data < -PI)
        {
            yaw_des.data += 2 * PI;
        }

        // the following lines are used to have a soft rotation

        if (yaw_des.data > 0 && temp > 0)
        {
            if (abs(yaw_des.data - temp) > 0.2) // ciò l'ho fatto per non avere rotazioni brusche
            {
                if ((yaw_des.data - temp) < 0)
                {
                    yaw_des.data += 0.015;
                }
                else
                {
                    yaw_des.data -= 0.015;
                }
            }
        }
        else if (yaw_des.data < 0 && temp < 0)
        {
            if (abs(yaw_des.data - temp) > 0.2) // ciò l'ho fatto per non avere rotazioni brusche
            {
                if ((yaw_des.data - temp) < 0)
                {
                    yaw_des.data += 0.015;
                }
                else
                {
                    yaw_des.data -= 0.015;
                }
            }
        }
        else if (yaw_des.data > 0 && temp < 0)
        {
            if (abs(yaw_des.data - temp) > 0.2 && abs(yaw_des.data - temp) < (2 * PI - 0.2)) // ciò l'ho fatto per non avere rotazioni brusche
            {
                if (abs(yaw_des.data - temp) > PI)
                {

                    yaw_des.data += 0.015;
                }
                else
                {
                    yaw_des.data -= 0.015;
                }
            }
        }
        else if (yaw_des.data < 0 && temp > 0)
        {
            if (abs(yaw_des.data - temp) > 0.2 && abs(yaw_des.data - temp) < (2 * PI - 0.2)) // ciò l'ho fatto per non avere rotazioni brusche
            {
                if (abs(yaw_des.data - temp) > PI)
                {
                    yaw_des.data -= 0.015;
                }
                else
                {
                    yaw_des.data += 0.015;
                }
            }
        }
        else
        {
            yaw_des.data = temp;
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "planner");
    PLAN mav_pl;
    mav_pl.run();
    return 0;
}
