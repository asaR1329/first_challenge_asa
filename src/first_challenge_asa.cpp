#include "first_challenge_asa/first_challenge_asa.h"
#include "math.h"
#include <tf/transform_listener.h>

FirstChallenge::FirstChallenge():private_nh_("~"){
    private_nh_.param("hz", hz_, {100});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback,this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr&msg){
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser_ = *msg;
}

void FirstChallenge::straight(){
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::turn(){
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0;
    cmd_vel_.cntl.angular.z = 0.7;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::stop(){
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0;
    cmd_vel_.cntl.angular.z = 0;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::show_odom(){
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" << odometry_.pose.pose.position.y << " z:" << odometry_.pose.pose.position.z << "\n" << std::endl;

    std::cout << "Quaternion" << ": x:" << odometry_.pose.pose.orientation.x << " y:" << odometry_.pose.pose.orientation.y << " z:" << odometry_.pose.pose.orientation.z << " w:" << odometry_.pose.pose.orientation.w << std::endl;
}

float FirstChallenge::show_scan(){
    float range_min = 1e6;
    for(int i=0; i<laser_.ranges.size(); i++){
        if(laser_.ranges[i] < range_min && laser_.ranges[i] > 0.350){   //35㎜~のみ
            range_min = laser_.ranges[i];
        }
    }
    std::cout << "scan: min:" << range_min << std::endl;
    return range_min;
}

void FirstChallenge::show_data(){
    show_odom();
    show_scan();
}

void FirstChallenge::process(){

    int count = 0;
    double yaw;
    float range_min;
    yaw = tf::getYaw(odometry_.pose.pose.orientation);

    ros::Rate loop_rate(hz_);

    while(ros::ok()){

        yaw = tf::getYaw(odometry_.pose.pose.orientation);
        show_odom();
        range_min = show_scan();

        if(odometry_.pose.pose.position.x <= 1.0){  //1mまで直進
            straight();
        }else{
            if(yaw >= 0 && count == 0){             //360°回転
                turn();
                if(yaw < 0) count = 1;
            }else{
                if(range_min >= 0.500){             //range_min<0.5mで停止
                    straight();
                }else{
                    stop();
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "first_challenge_asa");
    FirstChallenge first_challenge_asa;
    first_challenge_asa.process();
    ros::spin();
    return 0;
}

