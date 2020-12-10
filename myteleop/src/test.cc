#include "ros/ros.h"

#include <string>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <signal.h>

#include "./test.hh"

using namespace std;

void mysigintHandler(int sig){
    ROS_INFO("shutdown now.\n");
    ros::shutdown();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "myteleop", ros::init_options::NoSigintHandler); //ノードの名前を定義
    ros::NodeHandle n;
    ros::Publisher msg_pub = n.advertise<geometry_msgs::Twist>("/hexarobo_s/diff_drive_controller/cmd_vel", 1000); //cmd_velというトピックにgeometry_msgs::Twist型のメッセージを送る
    ros::Rate loop_rate(10);

    signal(SIGINT, mysigintHandler);

    geometry_msgs::Twist twist;
    bool flag = false;
    char c;
    while(ros::ok()){
        if(!flag){
            ROS_INFO("start publish\n");
            flag = true;
        }
        cin >> c; //キーの読み込み
        cout << c << endl; //読み込んだキーを出力

        msg_pub.publish(twist);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}