#include "ros/ros.h"

#include <string.h> //strlen()を使用する
#include <iostream> //入出力を使用する
#include <geometry_msgs/Twist.h> //ROSのgeometry_msgs/Twist型を使用する
#include <signal.h> //Ctrl-Cのシグナルを読み取って、ハンドラを起動するのに使用
#include <cstring> //strncmp()を使用する

#include "./test.hh"

using namespace std; //おまじない

void mysigintHandler(int sig){
    ROS_INFO("shutdown now.\n");
    ros::shutdown();
    exit(1);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "myteleop", ros::init_options::NoSigintHandler); //ノードの名前を定義, SigintHandler()が自動的に呼ばれるのを阻止
    ros::NodeHandle n;
    ros::Publisher msg_pub = n.advertise<geometry_msgs::Twist>("/hexarobo_s/diff_drive_controller/cmd_vel", 1000); //cmd_velというトピックにgeometry_msgs::Twist型のメッセージを送る
    ros::Rate loop_rate(10);

    signal(SIGINT, mysigintHandler);

    geometry_msgs::Twist twist;

    //初期値の設定
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    double value = 0.0;
    bool flag = false; //while()に1回目入る時の状態を記録
    char str[50]; //入力された文字列を格納する配列
    

    while(ros::ok()){
        if(!flag){
            ROS_INFO("start publish\n");
            flag = true;
        }
        
        cin >> str;
        cout << "you typewrite this : " << str << endl;
        cout << "your type count : " << strlen(str) << endl;

        if(strncmp(str, "rrr", 3)==0 && value<0.5){
            value += 0.02;
        }
        else if(strncmp(str, "bbb", 3)==0 && value >= 0.0){
            value -= 0.02;
        }

        twist.linear.x = value;
        msg_pub.publish(twist);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
