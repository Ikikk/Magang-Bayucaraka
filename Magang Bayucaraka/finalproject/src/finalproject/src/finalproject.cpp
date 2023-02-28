#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <stdio.h>
#include <fstream>
using namespace std;

float loc[3] = {0,0,3};
float waypoint[9][2] = {{0,0}, {-2,4}, {-6,6}, {-2,8}, {0,12}, {2,8}, {6,6}, {2,4}, {0,0}};

//callback function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped beforePos;
void pos_cb(geometry_msgs::PoseStamped posSent) {
    beforePos.pose.position.x=posSent.pose.position.x;
    beforePos.pose.position.y=posSent.pose.position.y;
    beforePos.pose.position.z=posSent.pose.position.z;
}

geometry_msgs::PoseStamped posenow;
int stabil(int x, int y, int z) {
    float miss=0.1;
    if(posenow.pose.position.x>loc[x]-miss&&posenow.pose.position.x<loc[x]+miss&&
        posenow.pose.position.y>loc[y]-miss&&posenow.pose.position.y<loc[y]+miss&&
        posenow.pose.position.z>loc[z]-miss&&posenow.pose.position.z<loc[z]+miss)
        return 1;
    return -1;
}

int main(int argc, char **argv)
{
    int way = 0;
    // int num;
    // FILE* pipe = popen("python example.py", {"-r"});
    // if (!pipe) {
    //     fprintf(stderr, "Failed to run Python script\n");
    //     return 1;
    // }
    // fscanf(pipe, "%d", &num);
    // pclose(pipe);

    ros::init(argc, argv, "finalproject");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = loc[0];
    pose.pose.position.y = loc[1];
    pose.pose.position.z = loc[2];

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if ( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
    } else {

        if (stabil(0,1,2) && way<9 && 
            (ros::Time::now() - last_request > ros::Duration(20.0))) 
            {
            loc[0]+=waypoint[way][0];
            loc[1]+=waypoint[way][1];
                pose.pose.position.x=loc[0];
                pose.pose.position.y=loc[1];
                for(int i = 100; ros::ok() && i > 0; --i){
                    local_pos_pub.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                }

                way++;
                ROS_INFO("Mission %d", way);
        } else if (way  == 9) {
            ROS_INFO("Mission complete");
            return 0;
        }

    }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}