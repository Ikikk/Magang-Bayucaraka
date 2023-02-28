#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//callback function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh; 

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb); //tingkat pengulangan adalah 10 Hz.
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //point start
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send setpoints
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //set offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //send request to arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //set request time
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        //arming
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Define the four corners of the square and one corners to make a triangle
        geometry_msgs::PoseStamped square_pose[6];
        square_pose[0].pose.position.x = 0;
        square_pose[0].pose.position.y = 8;
        square_pose[0].pose.position.z = 2;

        square_pose[1].pose.position.x = 10;
        square_pose[1].pose.position.y = 8;
        square_pose[1].pose.position.z = 2;

        square_pose[2].pose.position.x = 10;
        square_pose[2].pose.position.y = 1;
        square_pose[2].pose.position.z = 2;

        square_pose[3].pose.position.x = 0;
        square_pose[3].pose.position.y = 1;
        square_pose[3].pose.position.z = 2;

        square_pose[4].pose.position.x = 5;
        square_pose[4].pose.position.y = 12;
        square_pose[4].pose.position.z = 2;

        square_pose[5].pose.position.x = 10;
        square_pose[5].pose.position.y = 8;
        square_pose[5].pose.position.z = 2;

        // Move the drone to each corner
        for (int i = 0; i < 6; i++) {
            local_pos_pub.publish(square_pose[i]);
            ros::spinOnce();
            rate.sleep();
        }

        ros::Duration(5.0).sleep();
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}