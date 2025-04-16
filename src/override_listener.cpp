#include <ros/ros.h>
#include <ros/message_event.h>
#include <std_msgs/Float32MultiArray.h>
#include <svan_simple_control_msgs/SvanCommand.h>

ros::Publisher svan_command_pub;
bool joystick_override = false;
void joystickCallback(const ros::MessageEvent<std_msgs::Float32MultiArray const>& event){
    const std::string& publisher_name = event.getPublisherName();
    if (publisher_name == "/joy_commands" && joystick_override == false){
        svan_simple_control_msgs::SvanCommand svan_command;
        svan_command.operation_mode = svan_simple_control_msgs::SvanCommand::COMMAND_JOYSTICK_OVERRIDE;
        svan_command_pub.publish(svan_command);
        ROS_INFO("Applying Joystick Override");
        joystick_override = true;
    }
}

int main(int argc, char **argv){

    ros::init(argc,argv,"svan_override_listener");
    ros::NodeHandle n;

    svan_command_pub = n.advertise<svan_simple_control_msgs::SvanCommand>("svan/simple_control",1);
    ros::Subscriber joystick_sub = n.subscribe<std_msgs::Float32MultiArray>("svan/joystick_data", 1, joystickCallback);


}