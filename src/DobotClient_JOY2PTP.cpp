#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

#include "dobot/GetDeviceVersion.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/SetQueuedCmdStartExec.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPJumpParams.h"

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    double joy_x1 = msg->axes[3];
    double joy_y1 = msg->axes[4];
    double joy_x2 = msg->axes[0];
    double joy_y2 = msg->axes[1];

    ros::NodeHandle m;
    ros::ServiceClient client_sub;
    dobot::SetPTPCmd srv_s;

    client_sub = m.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");

    srv_s.request.ptpMode = 2;
    // srv_s.request.x = msg.position.x;
    // srv_s.request.y = msg.position.y;
    // srv_s.request.z = msg.position.z;
    // srv_s.request.r = msg.orientation.x;
    srv_s.request.x = (joy_y1 * 50) + 200;
    srv_s.request.y = (joy_x1 * 50);
    // srv_s.request.z = (joy_y2 + 1)*10;
    srv_s.request.z = -65.0;
    srv_s.request.r = 0;

    client_sub.call(srv_s);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "DobotClient_Topic");
    ros::NodeHandle n;
    ros::ServiceClient client;

    // SetCmdTimeout
    client =
        n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't "
                  "started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>(
        "/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>(
        "/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>(
        "/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion,
                 srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>(
        "/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    // srv5.request.xBias = 70;
    srv5.request.xBias = 0;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>(
            "/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>(
            "/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 3000;
        srv.request.xyzAcceleration = 2600;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        srv.request.isQueued = false;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>(
            "/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>(
            "/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 500;
        srv.request.accelerationRatio = 500;
        client.call(srv);
    } while (0);

    // client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    // dobot::SetPTPCmd srv;

    ros::Subscriber geom = n.subscribe("/joy", 1, joyCallback);
    ros::spin();
}
