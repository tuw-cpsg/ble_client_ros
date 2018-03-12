#include <ros/ros.h>
#include <ble_client_msgs/HandleUInt8Array.h>
#include <ble_client_msgs/HandleUInt16Array.h>

#include <ble_client_cpp/ble_client.h>

#include <stdlib.h>
#include <glib.h>
#include <sstream>
#include <signal.h>

#define DEFAULT_ADAPTER "hci0"
#define ADDR_TYPE "random"

bool keepRunning = true;
BleClient *ble_client;

ros::Publisher handle_pub;

void gattNotificationListener_cb(const uint16_t handle, const int size, const uint8_t *value)
{
    ble_client_msgs::HandleUInt8Array msg;
    msg.handle = handle;
    msg.data   = std::vector<uint8_t>(value, value + size * sizeof(uint8_t));

    handle_pub.publish(msg);
}

void gattWriteCmdUInt8Array_cb(const ble_client_msgs::HandleUInt8Array::ConstPtr& msg)
{
    if(ble_client != NULL && ble_client->isConnected() == true)
    {
        ble_client->gattWriteCmd(msg->handle, (const uint8_t *)(msg->data.data()), msg->data.size() * sizeof(uint8_t));
    }
}

void gattWriteCmdUInt16Array_cb(const ble_client_msgs::HandleUInt16Array::ConstPtr& msg)
{
    if(ble_client != NULL && ble_client->isConnected() == true)
    {
        ble_client->gattWriteCmd(msg->handle, (const uint8_t *)(msg->data.data()), msg->data.size() * sizeof(uint16_t));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ble_client_node", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    std::string adapter;
    n.param("adapter", adapter, std::string(DEFAULT_ADAPTER));

    std::string device;
    if(!n.getParam("device", device))
    {
        ROS_ERROR("No device specified!");
        exit(-1);
    }

    std::replace(device.begin(), device.end(), '_', ':');
    ble_client = new BleClient(adapter.c_str(), device.c_str(), ADDR_TYPE);
    std::replace(device.begin(), device.end(), ':', '_');

    ros::Subscriber sub8 = n.subscribe("/ble_client_node/" + device + "/gattWriteCmd/UInt8Array",
            1, gattWriteCmdUInt8Array_cb);
    ros::Subscriber sub16 = n.subscribe("/ble_client_node/" + device + "/gattWriteCmd/UInt16Array",
            1, gattWriteCmdUInt16Array_cb);

    if(!ble_client->connect())
    {
        ROS_ERROR("Failed to connect to BLE device.");
    }

    bool listenToNotifications;
    if(n.getParam("listenToNotifications", listenToNotifications)
            && listenToNotifications == true)
    {
        //uint16_t handle = strtoul(listenToHandle.c_str(), NULL, 0);
        handle_pub = n.advertise<ble_client_msgs::HandleUInt8Array>(
                "/ble_client_node/" + device + "/notifications", 1000);
        ble_client->gattListenToNotification(gattNotificationListener_cb);
    }

    ros::spin();

    ble_client->disconnect();

    //TODO
    // delete ble_client

    return 0;
}
