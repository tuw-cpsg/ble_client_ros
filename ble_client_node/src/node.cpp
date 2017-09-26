#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

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

void gattListen_cb(int size, const uint8_t *value)
{
    std_msgs::UInt8MultiArray msg;
    msg.data = std::vector<uint8_t>(value, value + size * sizeof(uint8_t));

    handle_pub.publish(msg);
}

void gattWriteCmd_cb(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    uint16_t *handle;
    uint8_t  *data;

    handle = (uint16_t*) &msg->data[0];
    data   = (uint8_t*)  &msg->data[2];

    if(ble_client != NULL && ble_client->isConnected() == true)
    {
        ble_client->gattWriteCmd(*handle, data, msg->data.size() - 2);
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

    ble_client = new BleClient(adapter.c_str(), device.c_str(), ADDR_TYPE);

    std::replace(device.begin(), device.end(), ':', '_');

    ros::Subscriber sub = n.subscribe("/ble_client_node/" + device + "/gattWriteCmd",
            1, gattWriteCmd_cb);

    if(!ble_client->connect())
    {
        ROS_ERROR("Failed to connect to BLE device.");
    }

    std::string listenToHandle;
    if(n.getParam("listenToHandle", listenToHandle))
    {
        uint16_t handle = strtoul(listenToHandle.c_str(), NULL, 0);
        handle_pub = n.advertise<std_msgs::UInt8MultiArray>(
                "/ble_client_node/" + device + "/" + listenToHandle, 1000);
        ROS_ERROR("%04X", handle);
        ble_client->gattListenToHandle(handle, gattListen_cb);
    }

    ros::spin();

    ble_client->disconnect();

    //TODO
    // delete ble_client

    return 0;
}
