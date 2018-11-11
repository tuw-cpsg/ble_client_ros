#include <ros/ros.h>
#include <ble_client_cpp/ble_client.h>

#include <stdlib.h>
#include <glib.h>
#include <sstream>
#include <signal.h>
#include <sensor_msgs/Joy.h>

#define DEFAULT_ADAPTER "hci0"
#define ADDR_TYPE "random"

bool keepRunning = true;
BleClient *ble_client;

typedef struct {
    uint16_t thrust = 0;
    uint16_t roll = 1024;
    uint16_t pitch = 1024;
    uint16_t yaw = 1024;
    uint16_t arm = 0;
    uint16_t acro = 0;
} controlData;


controlData joydaten;
//yaw ,thrust ,roll ,pitch ,arm ,acro


void joyCallback(const sensor_msgs::Joy::ConstPtr& data){

    joydaten.yaw     = uint16_t((((-1*data->axes[0])+1))*1023.5);
    joydaten.thrust = 0;
    if (data->axes[1] >= 0.0){
        joydaten.thrust  = uint16_t((data->axes[1])*2047);    
    }
        
    joydaten.roll    = uint16_t((((-1*data->axes[3])+1))*1023.5);  
    joydaten.pitch   = uint16_t((data->axes[4]+1)*1023.5);

    joydaten.arm = uint16_t(data->buttons[4]*2047);
    joydaten.acro = uint16_t(data->buttons[5]*2047);
}

void sendData(const ros::TimerEvent&){
    if(ble_client != NULL && ble_client->isConnected() == true)
    {
        //ROS_INFO("%d %d %d %d %d %d",steuerdaten[0],steuerdaten[1],steuerdaten[2],steuerdaten[3],steuerdaten[4],steuerdaten[5]);
        ble_client->gattWriteCmd(0x0014, (const uint8_t *)(&joydaten),  sizeof(joydaten));
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


    ros::Subscriber joyDaten = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);

    if(!ble_client->connect())
    {
        ROS_ERROR("Failed to connect to BLE device.");
    }

    ros::Timer timer1 = n.createTimer(ros::Duration(0.011), sendData);
    ros::spin();

    ble_client->disconnect();

    //TODO
    // delete ble_client

    return 0;
}
