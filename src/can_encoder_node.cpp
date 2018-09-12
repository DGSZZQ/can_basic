#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "ros/ros.h"
#include "can_basic/SendMsg.h"

#include <can_basic/controlcan.h>

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count = 0;
int channel = 0;

void CAN_init();
ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen=1 ,BYTE* data=new BYTE(1), BYTE externflag=1,\
               BYTE sendtype=0, BYTE remoteflag=0);
void Send_info_display(VCI_CAN_OBJ * send, int channel, int count);

void chatterCallback(const can_basic::SendMsg::ConstPtr& msg)
{
    VCI_CAN_OBJ send[1];
    BYTE * data = new BYTE(msg->len);
    for (int i=0; i<msg->len; i++) data[i] = char(msg->data[i]);
    if (Send_frame(send, channel, msg->ID, msg->len, data) == 1){
        Send_info_display(send, channel, count++);
        ROS_INFO("Send a message, ID = %d", msg->ID);
    }
}

main(int argc,  char **argv)
{

    ros::init(argc, argv, "can_encoder");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("can_tosend", 1000, chatterCallback);
    ros::Rate loop_rate(10);

    CAN_init();

    VCI_CAN_OBJ send[1];

    // receive thread
    int m_run0=1;
    int channel = 0;
    int count = 0;

    while (ros::ok()){

        int times = 5;
        BYTE data[3] = {0x34, 0x45, 0xf3};
        while(times--)
        {
            if(Send_frame(send, 1, 123, sizeof(data), data) == 1)
            {
                Send_info_display(send, 1, count++);
                send[0].ID+=1;
            }
            else continue;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

void CAN_init(){

    printf(">>this is hello !\r\n");//指示程序已运行

    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error!\n");
        //exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
    config.Timing1=0x1C;
    config.Mode=0;//正常模式


}

ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen ,BYTE* data, BYTE externflag,\
               BYTE sendtype, BYTE remoteflag){
   //VCI_CAN_OBJ send[1];
   send->ID=id;
   send->SendType=sendtype;
   send->RemoteFlag=remoteflag;
   send->ExternFlag=externflag;
   send->DataLen=datalen;

   int i=0;
   for(i=0; i<datalen; i++){
       send->Data[i] = data[i];
   }
   return VCI_Transmit(VCI_USBCAN2, 0, CAN_id, send, 1);
}

void Send_info_display(VCI_CAN_OBJ * send, int channel, int count){
    printf("Index:%04d  ",count);
    printf("CAN%d TX ID:0x%08X", channel+1, send[0].ID);
    if(send[0].ExternFlag==0) printf(" Standard ");
    if(send[0].ExternFlag==1) printf(" Extend   ");
    if(send[0].RemoteFlag==0) printf(" Data   ");
    if(send[0].RemoteFlag==1) printf(" Remote ");
    printf("DLC:0x%02X",send[0].DataLen);
    printf(" data:0x");

    for(int i=0;i<send[0].DataLen;i++)
    {
        printf(" %02X",send[0].Data[i]);
    }
    printf("\n");
}

