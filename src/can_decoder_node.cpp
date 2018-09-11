#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <can_decoder/controlcan.h>

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。

void CAN_init();
ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen=1 ,BYTE* data=new BYTE(1), BYTE externflag=1,\
               BYTE sendtype=0, BYTE remoteflag=0);
void Send_info_display(VCI_CAN_OBJ * send, int channel, int count);
void Receive_info_display(VCI_CAN_OBJ * rec, int channel, int reclen, int count);
void *receive_func(void* param);

main()
{
    CAN_init();
    //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];

    int m_run0=1;
    pthread_t threadid;
    int ret;
    ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

    int times = 5;
    BYTE data[3] = {0x34, 0x45, 0xf3};
    while(times--)
    {
        if(Send_frame(send, 0, 123, sizeof(data), data) == 1)
        {
            Send_info_display(send, 0, count++);
            send[0].ID+=1;
        }
        else continue;

        if(Send_frame(send, 1, 123, sizeof(data), data) == 1)
        {
            Send_info_display(send, 0, count++);
            send[0].ID+=1;
        }
        else continue;
    }
    usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
    m_run0=0;//线程关闭指令。
    pthread_join(threadid,NULL);//等待线程关闭。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    //除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
    //goto ext;
    return -1;
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

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
    {
        printf(">>Init can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }
    if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
    {
        printf(">>Start can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);

    }
}

void *receive_func(void* param)  //接收线程。
{
    int reclen=0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;

    int *run=(int*)param;//线程启动，退出控制。
    int ind=0;

    while((*run)&0x0f)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            Receive_info_display(rec, ind, reclen, count);
            count += reclen;
        }
        ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。
    }
    printf("run thread exit\n");//退出接收线程
    pthread_exit(0);
}

ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen=1 ,BYTE* data=new BYTE(1), BYTE externflag=1,\
               BYTE sendtype=0, BYTE remoteflag=0){
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

void Receive_info_display(VCI_CAN_OBJ * rec, int channel, int reclen, int count){
    for(int j=0;j<reclen;j++)
    {
        printf("Index:%04d  ",count); count++;//序号递增
        printf("CAN%d RX ID:0x%08X", channel+1, rec[j].ID);//ID
        if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
        if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
        if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
        if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
        printf("DLC:0x%02X",rec[j].DataLen);//帧长度
        printf(" data:0x");	//数据
        for(int i = 0; i < rec[j].DataLen; i++)
        {
            printf(" %02X", rec[j].Data[i]);
        }
        printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
        printf("\n");
    }
}
