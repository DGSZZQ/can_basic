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
#include "can_basic/ReceiveMsg.h"
#include "can_basic/SendMsg.h"
#include "can_basic/StimMsg.h"
#include <can_basic/controlcan.h>

// threads avails more parameters
typedef void * (*FUNCALLBACK)(void * arg1, void * arg2, void * arg3);
typedef struct{
   FUNCALLBACK callback;
   void * arg1;
   void * arg2;
   void * arg3;

} ARGS;
void * sub_thread_entry(void * arg)
{
    ARGS args;
    /* retrieve args */
    args.callback = ((ARGS *)arg)->callback;
    args.arg1 = ((ARGS *)arg)->arg1;
    args.arg2 = ((ARGS *)arg)->arg2;
    args.arg3 = ((ARGS *)arg)->arg3;
    free(arg);
    return args.callback(args.arg1, args.arg2, args.arg3);
}

int my_pthread_create(pthread_t *pthread, const pthread_attr_t *attr,
        FUNCALLBACK start_routine, void *arg1, void * arg2, void * arg3)
{
    ARGS * args;
    if (NULL == (args=(ARGS *)malloc(sizeof(ARGS))))     return -1;
    /* push args into buffer */
    args->callback = start_routine;
    args->arg1 = arg1;
    args->arg2 = arg2;
    args->arg3 = arg3;

    return pthread_create(pthread, attr, sub_thread_entry, args);
}

VCI_BOARD_INFO pInfo;//用来获取设备信息。
//int count = 0;
int sub_count = 0;

void CAN_init();
ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen=1 ,BYTE* data=new BYTE(1), BYTE externflag=1,\
               BYTE sendtype=0, BYTE remoteflag=0);
void Send_info_display(VCI_CAN_OBJ * send, int channel, int count);
void Receive_info_display(VCI_CAN_OBJ * rec, int channel, int reclen, int count);
void *receive_func(void* param, void* channel, void* Count);

//void chatterCallback(const can_basic::SendMsg::ConstPtr& msg)
//{
//    int channel = 1;
//    VCI_CAN_OBJ send[1];
//    BYTE * data = new BYTE(msg->len);
//    for (int i=0; i<msg->len; i++) data[i] = char(msg->data[i]);
//    if (Send_frame(send, channel, msg->ID, msg->len, data) == 1){
//        Send_info_display(send, channel, sub_count++);
//        ROS_INFO("CAN%d has sent a message, ID = %d", channel+1, msg->ID);
//    }
//}

class basic_can_control {
private:
    ros::NodeHandle m_node, m_privateNode;
    ros::Subscriber sub; // for can msg to send
    ros::Subscriber imu; // for imu msg
    void chatterCallback(const can_basic::SendMsg::ConstPtr& msg); // for can msg to send
    void chatterCallback_imu(const can_basic::StimMsg::ConstPtr& msg); // for can msg to send
public:
    BYTE data_FR_T[8];
    BYTE data_FL_T[8];
    float velocity;
    float delta_t;
    basic_can_control(const ros::NodeHandle &node, const ros::NodeHandle &prinode);
    ~basic_can_control() {}
    ULONG Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen=1 ,BYTE* data=new BYTE(1), BYTE externflag=1,\
                   BYTE sendtype=0, BYTE remoteflag=0);
    void Send_info_display(VCI_CAN_OBJ * send, int channel, int count);
};

basic_can_control::basic_can_control(const ros::NodeHandle &node, const ros::NodeHandle &prinode):
    m_node(node), m_privateNode(prinode) {
    velocity = 0;
    delta_t = 0.0005;
    BYTE x[8] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BYTE y[8] = {0x01, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
    for (int i=0; i<8; i++){
        data_FR_T[i] = x[i];
        data_FL_T[i] = y[i];
    }
    sub = m_node.subscribe("can_tosend", 1000, &basic_can_control::chatterCallback, this);
    imu = m_node.subscribe("stim", 1000, &basic_can_control::chatterCallback_imu, this);
}

void basic_can_control::chatterCallback(const can_basic::SendMsg::ConstPtr& msg){
    int channel = 1;
    VCI_CAN_OBJ send[1];
    BYTE * data = new BYTE(msg->len);
    for (int i=0; i<msg->len; i++) data[i] = char(msg->data[i]);
    if (this->Send_frame(send, channel, msg->ID, msg->len, data) == 1){
        Send_info_display(send, channel, sub_count++);
        ROS_INFO("CAN%d has sent a message, ID = %d", channel+1, msg->ID);
    }
}

void basic_can_control::chatterCallback_imu(const can_basic::StimMsg::ConstPtr& msg){
    velocity += msg->acc_x * delta_t;

}

ULONG basic_can_control::Send_frame(VCI_CAN_OBJ * send, int CAN_id, UINT id, BYTE datalen ,BYTE* data, BYTE externflag,\
                                    BYTE sendtype, BYTE remoteflag) {
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

void basic_can_control::Send_info_display(VCI_CAN_OBJ * send, int channel, int count){
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

main(int argc,  char **argv)
{

    ros::init(argc, argv, "can_decoder");
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("can_tosend", 1000, chatterCallback);
    ros::Rate loop_rate(10);
    basic_can_control Can_control(ros::NodeHandle(), ros::NodeHandle("~"));

    CAN_init();

    VCI_CAN_OBJ send[1];

    // receive thread
    int m_run0=1;
    int channel = 0;
    int count = 0;
    pthread_t threadid;


    int times = 2000;
    Can_control.data_FR_T[2] = 0x07;
    Can_control.data_FR_T[3] = 0x00;
    Can_control.data_FL_T[2] = 0xF8;
    Can_control.data_FL_T[3] = 0xFF;
    while(times--)
    {
        Can_control.Send_frame(send, 1, 0x0A510102, sizeof(Can_control.data_FR_T), Can_control.data_FR_T);
        Can_control.Send_frame(send, 1, 0x0A510101, sizeof(Can_control.data_FL_T), Can_control.data_FL_T);
    }

    while (ros::ok()){
        if(count==0)
            my_pthread_create(&threadid, NULL, receive_func, &m_run0, &channel, &count);

        int times = 100;
        Can_control.data_FR_T[2] = 0x04;
        Can_control.data_FR_T[3] = 0x00;
        Can_control.data_FL_T[2] = 0xFB;
        Can_control.data_FL_T[3] = 0xFF;
        while(times--)
        {
            Can_control.Send_frame(send, 1, 0x0A510102, sizeof(Can_control.data_FR_T), Can_control.data_FR_T);
            Can_control.Send_frame(send, 1, 0x0A510101, sizeof(Can_control.data_FL_T), Can_control.data_FL_T);
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

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1) {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    } else {
        printf(">>Init CAN1 success\n");
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1) {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    } else {
        printf(">>Start CAN1 success\n");
    }

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1) {
        printf(">>Init can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    } else {
        printf(">>Init CAN2 success\n");
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1) {
        printf(">>Start can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    } else {
        printf(">>Start CAN2 success\n");
    }


}

void *receive_func(void* param, void* channel, void* Count)  //接收线程。
{
    int reclen=0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;

    int *run=(int*)param;//线程启动，退出控制。
    int *ind=(int*)channel;
    int *count = (int*)Count;

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<can_basic::ReceiveMsg>("can_receive", 1000);

    while((*run)&0x0f)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,(*ind),rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            for(int k=0; k<reclen; k++){
                can_basic::ReceiveMsg recemsg;
                recemsg.ID = rec[k].ID;
                recemsg.channel = (*ind);
                recemsg.len = rec[k].DataLen;
                std::vector<uint16_t> data_rec(recemsg.len);
                for(int w=0; w<rec[k].DataLen; w++) {
                    data_rec[w] = uint16_t(rec[j].Data[w]);
                }
                recemsg.data = data_rec;
                pub.publish(recemsg);
                ROS_INFO("CAN%d has receive a message, ID = %d", recemsg.channel+1, recemsg.ID);
            }
            //Receive_info_display(rec, (*ind), reclen, *count);

            *count += reclen;
        }
    }
    printf("run thread exit\n");//退出接收线程
    pthread_exit(0);
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
