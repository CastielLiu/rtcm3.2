#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

extern "C"{
	#include "rtk/qxwz_rtcm.h"
}

#include "rtk/serial_open.h"
#define __NAME__ "rtk"

int fd_ = -1;
bool new_rtk_code_ = false;
qxwz_rtcm_status rtk_code_ = QXWZ_STATUS_NONE;

void  get_qxwz_sdk_account_info(void)
{
    qxwz_account_info *p_account_info = getqxwzAccount();
    if(p_account_info->appkey != NULL) {
	    printf("appkey=%s\n",p_account_info->appkey);
    }
    if(p_account_info->deviceID != NULL) {
	    printf("deviceID=%s\n",p_account_info->deviceID);
    }
    if(p_account_info->deviceType != NULL) {
	    printf("deviceType=%s\n",p_account_info->deviceType);
    }

    if(p_account_info->NtripUserName != NULL) {
	    printf("NtripUserName=%s\n",p_account_info->NtripUserName);
    }
    if(p_account_info->NtripPassword != NULL) {
	    printf("NtripPassword=%s\n",p_account_info->NtripPassword);
    }
    printf("账号即将到期 expire time : %ld\n",p_account_info->expire_time);
}

void qxwz_rtcm_response_callback(qxwz_rtcm data)
{
    printf("RTK is running...\t");
    printf("QXWZ_RTCM_DATA len:%ld\n",data.length);
    write(fd_, data.buffer, data.length);
}

void qxwz_status_response_callback(qxwz_rtcm_status code)
{
    rtk_code_ = code;
    new_rtk_code_ = true;
}

class RtkNode
{
public:
    RtkNode(){}
    bool init(const std::string& port_name)
    {
        ros::NodeHandle nh;
        pub_status_ = nh.advertise<std_msgs::String>("/rtk_status",1);
        pub_diagnostic_ = nh.advertise<std_msgs::String>("/sensors/diagnostic",1);
        timer_ = nh.createTimer(ros::Duration(0.1), &RtkNode::timerCallback, this);
        diagnostic_msg_.hardware_id = __NAME__;
        
        fd_ = dev_open(port_name.c_str());
        if(fd_ == -1)
            return false;
        
        //设置appKey和appSecret
        //apapKey申请详细见说明文档
        qxwz_config config;
        config.appkey="A48g21q9e7if";
        config.appSecret="07f5cfb3f9a6299f";
        config.deviceId="little_ant";
        config.deviceType="novatel";

        qxwz_setting(&config);
        
        return true;
    }
    
    void parseRtkcode()
    {
        new_rtk_code_ = false;
        static std::string accountInfo = "Account";
        static std::string logInfo = "Log";
        static std::string dataInfo = "Data";
        //test account expire
        if(rtk_code_ == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE)
        {
	        get_qxwz_sdk_account_info();
	        accountInfo = "Will Expire";
        }
        else if(rtk_code_ == QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED)
        {
            diagnostic_msg_.level = diagnostic_msg_.ERROR;
            diagnostic_msg_.message = "Account expired!";
	        printf("账号到期\n");
	        accountInfo = "Expired";
        }
        else if(rtk_code_ == QXWZ_STATUS_APPKEY_IDENTIFY_FAIL)
        {
	        printf("验证失败\n");
	        logInfo = "Identify fail";
        }
        else if(rtk_code_ == QXWZ_STATUS_APPKEY_IDENTIFY_SUCCESS)
        {
	        printf("验证成功\n");
	        logInfo = "Identify succese";
        }
        else if(rtk_code_ == QXWZ_STATUS_GGA_SEND_NOT_AVAIABLE)
        {
	        printf("非法的GPGGA数据\n");
	        dataInfo = "Error gpgga";
        }
        else if(rtk_code_ == QXWZ_STATUS_NTRIP_RECEIVING_DATA)
        {
	        printf("正在接收服务器数据\n");
	        dataInfo = "Receiving";
        }
        else if(rtk_code_ == QXWZ_STATUS_NTRIP_CONNECTED)
        {
	        printf("连接服务器成功\n");
	        logInfo = "Connected";
        }
        else if(rtk_code_ ==QXWZ_STATUS_NTRIP_DISCONNECTED)
        {
	        printf("服务器断开\n");
	        logInfo = "Disconnect";
        }
        else if(rtk_code_ ==QXWZ_STATUS_NETWORK_ERROR)
        {
	        printf("网络异常\n");
	        logInfo = "Disconnect";
        }
        else
        {
	        printf("QXWZ_RTCM_STATUS:%d\n",rtk_code_);
        }
        
        std_msgs::String status_msg;
        status_msg.data = accountInfo + " | " + logInfo + " | " + dataInfo;
        pub_status_.publish(status_msg);
    }
    
    void timerCallback(const ros::TimerEvent& e)
    {
        static double last_time = 0;
        static char gpgga_msg[200];
        double now_time = ros::Time::now().toSec();
        if(new_rtk_code_) 
            parseRtkcode();
        
        if(now_time-last_time < 1.0)
            return;
        
        int len = read(fd_,gpgga_msg,199);
		
	    //if(len <= 6) return;
		
	    gpgga_msg[len] = '\0';
	    
	    //qxwz_rtcm_sendGGAWithGGAString(gpgga_msg);
	    qxwz_rtcm_sendGGAWithGGAString("$GPGGA,014434.70,3817.13334637,N,12139.72994196,E,4,07,1.5,6.571,M,8.942,M,0.7,0016*7B\n");
    }
    
    
    void run()
    {
        qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);//启动rtcm sdk
        
        ros::spin();
        printf("qxwz_rtcm_stop here\r\n");
        qxwz_rtcm_stop();
        printf("qxwz_rtcm_stop done\r\n");
        
    }
    
private:
    ros::Publisher pub_diagnostic_;
    ros::Publisher pub_status_;
    int fd_;
    std::string rtk_status_;
    int rtk_status_code_;
    diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
    ros::Timer timer_;
};



int main(int argc, char * argv[]) 
{
    if(argc < 2)
    {
        std::cout << "No Port Name!" << std::endl;
        return 0;
    }
    ros::init(argc, argv, "rtk_node");

    RtkNode rtk;
    
    if(!rtk.init(argv[1])) return 0;
    rtk.run();
    
    return 0;
}


