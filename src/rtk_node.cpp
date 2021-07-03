#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <string>
#include <mutex>
#include <diagnostic_msgs/DiagnosticStatus.h>

extern "C"{
	#include "rtk/qxwz_rtcm.h"
}

#include "rtk/serial_open.h"
#define __NAME__ "rtk"

int fd_ = -1;
ros::Publisher pub_diagnostic_;
ros::Publisher pub_status_;
std::string rtk_status_;
diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
std::mutex diagnostic_msg_mutex_;
bool login = false;
double last_rtkdata_time = 0;

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

void parseRtkcode(qxwz_rtcm_status code)
{
    static std::string accountInfo = "Account";
    static std::string logInfo = "Log";
    static std::string dataInfo = "Data";
    
    std::lock_guard<std::mutex> lck(diagnostic_msg_mutex_);
    
    if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE)
    {
        diagnostic_msg_.level = diagnostic_msg_.WARN;
        diagnostic_msg_.message = "Account will expire!";
        
        get_qxwz_sdk_account_info();
        accountInfo = "Will Expire";
    }
    else if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED)
    {
        diagnostic_msg_.level = diagnostic_msg_.ERROR;
        diagnostic_msg_.message = "Account expired!";
        printf("账号到期\n");
        accountInfo = "Expired";
    }
    else if(code == QXWZ_STATUS_APPKEY_IDENTIFY_FAIL)
    {
        diagnostic_msg_.level = diagnostic_msg_.ERROR;
        diagnostic_msg_.message = "Identify fail!";
        
        printf("验证失败\n");
        logInfo = "Identify fail";
    }
    else if(code == QXWZ_STATUS_APPKEY_IDENTIFY_SUCCESS)
    {
        diagnostic_msg_.level = diagnostic_msg_.OK;
        diagnostic_msg_.message = "Identify success.";
        
        printf("验证成功\n");
        logInfo = "Identify succese";
    }
    else if(code == QXWZ_STATUS_GGA_SEND_NOT_AVAIABLE)
    {
        diagnostic_msg_.level = diagnostic_msg_.ERROR;
        diagnostic_msg_.message = "Illegal gpgga data!";
        printf("非法的GPGGA数据\n");
        dataInfo = "Error gpgga";
    }
    else if(code == QXWZ_STATUS_NTRIP_RECEIVING_DATA)
    {
        diagnostic_msg_.level = diagnostic_msg_.OK;
        diagnostic_msg_.message = "Data receiving...";
        printf("正在接收服务器数据\n");
        dataInfo = "Receiving";
    }
    else if(code == QXWZ_STATUS_NTRIP_CONNECTED)
    {
        diagnostic_msg_.level = diagnostic_msg_.OK;
        diagnostic_msg_.message = "Connected.";
        printf("连接服务器成功\n");
        logInfo = "Connected";
        login = true;
    }
    else if(code ==QXWZ_STATUS_NTRIP_DISCONNECTED)
    {
        diagnostic_msg_.level = diagnostic_msg_.ERROR;
        diagnostic_msg_.message = "Disconnect.";
        printf("服务器断开\n");
        logInfo = "Disconnect";
        login = false;
    }
    else if(code ==QXWZ_STATUS_NETWORK_ERROR)
    {
        diagnostic_msg_.level = diagnostic_msg_.ERROR;
        diagnostic_msg_.message = "Network error!";
        printf("网络异常\n");
        logInfo = "Network error";
        login = false;
    }
    else
    {
        diagnostic_msg_.level = diagnostic_msg_.WARN;
        diagnostic_msg_.message = std::string("Unknown code ") + std::to_string(code);
        printf("QXWZ_RTCM_STATUS:%d\n",code);
    }
    
    pub_diagnostic_.publish(diagnostic_msg_);
    
    std_msgs::String status_msg;
    status_msg.data = accountInfo + " | " + logInfo + " | " + dataInfo;
    pub_status_.publish(status_msg);
}

void qxwz_rtcm_response_callback(qxwz_rtcm data)
{
    printf("RTK is running...\t");
    printf("QXWZ_RTCM_DATA len:%ld\n",data.length);
    last_rtkdata_time = ros::Time::now().toSec();
    std::lock_guard<std::mutex> lck(diagnostic_msg_mutex_);
    
    diagnostic_msg_.level = diagnostic_msg_.OK;
    diagnostic_msg_.message = "Received rtk data.";
    pub_diagnostic_.publish(diagnostic_msg_);

    write(fd_, data.buffer, data.length);
}

void qxwz_status_response_callback(qxwz_rtcm_status code)
{
    parseRtkcode(code);
}

char * string2charPtr(const std::string& str)
{
    if(str.length() == 0)
        return NULL;
        
    char *ptr = new char[str.length()];
    strcpy(ptr, str.c_str());
}

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "rtk_node");

    ros::NodeHandle nh, nh_private("~");
    pub_status_ = nh.advertise<std_msgs::String>("/rtk_status",1);
    pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("/driverless/diagnostic",1);
    diagnostic_msg_.hardware_id = __NAME__;

    ros::Duration(3.0).sleep();
    fd_ = dev_open(nh_private.param<std::string>("port","/dev/ttyS3").c_str());
    if(fd_ == -1) return false;
    
    std::string appKey = nh_private.param<std::string>("appKey","");
    std::string appSecret = nh_private.param<std::string>("appSecret","");
    std::string deviceId  = nh_private.param<std::string>("deviceId","");
    std::string deviceType = nh_private.param<std::string>("deviceType","");
    
    //设置appKey和appSecret
    //apapKey申请详细见说明文档
    qxwz_config config;
    config.appkey = string2charPtr(appKey);
    config.appSecret = string2charPtr(appSecret);
    config.deviceId = string2charPtr(deviceId);
    config.deviceType = string2charPtr(deviceType);
    
//    std::cout << config.appkey << "  " << config.appSecret << std::endl;

    qxwz_setting(&config);
    qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);//启动rtcm sdk
    
    char gpgga_msg[200];
    ros::Rate loop_rate(1);
    int gpgga_timeout_cnt = 0; 
    last_rtkdata_time = ros::Time::now().toSec();
    while(ros::ok())
    {
        int len = read(fd_,gpgga_msg,199);
        if(len > 6)
        {
            gpgga_msg[len] = '\0';
            qxwz_rtcm_sendGGAWithGGAString(gpgga_msg);
            gpgga_timeout_cnt = 0;
        }

        if(len < 6 && gpgga_timeout_cnt++ > 3) //gpgga timeout
        {
            std::lock_guard<std::mutex> lck(diagnostic_msg_mutex_);
            diagnostic_msg_.level = diagnostic_msg_.ERROR;
            diagnostic_msg_.message = "No new gpgga data!";
            pub_diagnostic_.publish(diagnostic_msg_);
        }
        else if(!login)  //not login
        {
            std::lock_guard<std::mutex> lck(diagnostic_msg_mutex_);
            diagnostic_msg_.level = diagnostic_msg_.ERROR;
            diagnostic_msg_.message = "Disconnect!";
            pub_diagnostic_.publish(diagnostic_msg_);
        }
        else if(ros::Time::now().toSec() - last_rtkdata_time > 1.5) //login but data timeout
        {
            std::lock_guard<std::mutex> lck(diagnostic_msg_mutex_);
            diagnostic_msg_.level = diagnostic_msg_.ERROR;
            diagnostic_msg_.message = "Rtk data timeout!";
            pub_diagnostic_.publish(diagnostic_msg_);
        }

        loop_rate.sleep();
    }
    close(fd_);
    printf("qxwz_rtcm_stop here\r\n");
    qxwz_rtcm_stop();
    printf("qxwz_rtcm_stop done\r\n");
    
    
    return 0;
}


