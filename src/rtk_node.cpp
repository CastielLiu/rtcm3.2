#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <std_msgs/String.h>

extern "C"{
	#include "rtk/qxwz_rtcm.h"
}

#include "rtk/serial_open.h"
//#include "qxlog.h"

#undef QXLOGI
#define QXLOGI printf

//#define _QXWZ_TEST_START_STOP
int fd_rtcm;
std::string g_rtk_status;
int g_status_code;

qxwz_account_info *p_account_info = NULL;
void  get_qxwz_sdk_account_info(void);

//unsigned char buf_[] = "wendao\r\n";

void qxwz_rtcm_response_callback(qxwz_rtcm data){
    printf("RTK is running...\t");
    
    printf("QXWZ_RTCM_DATA len:%ld\n",data.length);
/*    printf("QXWZ_RTCM_DATA:%s\n",data.buffer);*/
    
    write(fd_rtcm, data.buffer, data.length);
}

void qxwz_status_response_callback(qxwz_rtcm_status code)
{
	static std::string accountInfo = "Account";
	static std::string logInfo = "Log";
	static std::string dataInfo = "Data";

	struct tm *ptr = NULL;
	//test account expire
	g_status_code = code;
	if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE)
	{
		get_qxwz_sdk_account_info();
		accountInfo = "Will Expire";
	}
	else if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED)
	{
		printf("账号到期\n");
		accountInfo = "Expired";
	}
	else if(code == QXWZ_STATUS_APPKEY_IDENTIFY_FAIL)
	{
		printf("验证失败\n");
		logInfo = "Identify fail";
	}
	else if(code == QXWZ_STATUS_APPKEY_IDENTIFY_SUCCESS)
	{
		printf("验证成功\n");
		logInfo = "Identify succese";
	}
	else if(code == QXWZ_STATUS_GGA_SEND_NOT_AVAIABLE)
	{
		printf("非法的GPGGA数据\n");
		dataInfo = "Error gpgga";
	}
	else if(code == QXWZ_STATUS_NTRIP_RECEIVING_DATA)
	{
		printf("正在接收服务器数据\n");
		dataInfo = "Receiving";
	}
	else if(code == QXWZ_STATUS_NTRIP_CONNECTED)
	{
		printf("连接服务器成功\n");
		logInfo = "Connected";
	}
	else if(code ==QXWZ_STATUS_NTRIP_DISCONNECTED)
	{
		printf("服务器断开\n");
		logInfo = "Disconnect";
	}
	else
	{
		QXLOGI("QXWZ_RTCM_STATUS:%d\n",code);
	}

	g_rtk_status =  accountInfo + " | " + logInfo + " | " + dataInfo;
}

void  get_qxwz_sdk_account_info(void)
{
	p_account_info = getqxwzAccount();
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


#ifdef _QXWZ_TEST_START_STOP
pthread_t qxwz_rtcm_test;
void test_qxwz_rtcm_start_stop(void);
#endif

char gpggaMsg[200];

int main(int argc, char * argv[]) 
{
	ros::init(argc, argv, "rtk_node");
	
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("/rtk_status",1);

	if(argc ==2)
		fd_rtcm = dev_open(argv[1]);
	else
		fd_rtcm = dev_open("/dev/ttyS4");
	if(fd_rtcm == -1)
		return 0;
		
    //设置appKey和appSecret
    //apapKey申请详细见说明文档
    qxwz_config config;
    //RTD
    config.appkey="A48g21q9e7if";
    config.appSecret="07f5cfb3f9a6299f";
    config.deviceId="little_ant";
    config.deviceType="novatel";

    qxwz_setting(&config);
    //启动rtcm sdk
    qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);

	#ifdef _QXWZ_TEST_START_STOP
    pthread_create(&qxwz_rtcm_test,NULL,test_qxwz_rtcm_start_stop,NULL);
	#endif
	
	std_msgs::String msg;
	
    //每秒发送gga以获取最新的rtcm数据流 
    while(ros::ok())
    {
    	usleep(980000);
		
		int len = read(fd_rtcm,gpggaMsg,199);
		
		if(len <= 6) continue;
			
		gpggaMsg[len] = '\0';
		
/*		printf("%s\n",gpggaMsg);*/
		
		qxwz_rtcm_sendGGAWithGGAString(gpggaMsg);
		
		msg.data = g_rtk_status;
		pub.publish(msg);
    }
    QXLOGI("qxwz_rtcm_stop here\r\n");
//    //关闭rtcm sdk
    qxwz_rtcm_stop();
    QXLOGI("qxwz_rtcm_stop done\r\n");
    
    return 0;
}

#ifdef _QXWZ_TEST_STARTSTOP
void test_qxwz_rtcm_start_stop(void)
{
	//sleep(2);
	qxwz_rtcm_stop();
    while(1)
    {
    	qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);
		sleep(50);
	    time_t time_stop_begin = time(NULL);
	    qxwz_rtcm_stop();
	    time_t time_stop_end = time(NULL);
	    QXLOGI("time_stop_begin:%d,time_stop_end:%d\n",time_stop_begin,time_stop_end);
		sleep(1);
    }
}
#endif
