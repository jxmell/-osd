#pragma once

#include <stdint.h>
#include <time.h>
#include "osdconfig.h"

typedef struct {
        uint32_t received_packet_cnt;
        uint32_t wrong_crc_cnt;
        int8_t current_signal_dbm;
	int8_t type;
	int signal_good;
} wifi_adapter_rx_status_t;

typedef struct {
        uint32_t received_packet_cnt;
        uint32_t wrong_crc_cnt;
        int8_t current_signal_dbm;
	int8_t type;
	int signal_good;
} wifi_adapter_rx_status_t_osd;

typedef struct {
        uint32_t received_packet_cnt;
        uint32_t wrong_crc_cnt;
        int8_t current_signal_dbm;
	int8_t type;
	int signal_good;
} wifi_adapter_rx_status_t_uplink;

typedef struct {
        time_t last_update;
        uint32_t received_block_cnt;
        uint32_t damaged_block_cnt;
	uint32_t lost_packet_cnt;
	uint32_t received_packet_cnt;
	uint32_t lost_per_block_cnt;
        uint32_t tx_restart_cnt;
	uint32_t kbitrate;
        uint32_t wifi_adapter_cnt;
        wifi_adapter_rx_status_t adapter[8];
} wifibroadcast_rx_status_t;

typedef struct {
        time_t last_update;
        uint32_t received_block_cnt;
        uint32_t damaged_block_cnt;
	uint32_t lost_packet_cnt;
	uint32_t received_packet_cnt;
	uint32_t lost_per_block_cnt;
        uint32_t tx_restart_cnt;
	uint32_t kbitrate;
        uint32_t wifi_adapter_cnt;
        wifi_adapter_rx_status_t adapter[8];
} wifibroadcast_rx_status_t_osd;

typedef struct {
        time_t last_update;
        uint32_t received_block_cnt;
        uint32_t damaged_block_cnt;
	uint32_t lost_packet_cnt;
	uint32_t received_packet_cnt;
	uint32_t lost_per_block_cnt;
        uint32_t tx_restart_cnt;
	uint32_t kbitrate;
        uint32_t wifi_adapter_cnt;
        wifi_adapter_rx_status_t adapter[8];
} wifibroadcast_rx_status_t_rc;

typedef struct {
        time_t last_update;
        uint32_t received_block_cnt;
        uint32_t damaged_block_cnt;
	uint32_t lost_packet_cnt;
	uint32_t received_packet_cnt;
	uint32_t lost_per_block_cnt;
        uint32_t tx_restart_cnt;
	uint32_t kbitrate;
        uint32_t wifi_adapter_cnt;
        wifi_adapter_rx_status_t adapter[8];
} wifibroadcast_rx_status_t_uplink;

typedef struct {
    time_t last_update;
    uint8_t cpuload;
    uint8_t temp;
    uint32_t injected_block_cnt;
    uint32_t skipped_fec_cnt;
    uint32_t injection_fail_cnt;
    long long injection_time_block;
    uint16_t bitrate_kbit;
    uint16_t bitrate_measured_kbit;
    uint8_t cts;
    uint8_t undervolt;
} wifibroadcast_rx_status_t_sysair;


typedef struct {
	uint32_t validmsgsrx;
	uint32_t datarx;

	float voltage;
	float ampere;
	int32_t mah;
	float baro_altitude;
	float altitude;
	double longitude;
	double latitude;
	float heading;
	float cog; //course over ground
	float speed;
	float airspeed;
	float roll, pitch;
	uint8_t sats;
	uint8_t fix;
	uint8_t armed;
	uint8_t rssi;

	uint8_t home_fix;

//#if defined(FRSKY)
	int16_t x, y, z; // also needed for smartport
	int16_t ew, ns;
//#endif

#if defined(SMARTPORT)
	uint8_t swr;
	float rx_batt;
	float adc1;
	float adc2;
	float vario;
#endif

#if defined(MAVLINK)
	uint32_t mav_flightmode;
	float mav_climb;
#endif

#if defined(LTM)
// ltm S frame
	uint8_t ltm_status;
	uint8_t ltm_failsafe;
	uint8_t ltm_flightmode;
// ltm N frame
	uint8_t ltm_gpsmode;
	uint8_t ltm_navmode;
	uint8_t ltm_navaction;
	uint8_t ltm_wpnumber;
	uint8_t ltm_naverror;
// ltm X frame
	uint16_t ltm_hdop;
	uint8_t ltm_hw_status;
	uint8_t ltm_x_counter;
	uint8_t ltm_disarm_reason;
// ltm O frame
	float ltm_home_altitude;
	double ltm_home_longitude;
	double ltm_home_latitude;
	uint8_t ltm_osdon;
	uint8_t ltm_homefix;
#endif


	wifibroadcast_rx_status_t *rx_status;
	wifibroadcast_rx_status_t_osd *rx_status_osd;
	wifibroadcast_rx_status_t_rc *rx_status_rc;
	wifibroadcast_rx_status_t_uplink *rx_status_uplink;
	wifibroadcast_rx_status_t_sysair *rx_status_sysair;
} telemetry_data_t;



typedef struct{
    volatile unsigned char     	Menu;                     //菜单
    volatile unsigned char     	Lang;                     //语言
    volatile unsigned char     	usepal;                   //视频信号制式
    volatile unsigned char     	Metric;                   //公制
    volatile int    			offset_x;
    volatile int    			offset_y;

    volatile float  			JD;                       //经度
    volatile float  			WD;                       //纬度
    volatile float  			Speed;                    //速度
    volatile float  			Altitude;                 //高度
    volatile float  			numSV;                    //卫星数
    volatile float  			Heading;                  //飞行方向
    volatile float  			JDhome;                   //起飞点经度
    volatile float  			WDhome;                   //起飞点纬度
    volatile float  			current;                  //电流值
    volatile float  			curmah;                   //消耗mAh
    volatile float  			osdflt;                   //图传电压
    volatile float  			powflt;                   //动力电压
    volatile float  			WPJD;                     //航点经度
    volatile float  			WPWD;                     //航点纬度
    volatile float  			WPGD;                     //航点高度
    volatile float  			WPID;                     //航点标号
    volatile float  			WPTIME;                   //航点剩余时间
    volatile float  			disthome;                 //回家距离
    volatile float  			distrt;                   //航程
    volatile float  			headingcrr;               //返航方向
    volatile unsigned int    	throtimecnt;                //飞行时间
    volatile unsigned char    	moldfilter;                 //健康值
    volatile unsigned int    	unusedc3;                    //起飞辅助油门
    volatile int    			agld1;                      //横滚角+-180
    volatile int    			agld2;                      //俯仰角+-180
    volatile int    			gpsplus1;                   //控制角度1
    volatile int    			gpsplus2;                   //控制角度2
    volatile unsigned char    	throosd;                    //油门控制值
    volatile unsigned char    	mode;                       //模式 	
	volatile unsigned char     	rssi;                       //信号强度
	volatile unsigned char 		astrange;                   //辅助手抛范围
    volatile unsigned char 		rssionosd;                  //信号强度显示
    volatile unsigned char 		jwonosd;		            //显示经纬度
    volatile unsigned short    	lowpoweralarm;              //低电压报警
	volatile unsigned short    	airspeed;					//空速计速度
	volatile unsigned char 		flapval;					//襟翼补偿值
}PACKET_STATE;




wifibroadcast_rx_status_t *telemetry_wbc_status_memory_open(void);
wifibroadcast_rx_status_t_osd *telemetry_wbc_status_memory_open_osd(void);
wifibroadcast_rx_status_t_rc *telemetry_wbc_status_memory_open_rc(void);
wifibroadcast_rx_status_t_uplink *telemetry_wbc_status_memory_open_uplink(void);
wifibroadcast_rx_status_t_sysair *telemetry_wbc_status_memory_open_sysair(void);
