/*
Copyright (c) 2015, befinitiv
Copyright (c) 2012, Broadcom Europe Ltd
modified by Samuel Brucksch https://github.com/SamuelBrucksch/wifibroadcast_osd
modified by Rodizio

All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <locale.h>

#include "render.h"
#include "osdconfig.h"
#include "telemetry.h"
#ifdef FRSKY
#include "frsky.h"
#elif defined(LTM)
#include "ltm.h"
#elif defined(MAVLINK)
#include "mavlink.h"
#elif defined(SMARTPORT)
#include "smartport.h"
#endif






///////////gcc协议实现部分///////////////////
/*串口中断中用来接收GCS协议数据并提取正确协议包*/
#define PACKET_STX0				0xFD
#define PACKET_STX1				0xFD
#define PACKET_STX2				0xFD
#define PACKET_STX3				0xFD

#define PACKET_HEAD_LENGTH		9      //STX0 STX1 STX2 STX3 TYPE SIZE  1 + 1 + 1 + 1 + 1 + 4

#define PACKET_TYPE_REQUEST 	0
#define PACKET_TYPE_ACK			1
#define PACKET_TYPE_STATE	    2
#define PACKET_TYPE_WPT			3
#define PACKET_TYPE_CMD			4

unsigned char 	enable_readbuf 	 = 0;  //读取中断接收包使能位(数据包更新时)
PACKET_STATE	packet_state_recv;


void unpack_state(unsigned char* recbuf)
{
    if((recbuf[4]==PACKET_TYPE_STATE)&&(enable_readbuf))
	{memcpy((void *)&packet_state_recv, (const void *)&recbuf[PACKET_HEAD_LENGTH], sizeof(PACKET_STATE));}
}


int gcs_protocol_recv_packet(uint8_t  * buf,telemetry_data_t *td)
{
    uint8_t  recBuf[263];
	unsigned char 			i = 0, start = 0;
	unsigned int				j = 0;
	unsigned int 			packet_size = 0, data_size = 0;
	unsigned short			check_calc = 0, check_recv = 0;
	static unsigned int  PACKET_SIZE_STATE 		= sizeof(PACKET_STATE)+20;
	uint8_t ch;
for(int n_unpack=0 ;(n_unpack<=PACKET_SIZE_STATE)&&(enable_readbuf!=1);n_unpack++)
{
    ch=buf[n_unpack];
	/* i:0--8  PACKET_HEAD_LENGTH: 1--9*/
	switch(i){
		case 0:
			if(PACKET_STX0 == ch){i++;} else {i = 0;}
			break;
		case 1:
			if(PACKET_STX1 == ch){i++;} else {i = 0;}
			break;
		case 2:
			if(PACKET_STX2 == ch){i++;} else {i = 0;}
			break;
		case 3:
			if(PACKET_STX3 == ch){
				start = 1;
				j = 0;
				i = 0;
				recBuf[j++] = PACKET_STX0; 
				recBuf[j++] = PACKET_STX1; 
				recBuf[j++] = PACKET_STX2;
				recBuf[j++] = PACKET_STX3;
				continue;				
			}
			i = 0;
			break;
		default:
			break;		
	}

    if(start == 1){
		recBuf[j] = ch;
		j++;
		if(j == PACKET_HEAD_LENGTH){
			data_size = *((unsigned int*)&recBuf[5]);
			if(data_size > 1000){
				i = 0;
				j = 0;
				start = 0;
				data_size = 0;
				packet_size = 0;
				check_recv = 0;
				check_calc = 0;
				continue;
			}
			packet_size = PACKET_HEAD_LENGTH + data_size + 2;
			continue;
		}					
		if (j > PACKET_HEAD_LENGTH && j <= packet_size){
			check_calc += ch;
			if(j == packet_size){
				check_calc -= (recBuf[j - 1] + recBuf[j - 2]);
				check_recv = ((unsigned short)recBuf[j - 1] << 8) | (unsigned short)recBuf[j - 2];
				if(check_calc == check_recv){
                    enable_readbuf = 1;
					//last_packet_size = packet_size;
				}
				
				i = 0;
				j = 0;
				start = 0;
				data_size = 0;
				packet_size = 0;
				check_recv = 0;
				check_calc = 0;
				continue; 
			}
		}
	}
}

  if(enable_readbuf==1)
  {
   unpack_state(recBuf);
   td.pitch=packet_state_recv.agld1;
   td.roll=packet_state_recv.agld2;
   td.speed=packet_state_recv.Speed;
   return 1;
  }

}




/////////////////////////////////////////////







long long current_timestamp() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    return milliseconds;
}

fd_set set;

struct timeval timeout;

int main(int argc, char *argv[]) {
    fprintf(stderr,"OSD started\n=====================================\n\n");

    setpriority(PRIO_PROCESS, 0, 10);

    setlocale(LC_ALL, "en_GB.UTF-8");

    uint8_t buf[263]; // Mavlink maximum packet length
    size_t n;

    long long fpscount_ts = 0;
    long long fpscount_ts_last = 0;
    int fpscount = 0;
    int fpscount_last = 0;
    int fps;

    int do_render = 0;
    int counter = 0;


#ifdef FRSKY
    frsky_state_t fs;
#endif

    struct stat fdstatus;
    signal(SIGPIPE, SIG_IGN);
    char fifonam[100];
    sprintf(fifonam, "/root/telemetryfifo1");

    int readfd;
    readfd = open(fifonam, O_RDONLY | O_NONBLOCK);
    if(-1==readfd) {
        perror("ERROR: Could not open /root/telemetryfifo1");
        exit(EXIT_FAILURE);
    }
    if(-1==fstat(readfd, &fdstatus)) {
        perror("ERROR: fstat /root/telemetryfifo1");
        close(readfd);
        exit(EXIT_FAILURE);
    }

    fprintf(stderr,"OSD: Initializing sharedmem ...\n");
    telemetry_data_t td;
    telemetry_init(&td);
    fprintf(stderr,"OSD: Sharedmem init done\n");

//    fprintf(stderr,"OSD: Initializing render engine ...\n");
    render_init();
//    fprintf(stderr,"OSD: Render init done\n");

    long long prev_time = current_timestamp();
    long long prev_time2 = current_timestamp();

    long long prev_cpu_time = current_timestamp();
    long long delta = 0;

    int cpuload_gnd = 0;
    int temp_gnd = 0;
    int undervolt_gnd = 0;
    FILE *fp;
    FILE *fp2;
    FILE *fp3;
    long double a[4], b[4];

    fp3 = fopen("/tmp/undervolt","r");
    if(NULL == fp3) {
        perror("ERROR: Could not open /tmp/undervolt");
        exit(EXIT_FAILURE);
    }
    fscanf(fp3,"%d",&undervolt_gnd);
    fclose(fp3);
//    fprintf(stderr,"undervolt:%d\n",undervolt_gnd);

    while(1) {
//		fprintf(stderr," start while ");
//		prev_time = current_timestamp();

	    FD_ZERO(&set);
	    FD_SET(readfd, &set);
	    timeout.tv_sec = 0;
	    timeout.tv_usec = 50 * 1000;
	    // look for data 50ms, then timeout
	    n = select(readfd + 1, &set, NULL, NULL, &timeout);
	    if(n > 0) { // if data there, read it and parse it
	        n = read(readfd, buf, sizeof(buf));
//	        printf("OSD: %d bytes read\n",n);
	        if(n == 0) { continue; } // EOF
		if(n<0) {
		    perror("OSD: read");
		    exit(-1);
		}
#ifdef FRSKY
		frsky_parse_buffer(&fs, &td, buf, n);
#elif defined(LTM)
		do_render = ltm_read(&td, buf, n);
#elif defined(MAVLINK)
		do_render = mavlink_read(&td, buf, n);
#elif defined(SMARTPORT)
		smartport_read(&td, buf, n);
#elif defined(GCS)
        gcs_protocol_recv_packet(buf,&td)
#endif
	    }
	    counter++;
//	    fprintf(stderr,"OSD: counter: %d\n",counter);
	    // render only if we have data that needs to be processed as quick as possible (attitude)
	    // or if three iterations (~150ms) passed without rendering
	    if ((do_render == 1) || (counter == 3)) {
//		fprintf(stderr," rendering! ");
		prev_time = current_timestamp();
		fpscount++;
		render(&td, cpuload_gnd, temp_gnd/1000, undervolt_gnd,fps);
		long long took = current_timestamp() - prev_time;
//		fprintf(stderr,"Render took %lldms\n", took);
		do_render = 0;
		counter = 0;
	    }

	    delta = current_timestamp() - prev_cpu_time;
	    if (delta > 1000) {
		prev_cpu_time = current_timestamp();
//		fprintf(stderr,"delta > 10000\n");

		fp2 = fopen("/sys/class/thermal/thermal_zone0/temp","r");
		fscanf(fp2,"%d",&temp_gnd);
		fclose(fp2);
//		fprintf(stderr,"temp gnd:%d\n",temp_gnd/1000);

		fp = fopen("/proc/stat","r");
		fscanf(fp,"%*s %Lf %Lf %Lf %Lf",&a[0],&a[1],&a[2],&a[3]);
		fclose(fp);

		cpuload_gnd = (((b[0]+b[1]+b[2]) - (a[0]+a[1]+a[2])) / ((b[0]+b[1]+b[2]+b[3]) - (a[0]+a[1]+a[2]+a[3]))) * 100;
//		fprintf(stderr,"cpuload gnd:%d\n",cpuload_gnd);

		fp = fopen("/proc/stat","r");
		fscanf(fp,"%*s %Lf %Lf %Lf %Lf",&b[0],&b[1],&b[2],&b[3]);
		fclose(fp);
	    }
//		long long took = current_timestamp() - prev_time;
//		fprintf(stderr,"while took %lldms\n", took);

		long long fpscount_timer = current_timestamp() - fpscount_ts_last;
		if (fpscount_timer > 2000) {
		    fpscount_ts_last = current_timestamp();
		    fps = (fpscount - fpscount_last) / 2;
		    fpscount_last = fpscount;
//		    fprintf(stderr,"OSD FPS: %d\n", fps);
		}
    }
    return 0;
}
