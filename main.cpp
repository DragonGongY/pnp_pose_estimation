#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
//#include "serial/serial.h"
//#include "HikvisionCamera.h"
#include "infrared_solvePnP.h"
//#include "IRDistanceDetection.h" damon


using namespace std;
using namespace cv;
#if 0
#define SERIAL_PORT "/dev/serial0"
#define SERIAL_PORT2 "/dev/serial1"

#define  IR_SENSOR_NUM  8
#define  FILTER_BUF_LEN  8


FILE *fp = NULL;
FILE *fpErr = NULL;
#if 1

int m_FilterBuf[IR_SENSOR_NUM][FILTER_BUF_LEN];
int m_Channel[IR_SENSOR_NUM];
short m_dis[8];
unsigned short CRC16(unsigned char *ptr, unsigned short len)
{
    unsigned char i;
    unsigned short crc = 0xFFFF;
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i<8; i++)
        {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return(crc);
}


int put_data_into_buf(int addata, int ch)
{
	m_FilterBuf[ch][m_Channel[ch] & (FILTER_BUF_LEN - 1)] = addata;
	m_Channel[ch]++;
	return 0;
}
int get_ch_middle_num(int ch)
{
		int i, j;// ѭ������
		int bTemp;
		int tempArry[FILTER_BUF_LEN];
		//if (iFilterLen > FILTER_BUF_LEN) return 0;
		memcpy(tempArry, m_FilterBuf[ch], sizeof(int)*FILTER_BUF_LEN);
		// ��ð�ݷ��������������
		for (j = 0; j < FILTER_BUF_LEN - 1; j++)
		{
			for (i = 0; i < FILTER_BUF_LEN - j - 1; i++)
			{
				if (tempArry[i] > tempArry[i + 1])
				{
					// ����
					bTemp = tempArry[i];
					tempArry[i] = tempArry[i + 1];
					tempArry[i + 1] = bTemp;
				}
			}
		}
		// ������ֵ
		if ((FILTER_BUF_LEN & 1) > 0)
		{
			// ������������Ԫ�أ������м�һ��Ԫ��
			bTemp = tempArry[(FILTER_BUF_LEN + 1) / 2];
		}
		else
		{
			// ������ż����Ԫ�أ������м�����Ԫ��ƽ��ֵ
			bTemp = (tempArry[FILTER_BUF_LEN / 2] + tempArry[FILTER_BUF_LEN / 2 + 1]) / 2;
		}
		return bTemp;
}


double vol2distance(double vol)
{
	typedef struct {
		double vol;
		double dis;
	}vol_dis_t;
	//С��IR���봫����
#if 1
	//const std::vector<vol_dis_t> table = { {460,800},{470,700},{480,600},
	//								{520,500},{610,400},{750,300},
	//								{880,250},{1080,200},{1410,150},
	//								{2057,100},{2500,50} };
	const std::vector<vol_dis_t> table = {/*{4000,600},{4536,500},*/{7064,400},
										{12234,300},{18321,200},{32529, 100},
										{40797,50}};
#else  
	//���IR������
	const std::vector<vol_dis_t> table = { {670,900},{760,800},{830,700},
							{940,600},{1100,500},{1329,400},
								{1690,300},{2060,200},{2297,150} };

#endif
	double dis = 0;
	for (int i = 0;i < table.size();i++)
	{
		if (vol < table[0].vol)  break;

		if (vol > table[i].vol)
			continue;
		
		if (i != 0)
		{
			//����ֱ�߷���
			double k = (table[i].dis - table[i - 1].dis) / (table[i].vol - table[i-1].vol);
			//printf("[%d] k=%lf\n", i, k);
			dis = k * (vol - table[i-1].vol) + table[i-1].dis;
			break;
		}
	}
	return dis;
	//return b*pow(vol,3)+c*pow(vol,2)+d*pow(vol,1)+e;
	//return a*pow(vol,4)+b*pow(vol,3)+c*pow(vol,2)+d*pow(vol,1)+e;
}

#if 0
int main(int argc, char **argv) {

	unsigned char cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x0, 0x0};
	string port(SERIAL_PORT);
    
	serial::Serial serialHandle(port, 9600, serial::Timeout::simpleTimeout(100));
	unsigned char *buff = new unsigned char[128];
	
	if (!serialHandle.isOpen()) {
		cout << SERIAL_PORT << "Open fail" << endl;
		return -1;
	}	

	unsigned short crc = CRC16(cmd, 6);
	cmd[6] = (char)crc;
    cmd[7] = (char)(crc >> 8);
	
	printf("crc 0x%x 0x%x\n", cmd[6], cmd[7]);
	#if  0
	string port2(SERIAL_PORT2);
	serial::Serial serialHandle2(port2, 115200, serial::Timeout::simpleTimeout(100));
	
	if (!serialHandle2.isOpen()) {
		cout << SERIAL_PORT2 << "Open fail2" << endl;
		return -1;
	}
	char pnpStr[128];
	unsigned char pnpData[128]; 
	pnpData[0] = 0xab;
	pnpData[1] = 0xcd;
	pnpData[2] = 1;//cmd
	pnpData[3] = 24;//len 2 + 2 + 2 + 2
    short dis[8] = {0};
	unsigned char sum = 0;
	posture_result pnpResult;
	bool  flag;
	short int  x_rotation;
	#endif
	while (1) {
		int writeRet = serialHandle.write(cmd, 8); 
		//printf("writeRet=%d\n", writeRet);
		
		int readRet = serialHandle.read (buff, 128);
		printf("readRet=%d\n", readRet);
		
		for (int i = 0; i < readRet; i++) {
			printf("0x%x ", buff[i]);
		}
		printf("\n");
		
		for (int i = 0; i < IR_SENSOR_NUM;i++) {
			double vol;
			unsigned short ao = 0;
			*((unsigned char *)&ao) = buff[i*2+3 + 1]; 
			*(((unsigned char *)&ao) + 1) =  buff[i*2+3];
			//printf("[%d] 0x%x\n", i, ao);
			put_data_into_buf(ao, i);
			vol = get_ch_middle_num(i);
			//printf("vol = %lf\n", vol);
			m_dis[i] = vol2distance(vol);
			
			if(i<4)
				printf("==vol[%d]��%d  dis[%d]��%d \n", i, ao, i,m_dis[i]);
		}	
					
		//usleep(50000);
	
	}
	return 0;
	
	
}
#endif
#endif

//posture_result pnpResult;
//short int  x_rotation = 0;
//bool  flag = false;
//pthread_mutex_t mutex;
//short dis[8] = {0};

#if 0
void *ComUart(void *arg) {
	FILE *fp2 = fopen("/home/damon/cmdlog", "a");
	FILE *fp3 = fopen("/home/damon/cmderrlog", "a");
	string port(SERIAL_PORT);
	unsigned char sum = 0;
    unsigned char pnpData[128]; 
	pnpData[0] = 0xab;
	pnpData[1] = 0xcd;
	pnpData[2] = 1;//cmd
	pnpData[3] = 24;//len 2 + 2 + 2 + 2	
	
	
	serial::Serial serialHandle(port, 115200, serial::Timeout::simpleTimeout(30));
	
	if (!serialHandle.isOpen()) {
		cout << SERIAL_PORT << "Open fail" << endl;
		fprintf(fp2, "[%d]Open com1 fail\n", getpid());
		fflush(fp2);
		return (void *)0;
	}	
	fprintf(fp2, "[%d]Open com1 OK\n", getpid());
	fflush(fp2);	
	
	time_t tt;
	struct tm *t;
	struct timeval tv;
	struct timeval end;
	struct timezone tz;
	int lasttime = 0;
	struct timeval lastVal = {0, 0};
	while (1) {
		double time1 = (double)cvGetTickCount() / cvGetTickFrequency();
		int costtime = 0;
		if (lasttime != 0) {
			costtime = time1 / 1000 - lasttime;
		}
		else {
			lasttime = time1 / 1000;
		}
		
		if (costtime <= 60) {
			int a = 0;
			continue;
		}
		lasttime = time1 / 1000;


		        gettimeofday(&tv, &tz);
		        t = localtime(&tv.tv_sec);
				fprintf(fp2, "-----------[%d][%d](%d)time=%02d:%02d:%02d:%02ld\n", 0, 0, 0, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
				fflush(fp2);


		        double start = static_cast<double>(cvGetTickCount());
		        /*pthread_mutex_lock(&mutex);
		        memcpy(pnpData + 4, &x_rotation, sizeof(x_rotation));
				memcpy(pnpData + 6, &pnpResult.x_translation, sizeof(pnpResult.x_translation));
				memcpy(pnpData + 8, &pnpResult.z_translation, sizeof(pnpResult.z_translation));
				memcpy(pnpData + 10, &flag, sizeof(flag));
				flag = 0;
				pthread_mutex_unlock(&mutex);
				memcpy(pnpData + 12, &dis, 8*sizeof(short));
				
				
				
                sum = 0;
				for (int i = 2; i < 28; i++) {
					sum += pnpData[i];
				}
				sum = sum & 0xff;
				pnpData[28] = sum;*/
				
                //time(&tt);
               // t = localtime(&tt); 

				//struct tm *t;




				//int writeLen = serialHandle.write(pnpData, 29); 
					
					
				//fprintf(fp2, "x_rotation=%f x_translation=%d z_translation=%d detect_flag=%d\n", pnpResult.x_rotation, pnpResult.x_translation, pnpResult.z_translation, pnpResult.detect_flag);
				
				//printf("x_rotation=%f x_translation=%d z_translation=%d detect_flag=%d\n", pnpResult.x_rotation, pnpResult.x_translation, pnpResult.z_translation, pnpResult.detect_flag);
				
				//printf("getOneFrameTimeout[%d]  width=%d height=%d pix=%d\n", ret, imgInfo.width, imgInfo.height, imgInfo.pixelType);
				//imshow("img", img);
				

	

				double time2 = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
				int syncTime = time2 / 1000;

				//fprintf(fp2, "[%d][%d](%d)time=%02d:%02d:%02d:%02ld\n", writeLen, costtime, syncTime,  t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
				//fflush(fp2);

				//fprintf(fp, "loop time=%d\n", costtime);

				gettimeofday(&end, NULL);
				if (lastVal.tv_sec == 0) {
					lastVal.tv_sec = end.tv_sec;
					lastVal.tv_usec = end.tv_usec;
				}
				long diff = (end.tv_sec - lastVal.tv_sec) * 1000 + (end.tv_usec - lastVal.tv_usec) / 1000;

				lastVal.tv_sec = end.tv_sec;
				lastVal.tv_usec = end.tv_usec;

				if (diff > 100) {
					fprintf(fp3, "[%d][%ld](%d)time=%02d:%02d:%02d:%02ld\n", 0, diff, syncTime, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
					fflush(fp3);
				}
				
				
				
				//usleep(60000);
				gettimeofday(&tv, &tz);
				t = localtime(&tv.tv_sec);
				fprintf(fp2, "[%d][%d](%d)time=%02d:%02d:%02d:%02ld\n", 0, 0, syncTime, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
				fflush(fp2);
	}
	
	
	return (void *)0;
}
#endif
#if 0
void *ComUart(void *arg) {
	FILE *fp = fopen("/home/damon/comlog", "a");
	string port(SERIAL_PORT);
	unsigned char sum = 0;
	unsigned char pnpData[128];
	pnpData[0] = 0xab;
	pnpData[1] = 0xcd;
	pnpData[2] = 1;//cmd
	pnpData[3] = 24;//len 2 + 2 + 2 + 2	


	serial::Serial serialHandle(port, 115200, serial::Timeout::simpleTimeout(30));
	if (!serialHandle.isOpen()) {
		cout << SERIAL_PORT << "Open fail" << endl;
		fprintf(fp, "[%d]Open com1 fail\n", getpid());
		fflush(fp);
		return (void *)0;
	}
	fprintf(fp, "[%d]Open com1 OK\n", getpid());
	fflush(fp);

	struct tm *t;
	struct timeval begin;
	struct timeval lastVal = { 0, 0};
	while (1) {
		gettimeofday(&begin, NULL);
		t = localtime(&begin.tv_sec);
		
		pthread_mutex_lock(&mutex);
		memcpy(pnpData + 4, &x_rotation, sizeof(x_rotation));
		memcpy(pnpData + 6, &pnpResult.x_translation, sizeof(pnpResult.x_translation));
		memcpy(pnpData + 8, &pnpResult.z_translation, sizeof(pnpResult.z_translation));
		memcpy(pnpData + 10, &flag, sizeof(flag));
		flag = 0;
		memcpy(pnpData + 12, &dis, 8*sizeof(short));
		printf("x_rotation=%f x_translation=%d z_translation=%d detect_flag=%d\n", pnpResult.x_rotation, pnpResult.x_translation, pnpResult.z_translation, pnpResult.detect_flag);
        pthread_mutex_unlock(&mutex);
		
		
		sum = 0;
		for (int i = 2; i < 28; i++) {
			sum += pnpData[i];
		}
		sum = sum & 0xff;
		pnpData[28] = sum;

		int writeLen = serialHandle.write(pnpData, 29); 
        if (writeLen != 29) {
			fprintf(fp, "Error COM1 write error=%d [%02d:%02d:%02d:%02ld]\n", writeLen, t->tm_hour, t->tm_min, t->tm_sec, begin.tv_usec);
			fflush(fp);				
		}

		//printf("x_rotation=%f x_translation=%d z_translation=%d detect_flag=%d\n", pnpResult.x_rotation, pnpResult.x_translation, pnpResult.z_translation, pnpResult.detect_flag);

		//printf("getOneFrameTimeout[%d]  width=%d height=%d pix=%d\n", ret, imgInfo.width, imgInfo.height, imgInfo.pixelType);
		//imshow("img", img);


		//fprintf(fp2, "[%d][%d](%d)time=%02d:%02d:%02d:%02ld\n", writeLen, costtime, syncTime,  t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
		//fflush(fp2);

		//fprintf(fp, "loop time=%d\n", costtime);

		gettimeofday(&begin, NULL);
		t = localtime(&begin.tv_sec);
		if (lastVal.tv_sec == 0) {
			lastVal.tv_sec = begin.tv_sec;
			lastVal.tv_usec = begin.tv_usec;
		}
		long diff = (begin.tv_sec - lastVal.tv_sec) * 1000 + (begin.tv_usec - lastVal.tv_usec) / 1000;		
		if (diff > 100) {
			fprintf(fp, "Error LOOP TIME=%ld[%02d:%02d:%02d:%02ld]\n", diff, t->tm_hour, t->tm_min, t->tm_sec, begin.tv_usec);
			fflush(fp);
		}
		lastVal.tv_sec = begin.tv_sec;
		lastVal.tv_usec = begin.tv_usec;


		//gettimeofday(&tv, &tz);
		
		//fprintf(fp2, "[%d][%ld]time=%02d:%02d:%02d:%02ld\n", writeLen, diff, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
		
		//fflush(fp2);


		usleep(60000);
		
	}


	return (void *)0;
}

#if 0
int main1(int argc, char **argv)
{
	pthread_t ntid;
	int err = pthread_create(&ntid, NULL, ComUart, NULL);

	HikvisionCamera *cam = new HikvisionCamera("025845");
	ImageInfo imgInfo;
	pthread_mutex_init(&mutex, NULL);
	FILE *fp = fopen("/home/damon/syslog", "a");
	
	//CDisDetect  ins
	
	cam->open();
	if (cam->isOpen()) {
		int ret = cam->setPixelType(MONO8_PIXEL);
		printf("setPixelType ret=%d\n", ret);
		imgInfo.data = (unsigned char *)malloc(cam->getPayloadSize());
		imgInfo.size = cam->getPayloadSize();

		cam->setTriggerMode(TRIGGER_MODE_OFF);
		//cam->setTriggerSource(TRIGGER_SOURCE_SOFTWARE);

		ret = cam->setExposureTime(16000);
		cam->startGrabbing();
	
	    fprintf(fp, "[%d]camera open OK startGrabbing \n", getpid());
	    fflush(fp);
	}
	else {
		printf("camera open fail\n");
	    fprintf(fp, "[%d]camera open fail\n", getpid());
	    fflush(fp);
		return -1;
	}

    unsigned char cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x0, 0x0};
	string port2(SERIAL_PORT2);

	serial::Serial serialHandle2(port2, 9600, serial::Timeout::simpleTimeout(30));
	unsigned char *buff = new unsigned char[128];

	
	int readLen = 64;
	int ret = -1;
	Mat img;

	infrared_solvePnP_parameter parameter;
	int  namecont = 0;


	while (1) {
		string result;// = serialHandle.read(readLen);
		result = "S1";
		
		if (result == "S1") {
			//cam->softwareTrigger();
			ret = cam->getOneFrameTimeout(imgInfo, 100);
			if (ret == 0) {
				img = Mat(imgInfo.height, imgInfo.width, CV_8UC1, imgInfo.data);
                pthread_mutex_lock(&mutex);
				pnpResult = infrared_solvePnP(img, parameter); 
				flag = pnpResult.detect_flag;
				x_rotation = pnpResult.x_rotation * 10;
				pthread_mutex_unlock(&mutex);
#if 0
  				ret = serialHandle2.write(cmd, 8);
				if (ret != 8) {
					fprintf(fp, "COM2 write error=%d[%02d:%02d:%02d:%02ld]\n", ret, t->tm_hour, t->tm_min, t->tm_sec, begin.tv_usec);
					fflush(fp);
					pthread_mutex_lock(&mutex);
					memset(&dis, 0, 16);
					pthread_mutex_unlock(&mutex);
				} else {
					int readRet = serialHandle2.read(buff, 19); 
					if (readRet < 19) {
						fprintf(fp, "COM2 read error=%d[%02d:%02d:%02d:%02ld]\n", readRet, t->tm_hour, t->tm_min, t->tm_sec, begin.tv_usec);
						fflush(fp);
						pthread_mutex_lock(&mutex);
					    memset(&dis, 0, 16);
					    pthread_mutex_unlock(&mutex);
					} else {				
						/*for (int i = 0; i < readRet; i++) {
							printf("0x%x ", buff[i]);
						}
						printf("\n");	*/	
						for (int i = 0; i < IR_SENSOR_NUM;i++) {
							double vol;
							unsigned short ao = 0;
							*((unsigned char *)&ao) = buff[i*2+3 + 1]; 
							*(((unsigned char *)&ao) + 1) =  buff[i*2+3];
							//printf("[%d] 0x%x\n", i, ao);
							put_data_into_buf(ao, i);
							vol = get_ch_middle_num(i);
							//printf("vol = %lf\n", vol);
							pthread_mutex_lock(&mutex);
							dis[i] = vol2distance(vol);
							pthread_mutex_unlock(&mutex);
							
							if(i<4)
								printf("==vol[%d]��%d  dis[%d]��%d \n", i, ao, i,dis[i]);
						}					
					}
				}

	if (cam->isOpen()) {
		free(imgInfo.data);
		cam->close();
		delete cam;
	}

	return 0;
}
#endif
#endif
#endif
#endif

int main()
{
    posture_result pnpResult;
    infrared_solvePnP_parameter parameter;
    short int x_translation;
    float x_rotation;
    short int z_translation;
    bool detect_flag;
    Mat img = imread("/media/dp/LinuxData/Projects/changeBattery/changeBattery/save_image/516.jpg", 0);
    pnpResult = infrared_solvePnP(img, parameter);
    x_translation = pnpResult.x_translation;
    x_rotation = pnpResult.x_rotation;
    z_translation = pnpResult.z_translation;
    detect_flag = pnpResult.detect_flag;
    cout << "x_translation: " << x_translation << endl;
    cout << "x_rotation: " << x_rotation << endl;
    cout << "z_translation: " << z_translation << endl;
    cout << "detect_flag: " << detect_flag << endl;

}