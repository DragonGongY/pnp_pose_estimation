#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include "MvCameraControl.h"
#include "HikvisionCamera.h"



int HikvisionCamera::open() {
    void *m_handle = NULL;
    printf("open first\n");
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE , &m_stDevList);
    if (MV_OK != ret) {
        printf("error: EnumDevices fail ret=[%x]\n", ret);
        return BZL_E_DEVICE_ENUM;
    }
    if (m_stDevList.nDeviceNum <= 0) {
    	printf("error: no device\n");
    	return BZL_E_DEVICE_NO;
    }

    int index = 0;
    for (; index < m_stDevList.nDeviceNum; index++) {
    	//printf("camid=%s\n", _cameraID.c_str());
    	printf("camid=%s\n", m_stDevList.pDeviceInfo[index]->SpecialInfo.stUsb3VInfo.chSerialNumber);
		if (1) {
    	//if (_cameraID.compare((char *)m_stDevList.pDeviceInfo[index]->SpecialInfo.stUsb3VInfo.chSerialNumber) == 0) {
    		ret = MV_CC_CreateHandle(&m_handle, m_stDevList.pDeviceInfo[index]);
    		if (MV_OK != ret) {
    			printf("error: CreateHandle fail [%x]\n", ret);
    			return BZL_E_DEVICE_CREATE_HANDLE;
    		} else {
    			break;			
    		}		
    	}
    }

    if (m_stDevList.nDeviceNum == index) {
    	printf("not find the device\n");
    	return BZL_E_DEVICE_FIND;
    }
    

    ret = MV_CC_OpenDevice(m_handle);
    if (MV_OK != ret) {
        printf("error: OpenDevice fail [%x]\n", ret);
        return BZL_E_DEVICE_OPEN;
    }  
    
    _cameraHandle = m_handle;
    
    return BZL_OK;
    //MV_CC_SetPixelFormat(_cameraHandle, PixelType_Gvsp_RGB8_Packed);
    
}   

void HikvisionCamera::close() {
    int nRet = MV_CC_CloseDevice(_cameraHandle);
    if (MV_OK != nRet) {
        printf("error: CloseDevice fail [%x]\n", nRet);
        return;
    }


    nRet = MV_CC_DestroyHandle(_cameraHandle);
    if (MV_OK != nRet) {
        printf("error: DestroyHandle fail [%x]\n", nRet);
        return;
    }    		
    
    _cameraHandle = NULL;
}

bool HikvisionCamera::isOpen() {
    return _cameraHandle != NULL ? true : false;	
}


int HikvisionCamera::startGrabbing() {
    int ret = MV_CC_StartGrabbing(_cameraHandle);
    if (ret != MV_OK) {
    	return BZL_E_DEVICE_START;
    }
    
    return BZL_OK; 
}

int HikvisionCamera::stopGrabbing() {
    int ret = MV_CC_StopGrabbing(_cameraHandle);	
    if (ret != MV_OK) {
    	return BZL_E_DEVICE_STOP;
    }	
    
    return BZL_OK;
}      


int HikvisionCamera::getOneFrameTimeout(ImageInfo &imgInfo, int nMsec) {
    MV_FRAME_OUT_INFO_EX stInfo;
    memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    
    int ret = MV_CC_GetOneFrameTimeout(_cameraHandle, imgInfo.data, imgInfo.size, &stInfo, nMsec);
	
    if (MV_OK == ret) {
    	imgInfo.width = stInfo.nWidth;
    	imgInfo.height = stInfo.nHeight;
    	if (PixelType_Gvsp_RGB8_Packed == stInfo.enPixelType) {
    		imgInfo.pixelType = RGB24_PIXEL;
    	} else if (PixelType_Gvsp_Mono8 == stInfo.enPixelType) {
    		imgInfo.pixelType = MONO8_PIXEL;
    	} else {
    		imgInfo.pixelType = NONE_PIXEL;
    	}
#if 0    	
    		//图片数据输入输出参数            
            MV_SAVE_IMAGE_PARAM stParam;
                              
            //源数据                 
            stParam.pData         = imgInfo.data;                //原始图像数据
            stParam.nDataLen      = imgInfo.size;    //原始图像数据长度
            stParam.enPixelType   = stInfo.enPixelType;  //原始图像数据的像素格式
            stParam.nWidth        = stInfo.nWidth;       //图像宽
            stParam.nHeight       = stInfo.nHeight;      //图像高   

            //目标数据
            stParam.enImageType   = MV_Image_Jpeg;            //需要保存的图像类型，转换成JPEG格式
            stParam.nBufferSize   = imgInfo.size;                 //存储节点的大小
            unsigned char* pImage = (unsigned char*)malloc(imgInfo.size);
            stParam.pImageBuffer  = pImage;                   //输出数据缓冲区，存放转换之后的图片数据         

            ret = MV_CC_SaveImage(&stParam);
 

            //将转换之后图片数据保存成文件
            FILE* fp = fopen("image.jpeg", "wb");
            fwrite(pImage, 1, stParam.nImageLen, fp);
            fclose(fp);
            free(pImage);
#endif    
    	
    	
    	return BZL_OK;
    } else {
    	return BZL_E_DEVICE_CAPTURE;
    }

    //stInfo->nFrameLen;    图像大小	
}



int HikvisionCamera::getPayloadSize() {
    MVCC_INTVALUE stIntvalue = {0};
    
    int ret = MV_CC_GetIntValue(_cameraHandle, "PayloadSize", &stIntvalue);
    if (ret != MV_OK) {
        printf("Get PayloadSize failed! nRet [%x]\n", ret);
        return BZL_E_DEVICE_GET_PARAM;
    }
    
    return stIntvalue.nCurValue; 
}





int HikvisionCamera::setTriggerMode(CAMERA_TRIGGER_MODE triggerMode) {
    MV_CAM_TRIGGER_MODE modeVal = MV_TRIGGER_MODE_OFF;//default continues
    
    if (TRIGGER_MODE_OFF == triggerMode) {
    	modeVal = MV_TRIGGER_MODE_OFF;
    } else if (TRIGGER_MODE_ON == triggerMode) {
    	modeVal = MV_TRIGGER_MODE_ON;
    }
    int ret = MV_CC_SetEnumValue(_cameraHandle, "TriggerMode", modeVal);
    if (ret != MV_OK) {
    	return BZL_E_DEVICE_SET_PARAM;
    } else {
    	return BZL_OK;
    }
    
}


CAMERA_TRIGGER_MODE HikvisionCamera::getTriggerMode() {
    CAMERA_TRIGGER_MODE modeVal = TRIGGER_MODE_NONE;
    MVCC_ENUMVALUE enumValue = {0};
    
    int ret = MV_CC_GetEnumValue(_cameraHandle, "TriggerMode", &enumValue);
    if (MV_OK != ret) {
    	return modeVal;
    }
    
    if (MV_TRIGGER_MODE_ON == enumValue.nCurValue) {
    	modeVal = TRIGGER_MODE_ON;
    } else if (MV_TRIGGER_MODE_OFF == enumValue.nCurValue) {
    	modeVal = TRIGGER_MODE_OFF;
    }
    return modeVal;
}
 
 
int HikvisionCamera::setTriggerSource(CAMERA_TRIGGER_SOURCE triggerSource) {
    MV_CAM_TRIGGER_SOURCE sourceVal = MV_TRIGGER_SOURCE_SOFTWARE;//default software
    
    if (TRIGGER_SOURCE_SOFTWARE == triggerSource) {
    	sourceVal = MV_TRIGGER_SOURCE_SOFTWARE;
    } else if (TRIGGER_SOURCE_LINE0 == triggerSource) {
    	sourceVal = MV_TRIGGER_SOURCE_LINE0;
    }
    
    int ret = MV_CC_SetEnumValue(_cameraHandle, "TriggerSource", sourceVal);
    if (MV_OK != ret) {
    	return BZL_E_DEVICE_SET_PARAM;
    } else {
    	return BZL_OK;
    }	
}



CAMERA_TRIGGER_SOURCE  HikvisionCamera::getTriggerSource() {
    CAMERA_TRIGGER_SOURCE sourceVal = TRIGGER_SOURCE_NONE;
    MVCC_ENUMVALUE enumValue = {0}; 
    
    int ret = MV_CC_GetEnumValue(_cameraHandle, "TriggerSource", &enumValue);
    if (MV_OK != ret) {
    	return sourceVal;
    }
    
    if (MV_TRIGGER_SOURCE_SOFTWARE == enumValue.nCurValue) {
    	sourceVal = TRIGGER_SOURCE_SOFTWARE;
    } else if (MV_TRIGGER_SOURCE_LINE0 == enumValue.nCurValue) {
    	sourceVal = TRIGGER_SOURCE_LINE0;
    }
    return sourceVal;	
}


int HikvisionCamera::softwareTrigger() {
    int ret = MV_CC_SetCommandValue(_cameraHandle, "TriggerSoftware");
    if (MV_OK != ret) {
    	return BZL_E_DEVICE_SOFTWARE_TRIGGER;
    } else {
    	return BZL_OK;
    }
}

int HikvisionCamera::setPixelType(IMAGE_PIXEL_TYPE pixelType) {
    unsigned int pixelVal = PixelType_Gvsp_Mono8; //default RGB24_PIXEL

    if (MONO8_PIXEL == pixelType) {
    	pixelVal = PixelType_Gvsp_Mono8;
    } else if (RGB24_PIXEL == pixelType) {
    	pixelVal = PixelType_Gvsp_RGB8_Packed;
    }
    
    int ret = MV_CC_SetPixelFormat(_cameraHandle, pixelVal);
    if (MV_OK != ret) {
        printf("error: SetPixelFormat fail [%x]\n", ret);
        return BZL_E_DEVICE_SET_PARAM;
    } else {
    	return BZL_OK;
    }	
    
}

IMAGE_PIXEL_TYPE HikvisionCamera::getPixelType() {
    IMAGE_PIXEL_TYPE pixelVal = NONE_PIXEL;
    
    MVCC_ENUMVALUE enumValue = {0};
    int ret = MV_CC_GetPixelFormat(_cameraHandle, &enumValue);
    if (MV_OK != ret) {
        printf("error: GetPixelFormat fail [%x]\n", ret);
        return pixelVal;
    }
    printf("MV_CC_GetPixelFormat=%d\n", enumValue.nCurValue);
    if (PixelType_Gvsp_Mono8 == enumValue.nCurValue) {
    	pixelVal = MONO8_PIXEL;
    } else if (PixelType_Gvsp_RGB8_Packed == enumValue.nCurValue) {
    	pixelVal = RGB24_PIXEL;
    }	
    
    return pixelVal;
}


int HikvisionCamera::setExposureTime(int nUs) {
    int ret = MV_CC_SetFloatValue(_cameraHandle, "ExposureTime", nUs);
    if (MV_OK != ret) {
        printf("error: SetFloatValue fail [%x]\n", ret);
        return BZL_E_DEVICE_SET_PARAM;
    } else {
    	return BZL_OK;
    }	
}

int HikvisionCamera::getExposureTime() {
    int exposureTime = 0.0; 
    MVCC_FLOATVALUE struFloatValue = {0}; 

    int ret = MV_CC_GetFloatValue(_cameraHandle, "ExposureTime", &struFloatValue);
    if (MV_OK != ret) {
        printf("error: GetFloatValue fail [%x]\n", ret); 
    } else {
    	exposureTime = struFloatValue.fCurValue;
    }
    
    return exposureTime;	
}


void HikvisionCamera::enableLightSync() {
    //LineSelector  LINE1->1
    //LineMode     OUTPUT->1
    //StrobeEnable  bool
    //StrobeLineDuration int us
    //设置Enum型参数
   unsigned int enMode = 1; //设置触发源为软触发

    int ret = MV_CC_SetEnumValue(_cameraHandle, "LineSelector", enMode);
    if (MV_OK != ret) {
        printf("1error: SetEnumValue fail [%x]\n", ret);
        return;
    }
    /*enMode = 1;
    ret = MV_CC_SetEnumValue(_cameraHandle, "LineMode", enMode);
    if (MV_OK != ret) {
        printf("2error: SetEnumValue fail [%x]\n", ret);
        return;
    }*/    


    ret = MV_CC_SetBoolValue(_cameraHandle, "StrobeEnable", true);
    if (MV_OK != ret) {
        printf("error: SetBoolValue fail [%x]\n", ret);
        return;
    }
    
    //设置int型参数
    unsigned int nValue = getExposureTime(); 
    ret = MV_CC_SetIntValue(_cameraHandle, "StrobeLineDuration", nValue);
    if (MV_OK != ret) {
        printf("error: SetIntValue fail [%x]\n", ret);
        return;
    }

    
}




     


