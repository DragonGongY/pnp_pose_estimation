#ifndef HIKVISION_CAMERA_H
#define HIKVISION_CAMERA_H

#include "BaseCamera.h"


class HikvisionCamera :public BaseCamera
{
public:
    HikvisionCamera(string CameraID): BaseCamera(CameraID), _cameraHandle(NULL) {
    	
    }
    
    ~HikvisionCamera() {
    	if (isOpen()) {
    		close();
    	}
    }
    
    
    virtual int open();    
    virtual void close();
    virtual bool isOpen();
    
    virtual int startGrabbing();
    virtual int stopGrabbing();        

    virtual int getPayloadSize();
    virtual int getOneFrameTimeout(ImageInfo &imgInfo, int nMsec);
    

    virtual int  setTriggerMode(CAMERA_TRIGGER_MODE triggerMode);
    virtual CAMERA_TRIGGER_MODE getTriggerMode();
    virtual int setTriggerSource(CAMERA_TRIGGER_SOURCE triggerSource);
    virtual CAMERA_TRIGGER_SOURCE  getTriggerSource();             
    virtual int softwareTrigger();
    virtual int setPixelType(IMAGE_PIXEL_TYPE pixelType);
    virtual IMAGE_PIXEL_TYPE getPixelType();

    virtual int setExposureTime(int nUs);
    virtual int getExposureTime();
    void enableLightSync();
private:
    void *_cameraHandle;    
   
};

#endif 
