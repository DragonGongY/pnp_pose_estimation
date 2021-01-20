#ifndef BASE_CAMERA_H
#define BASE_CAMERA_H

#include "CameraConfig.h"

class BaseCamera
{
public:
    BaseCamera(string cameraID): _cameraID(cameraID) {
    }
    
    virtual ~BaseCamera() {
    	
    }
    
    	
    virtual int open() = 0;    
    virtual void close() = 0;
    virtual bool isOpen() = 0;
    
    virtual int startGrabbing() = 0;
    virtual int stopGrabbing() = 0;        
    virtual int getPayloadSize() = 0;
    virtual int getOneFrameTimeout(ImageInfo &imgInfo, int nMsec) = 0;
    

    virtual int  setTriggerMode(CAMERA_TRIGGER_MODE triggerMode) = 0;
    virtual CAMERA_TRIGGER_MODE getTriggerMode() = 0;
    virtual int setTriggerSource(CAMERA_TRIGGER_SOURCE triggerSource) = 0;
    virtual CAMERA_TRIGGER_SOURCE  getTriggerSource() = 0;           
    virtual int softwareTrigger() = 0;
    virtual int setPixelType(IMAGE_PIXEL_TYPE pixelType) = 0;
    virtual IMAGE_PIXEL_TYPE getPixelType() = 0;
    
    virtual int setExposureTime(int nUs) = 0;
    virtual int getExposureTime() = 0;
protected:
    string _cameraID;

};


#endif 
