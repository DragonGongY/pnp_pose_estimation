#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include <string>
#include <vector>

using namespace std;


#define CAMERA_NUMBER       "CameraNumber"
#define CAMERA_DEVICE       "CameraDevice"
#define VENDOR_TYPE         "VendorType"
#define INTERFACE_TYPE      "InterfaceType"
#define DIMENSION_TYPE      "DimensionType"  
#define MODEL_NAME          "ModelName"
#define SERIAL_NUMBER       "SerialNumber"
#define USER_NAME           "UserName"

#define TRIGGER_MODE        "TriggerMode"
#define TRIGGER_SOURCE      "TriggerSource"
#define EXPOSURE_TIME       "ExposureTime"
#define PIXEL_TYPE          "PixelType"


typedef enum CAMERA_VENDOR_TYPE_ {
    NONE_VENDOR      = -1,
    HIKVISION_VENDOR  = 0,
    BASLER_VENDOR     = 1,
    IDLE_VENDOR
} CAMERA_VENDOR_TYPE;    

typedef enum CAMERA_INTERFACE_TYPE_ {
    NONE_INTERFACE = -1,
    GIGE_INTERFACE = 0,
    USB3_INTERFACE = 1,
    IDLE_INTERFACE
} CAMERA_INTERFACE_TYPE;

typedef enum CAMERA_DIMENSION_TYPE_ {
    NONE_DIMENSION    = -1,
    TWO_DIMENSION     = 0,
    THREE_DIMENSION   = 1,
    IDLE_DIMENSION
} CAMERA_DIMENSION_TYPE;    


typedef enum IMAGE_PIXEL_TYPE_ {
    NONE_PIXEL = -1,
    MONO8_PIXEL = 0,
    RGB24_PIXEL = 1,
	BGR24_PIXEL = 2,
	BAYER_GB_PIXEL = 3,
    DEPTH16_PIXEL = 4,
    IDLE_PIXEL
} IMAGE_PIXEL_TYPE;

typedef enum CAMERA_TRIGGER_MODE_{
    TRIGGER_MODE_NONE        = -1,
    TRIGGER_MODE_OFF         = 0,            // continues
    TRIGGER_MODE_ON          = 1,            // not continues
    TRIGGER_MODE_IDLE
} CAMERA_TRIGGER_MODE;
    
typedef enum CAMERA_TRIGGER_SOURCE_{
    TRIGGER_SOURCE_NONE           = -1,
    TRIGGER_SOURCE_SOFTWARE       = 0,           
    TRIGGER_SOURCE_LINE0          = 1,
    TRIGGER_SOURCE_IDLE
} CAMERA_TRIGGER_SOURCE;


enum ERROR_CODE {
    BZL_OK                             = 0,
    BZL_E_COMMON                       = -1,
    BZL_E_DEVICE_ENUM                  = -2,
    BZL_E_DEVICE_NO                    = -3,
    BZL_E_DEVICE_FIND                  = -4,
    BZL_E_DEVICE_CREATE_HANDLE         = -5, 
    BZL_E_DEVICE_OPEN                  = -6, 
    BZL_E_DEVICE_START                 = -7,
    BZL_E_DEVICE_STOP                  = -8,
    BZL_E_DEVICE_CAPTURE               = -9,
    BZL_E_DEVICE_SET_PARAM             = -10,
    BZL_E_DEVICE_GET_PARAM             = -11,
    BZL_E_DEVICE_SOFTWARE_TRIGGER      = -12,
	BZL_E_DEVICE_NOT_OPEN              = -13,
};


struct CameraConfig {
    CAMERA_VENDOR_TYPE    vendorType;
    CAMERA_INTERFACE_TYPE interfaceType;
    CAMERA_DIMENSION_TYPE dimensionType;
    string modelName;
    string serialNumber;
    string userName;
    CAMERA_TRIGGER_MODE triggerMode;
    CAMERA_TRIGGER_SOURCE triggerSource;
    int exposureTime;
    IMAGE_PIXEL_TYPE pixelType;    
};


struct ImageInfo {
    unsigned char  *data;           // ch:用于保存图像数据的缓存地址
    unsigned int   size;            // ch:缓存大小
    unsigned int   width;           // ch:图像宽 | en:Image Width
    unsigned int   height;          // ch:图像高 | en:Image Height
    IMAGE_PIXEL_TYPE    pixelType;  // ch:像素格式 | en:Pixel Type
    //unsigned int channel;         // ch:通道数 | en:image channel
};    

void CameraLoadConfig(string path, vector<CameraConfig> &configs);

#endif