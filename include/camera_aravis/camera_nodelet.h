// camera_aravis
//
// This is a ROS node that operates GenICam-based cameras via the Aravis library.
// Commonly available camera features are supported through the dynamic_reconfigure user-interface and GUI,
// and for those features not in the GUI but that are specific to a camera, they can be set in the
// camera by setting the appropriate parameter at startup.  This code reads those parameters, and
// if any of them match a camera feature, then the camera is written to.
//
// For example, if a camera has a feature called "IRFormat" that is an integer 0, 1, or 2, you can do
// rosparam set camnode/IRFormat 2
// and this driver will write it to the camera at startup.  Note that the datatype of the parameter
// must be correct for the camera feature (e.g. bool, int, double, string, etc), so for example you should use
// rosparam set camnode/GainAuto true
// and NOT
// rosparam set camnode/GainAuto 1
//

#include <arv.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <glib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>

#include <boost/thread.hpp>

#include "XmlRpc.h"

//#define TUNING	// Allows tuning the gains for the timestamp controller.  Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd

#define CLIP(x,lo,hi)	MIN(MAX((lo),(x)),(hi))
#define THROW_ERROR(m) throw std::string((m))

#define TRIGGERSOURCE_SOFTWARE	0
#define TRIGGERSOURCE_LINE1		1
#define TRIGGERSOURCE_LINE2		2

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)

namespace camera_aravis
{

typedef struct
{
  const char *szName;
  const char *szTag;
  ArvDomNode *pNode;
  ArvDomNode *pNodeSibling;
} NODEEX;


static volatile gboolean                  bCancel;

void set_cancel (int signal)
{
    bCancel = TRUE;
}

class CameraNodelet : public nodelet::Nodelet
{

  typedef camera_aravis::CameraAravisConfig Config;



  typedef struct
  {
    GMainLoop  *main_loop;
    int         nBuffers;	// Counter for Hz calculation.
  } ApplicationData;
  // ------------------------------------

  boost::thread init_thread_;

  // Conversions from integers to Arv types.
  const static char *szBufferStatusFromInt[];

  ArvGvStream *CreateStream(void);

  void RosReconfigure_callback(Config &config, uint32_t level);

  static void NewBuffer_callback(ArvStream *pStream, gpointer *data);

  static void ControlLost_callback(ArvGvDevice *pGvDevice, gpointer* data);

  static gboolean SoftwareTrigger_callback(void *pCamera);

  // PeriodicTask_callback()
  // Check for termination, and spin for ROS.
  static gboolean PeriodicTask_callback (void *data);

  // Get the child and the child's sibling, where <p___> indicates an indirection.
  NODEEX GetGcFirstChild(ArvGc *pGenicam, NODEEX nodeex);

  // Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
  NODEEX GetGcNextSibling(ArvGc *pGenicam, NODEEX nodeex);

  // Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
  void PrintDOMTree(ArvGc *pGenicam, NODEEX nodeex, int nIndent);

  // WriteCameraFeaturesFromRosparam()
  // Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
  // camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
  // in the camera.
  //
  // Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
  // looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
  // integers are integers, doubles are doubles, etc.
  //
  void WriteCameraFeaturesFromRosparam(ros::NodeHandle &nh);

  const char* GetPixelEncoding(ArvPixelFormat pixel_format);

  virtual void onInit();

  void onInitImpl();

private:
  ApplicationData                         applicationData;

  image_transport::CameraPublisher        publisher;
  camera_info_manager::CameraInfoManager *pCameraInfoManager;

  gint                                    width, height; // buffer->width and buffer->height not working, so I used a global.
  Config                                  config;
  Config                                  configMin;
  Config                                  configMax;
  int                                     idSoftwareTriggerTimer;

  int                                     isImplementedAcquisitionFrameRate;
  int                                     isImplementedAcquisitionFrameRateEnable;
  int                                     isImplementedGain;
  int                                     isImplementedExposureTimeAbs;
  int                                     isImplementedExposureAuto;
  int                                     isImplementedGainAuto;
  int                                     isImplementedFocusPos;
  int                                     isImplementedTriggerSelector;
  int                                     isImplementedTriggerSource;
  int                                     isImplementedTriggerMode;
  int                                     isImplementedAcquisitionMode;
  int                                     isImplementedMtu;

  int                                     xRoi;
  int                                     yRoi;
  int                                     widthRoi;
  int                                     widthRoiMin;
  int                                     widthRoiMax;
  int                                     heightRoi;
  int                                     heightRoiMin;
  int                                     heightRoiMax;

  int                                     widthSensor;
  int                                     heightSensor;

  const char                             *pszPixelformat;
  unsigned                                nBytesPixel;
  ArvCamera                              *pCamera;
  ArvDevice                              *pDevice;
  int                                     mtu;
  int                                     Acquire;
  const char                             *keyAcquisitionFrameRate;

};
}
