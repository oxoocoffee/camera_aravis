#include <camera_aravis/camera_nodelet.h>


namespace camera_aravis
{

// Conversions from integers to Arv types.
const char* CameraNodelet::szBufferStatusFromInt[] = {
  "ARV_BUFFER_STATUS_SUCCESS",
  "ARV_BUFFER_STATUS_CLEARED",
  "ARV_BUFFER_STATUS_TIMEOUT",
  "ARV_BUFFER_STATUS_MISSING_PACKETS",
  "ARV_BUFFER_STATUS_WRONG_PACKET_ID",
  "ARV_BUFFER_STATUS_SIZE_MISMATCH",
  "ARV_BUFFER_STATUS_FILLING",
  "ARV_BUFFER_STATUS_ABORTED"
};

ArvGvStream* CameraNodelet::CreateStream(void)
{
    gboolean 		bAutoBuffer = FALSE;
    gboolean 		bPacketResend = TRUE;
    unsigned int 	timeoutPacket = 40; // milliseconds
    unsigned int 	timeoutFrameRetention = 200;

    ArvGvStream* pStream = (ArvGvStream *)arv_device_create_stream (pDevice, NULL, NULL);
    if (pStream)
    {
        ArvBuffer	*pBuffer;
        gint 		 nbytesPayload;


        if (!ARV_IS_GV_STREAM (pStream))
            ROS_WARN("Stream is not a GV_STREAM");

        if (bAutoBuffer)
            g_object_set (pStream,
                          "socket-buffer",
                          ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                          "socket-buffer-size", 0,
                          NULL);
        if (!bPacketResend)
            g_object_set (pStream,
                          "packet-resend",
                          bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                          NULL);
        g_object_set (pStream,
                      "packet-timeout",
                      (unsigned) timeoutPacket * 1000,
                      "frame-retention", (unsigned) timeoutFrameRetention * 1000,
                      NULL);

        // Load up some buffers.
        nbytesPayload = arv_camera_get_payload (pCamera);
        for (int i=0; i<50; i++)
        {
            pBuffer = arv_buffer_new (nbytesPayload, NULL);
            arv_stream_push_buffer ((ArvStream *)pStream, pBuffer);
        }
    }
    return pStream;
} // CreateStream()



void CameraNodelet::RosReconfigure_callback(Config &newconfig, uint32_t level)
{
    int             changedAcquire;
    int             changedAcquisitionFrameRate;
    int             changedExposureAuto;
    int             changedGainAuto;
    int             changedExposureTimeAbs;
    int             changedGain;
    int             changedAcquisitionMode;
    int             changedTriggerMode;
    int             changedTriggerSource;
    int             changedSoftwarerate;
    int             changedFrameid;
    int             changedFocusPos;
    int             changedMtu;
    int             changedBinning;

    if (newconfig.frame_id == "")
        newconfig.frame_id = "camera";


    // Find what the user changed.
    changedAcquire    			= (newconfig.Acquire != config.Acquire);
    changedAcquisitionFrameRate         = (newconfig.AcquisitionFrameRate != config.AcquisitionFrameRate);
    changedExposureAuto 		= (newconfig.ExposureAuto != config.ExposureAuto);
    changedExposureTimeAbs  	        = (newconfig.ExposureTimeAbs != config.ExposureTimeAbs);
    changedGainAuto     		= (newconfig.GainAuto != config.GainAuto);
    changedGain         		= (newconfig.Gain != config.Gain);
    changedAcquisitionMode 		= (newconfig.AcquisitionMode != config.AcquisitionMode);
    changedTriggerMode  		= (newconfig.TriggerMode != config.TriggerMode);
    changedTriggerSource		= (newconfig.TriggerSource != config.TriggerSource);
    changedSoftwarerate  		= (newconfig.softwaretriggerrate != config.softwaretriggerrate);
    changedFrameid      		= (newconfig.frame_id != config.frame_id);
    changedFocusPos     		= (newconfig.FocusPos != config.FocusPos);
    changedMtu          		= (newconfig.mtu != config.mtu);
    changedBinning          		= (newconfig.Binning != config.Binning);

    // Limit params to legal values.
    newconfig.AcquisitionFrameRate      = CLIP(newconfig.AcquisitionFrameRate,         configMin.AcquisitionFrameRate, 	configMax.AcquisitionFrameRate);
    newconfig.ExposureTimeAbs      	= CLIP(newconfig.ExposureTimeAbs,  	configMin.ExposureTimeAbs,  		configMax.ExposureTimeAbs);
    newconfig.Gain          		= CLIP(newconfig.Gain,          		configMin.Gain,         			configMax.Gain);
    newconfig.FocusPos       		= CLIP(newconfig.FocusPos,      		configMin.FocusPos,      		    configMax.FocusPos);


    // Adjust other controls dependent on what the user changed.
    if (changedExposureTimeAbs || changedGainAuto || ((changedAcquisitionFrameRate || changedGain || changedFrameid
                                                       || changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) && newconfig.ExposureAuto=="Once"))
        newconfig.ExposureAuto = "Off";

    if (changedGain || changedExposureAuto || ((changedAcquisitionFrameRate || changedExposureTimeAbs || changedFrameid
                                                || changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) && newconfig.GainAuto=="Once"))
        newconfig.GainAuto = "Off";

    if (changedAcquisitionFrameRate)
        newconfig.TriggerMode = "Off";


    // Find what changed for any reason.
    changedAcquire    			= (newconfig.Acquire != config.Acquire);
    changedAcquisitionFrameRate = (newconfig.AcquisitionFrameRate != config.AcquisitionFrameRate);
    changedExposureAuto 		= (newconfig.ExposureAuto != config.ExposureAuto);
    changedExposureTimeAbs     	= (newconfig.ExposureTimeAbs != config.ExposureTimeAbs);
    changedGainAuto     		= (newconfig.GainAuto != config.GainAuto);
    changedGain            		= (newconfig.Gain != config.Gain);
    changedAcquisitionMode 		= (newconfig.AcquisitionMode != config.AcquisitionMode);
    changedTriggerMode  		= (newconfig.TriggerMode != config.TriggerMode);
    changedTriggerSource		= (newconfig.TriggerSource != config.TriggerSource);
    changedSoftwarerate  		= (newconfig.softwaretriggerrate != config.softwaretriggerrate);
    changedFrameid      		= (newconfig.frame_id != config.frame_id);
    changedFocusPos     		= (newconfig.FocusPos != config.FocusPos);
    changedMtu          		= (newconfig.mtu != config.mtu);
    changedBinning          		= (newconfig.Binning != config.Binning);


    // Set params into the camera.
    if (changedExposureTimeAbs)
    {
        if (isImplementedExposureTimeAbs)
        {
            ROS_INFO ("Set ExposureTimeAbs = %f", newconfig.ExposureTimeAbs);
            arv_device_set_float_feature_value (pDevice, "ExposureTimeAbs", newconfig.ExposureTimeAbs);
        }
        else
            ROS_INFO ("Camera does not support ExposureTimeAbs.");
    }

    if (changedGain)
    {
        if (isImplementedGain)
        {
            ROS_INFO ("Set gain = %f", newconfig.Gain);
            //arv_device_set_integer_feature_value (pDevice, "GainRaw", newconfig.GainRaw);
            arv_camera_set_gain (pCamera, newconfig.Gain);
        }
        else
            ROS_INFO ("Camera does not support Gain or GainRaw.");
    }

    if (changedExposureAuto)
    {
        if (isImplementedExposureAuto && isImplementedExposureTimeAbs)
        {
            ROS_INFO ("Set ExposureAuto = %s", newconfig.ExposureAuto.c_str());
            arv_device_set_string_feature_value (pDevice, "ExposureAuto", newconfig.ExposureAuto.c_str());
            if (newconfig.ExposureAuto=="Once")
            {
                ros::Duration(2.0).sleep();
                newconfig.ExposureTimeAbs = arv_device_get_float_feature_value (pDevice, "ExposureTimeAbs");
                ROS_INFO ("Get ExposureTimeAbs = %f", newconfig.ExposureTimeAbs);
                newconfig.ExposureAuto = "Off";
            }
        }
        else
            ROS_INFO ("Camera does not support ExposureAuto.");
    }
    if (changedGainAuto)
    {
        if (isImplementedGainAuto && isImplementedGain)
        {
            ROS_INFO ("Set GainAuto = %s", newconfig.GainAuto.c_str());
            arv_device_set_string_feature_value (pDevice, "GainAuto", newconfig.GainAuto.c_str());
            if (newconfig.GainAuto=="Once")
            {
                ros::Duration(2.0).sleep();
                //newconfig.GainRaw = arv_device_get_integer_feature_value (pDevice, "GainRaw");
                newconfig.Gain = arv_camera_get_gain (pCamera);
                ROS_INFO ("Get Gain = %f", newconfig.Gain);
                newconfig.GainAuto = "Off";
            }
        }
        else
            ROS_INFO ("Camera does not support GainAuto.");
    }

    if (changedAcquisitionFrameRate)
    {
        if (isImplementedAcquisitionFrameRate)
        {
            ROS_INFO ("Set %s = %f", keyAcquisitionFrameRate, newconfig.AcquisitionFrameRate);
            arv_device_set_float_feature_value (pDevice, keyAcquisitionFrameRate, newconfig.AcquisitionFrameRate);
        }
        else
            ROS_INFO ("Camera does not support AcquisitionFrameRate.");
    }

    if (changedTriggerMode)
    {
        if (isImplementedTriggerMode)
        {
            ROS_INFO ("Set TriggerMode = %s", newconfig.TriggerMode.c_str());
            arv_device_set_string_feature_value (pDevice, "TriggerMode", newconfig.TriggerMode.c_str());
        }
        else
            ROS_INFO ("Camera does not support TriggerMode.");
    }

    if (changedTriggerSource)
    {
        if (isImplementedTriggerSource)
        {
            ROS_INFO ("Set TriggerSource = %s", newconfig.TriggerSource.c_str());
            arv_device_set_string_feature_value (pDevice, "TriggerSource", newconfig.TriggerSource.c_str());
        }
        else
            ROS_INFO ("Camera does not support TriggerSource.");
    }

    if ((changedTriggerMode || changedTriggerSource || changedSoftwarerate) && newconfig.TriggerMode=="On" && newconfig.TriggerSource=="Software")
    {
        if (isImplementedAcquisitionFrameRate)
        {
            // The software rate is limited by the camera's internal framerate.  Bump up the camera's internal framerate if necessary.
            newconfig.AcquisitionFrameRate = configMax.AcquisitionFrameRate;
            ROS_INFO ("Set %s = %f", keyAcquisitionFrameRate, newconfig.AcquisitionFrameRate);
            arv_device_set_float_feature_value (pDevice, keyAcquisitionFrameRate, newconfig.AcquisitionFrameRate);
        }
    }

    if (changedTriggerSource || changedSoftwarerate)
    {
        // Recreate the software trigger callback.
        if (idSoftwareTriggerTimer)
        {
            g_source_remove(idSoftwareTriggerTimer);
            idSoftwareTriggerTimer = 0;
        }
        if (!strcmp(newconfig.TriggerSource.c_str(),"Software"))
        {
            ROS_INFO ("Set softwaretriggerrate = %f", 1000.0/ceil(1000.0 / newconfig.softwaretriggerrate));

            // Turn on software timer callback.
            idSoftwareTriggerTimer = g_timeout_add ((guint)ceil(1000.0 / newconfig.softwaretriggerrate), SoftwareTrigger_callback, pDevice);
        }
    }
    if (changedFocusPos)
    {
        if (isImplementedFocusPos)
        {
            ROS_INFO ("Set FocusPos = %d", newconfig.FocusPos);
            arv_device_set_integer_feature_value(pDevice, "FocusPos", newconfig.FocusPos);
            ros::Duration(1.0).sleep();
            newconfig.FocusPos = arv_device_get_integer_feature_value(pDevice, "FocusPos");
            ROS_INFO ("Get FocusPos = %d", newconfig.FocusPos);
        }
        else
            ROS_INFO ("Camera does not support FocusPos.");
    }
    if (changedMtu)
    {
        if (isImplementedMtu)
        {
            ROS_INFO ("Set mtu = %d", newconfig.mtu);
            arv_device_set_integer_feature_value(pDevice, "GevSCPSPacketSize", newconfig.mtu);
            ros::Duration(1.0).sleep();
            newconfig.mtu = arv_device_get_integer_feature_value(pDevice, "GevSCPSPacketSize");
            ROS_INFO ("Get mtu = %d", newconfig.mtu);
        }
        else
            ROS_INFO ("Camera does not support mtu (i.e. GevSCPSPacketSize).");
    }

    if (changedAcquisitionMode)
    {
        if (isImplementedAcquisitionMode)
        {
            ROS_INFO ("Set AcquisitionMode = %s", newconfig.AcquisitionMode.c_str());
            arv_device_set_string_feature_value (pDevice, "AcquisitionMode", newconfig.AcquisitionMode.c_str());

            ROS_INFO("AcquisitionStop");
            arv_device_execute_command (pDevice, "AcquisitionStop");
            ROS_INFO("AcquisitionStart");
            arv_device_execute_command (pDevice, "AcquisitionStart");
        }
        else
            ROS_INFO ("Camera does not support AcquisitionMode.");
    }

    if (changedAcquire)
    {
        if (newconfig.Acquire)
        {
            ROS_INFO("AcquisitionStart");
            arv_device_execute_command (pDevice, "AcquisitionStart");
        }
        else
        {
            ROS_INFO("AcquisitionStop");
            arv_device_execute_command (pDevice, "AcquisitionStop");
        }
    }

    if (changedBinning)
    {
        if(isImplementedBinning)
	{
	    if(newconfig.Binning == "Full")
	    {
	        if (dxMin <= 1 && dxMax >= 1 && dyMin <= 1 && dyMax >= 1)
		{
		    arv_camera_set_binning(pCamera, 1, 1);
		    ROS_INFO("Set Full Binning");
		}
		else
		{
		    ROS_ERROR("Full Binning is not supported, this is weird");
		}
	    }
	    else if (newconfig.Binning == "Half")
	    {
	        if (dxMin <= 2 && dxMax >= 2 && dyMin <= 2 && dyMax >= 2)
		{
		    arv_camera_set_binning(pCamera, 2, 2);
		    ROS_INFO("Set Half Binning");
		}
		else
		{
		    ROS_ERROR("Half Binning is not supported");
		}
	    }
	    else
	    {
		ROS_ERROR("Binning configuration %s is not implemented", newconfig.Binning.c_str());
	    }
	}
        else
            ROS_INFO ("Camera does not support Binning.");
    }

    config = newconfig;

} // RosReconfigure_callback()


void CameraNodelet::NewBuffer_callback (ArvStream *pStream, gpointer* data)
{
    CameraNodelet* This =reinterpret_cast<CameraNodelet*>(data);

    static uint64_t  cm = 0L;	// Camera time prev
    uint64_t  		 cn = 0L;	// Camera time now

#ifdef TUNING
    static uint64_t  rm = 0L;	// ROS time prev
#endif
    uint64_t  		 rn = 0L;	// ROS time now

    static uint64_t	 tm = 0L;	// Calculated image time prev
    uint64_t		 tn = 0L;	// Calculated image time now

    static int64_t   em = 0L;	// Error prev.
    int64_t  		 en = 0L;	// Error now between calculated image time and ROS time.
    int64_t  		 de = 0L;	// derivative.
    int64_t  		 ie = 0L;	// integral.
    int64_t			 u = 0L;	// Output of controller.

    int64_t			 kp1 = 0L;		// Fractional gains in integer form.
    int64_t			 kp2 = 1024L;
    int64_t			 kd1 = 0L;
    int64_t			 kd2 = 1024L;
    int64_t			 ki1 = -1L;		// A gentle pull toward zero.
    int64_t			 ki2 = 1024L;

    static uint32_t	 iFrame = 0;	// Frame counter.

    ArvBuffer		*pBuffer;

    pBuffer = arv_stream_try_pop_buffer (pStream);
    if (pBuffer != NULL)
    {
        if (arv_buffer_get_status (pBuffer) == ARV_BUFFER_STATUS_SUCCESS)
        {
            sensor_msgs::Image msg;

            size_t buffer_size;
            char *buffer_data = (char *) arv_buffer_get_data (pBuffer, &buffer_size);

            This->applicationData.nBuffers++;
            std::vector<uint8_t> this_data(buffer_size);
            memcpy(&this_data[0], buffer_data, buffer_size);


            // Camera/ROS Timestamp coordination.
            cn				= (uint64_t)arv_buffer_get_timestamp (pBuffer);	// Camera now
            rn	 			= ros::Time::now().toNSec();					// ROS now

            if (iFrame < 10)
            {
                cm = cn;
                tm  = rn;
            }

            // Control the error between the computed image timestamp and the ROS timestamp.
            en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
            de = en-em;
            ie += en;
            u = kp1*(en/kp2) + ki1*(ie/ki2) + kd1*(de/kd2);  // kp<0, ki<0, kd>0

            // Compute the new timestamp.
            tn = (uint64_t)((int64_t)tm + (int64_t)cn-(int64_t)cm + u);

#ifdef TUNING
            ROS_WARN("en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de, kp1*(en/kp2), ki1*(ie/ki2), kd1*(de/kd2), u);
            ROS_WARN("cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn-cm, rn-rm, (int64_t)tn-(int64_t)tm, tn-rn);
            msgInt64.data = tn-rn; //cn-cm+tn-tm; //
            ppubInt64->publish(msgInt64);
            rm = rn;
#endif

            // Save prior values.
            cm = cn;
            tm = tn;
            em = en;

            // Construct the image message.
            msg.header.stamp.fromNSec(tn);
            msg.header.seq = arv_buffer_get_frame_id (pBuffer);
            msg.header.frame_id = This->config.frame_id;
            msg.width = This->widthRoi;
            msg.height = This->heightRoi;
            msg.encoding = This->pszPixelformat;
            msg.step = msg.width * This->nBytesPixel;
            msg.data = this_data;

            // get current CameraInfo data
            sensor_msgs::CameraInfo camerainfo = This->pCameraInfoManager->getCameraInfo();
            camerainfo.header.stamp = msg.header.stamp;
            camerainfo.header.seq = msg.header.seq;
            camerainfo.header.frame_id = msg.header.frame_id;
            camerainfo.width = This->widthRoi;
            camerainfo.height = This->heightRoi;

            This->publisher.publish(msg, camerainfo);

        }
        else
            ROS_WARN_THROTTLE (5, "Frame error: %s", szBufferStatusFromInt[arv_buffer_get_status (pBuffer)]);

        arv_stream_push_buffer (pStream, pBuffer);
        iFrame++;
    }
} // NewBuffer_callback()


void CameraNodelet::ControlLost_callback (ArvGvDevice *pGvDevice, gpointer* data)
{
    ROS_ERROR ("Control lost.");
    bCancel = TRUE;
}

gboolean CameraNodelet::SoftwareTrigger_callback (void *device)
{
    arv_device_execute_command ((ArvDevice *)device, "TriggerSoftware");

    return TRUE;
}


// PeriodicTask_callback()
// Check for termination, and spin for ROS.
gboolean CameraNodelet::PeriodicTask_callback (void *data)
{
    CameraNodelet* This =reinterpret_cast<CameraNodelet*>(data);
    ApplicationData *pData = &This->applicationData;

    //  ROS_INFO ("Frame rate = %d Hz", pData->nBuffers);
    pData->nBuffers = 0;

    if (bCancel)
    {
        g_main_loop_quit (pData->main_loop);
        return FALSE;
    }

    ros::spinOnce();

    return TRUE;
} // PeriodicTask_callback()


// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX CameraNodelet::GetGcFirstChild(ArvGc *pGenicam, NODEEX nodeex)
{
    const char *szName=0;

    if (nodeex.pNode)
    {
        nodeex.pNode = arv_dom_node_get_first_child(nodeex.pNode);
        if (nodeex.pNode)
        {
            nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
            nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

            // Do the indirection.
            if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
            {
                szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
                nodeex.pNode  = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
                if (nodeex.pNode)
                {
                    nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
                }
            }
            else
            {
                nodeex.szTag = nodeex.szName;
            }
        }
        else
            nodeex.pNodeSibling = NULL;
    }
    else
    {
        nodeex.szName = NULL;
        nodeex.szTag = NULL;
        nodeex.pNodeSibling = NULL;
    }

    //ROS_INFO("GFC name=%s, node=%p, sib=%p", szNameChild, nodeex.pNode, nodeex.pNodeSibling);


    return nodeex;
} // GetGcFirstChild()


// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
NODEEX CameraNodelet::GetGcNextSibling(ArvGc *pGenicam, NODEEX nodeex)
{
    const char *szName=0;

    // Go to the sibling.
    nodeex.pNode = nodeex.pNodeSibling;
    if (nodeex.pNode)
    {
        nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
        nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

        // Do the indirection.
        if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
        {
            szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
            nodeex.pNode = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
            if (nodeex.pNode)
            {
                nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
            }
        }
        else
        {
            nodeex.szTag = nodeex.szName;
        }
    }
    else
    {
        nodeex.szName = NULL;
        nodeex.szTag = NULL;
        nodeex.pNodeSibling = NULL;
    }

    //ROS_INFO("GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode, nodeex.pNodeSibling);


    return nodeex;
} // GetGcNextSibling()


// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
void CameraNodelet::PrintDOMTree(ArvGc *pGenicam, NODEEX nodeex, int nIndent, bool debug)
{
    char		*szIndent=0;
    const char *szFeature=0;
    const char *szDomName=0;
    const char *szFeatureValue=0;

    szIndent = new char[nIndent+1];
    memset(szIndent,' ',nIndent);
    szIndent[nIndent]=0;

    nodeex = GetGcFirstChild(pGenicam, nodeex);
    if (nodeex.pNode)
    {
        do
        {
            if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode *)nodeex.pNode))
            {
                szDomName = arv_dom_node_get_node_name(nodeex.pNode);
                szFeature = arv_gc_feature_node_get_name((ArvGcFeatureNode *)nodeex.pNode);
                szFeatureValue = arv_gc_feature_node_get_value_as_string((ArvGcFeatureNode *)nodeex.pNode, NULL);
                if (szFeature && szFeatureValue && szFeatureValue[0])
                {
                    if (debug)
                    {
                        ROS_DEBUG("FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature, szFeatureValue);
                    }
                    else
                    {
                        ROS_INFO("FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature, szFeatureValue);
                    }
                }
            }
            PrintDOMTree(pGenicam, nodeex, nIndent+4, debug);

            // Go to the next sibling.
            nodeex = GetGcNextSibling(pGenicam, nodeex);
        } while (nodeex.pNode && nodeex.pNodeSibling);
    }
} //PrintDOMTree()


// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
void CameraNodelet::WriteCameraFeaturesFromRosparam(ros::NodeHandle& nh)
{
    XmlRpc::XmlRpcValue	 			 xmlrpcParams;
    XmlRpc::XmlRpcValue::iterator	 iter;
    ArvGcNode						*pGcNode;
    GError							*error=NULL;


    nh.getParam (ros::this_node::getNamespace(), xmlrpcParams);

    if (xmlrpcParams.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        for (iter=xmlrpcParams.begin(); iter!=xmlrpcParams.end(); iter++)
        {
            std::string		key = iter->first;

            pGcNode = arv_device_get_feature (pDevice, key.c_str());
            if (pGcNode && arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error))
            {
                //				unsigned long	typeValue = arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
                //				ROS_INFO("%s cameratype=%lu, rosparamtype=%d", key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

                // We'd like to check the value types too, but typeValue is often given as G_TYPE_INVALID, so ignore it.
                switch (iter->second.getType())
                {
                case XmlRpc::XmlRpcValue::TypeBoolean://if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))// && (typeValue==G_TYPE_INT64))
                {
                    int			value = (bool)iter->second;
                    arv_device_set_integer_feature_value(pDevice, key.c_str(), value);
                    ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
                }
                    break;

                case XmlRpc::XmlRpcValue::TypeInt: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))// && (typeValue==G_TYPE_INT64))
                {
                    int			value = (int)iter->second;
                    arv_device_set_integer_feature_value(pDevice, key.c_str(), value);
                    ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
                }
                    break;

                case XmlRpc::XmlRpcValue::TypeDouble: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))// && (typeValue==G_TYPE_DOUBLE))
                {
                    double		value = (double)iter->second;
                    arv_device_set_float_feature_value(pDevice, key.c_str(), value);
                    ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
                }
                    break;

                case XmlRpc::XmlRpcValue::TypeString: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))// && (typeValue==G_TYPE_STRING))
                {
                    std::string	value = (std::string)iter->second;
                    arv_device_set_string_feature_value(pDevice, key.c_str(), value.c_str());
                    ROS_INFO("Read parameter (string) %s: %s", key.c_str(), value.c_str());
                }
                    break;

                case XmlRpc::XmlRpcValue::TypeInvalid:
                case XmlRpc::XmlRpcValue::TypeDateTime:
                case XmlRpc::XmlRpcValue::TypeBase64:
                case XmlRpc::XmlRpcValue::TypeArray:
                case XmlRpc::XmlRpcValue::TypeStruct:
                default:
                    ROS_WARN("Unhandled rosparam type in WriteCameraFeaturesFromRosparam()");
                }
            }
        }
    }
} // WriteCameraFeaturesFromRosparam()

const char* CameraNodelet::GetPixelEncoding(ArvPixelFormat pixel_format)
{
    static std::string none;
    switch(pixel_format)
    {
    using namespace sensor_msgs::image_encodings;

    // supported grayscale encodings
    case ARV_PIXEL_FORMAT_MONO_8:         return MONO8.c_str();
    case ARV_PIXEL_FORMAT_MONO_8_SIGNED:  return TYPE_8SC1.c_str(); // OpenCV type
    case ARV_PIXEL_FORMAT_MONO_16:        return MONO16.c_str();

    // supported color encodings
    case ARV_PIXEL_FORMAT_RGB_8_PACKED:   return RGB8.c_str();
    case ARV_PIXEL_FORMAT_BGR_8_PACKED:   return BGR8.c_str();
    case ARV_PIXEL_FORMAT_RGBA_8_PACKED:  return RGBA8.c_str();
    case ARV_PIXEL_FORMAT_BGRA_8_PACKED:  return BGRA8.c_str();
    case ARV_PIXEL_FORMAT_YUV_422_PACKED: return YUV422.c_str();

    // supported bayer encodings
    case ARV_PIXEL_FORMAT_BAYER_GR_8:     return BAYER_GRBG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_RG_8:     return BAYER_RGGB8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GB_8:     return BAYER_GBRG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_BG_8:     return BAYER_BGGR8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GR_16:    return BAYER_GRBG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_RG_16:    return BAYER_RGGB16.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GB_16:    return BAYER_GBRG16.c_str();
    case ARV_PIXEL_FORMAT_BAYER_BG_16:    return BAYER_BGGR16.c_str();

// unsupported encodings
//  case ARV_PIXEL_FORMAT_BAYER_GR_10:
//  case ARV_PIXEL_FORMAT_BAYER_RG_10:
//  case ARV_PIXEL_FORMAT_BAYER_GB_10:
//  case ARV_PIXEL_FORMAT_BAYER_BG_10:
//  case ARV_PIXEL_FORMAT_BAYER_GR_12:
//  case ARV_PIXEL_FORMAT_BAYER_RG_12:
//  case ARV_PIXEL_FORMAT_BAYER_GB_12:
//  case ARV_PIXEL_FORMAT_BAYER_BG_12:
//  case ARV_PIXEL_FORMAT_BAYER_GR_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_RG_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_GB_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_BG_12_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_10_PACKED:
//  case ARV_PIXEL_FORMAT_BGR_10_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_12_PACKED:
//  case ARV_PIXEL_FORMAT_BGR_12_PACKED:
//  case ARV_PIXEL_FORMAT_YUV_411_PACKED:
//  case ARV_PIXEL_FORMAT_YUV_444_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_8_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_10_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_12_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_16_PLANAR:
//  case ARV_PIXEL_FORMAT_YUV_422_YUYV_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GR_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_RG_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GB_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_BG_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_YUV_422_YUYV_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GR_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_RG_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GB_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_BG_16:
    }

    return 0;
} // GetPixelEncoding()

void CameraNodelet::onInit()
{
    NODELET_DEBUG("Starting Camera Nodelet");
    //! We will be retrying to open camera until it is open, which may block the
    //! thread. Nodelet::onInit() should not block, hence spawning a new thread
    //! to do initialization.
    init_thread_ = boost::thread(boost::bind(&CameraNodelet::onInitImpl, this));
}

void CameraNodelet::onInitImpl()
{

//    ros::NodeHandle& nh = getNodeHandle(); // unused??
    ros::NodeHandle& nh = getPrivateNodeHandle();

    char   		*pszGuid = NULL;
    char    	 szGuid[512];
    int			 nInterfaces = 0;
    int			 nDevices = 0;
    int 		 i = 0;
    const char	*pkeyAcquisitionFrameRate[2] = {"AcquisitionFrameRate", "AcquisitionFrameRateAbs"};
    ArvGcNode	*pGcNode;
    GError		*error=NULL;

    applicationData.nBuffers = 0;
    applicationData.main_loop = 0;
    bCancel = FALSE;

    // TODO: support parameters, not just dynamic reconfigure
    config = config.__getDefault__();
    idSoftwareTriggerTimer = 0;

    // Print out some useful info.
    ROS_INFO ("Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    ROS_INFO ("# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    ROS_INFO ("# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
        ROS_INFO ("Device%d: %s", i, arv_get_device_id(i));

    // TODO: how did this work with multiple devices?
    if (nDevices>0)
    {

        if (nh.hasParam("guid"))
        {
            std::string		stGuid;

            nh.getParam("guid", stGuid);
            strcpy (szGuid, stGuid.c_str());
            pszGuid = szGuid;
        }
        else
            pszGuid = NULL;


        // Open the camera, and set it up.
        ROS_INFO("Opening: %s", pszGuid ? pszGuid : "(any)");
        while (TRUE)
        {
            pCamera = arv_camera_new(pszGuid);
            if (pCamera)
                break;
            else
            {
                ROS_WARN ("Could not open camera %s.  Retrying...", pszGuid);
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
        }

        pDevice = arv_camera_get_device(pCamera);
        ROS_INFO("Opened: %s-%s", arv_device_get_string_feature_value (pDevice, "DeviceVendorName"), arv_device_get_string_feature_value (pDevice, "DeviceID"));

        // Start the dynamic_reconfigure server. Don't set the callback yet so that we can override the default configuration
        dynamic_reconfigure::Server<Config>                    reconfigureServer;
        dynamic_reconfigure::Server<Config>::CallbackType      reconfigureCallback;
	reconfigureCallback = boost::bind(&CameraNodelet::RosReconfigure_callback, this,  _1, _2);
//        ros::Duration(2.0).sleep();

        // See if some basic camera features exist.
        pGcNode = arv_device_get_feature (pDevice, "AcquisitionMode");
        isImplementedAcquisitionMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "GainRaw");
        isImplementedGain = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
        pGcNode = arv_device_get_feature (pDevice, "Gain");
        isImplementedGain |= ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "ExposureTimeAbs");
        isImplementedExposureTimeAbs = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "ExposureAuto");
        isImplementedExposureAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "GainAuto");
        isImplementedGainAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "TriggerSelector");
        isImplementedTriggerSelector = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "TriggerSource");
        isImplementedTriggerSource = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "TriggerMode");
        isImplementedTriggerMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "FocusPos");
        isImplementedFocusPos = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "GevSCPSPacketSize");
        isImplementedMtu = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

        pGcNode = arv_device_get_feature (pDevice, "AcquisitionFrameRateEnableBinning");
        isImplementedAcquisitionFrameRateEnable = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

	isImplementedBinning = arv_camera_is_binning_available(pCamera);

        // Find the key name for framerate.
        keyAcquisitionFrameRate = NULL;
        for (i=0; i<2; i++)
        {
            pGcNode = arv_device_get_feature (pDevice, pkeyAcquisitionFrameRate[i]);
            isImplementedAcquisitionFrameRate = pGcNode ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
            if (isImplementedAcquisitionFrameRate)
            {
                keyAcquisitionFrameRate = pkeyAcquisitionFrameRate[i];
                break;
            }
        }

        // Get parameter bounds.
        arv_camera_get_exposure_time_bounds	(pCamera, &configMin.ExposureTimeAbs, &configMax.ExposureTimeAbs);
        arv_camera_get_gain_bounds			(pCamera, &configMin.Gain, &configMax.Gain);
        arv_camera_get_sensor_size			(pCamera, &widthSensor, &heightSensor);
        arv_camera_get_width_bounds			(pCamera, &widthRoiMin, &widthRoiMax);
        arv_camera_get_height_bounds		(pCamera, &heightRoiMin, &heightRoiMax);

	dxMin=1; dxMax=1; dyMin=1; dyMax=1;
	if (isImplementedBinning)
	{
	    arv_camera_get_x_binning_bounds(pCamera, &dxMin, &dxMax);
	    arv_camera_get_y_binning_bounds(pCamera, &dyMin, &dyMax);
	}

        if (isImplementedFocusPos)
        {
            gint64 focusMin64, focusMax64;
            arv_device_get_integer_feature_bounds (pDevice, "FocusPos", &focusMin64, &focusMax64);
            configMin.FocusPos = focusMin64;
            configMax.FocusPos = focusMax64;
        }
        else
        {
            configMin.FocusPos = 0;
            configMax.FocusPos = 0;
        }

        configMin.AcquisitionFrameRate =    0.0;
        configMax.AcquisitionFrameRate = 1000.0;

        // Initial camera settings
	// TODO: for now these are the only parameters that can be set without dynamic reconfigure
	// TODO: using getParam to not set a value when it is undefined is a little jenky, use nh.param with defaults instead?
	nh.getParam("ExposureTimeAbs", config.ExposureTimeAbs);
	nh.getParam("Gain", config.Gain);
	nh.getParam("AcquisitionFrameRate", config.AcquisitionFrameRate);
	nh.getParam("Binning", config.Binning);
	reconfigureServer.updateConfig(config); // sync up with dynamic reconfig so everyone has the same config
        if (isImplementedExposureTimeAbs)
            arv_device_set_float_feature_value(pDevice, "ExposureTimeAbs", config.ExposureTimeAbs);
        if (isImplementedGain)
            arv_camera_set_gain(pCamera, config.Gain);
        //arv_device_set_integer_feature_value(pDevice, "GainRaw", config.GainRaw);
        if (isImplementedAcquisitionFrameRateEnable)
            arv_device_set_integer_feature_value(pDevice, "AcquisitionFrameRateEnable", 1);
        if (isImplementedAcquisitionFrameRate)
            arv_device_set_float_feature_value(pDevice, keyAcquisitionFrameRate, config.AcquisitionFrameRate);
	if(isImplementedBinning)
	{
	    if(config.Binning == "Full")
	    {
	        if (dxMin <= 1 && dxMax >= 1 && dyMin <= 1 && dyMax >= 1)
		{
		    arv_camera_set_binning(pCamera, 1, 1);
		    ROS_INFO("Setting Full Binning");
		}
		else
		{
		    ROS_ERROR("Full Binning is not supported, this is weird");
		}
	    }
	    else if (config.Binning == "Half")
	    {
	        if (dxMin <= 2 && dxMax >= 2 && dyMin <= 2 && dyMax >= 2)
		{
		    arv_camera_set_binning(pCamera, 2, 2);
		    ROS_INFO("Setting Half Binning");
		}
		else
		{
		    ROS_ERROR("Half Binning is not supported");
		}
	    }
	    else
	    {
	        ROS_ERROR("Binning configuration is not implemented");
	    }
	}

        // Set up the triggering.
        if (isImplementedTriggerMode)
        {
            if (isImplementedTriggerSelector && isImplementedTriggerMode)
            {
                arv_device_set_string_feature_value(pDevice, "TriggerSelector", "AcquisitionStart");
                arv_device_set_string_feature_value(pDevice, "TriggerMode", "Off");
                arv_device_set_string_feature_value(pDevice, "TriggerSelector", "FrameStart");
                arv_device_set_string_feature_value(pDevice, "TriggerMode", "Off");
            }
        }


	// TODO: why are we writing the ros params?
        // WriteCameraFeaturesFromRosparam (nh);

        // Get parameter current values.
        xRoi=0; yRoi=0; widthRoi=0; heightRoi=0;
	dx=1; dy=1;
	arv_camera_get_binning(pCamera, &dx, &dy);
        arv_camera_get_region (pCamera, &xRoi, &yRoi, &widthRoi, &heightRoi);
        config.ExposureTimeAbs 	= isImplementedExposureTimeAbs ? arv_device_get_float_feature_value (pDevice, "ExposureTimeAbs") : 0;
        config.Gain      		= isImplementedGain ? arv_camera_get_gain (pCamera) : 0.0;
        pszPixelformat   		= GetPixelEncoding(arv_camera_get_pixel_format(pCamera));
        if(!pszPixelformat)
        {
            pszPixelformat = g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(pDevice, "PixelFormat")))->str;
            ROS_WARN("Pixelformat %s unsupported", pszPixelformat);
        }

        nBytesPixel      		= ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(arv_device_get_integer_feature_value(pDevice, "PixelFormat"));
        config.FocusPos  		= isImplementedFocusPos ? arv_device_get_integer_feature_value (pDevice, "FocusPos") : 0;


        // Print information.
        ROS_INFO ("    Using Camera Configuration:");
        ROS_INFO ("    ---------------------------");
        ROS_INFO ("    Vendor name          = %s", arv_device_get_string_feature_value (pDevice, "DeviceVendorName"));
        ROS_INFO ("    Model name           = %s", arv_device_get_string_feature_value (pDevice, "DeviceModelName"));
        ROS_INFO ("    Device id            = %s", arv_device_get_string_feature_value (pDevice, "DeviceID"));
        ROS_INFO ("    Sensor width         = %d", widthSensor);
        ROS_INFO ("    Sensor height        = %d", heightSensor);
        ROS_INFO ("    ROI x,y,w,h          = %d, %d, %d, %d", xRoi, yRoi, widthRoi, heightRoi);
        ROS_INFO ("    Pixel format         = %s", pszPixelformat);
        ROS_INFO ("    BytesPerPixel        = %d", nBytesPixel);
        ROS_INFO ("    Acquisition Mode     = %s", isImplementedAcquisitionMode ? arv_device_get_string_feature_value (pDevice, "AcquisitionMode") : "(not implemented in camera)");
        ROS_INFO ("    Trigger Mode         = %s", isImplementedTriggerMode ? arv_device_get_string_feature_value (pDevice, "TriggerMode") : "(not implemented in camera)");
        ROS_INFO ("    Trigger Source       = %s", isImplementedTriggerSource ? arv_device_get_string_feature_value(pDevice, "TriggerSource") : "(not implemented in camera)");
        ROS_INFO ("    Can set FrameRate:     %s", isImplementedAcquisitionFrameRate ? "True" : "False");
        if (isImplementedAcquisitionFrameRate)
        {
            config.AcquisitionFrameRate = arv_device_get_float_feature_value (pDevice, keyAcquisitionFrameRate);
            ROS_INFO ("    AcquisitionFrameRate = %g hz", config.AcquisitionFrameRate);
        }
        if (isImplementedBinning)
        {
	    ROS_INFO ("    Bin ranges [x], [y]  = [%d to %d], [%d to %d]", dxMin, dxMax, dyMin, dyMax);
	    ROS_INFO ("    Binning x, y         = %d, %d", dx, dy);
	}

        ROS_INFO ("    Can set Exposure:      %s", isImplementedExposureTimeAbs ? "True" : "False");
        if (isImplementedExposureTimeAbs)
        {
            ROS_INFO ("    Can set ExposureAuto:  %s", isImplementedExposureAuto ? "True" : "False");
            ROS_INFO ("    Exposure             = %g us in range [%g,%g]", config.ExposureTimeAbs, configMin.ExposureTimeAbs, configMax.ExposureTimeAbs);
        }

        ROS_INFO ("    Can set Gain:          %s", isImplementedGain ? "True" : "False");
        if (isImplementedGain)
        {
            ROS_INFO ("    Can set GainAuto:      %s", isImplementedGainAuto ? "True" : "False");
            ROS_INFO ("    Gain                 = %f %% in range [%f,%f]", config.Gain, configMin.Gain, configMax.Gain);
        }

        ROS_INFO ("    Can set FocusPos:      %s", isImplementedFocusPos ? "True" : "False");

        if (isImplementedMtu)
            ROS_INFO ("    Network mtu          = %lu", arv_device_get_integer_feature_value(pDevice, "GevSCPSPacketSize"));

        ROS_INFO ("    ---------------------------");


	// Print the tree of camera features, with their values.
	ROS_DEBUG ("    ----------------------------------------------------------------------------------");
	NODEEX		 nodeex;
	ArvGc	*pGenicam=0;
	pGenicam = arv_device_get_genicam(pDevice);

	nodeex.szName = "Root";
	nodeex.pNode = (ArvDomNode	*)arv_gc_get_node(pGenicam, nodeex.szName);
	nodeex.pNodeSibling = NULL;
	const bool USE_ROS_DEBUG = true;
	PrintDOMTree(pGenicam, nodeex, 0, USE_ROS_DEBUG); // use ROS_DEBUG
	ROS_DEBUG ("    ----------------------------------------------------------------------------------");

        // Start the camerainfo manager.
        pCameraInfoManager = new camera_info_manager::CameraInfoManager(nh, arv_device_get_string_feature_value (pDevice, "DeviceID"));

	// TODO: some camera config changed from when parameters were set, should we sync up the config with the current camera settings? or just keep them at what was requested?
	reconfigureServer.setCallback(reconfigureCallback);

        ArvGvStream *pStream = NULL;
        while (TRUE)
        {
            pStream = CreateStream();
            if (pStream)
                break;
            else
            {
                ROS_WARN("Could not create image stream for %s.  Retrying...", pszGuid);
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
        }


        // Set up image_raw.
        image_transport::ImageTransport		*pTransport = new image_transport::ImageTransport(nh);
//        publisher = pTransport->advertiseCamera("image_raw", 1);
        publisher = pTransport->advertiseCamera("image", 1);

        // Connect signals with callbacks.
        g_signal_connect (pStream, "new-buffer",   G_CALLBACK (NewBuffer_callback),   this);
        g_signal_connect (pDevice, "control-lost", G_CALLBACK (ControlLost_callback), this);
        g_timeout_add_seconds (1, PeriodicTask_callback, (void *)this);
        arv_stream_set_emit_signals ((ArvStream *)pStream, TRUE);


        void (*pSigintHandlerOld)(int);
        pSigintHandlerOld = signal (SIGINT, set_cancel);

        arv_device_execute_command (pDevice, "AcquisitionStart");

        applicationData.main_loop = g_main_loop_new (NULL, FALSE);
        g_main_loop_run (applicationData.main_loop);

        if (idSoftwareTriggerTimer)
        {
            g_source_remove(idSoftwareTriggerTimer);
            idSoftwareTriggerTimer = 0;
        }

        signal (SIGINT, pSigintHandlerOld);

        g_main_loop_unref (applicationData.main_loop);

        guint64 n_completed_buffers;
        guint64 n_failures;
        guint64 n_underruns;
        guint64 n_resent;
        guint64 n_missing;
        arv_stream_get_statistics ((ArvStream *)pStream, &n_completed_buffers, &n_failures, &n_underruns);
        ROS_INFO ("Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
        ROS_INFO ("Failures          = %Lu", (unsigned long long) n_failures);
        ROS_INFO ("Underruns         = %Lu", (unsigned long long) n_underruns);
        arv_gv_stream_get_statistics (pStream, &n_resent, &n_missing);
        ROS_INFO ("Resent buffers    = %Lu", (unsigned long long) n_resent);
        ROS_INFO ("Missing           = %Lu", (unsigned long long) n_missing);

        arv_device_execute_command (pDevice, "AcquisitionStop");

        g_object_unref (pStream);

    }
    else
        ROS_ERROR ("No cameras detected.");

    ros::shutdown();

    return;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_aravis::CameraNodelet, nodelet::Nodelet);
