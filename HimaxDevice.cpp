// This is the main DLL file.

#include "stdafx.h"

#include "HimaxDevice.h"

//for vc2008
// include file : add Include_open_NI , Include_XnLib
// lib : Lib_win32 --> openNI.lib
/**
 * This is a Himax device driver module for OpenNI 2.2+
 *
 * It doesn't map to any physical device, but a dummy device.
 * You can send the map you want to it, and it will let OpenNI to use it.
 *
 * It also provide a property pool to accept any property.
 *
 * http://viml.nchc.org.tw/home/
 *
 * version 0.4 @2013/09/14
 */

// C Header
#include <string.h>

// STL Header
#include <array>
#include <map>
#include <string>
#include <vector>

// C Time header
#include <ctime>

// for debug only
#include <iostream>

// OpenNI Header
#include "Driver/OniDriverAPI.h"
#include "XnLib.h"

// HimaxDevice command
//#include "HimaxDevice.h"    
                               
typedef std::vector<unsigned char> CharVector;
typedef std::map< int, CharVector> MapData;
typedef MapData::iterator MapDataIter;

typedef std::pair<OniDeviceInfo*, oni::driver::DeviceBase*> DevicePair;
typedef std::map< std::string, DevicePair> MapDevice;
typedef MapDevice::iterator MapDeviceIter;

#ifdef _MSC_VER
#define __func__ __FUNCTION__
#endif
                               
#pragma region inline functions for propertry data
template<typename _T>
_T* PropertyConvert( oni::driver::DriverServices& rService, size_t uSize, void* pData )
{
    if( sizeof(_T) == uSize )
        return reinterpret_cast<_T*>( pData );

    rService.errorLoggerAppend( "The required property data size not match: %d != %d\n", uSize, sizeof(_T) );
    return NULL;
}

template<typename _T>
const _T* PropertyConvert( oni::driver::DriverServices& rService, size_t uSize, const void* pData )
{
    if( sizeof(_T) == uSize )
        return reinterpret_cast<const _T*>( pData );

    rService.errorLoggerAppend( "The required property data size not match: %d != %d\n", uSize, sizeof(const _T) );
    return NULL;
}

template<typename _T>
bool GetProperty( oni::driver::DriverServices& rService, size_t uSize, void* pData, _T& rValue )
{
    _T* pTData = PropertyConvert<_T>( rService, uSize, pData );
    if( pTData != NULL )
    {
        *pTData = rValue;
        return true;
    }
    return false;
}

template<typename _T>
bool SetProperty( oni::driver::DriverServices& rService, size_t uSize, const void* pData, _T& rValue )
{
    const _T* pTData = PropertyConvert<_T>( rService, uSize, pData );
    if( pTData != NULL )
    {
        rValue = *pTData;
        return true;
    }
    return false;
}

#pragma endregion

/**
 * This is a property pool to store any type of property
 */
class PropertyPool
{
public:
    MapData    m_Data;

public:
    PropertyPool( oni::driver::DriverServices& rService ) : m_Service( rService )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
    }

    bool GetProperty( int propertyId, void* data, int* pDataSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        MapDataIter itData = m_Data.find( propertyId );
        if( itData == m_Data.end() )
        {
            //std::cout << "request unsaved property: " << propertyId << std::endl;
            m_Service.errorLoggerAppend( "Required property '%d' not set.", propertyId );
            return false;
        }

        CharVector vData = itData->second;
        if( vData.size() == *pDataSize )
        {
            //memcpy( data, vData.data(), vData.size() );  
            memcpy( data, &vData[0], vData.size() ); //&vector[0]
            return true;
        }

        m_Service.errorLoggerAppend( "Required property '%d' data size not match: '%d != '%d''.", propertyId, vData.size(), *pDataSize );
        return false;
    }

    bool SetProperty( int propertyId, const void* data, int iSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        MapDataIter itData = m_Data.find( propertyId );
        std::vector<unsigned char>* pData = NULL;
        if( itData == m_Data.end() )
        {
            pData = &(m_Data[propertyId]);
            pData->resize( iSize );
        }
        else
        {
            m_Service.errorLoggerAppend( "Overwrite property '%d'", propertyId );
            pData = &(itData->second);
        }

        if( pData->size() == iSize )
        {
            //memcpy( pData->data(), data, pData->size() );
            memcpy( &pData[0], data, pData->size() ); //&vector[0]
            return true;
        }

        m_Service.errorLoggerAppend( "Required property '%d' data size not match: '%d != '%d''.", propertyId, pData->size(), iSize );
        return false;
    }

protected:
    oni::driver::DriverServices&                m_Service;

private:
    void operator=( const PropertyPool&);
};

/**
 *
 */
class OpenNIHimaxStream : public oni::driver::StreamBase
{
public:
    /**
     * Constructor
     */
    OpenNIHimaxStream( OniSensorType eSeneorType, oni::driver::DriverServices& driverServices ) : oni::driver::StreamBase(), m_rDriverServices(driverServices), m_Properties(driverServices)
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        m_eSensorType        = eSeneorType;
        m_bStarted            = false;
        m_iFrameId            = 0;

        m_bConfigDone                = false;

        // default video mode
        m_mVideoMode.resolutionX    = 320;
        m_mVideoMode.resolutionY    = 240;
        m_mVideoMode.fps            = 1;
        m_mVideoMode.pixelFormat    = ONI_PIXEL_FORMAT_DEPTH_1_MM;

        // default cropping
        m_mCropping.enabled    = false;
        m_mCropping.width    = m_mVideoMode.resolutionX;
        m_mCropping.height    = m_mVideoMode.resolutionY;
        m_mCropping.originX    = 0;
        m_mCropping.originY    = 0;
    }

    /**
     * Start load image
     */
    OniStatus start()
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        if( m_bConfigDone )
        {
            m_bStarted = true;
            return ONI_STATUS_OK;
        }
        m_rDriverServices.errorLoggerAppend( "Please assign VideoMode before start" );
        return ONI_STATUS_ERROR;
    }

    /**
     * Stop load image
     */
    void stop()
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        m_bStarted = false;
    }

    /**
     * Check if the property is supported
     */
    OniBool isPropertySupported( int )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        return true;
    }

    /**
     * get property
     */
    OniStatus getProperty( int propertyId, void* data, int* pDataSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch( propertyId )
        {
        case ONI_STREAM_PROPERTY_VIDEO_MODE:
            if( GetProperty( m_rDriverServices, *pDataSize, data, m_mVideoMode ) )
                return ONI_STATUS_OK;
            break;

        case ONI_STREAM_PROPERTY_CROPPING:
            if( GetProperty( m_rDriverServices, *pDataSize, data, m_mCropping ) )
                return ONI_STATUS_OK;
            break;

        case ONI_STREAM_PROPERTY_STRIDE:
            {
                int iStride = int(m_uStride);
                if( GetProperty( m_rDriverServices, *pDataSize, data, iStride ) )
                    return ONI_STATUS_OK;
            }
            break;

        default:
            if( m_Properties.GetProperty( propertyId, data, pDataSize ) )
                return ONI_STATUS_OK;
        }
        //std::cerr << " >>> Request Stream Property: " << propertyId << std::endl;
        return ONI_STATUS_ERROR;
    }

    /**
     * set property
     */
    OniStatus setProperty( int propertyId, const void* data, int dataSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch( propertyId )
        {
        case ONI_STREAM_PROPERTY_VIDEO_MODE:
            if( SetProperty( m_rDriverServices, dataSize, data, m_mVideoMode ) )
            {
                switch( m_mVideoMode.pixelFormat )
                {
                case ONI_PIXEL_FORMAT_RGB888:
                    m_uStride = m_mVideoMode.resolutionX * sizeof( OniRGB888Pixel );
                    break;

                case ONI_PIXEL_FORMAT_DEPTH_1_MM:
                case ONI_PIXEL_FORMAT_DEPTH_100_UM:
                    m_uStride = m_mVideoMode.resolutionX * sizeof( OniDepthPixel );
                    break;

                default:
                    m_rDriverServices.errorLoggerAppend( "Unsupported pixel format: %d", m_mVideoMode.pixelFormat );
                    return ONI_STATUS_ERROR;
                }
                
                m_uDataSize = m_uStride * m_mVideoMode.resolutionY;

                m_bConfigDone = true;
                return ONI_STATUS_OK;
            }
            break;

        case ONI_STREAM_PROPERTY_CROPPING:
            if( SetProperty( m_rDriverServices, dataSize, data, m_mCropping ) )
                return ONI_STATUS_OK;
            break;

        default:
            if( m_Properties.SetProperty( propertyId, data, dataSize ) )
                return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    OniStatus invoke( int commandId, void* data, int dataSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch( commandId )
        {
        case GET_HIMAX_STREAM_IMAGE:
            if( m_bStarted )
            {
                OniFrame** pFrame = PropertyConvert<OniFrame*>( m_rDriverServices, dataSize, data );
                if( pFrame != NULL )
                {
                    *pFrame = CreateeNewFrame();
                    if( pFrame != NULL )
                    {
                        (*pFrame)->croppingEnabled = m_mCropping.enabled;
                        (*pFrame)->cropOriginX = m_mCropping.originX;
                        (*pFrame)->cropOriginY = m_mCropping.originY;
                        (*pFrame)->height = m_mCropping.height;
                        (*pFrame)->width = m_mCropping.width;
                        
                        return ONI_STATUS_OK;
                    }
                }
            }
            else
            {
                return ONI_STATUS_ERROR;
            }
            break;

        case SET_HIMAX_STREAM_IMAGE:
            if( m_bStarted )
            {
                OniFrame** pFrame = PropertyConvert<OniFrame*>( m_rDriverServices, dataSize, data );
                if( pFrame != NULL )
                {
                    if( SendNewFrame( *pFrame ) )
                        return ONI_STATUS_OK;
                }
            }
            else
            {
                return ONI_STATUS_ERROR;
            }
            break;
        }
        return ONI_STATUS_NOT_IMPLEMENTED;
    }

    OniBool isCommandSupported( int commandId )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch( commandId )
        {
        case GET_HIMAX_STREAM_IMAGE:
        case SET_HIMAX_STREAM_IMAGE:
            return true;
            break;
        }

        return FALSE;
    }

    void notifyAllProperties()
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        for( MapDataIter itProp = m_Properties.m_Data.begin(); itProp != m_Properties.m_Data.end(); ++ itProp ) {
            //raisePropertyChanged( itProp->first, itProp->second.data(), itProp->second.size() );
            CharVector vData = itProp->second;
            raisePropertyChanged( itProp->first, &vData[0], itProp->second.size() );
        }
    }

protected:
    OniFrame* CreateeNewFrame()
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        OniFrame* pFrame = getServices().acquireFrame();
        if( pFrame != NULL )
        {
            // update metadata
            pFrame->frameIndex        = ++m_iFrameId;
            pFrame->videoMode        = m_mVideoMode;
            pFrame->width            = m_mVideoMode.resolutionX;
            pFrame->height            = m_mVideoMode.resolutionY;
            pFrame->cropOriginX        = pFrame->cropOriginY = 0;
            pFrame->croppingEnabled    = FALSE;
            pFrame->sensorType        = ONI_SENSOR_COLOR;
            pFrame->stride            = int( m_uStride );

            time_t    tNow;
            time( &tNow );
            pFrame->timestamp        = tNow;
        }
        return pFrame;
    }

    bool SendNewFrame( OniFrame* pFrame )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        if( pFrame->videoMode.pixelFormat == m_mVideoMode.pixelFormat &&
            pFrame->videoMode.resolutionX == m_mVideoMode.resolutionX &&
            pFrame->videoMode.resolutionY == m_mVideoMode.resolutionY )
        {
            raiseNewFrame( pFrame );
            getServices().releaseFrame( pFrame );
            return true;
        }
        getServices().releaseFrame( pFrame );
        return false;
    }

protected:
    bool            m_bStarted;
    bool            m_bConfigDone;

    OniSensorType    m_eSensorType;
    OniVideoMode    m_mVideoMode;
    OniCropping        m_mCropping;

    int                m_iFrameId;
    size_t            m_uDataSize;
    size_t            m_uStride;

    oni::driver::DriverServices&    m_rDriverServices;
    PropertyPool                    m_Properties;

private:
    OpenNIHimaxStream( const OpenNIHimaxStream& );
    void operator=( const OpenNIHimaxStream& );
};

//typedef std::array<OpenNIHimaxStream*,2>::iterator StreamIter;
typedef std::tr1::array<OpenNIHimaxStream*,2>::iterator StreamIter;

/**
 * Device
 */
class OpenNIHimaxDevice : public oni::driver::DeviceBase
{
public:
    /**
     * Constructor
     */
    OpenNIHimaxDevice( OniDeviceInfo* pInfo, oni::driver::DriverServices& driverServices ) : m_pInfo(pInfo), m_rDriverServices(driverServices)
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        m_bCreated = false;

        // set depth sensor
        m_aStream[0] = NULL;
        m_aSensor[0].sensorType = ONI_SENSOR_DEPTH;
        m_aSensor[0].numSupportedVideoModes    = 1;
        // set dummy supported video mode
        m_aSensor[0].pSupportedVideoModes    = new OniVideoMode[1];
        m_aSensor[0].pSupportedVideoModes[0].resolutionX    = 1;
        m_aSensor[0].pSupportedVideoModes[0].resolutionY    = 1;
        m_aSensor[0].pSupportedVideoModes[0].fps            = 1;
        m_aSensor[0].pSupportedVideoModes[0].pixelFormat    = ONI_PIXEL_FORMAT_DEPTH_1_MM;

        // set depth sensor
        m_aStream[1] = NULL;
        m_aSensor[1].sensorType = ONI_SENSOR_COLOR;
        m_aSensor[1].numSupportedVideoModes    = 1;
        // set dummy supported video mode
        m_aSensor[1].pSupportedVideoModes    = new OniVideoMode[1];
        m_aSensor[1].pSupportedVideoModes[0].resolutionX    = 1;
        m_aSensor[1].pSupportedVideoModes[0].resolutionY    = 1;
        m_aSensor[1].pSupportedVideoModes[0].fps            = 1;
        m_aSensor[1].pSupportedVideoModes[0].pixelFormat    = ONI_PIXEL_FORMAT_RGB888;

        m_bCreated = true;
    }

    /**
     * Destructor
     */
    ~OpenNIHimaxDevice(){
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
    }

    /**
     * getSensorInfoList
     */
    OniStatus getSensorInfoList( OniSensorInfo** pSensors, int* numSensors )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        *numSensors    = m_aSensor.size();
        *pSensors    = m_aSensor.data();

        return ONI_STATUS_OK;
    }

    /**
     * create Stream
     */
    oni::driver::StreamBase* createStream( OniSensorType sensorType )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        size_t idx = GetSensorIdx( sensorType );
        if( idx < m_aStream.size() )
        {
            if( m_aStream[idx] == NULL )
                m_aStream[idx] = new OpenNIHimaxStream( sensorType, m_rDriverServices );
            return m_aStream[idx];
        }

        m_rDriverServices.errorLoggerAppend( "The given sensor type '%d' is not supported", sensorType );
        return NULL;
    }

    /**
     * destroy Stream
     */
    void destroyStream( oni::driver::StreamBase* pStream )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        for( StreamIter itStream = m_aStream.begin(); itStream != m_aStream.end(); ++ itStream )
        {
            if( *itStream == pStream )
            {
                *itStream = NULL;
                delete pStream;
                break;
            }
        }
    }

    /**
     * get Property
     */
    OniStatus getProperty( int propertyId, void* data, int* pDataSize )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch (propertyId)
        {
        case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
            {
                OniVersion* pVersion = PropertyConvert<OniVersion>( m_rDriverServices, *pDataSize, data );
                if( pVersion != NULL )
                {
                    pVersion->major            = 0;
                    pVersion->minor            = 1;
                    pVersion->maintenance    = 0;
                    pVersion->build            = 0;
                    return ONI_STATUS_OK;
                }
            }
            break;

        default:
            m_rDriverServices.errorLoggerAppend( "Unknown property: %d\n", propertyId );
            std::cerr << " >>> Request Device Property: " << propertyId << std::endl;
            return ONI_STATUS_NOT_IMPLEMENTED;
        }
        return ONI_STATUS_ERROR;
    }

    /**
     * make sure if this device is created
     */
    bool Created() const
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        return m_bCreated;
    }

protected:
    size_t GetSensorIdx( OniSensorType sensorType )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        switch( sensorType )
        {
        case ONI_SENSOR_DEPTH:
            return 0;

        case ONI_SENSOR_COLOR:
            return 1;
        }
        return 100;
    }

private:
    OpenNIHimaxDevice( const OpenNIHimaxDevice& );
    void operator=( const OpenNIHimaxDevice& );

    bool            m_bCreated;
    OniDeviceInfo*    m_pInfo;
//    std::array<OniSensorInfo,2>            m_aSensor;
//    std::array<OpenNIHimaxStream*,2>    m_aStream;
    std::tr1::array<OniSensorInfo,2>    m_aSensor;
    std::tr1::array<OpenNIHimaxStream*,2>    m_aStream;
    oni::driver::DriverServices&        m_rDriverServices;
};

/**
 * Driver
 */
class OpenNIHimaxDriver : public oni::driver::DriverBase
{
public:
    /**
     * Constructor
     */
    OpenNIHimaxDriver( OniDriverServices* pDriverServices ) : DriverBase( pDriverServices )
    {
        //marky : here assign name
        // default values
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        m_sDeviceName    = "\\OpenNI2\\HimaxDevice\\";
        m_sVendorName    = "OpenNI2 Himax Device by Heresy";
        //m_sDeviceName    = "\\OpenCV\\Camera\\";
        //m_sVendorName    = "OpenCV Camera by Heresy";

        m_bListDevice    = true;
        m_iMaxTestNum    = 10;
        /*
        try
        {
            std::ifstream fileSetting( "OpenCVCamera.ini" );
            if( fileSetting.is_open() )
            {
                std::string sLine;
                while( std::getline( fileSetting, sLine ) )
                {
                    // ignore comment begin with ';'
                    if( sLine.size() < 5 || sLine[0] == ';' )
                        continue;
    
                    // split with '='
                    size_t uPos = sLine.find_first_of( '=' );
                    if( uPos != std::string::npos )
                    {
                        std::string sName    = sLine.substr( 0, uPos );
                        std::string sValue    = sLine.substr( uPos+1 ); 
                        if( sName == "device_name" )
                        {
                            m_sDeviceName = sValue;
                        }
                        else if( sName == "list_device" )
                        {
                            if( sValue == "0" )
                                m_bListDevice = false;
                        }
                        else if( sName == "max_device_num" )
                        {
                            std::stringstream(sValue) >> m_iMaxTestNum;
                        }
                        else if( sName == "test_mode" )
                        {
                            OniVideoMode mMode;
                            std::stringstream(sValue) >> mMode;
                            m_vModeToTest.push_back( mMode );
                        }
                    }
                }
                fileSetting.close();
            }
            else
            {
                // default video mode
                m_vModeToTest.push_back( BuildMode( 320, 240, 30 ) );
                m_vModeToTest.push_back( BuildMode( 640, 480, 30 ) );
            }
        }
        catch( std::exception e )
        {
            getServices().errorLoggerAppend( "Setting file read error '%s'", e.what() );
        }
        */
    }

    /**
     * Initialize
     */
    OniStatus initialize(   oni::driver::DeviceConnectedCallback connectedCallback,
                            oni::driver::DeviceDisconnectedCallback disconnectedCallback,
                            oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
                            void* pCookie )
    {
        m_bListDevice    = true;
        m_iMaxTestNum    = 10;
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        //return oni::driver::DriverBase::initialize( connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie );
        OniStatus eRes = oni::driver::DriverBase::initialize( connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie );
        if( eRes == ONI_STATUS_OK )
        {
            //TODO
            //1.read from ini file
            //2.open device by tryDevice
            
            //add by marky for ni viwer work
            //int iCounter = 0;
            std::string sUri = m_sDeviceName;
            // construct URI
            //std::stringstream ss;
            //ss << m_sDeviceName << ( iCounter );
            // create device info
            CreateDeviceInfo( sUri, 0 );   //in order to deviceConnected, if not will failed
           
           
            /*
            if( m_bListDevice )
            {
                // test how many OpenCV devices work on this machine
                int iCounter = 0;
                while( iCounter < m_iMaxTestNum )
                {
                    cv::VideoCapture mCamera( iCounter );
                    if( mCamera.isOpened() )
                    {
                        mCamera.release();

                        // construct URI
                        std::stringstream ss;
                        ss << m_sDeviceName << ( iCounter );

                        // create device info
                        CreateDeviceInfo( ss.str(), uint16_t(iCounter) );

                        ++iCounter;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            return ONI_STATUS_OK;
            */
        }
        else {
         
        }
        printf("himax %s, %d eRes=%d\n",__func__,__LINE__, eRes);  //add by marky for debug
        return eRes;

    }

    /**
     * Open device
     */
    oni::driver::DeviceBase* deviceOpen( const char* uri, const char* /*mode*/ )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        std::string sUri = uri;
        // find if the device is already in the list
        MapDeviceIter itDevice = m_mDevices.find( sUri );
        if( itDevice != m_mDevices.end() )
        {
            DevicePair& rDeviceData = itDevice->second;
            if( rDeviceData.second == NULL )
            {
                // create device if not created
                OpenNIHimaxDevice* pDevice = new OpenNIHimaxDevice( rDeviceData.first, getServices() );
                if( pDevice->Created() )
                {
                    rDeviceData.second = pDevice;
                    return pDevice;
                }
                else
                {
                    getServices().errorLoggerAppend( "Device '%s' create error", uri );
                    delete pDevice;
                    return NULL;
                }
            }
            else
            {
                // use created device directly
                return rDeviceData.second;
            }
        }

        getServices().errorLoggerAppend( "Can't find device: '%s'", uri );
        return NULL;
    }

    /**
     * Close device
     */
    void deviceClose( oni::driver::DeviceBase* pDevice )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        // find in device list
        for( MapDeviceIter itDevice = m_mDevices.begin(); itDevice != m_mDevices.end(); ++ itDevice )
        {
            DevicePair& rDeviceData = itDevice->second;
            if( rDeviceData.second == pDevice )
            {
                rDeviceData.second = NULL;
                delete pDevice;
                return;
            }
        }
    }

    /**
     * Test given URI
     */
    OniStatus tryDevice( const char* uri )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        std::string sUri = uri;
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        // Find in list first
        MapDeviceIter itDevice = m_mDevices.find( sUri );
        if( itDevice != m_mDevices.end() )
        {
            return ONI_STATUS_OK;
        }
        else
        {
            //marky : here check name
            printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
            // check if URI prefix is correct
            if( sUri.substr( 0, m_sDeviceName.length() ) == m_sDeviceName )
            {
                // get id
                try
                {
                    CreateDeviceInfo( sUri , 0);
                    return ONI_STATUS_OK;
                }
                catch( ... )
                {
                    getServices().errorLoggerAppend( "given uri '%s' parsing error", uri );
                    return ONI_STATUS_ERROR;
                }
            }
        }

        return DriverBase::tryDevice(uri);
    }

    /**
     * Shutdown
     */
    void shutdown()
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        for( MapDeviceIter itDevice = m_mDevices.begin(); itDevice != m_mDevices.end(); ++ itDevice )
        {
            DevicePair& rDeviceData = itDevice->second;
            delete rDeviceData.first;
            delete rDeviceData.second;
        }
    }

protected:
    /**
     * prepare OniDeviceInfo and device list
     */
    void CreateDeviceInfo( const std::string& sUri , uint16_t idx )
    {
        printf("himax %s, %d \n",__func__,__LINE__);  //add by marky for debug
        // Construct OniDeviceInfo
        OniDeviceInfo* pInfo = new OniDeviceInfo();
        strncpy_s( pInfo->vendor,    m_sVendorName.c_str(),    ONI_MAX_STR );
        strncpy_s( pInfo->name,    sUri.c_str(),            ONI_MAX_STR );
        strncpy_s( pInfo->uri,        sUri.c_str(),            ONI_MAX_STR );
        pInfo->usbProductId = uint16_t( idx );   //copy from opencv 

        // save device info
        m_mDevices[sUri] = std::make_pair( pInfo, (oni::driver::DeviceBase*)NULL );
        deviceConnected( pInfo );
        deviceStateChanged( pInfo, 0 );
    }

protected:
    /* add by marky*/
    bool                        m_bListDevice;
    int                            m_iMaxTestNum;   
    /**/
    std::string                    m_sDeviceName;
    std::string                    m_sVendorName;
    MapDevice m_mDevices;
};

ONI_EXPORT_DRIVER(OpenNIHimaxDriver);


