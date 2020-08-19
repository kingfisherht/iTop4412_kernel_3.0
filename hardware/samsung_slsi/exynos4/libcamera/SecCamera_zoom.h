/*
**
** Copyright 2008, The Android Open Source Project
** Copyright 2010, Samsung Electronics Co. LTD
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/
  
#ifndef ANDROID_HARDWARE_CAMERA_SEC_H
#define ANDROID_HARDWARE_CAMERA_SEC_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>

#include <utils/RefBase.h>
#include <utils/threads.h>
#include <hardware/camera.h>
#include <videodev2.h>
#include <videodev2_samsung.h>
#include "sec_utils_v4l2.h"
#include "s5p_fimc.h"

#include "SecBuffer.h"
#include "exynos_mem.h"

#include <utils/String8.h>

#ifdef SAMSUNG_EXYNOS4210
#include "jpeg_api.h"
#endif

#ifdef SAMSUNG_EXYNOS4x12
#include "jpeg_hal.h"
#include "swscaler.h"
#endif

#include "Exif.h"
namespace android {

#ifndef BOARD_USE_V4L2
//#define ENABLE_ESD_PREVIEW_CHECK
//#define ZERO_SHUTTER_LAG
//#define IS_FW_DEBUG
#define VIDEO_SNAPSHOT

#if defined VIDEO_SNAPSHOT
#define ZERO_SHUTTER_LAG
#endif

//#if 1	//limeng 0705:redefine scalado Burst/HDR/Lowlight :pls don't modify correlative code without my review!

#if 0  //dg  change for ov5640 debug
#if defined ZERO_SHUTTER_LAG //limeng 120418
#define BURST_SHOT
//#define BURST_SHOT_PREV   //continuous shot request previous continuous frames
#define BURST_SHOT_NEXT   //SCF request subsequent continuous frames for burstshot
#define BURST_HDR   
#define EV_SKIP_FRAME 3
//for EV bracketing
#endif
#endif

#define USE_TORCH_SCENARIO   //limeng 120727
//add begin by suojp 2012-06-06
#define SCALADO_SCF
//add end by suojp 2012-06-06

#define USE_HUE //suojp 2012-04-26
//#define IS_FW_DEBUG

#define USE_FACE_DETECTION
#endif
#define USE_TOUCH_AF

//ISP HW reset when polling data failed, disabled in deflaut. Jiangshanbin 2012/08/09
#define SUPPORT_ISP_RESET

//Add work around to re-set param after streamon. Jiangshanbin 2012/07/27
#define SUPPORT_RESET_PARA_AFTER_STREAMON

#define S3_FRONT_CAMERA_CROP_ZOOM //limeng 0618:change zoom path,cyrstal modified for ov2675 zoom func ,2012-10-25

//#define S3_FRONT_CAMERA_CROP

#define TAF_CAF_OLDSCENARIO
#if 1 //limeng 0608:s3 decide to use which scenario
#define TAF_CAF_SCENARIO1 //CONTINUOUS TAF->LOST AF->CAF
#else
#define TAF_CAF_SCENARIO2 //SINGLE TAF>TOUCH CAF->CENTER CAF
#endif
#define TAF_CAF_FLASH //##mmkim 2012/08/09 -- for torch flash.

#define LOG_NDEBUG 0
#if defined(LOG_NDEBUG) && (LOG_NDEBUG == 0)
#define LOG_CAMERA LOGD
#define LOG_CAMERA_PREVIEW LOGD

#define LOG_TIME_DEFINE(n) \
    struct timeval time_start_##n, time_stop_##n; unsigned long log_time_##n = 0;

#define LOG_TIME_START(n) \
    gettimeofday(&time_start_##n, NULL);

#define LOG_TIME_END(n) \
    gettimeofday(&time_stop_##n, NULL); log_time_##n = measure_time_camera(&time_start_##n, &time_stop_##n);

#define LOG_TIME(n) \
    log_time_##n

#else
#define LOG_CAMERA(...)
#define LOG_CAMERA_PREVIEW(...)
#define LOG_TIME_DEFINE(n)
#define LOG_TIME_START(n)
#define LOG_TIME_END(n)
#define LOG_TIME(n)
#endif

#define FRM_RATIO(w, h)                 ((w)*100/(h))
#define SIZE_4K                         (1 << 12)

#define JOIN(x, y) JOIN_AGAIN(x, y)
#define JOIN_AGAIN(x, y) x ## y

//#define SC1_DVT1 //if use evt define 0 add.lzy//jmq.define Android.mk now

#if defined (SC1_DVT1)
#define FRONT_CAM HI253
#else
#define FRONT_CAM S5K8AAYX
#endif
#define BACK_CAM  S5K3H7

#if !defined (FRONT_CAM) || !defined(BACK_CAM)
#error "Please define the Camera module"
#endif

#define S5K3H7_PREVIEW_WIDTH             1280//640
#define S5K3H7_PREVIEW_HEIGHT            720//480
#define S5K3H7_SNAPSHOT_WIDTH            3232//3248//3264
#define S5K3H7_SNAPSHOT_HEIGHT           2424//2436//2448

#define S5K3H7_THUMBNAIL_WIDTH           320
#define S5K3H7_THUMBNAIL_HEIGHT          240
#define S5K3H7_THUMBNAIL_BPP             16

#define S5K3H7_FPS                       120//30

/* focal length of 3.43mm */
#define S5K3H7_FOCAL_LENGTH              343

#if defined (SC1_DVT1)

#define HI253_PREVIEW_WIDTH            640
#define HI253_PREVIEW_HEIGHT          480
#define HI253_SNAPSHOT_WIDTH          1280//1600//kun.bi modify for switch from 1920x1080(or other not 4:3) rear camera to 1600x1200 front camera,the preview screen is stretched
#define HI253_SNAPSHOT_HEIGHT         720//1200

#define HI253_THUMBNAIL_WIDTH         320///160
#define HI253_THUMBNAIL_HEIGHT        240///120
#define HI253_THUMBNAIL_BPP           16

#define HI253_FPS                    120
#define HI253_FOCAL_LENGTH            155
#else
#define S5K8AAYX_PREVIEW_WIDTH           640
#define S5K8AAYX_PREVIEW_HEIGHT          480
#define S5K8AAYX_SNAPSHOT_WIDTH          1280//640//kun.bi like HI253XXX
#define S5K8AAYX_SNAPSHOT_HEIGHT         720//480

#define S5K8AAYX_THUMBNAIL_WIDTH         320///160
#define S5K8AAYX_THUMBNAIL_HEIGHT        240///120
#define S5K8AAYX_THUMBNAIL_BPP           16

#define S5K8AAYX_FPS                    120

/***crystal copy from previous 
#define MT9D115_PREVIEW_WIDTH             640
#define MT9D115_PREVIEW_HEIGHT            480
#define MT9D115_SNAPSHOT_WIDTH            1600
#define MT9D115_SNAPSHOT_HEIGHT           1200
#define MT9D115_THUMBNAIL_WIDTH           320
#define MT9D115_THUMBNAIL_HEIGHT          240
#define MT9D115_THUMBNAIL_BPP             16
#define MT9D115_FPS                       30
#define MT9D115_FOCAL_LENGTH              90 	*/
/* focal length of 0.9mm */
#define S5K8AAYX_FOCAL_LENGTH            90
#endif

#define MAX_METERING_AREA              64

#define MAX_BACK_CAMERA_PREVIEW_WIDTH       JOIN(BACK_CAM,_PREVIEW_WIDTH)
#define MAX_BACK_CAMERA_PREVIEW_HEIGHT      JOIN(BACK_CAM,_PREVIEW_HEIGHT)
#define MAX_BACK_CAMERA_SNAPSHOT_WIDTH      JOIN(BACK_CAM,_SNAPSHOT_WIDTH)
#define MAX_BACK_CAMERA_SNAPSHOT_HEIGHT     JOIN(BACK_CAM,_SNAPSHOT_HEIGHT)

#define BACK_CAMERA_THUMBNAIL_WIDTH         JOIN(BACK_CAM,_THUMBNAIL_WIDTH)
#define BACK_CAMERA_THUMBNAIL_HEIGHT        JOIN(BACK_CAM,_THUMBNAIL_HEIGHT)
#define BACK_CAMERA_THUMBNAIL_BPP           JOIN(BACK_CAM,_THUMBNAIL_BPP)

#define BACK_CAMERA_FPS                     JOIN(BACK_CAM,_FPS)

#define BACK_CAMERA_FOCAL_LENGTH            JOIN(BACK_CAM,_FOCAL_LENGTH)

#define MAX_FRONT_CAMERA_PREVIEW_WIDTH      JOIN(FRONT_CAM,_PREVIEW_WIDTH)
#define MAX_FRONT_CAMERA_PREVIEW_HEIGHT     JOIN(FRONT_CAM,_PREVIEW_HEIGHT)
#define MAX_FRONT_CAMERA_SNAPSHOT_WIDTH     JOIN(FRONT_CAM,_SNAPSHOT_WIDTH)
#define MAX_FRONT_CAMERA_SNAPSHOT_HEIGHT    JOIN(FRONT_CAM,_SNAPSHOT_HEIGHT)

#define FRONT_CAMERA_THUMBNAIL_WIDTH        JOIN(FRONT_CAM,_THUMBNAIL_WIDTH)
#define FRONT_CAMERA_THUMBNAIL_HEIGHT       JOIN(FRONT_CAM,_THUMBNAIL_HEIGHT)
#define FRONT_CAMERA_THUMBNAIL_BPP          JOIN(FRONT_CAM,_THUMBNAIL_BPP)

#define FRONT_CAMERA_FPS                    JOIN(FRONT_CAM,_FPS)

#define FRONT_CAMERA_FOCAL_LENGTH           JOIN(FRONT_CAM,_FOCAL_LENGTH)

#define DEFAULT_JPEG_THUMBNAIL_WIDTH        256
#define DEFAULT_JPEG_THUMBNAIL_HEIGHT       192

#ifdef BOARD_USE_V4L2
#define CAMERA_DEV_NAME   "/dev/video1"
#else
#define CAMERA_DEV_NAME   "/dev/video0"
#endif

#ifdef SAMSUNG_EXYNOS4210
#define CAMERA_DEV_NAME3  "/dev/video2"
#endif

#ifdef SAMSUNG_EXYNOS4x12
#ifdef BOARD_USE_V4L2
#define CAMERA_DEV_NAME3  "/dev/video3"
#else
#define CAMERA_DEV_NAME3  "/dev/video1"
#endif
#ifdef ZERO_SHUTTER_LAG
#define CAMERA_DEV_NAME2  "/dev/video2"
#endif
#endif

#define DEV_EXYNOS_MEM    "/dev/fimg2d"
#define PFX_NODE_MEM   "/dev/fimg2d"

#define DEV_NAME3_RESERVED_SIZE  7168
#define DEV_NAME2_RESERVED_SIZE    76800//61440  //32768

#define CAMERA_DEV_NAME_TEMP "/data/videotmp_000"
#ifdef IS_FW_DEBUG
#define CAMERA_DEV_NAME2_TEMP "/data/videotemp_002"
#endif

#define FIMC_IS_DEV_NAME   "/dev/fimc_is"


#define BPP             2
#define MIN(x, y)       (((x) < (y)) ? (x) : (y))
#define MAX_BUFFERS     6

#ifdef ZERO_SHUTTER_LAG
#define CAP_BUFFERS     6
#define BURST_CAP_BUFFERS 5
#else
#define CAP_BUFFERS     1
#endif

#ifdef BOARD_USE_V4L2
#define MAX_PLANES      (3)
#define V4L2_BUF_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#else
#define MAX_PLANES      (1)
#define V4L2_BUF_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE
#endif

#ifdef BOARD_USE_V4L2_ION
#define V4L2_MEMORY_TYPE V4L2_MEMORY_USERPTR
#define RECORD_PIX_FMT V4L2_PIX_FMT_NV12M
#define PREVIEW_NUM_PLANE (3)
#define RECORD_NUM_PLANE (2)
#else
#define V4L2_MEMORY_TYPE V4L2_MEMORY_MMAP
#define RECORD_PIX_FMT V4L2_PIX_FMT_NV12
#define PREVIEW_NUM_PLANE (1)
#define RECORD_NUM_PLANE (1)
#endif

/*
 * V 4 L 2   F I M C   E X T E N S I O N S
 *
 */
#define V4L2_CID_ROTATION                   (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PADDR_Y                    (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PADDR_CB                   (V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PADDR_CR                   (V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_PADDR_CBCR                 (V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_STREAM_PAUSE               (V4L2_CID_PRIVATE_BASE + 53)

#define V4L2_CID_CAM_JPEG_MAIN_SIZE         (V4L2_CID_PRIVATE_BASE + 32)
#define V4L2_CID_CAM_JPEG_MAIN_OFFSET       (V4L2_CID_PRIVATE_BASE + 33)
#define V4L2_CID_CAM_JPEG_THUMB_SIZE        (V4L2_CID_PRIVATE_BASE + 34)
#define V4L2_CID_CAM_JPEG_THUMB_OFFSET      (V4L2_CID_PRIVATE_BASE + 35)
#define V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET   (V4L2_CID_PRIVATE_BASE + 36)
#define V4L2_CID_CAM_JPEG_QUALITY           (V4L2_CID_PRIVATE_BASE + 37)

#define TPATTERN_COLORBAR           1
#define TPATTERN_HORIZONTAL         2
#define TPATTERN_VERTICAL           3

#define V4L2_PIX_FMT_YVYU           v4l2_fourcc('Y', 'V', 'Y', 'U')

/* FOURCC for FIMC specific */
#define V4L2_PIX_FMT_VYUY           v4l2_fourcc('V', 'Y', 'U', 'Y')
#define V4L2_PIX_FMT_NV16           v4l2_fourcc('N', 'V', '1', '6')
#define V4L2_PIX_FMT_NV61           v4l2_fourcc('N', 'V', '6', '1')
#define V4L2_PIX_FMT_NV12T          v4l2_fourcc('T', 'V', '1', '2')
/*
 * U S E R   D E F I N E D   T Y P E S
 *
 */
#define PREVIEW_MODE 1
#define CAPTURE_MODE 2
#define RECORD_MODE 3
#define CAMERA_MAX_FACES 5

#ifdef IS_FW_DEBUG
#define FIMC_IS_FW_DEBUG_REGION_SIZE 512000
#define FIMC_IS_FW_DEBUG_REGION_ADDR 0x840000
#define SIZE_4K 4096
#define LOGD_IS(...) ((void)LOG(LOG_DEBUG, "IS_FW_DEBUG", __VA_ARGS__))
#endif

struct ExynosRect {
    int left;
    int top;
    int right;
    int bottom;
};

struct ExynosRect2 {
    int x;
    int y;
    int w;
    int h;
};

struct yuv_fmt_list {
    const char  *name;
    const char  *desc;
    unsigned int    fmt;
    int     depth;
    int     planes;
};

struct camsensor_date_info {
    unsigned int year;
    unsigned int month;
    unsigned int date;
};

class SecCamera : public virtual RefBase {
public:

    enum CAMERA_ID {
        CAMERA_ID_BACK  = 0,
        CAMERA_ID_FRONT = 1,
    };

    enum JPEG_QUALITY {
        JPEG_QUALITY_ECONOMY    = 0,
        JPEG_QUALITY_NORMAL     = 50,
        JPEG_QUALITY_SUPERFINE  = 100,
        JPEG_QUALITY_MAX,
    };

    enum OBJECT_TRACKING {
        OBJECT_TRACKING_OFF,
        OBJECT_TRACKING_ON,
        OBJECT_TRACKING_MAX,
    };

    /*VT call*/
    enum VT_MODE {
        VT_MODE_OFF,
        VT_MODE_ON,
        VT_MODE_MAX,
    };

    /*Camera sensor mode - Camcorder fix fps*/
    enum SENSOR_MODE {
        SENSOR_MODE_CAMERA,
        SENSOR_MODE_MOVIE,
    };

    /*Camera Shot mode*/
    enum SHOT_MODE {
        SHOT_MODE_SINGLE        = 0,
        SHOT_MODE_CONTINUOUS    = 1,
        SHOT_MODE_PANORAMA      = 2,
        SHOT_MODE_SMILE         = 3,
        SHOT_MODE_SELF          = 6,
    };

    enum CHK_DATALINE {
        CHK_DATALINE_OFF,
        CHK_DATALINE_ON,
        CHK_DATALINE_MAX,
    };

    enum CAM_MODE {
        PREVIEW     = 0,
        PICTURE     = 1,
        RECORDING   = 2,
        FLASHLIGHT =3,
    };

    int m_touch_af_start_stop;

    SecCamera();
    virtual ~SecCamera();

    static SecCamera* createInstance(void)
    {
        static SecCamera singleton;
        return &singleton;
    }
    status_t dump(int fd);

    bool            CreateCamera(int index);
    bool            DestroyCamera(void);
    int             getCameraId(void);
    void            initParameters(int index);
#ifdef SUPPORT_RESET_PARA_AFTER_STREAMON
    void            resetParameters(void);
#endif
#ifdef SUPPORT_ISP_RESET
    void            resetPreviewCamera(void);
#endif
    int             setMode(int recording_en);
    int             openExynosMemDev(int *fp);

    int             createFimc(int *fp, char *dev_name, int mode, int index);
    int             setFimc(void);
    int             setFimcForPreview(void);
    int             setFimcForRecord(void);
    int             setFimcForSnapshot(void);
    int             setFimcForBurstshot(void);
    int             ClearBurstshotFlag(void);
	int 			EnableContinuesFlag(void);
	int 			DisableContinuesFlag(void);
    int             setFimcSrc(int fd, int width, int height, camera_frame_metadata_t *facedata);
    int             setFimcDst(int fd, int width, int heifht, int pix_fmt, unsigned int addr);
    int             clearFimcBuf(int fd);
    int             runPreviewFimcOneshot(unsigned int src_addr, camera_frame_metadata_t *facedata);
    int             runPreviewFimcOneshotForVideoSnapshot(unsigned int src_addr);
    int             runRecordFimcOneshot(int index, unsigned int src_addr);
    int             runSnapshotFimcOneshot(unsigned int src_addr);
   int              runBurstshotFimcOneshot(int index,unsigned int src_addr);
    unsigned int    getShareBufferAddr(int index);
    int    getShareBufferFaceCount(int index);
    int    getShareBufferBlinkLevel(int index, int *blink_level);
    int             getSnapshotAddr(int index, SecBuffer *buffer);

    char *          getMappedAddr(void);
    int             startPreview(void);
    int             stopPreview(void);
    int             getPreviewState(void)
    {
        return m_preview_state;
    }
    void            clearPreviewState(void)
    {
        m_preview_state = 0;
    }
	void *    getVirtualAddr(unsigned int phs );

    int             startSnapshot(SecBuffer *yuv_buf);
    int             stopSnapshot(void);
    int             getSnapshot(void);
    int             setSnapshotFrame(int index);

    int             startRecord(bool recordHint);
    int             stopRecord(void);
    int             setPreviewFrame(int index);
    int             getRecordFrame(void);
    int             releaseRecordFrame(int index);
    int             getRecordAddr(int index, SecBuffer *buffer);
    int             getRecordPhysAddr(int index, SecBuffer *buffer);

    int             getPreview(camera_frame_metadata_t *facedata);
    int             setPreviewSize(int width, int height, int pixel_format);
    int             getPreviewSize(int *width, int *height, int *frame_size);
    int             getPreviewMaxSize(int *width, int *height);
    int             getPreviewPixelFormat(void);
    int             getPreviewSrcSize(int *width, int *height, int *frame_size);
    int             setPreviewImage(int index, unsigned char *buffer, int size);

    int             setVideosnapshotSize(int width, int height);
    int             getVideosnapshotSize(int *width, int *height, int *frame_size);
    int             setSnapshotSize(int width, int height);
    int             getSnapshotSize(int *width, int *height, int *frame_size);
    int             getSnapshotMaxSize(int *width, int *height);
    int             setSnapshotPixelFormat(int pixel_format);
    int             getSnapshotPixelFormat(void);

    unsigned char*  getJpeg(unsigned char *snapshot_data, int snapshot_size, int *size);
    unsigned char*  yuv2Jpeg(unsigned char *raw_data, int raw_size,
                                int *jpeg_size,
                                int width, int height, int pixel_format);

    int             setJpegThumbnailSize(int width, int height);
    int             getJpegThumbnailSize(int *width, int *height);

    int             setJpegThumbnailQuality(int jpeg_thumbnail_quality);
    int             getJpegThumbnailQuality(void);

    int             initSetParams(void);

    int             setAutofocus(void);
#ifdef TAF_CAF_FLASH	
    int             setTouchAF(int flash_mode);
#else
    int             setTouchAF(void);
#endif

    int             SetRotate(int angle);
    int             getRotate(void);

    int             setVerticalMirror(void);
    int             setHorizontalMirror(void);

    int             setWhiteBalance(int white_balance);
    int             getWhiteBalance(void);

    int             setBrightness(int brightness);
    int             getBrightness(void);

    int             setExposure(int exposure);
    int             getExposure(void);

    int             setImageEffect(int image_effect);
    int             getImageEffect(void);

    int             setSceneMode(int scene_mode);
    int             getSceneMode(void);

    int             setFlashMode(int flash_mode);
    int             getFlashMode(void);

#ifdef USE_TORCH_SCENARIO
    int             setTorchMode(int torch_mode);
    int             getTorchMode(void);
#endif

    int             setMetering(int metering_value);
    int             getMetering(void);
#if  1 //##mmkim 0608 -- AE area coordinate transition form google to YUV
    bool            setMeteringAreas(int num, ExynosRect *rects, int *weights);
#else
    bool            setMeteringAreas(int num, ExynosRect2 *rect2s, int *weights);
#endif
    int             getMaxNumMeteringAreas(void);

    int             setAutoExposureLock(int toggle);
    int             setAutoWhiteBalanceLock(int toggle);

    int             setISO(int iso_value);
    int             getISO(void);

    int             setContrast(int contrast_value);
    int             getContrast(void);

    int             setSaturation(int saturation_value);
    int             getSaturation(void);

    int             setSharpness(int sharpness_value);
    int             getSharpness(void);

    int             setHue(int hue_value);
    int             getHue(void);

    int             setWDR(int wdr_value);
    int             getWDR(void);

    int             setAntiShake(int anti_shake);
    int             getAntiShake(void);

    int             setJpegQuality(int jpeg_qality);
    int             getJpegQuality(void);

#ifdef S3_FRONT_CAMERA_CROP_ZOOM
    int             setFrontCameraZoom(int zoom_level);
#endif
#ifdef S3_FRONT_CAMERA_CROP
	int 			setFrontCameraCrop();
#endif
    int             setZoom(int zoom_level);
    int             getZoom(void);

    int             setObjectTracking(int object_tracking);
    int             getObjectTracking(void);
    int             getObjectTrackingStatus(void);

    int             setSmartAuto(int smart_auto);
    int             getSmartAuto(void);
    int             getAutosceneStatus(void);

    int             setBeautyShot(int beauty_shot);
    int             getBeautyShot(void);

    int             setVintageMode(int vintage_mode);
    int             getVintageMode(void);

    int             setFocusMode(int focus_mode);
    int             getFocusMode(void);

    int             setFaceDetect(int face_detect);
    int             getFaceDetect(void);

    int             setGPSLatitude(const char *gps_latitude);
    int             setGPSLongitude(const char *gps_longitude);
    int             setGPSAltitude(const char *gps_altitude);
    int             setGPSTimeStamp(const char *gps_timestamp);
    int             setGPSProcessingMethod(const char *gps_timestamp);
    int             cancelAutofocus(void);
    int             setFaceDetectLockUnlock(int facedetect_lockunlock);
	void 			m_secRect2toSecRect(ExynosRect2 *rect2, ExynosRect *rect);
	int 			setFocusAreas(int num, ExynosRect2* rect2s, int *weights);
	int 			setFocusAreas(int num, ExynosRect* rects, int *weights);
    int             setObjectPosition(int x, int y);
    int             setObjectTrackingStartStop(int start_stop);
    int             setTouchAFStartStop(int start_stop);
    int             setCAFStatus(int on_off);
    int             getAutoFocusResult(void);
    int             setAntiBanding(int anti_banding);
    int             getPostview(void);
    int             setRecordingSize(int width, int height);
    int             getRecordingSize(int *width, int *height);
    int             setGamma(int gamma);
    int             setSlowAE(int slow_ae);
    int             setExifOrientationInfo(int orientationInfo);
    int             setBatchReflection(void);
    int             setSnapshotCmd(void);
    int             endSnapshot(void);
    int             setCameraSensorReset(void);
    int             setSensorMode(int sensor_mode); /* Camcorder fix fps */
    int             setShotMode(int shot_mode);     /* Shot mode */
    int             setDataLineCheck(int chk_dataline);
    int             getDataLineCheck(void);
    int             setDataLineCheckStop(void);
    int             setDefultIMEI(int imei);
    int             getDefultIMEI(void);
    const __u8*     getCameraSensorName(void);
    bool             getUseInternalISP(void);
    bool             getFlashResult(void);
    bool             getFlashFrame(int index);
    int 		getEVValue(void);
    int                getEVFrame(int index);
    void             setCapIndex(int value);
    int               getCapIndex(void);
#ifdef ENABLE_ESD_PREVIEW_CHECK
    int             getCameraSensorESDStatus(void);
#endif // ENABLE_ESD_PREVIEW_CHECK

    int setFrameRate(int frame_rate);
    unsigned char*  getJpeg(int *jpeg_size,
                            int *thumb_size,
                            unsigned int *thumb_addr,
                            unsigned int *phyaddr);
    int             getSnapshotAndJpeg(SecBuffer *yuv_buf,
                                       int index,
                                       unsigned char *jpeg_buf,
                                       int *output_size);
    int             getVideoSnapshotAndJpeg(SecBuffer *yuv_buf,
                                       int index,
                                       unsigned char *jpeg_buf,
                                       int *output_size,
                                       unsigned char *thumnail_yuv,
                                       int thumbnail_width,
                                       int thumbnail_height);

    int             getExif(unsigned char *pExifDst, unsigned char *pThumbSrc, int thumbSize);

    void            getPostViewConfig(int*, int*, int*);
    void            getThumbnailConfig(int *width, int *height, int *size);

    int             getPostViewOffset(void);
    int             getCameraFd(enum CAM_MODE);
    int             getJpegFd(void);
    void            SetJpgAddr(unsigned char *addr);
    int             getPreviewAddr(int index, SecBuffer *buffer);
    int             getCaptureAddr(int index, SecBuffer *buffer);
    int             getBurstPhysAddr(int index, SecBuffer *buffer);
    int             pollAfLostState(unsigned int value);
    int             getAfLostState(void);
     int            AfLostPoll(void);
#ifdef IS_FW_DEBUG
    int             getDebugAddr(unsigned int *vaddr);
#endif
#ifdef BOARD_USE_V4L2_ION
    void            setUserBufferAddr(void *ptr, int index, int mode);
#endif
    static void     setJpegRatio(double ratio)
    {
        if((ratio < 0) || (ratio > 1))
            return;

        jpeg_ratio = ratio;
    }

    static double   getJpegRatio()
    {
        return jpeg_ratio;
    }

    static void     setInterleaveDataSize(int x)
    {
        interleaveDataSize = x;
    }

    static int      getInterleaveDataSize()
    {
        return interleaveDataSize;
    }

    static void     setJpegLineLength(int x)
    {
        jpegLineLength = x;
    }

    static int      getJpegLineLength()
    {
        return jpegLineLength;
    }

	int             getMiniExif(unsigned char *pExifDst);

private:
    v4l2_streamparm m_streamparm;
    struct sec_cam_parm   *m_params;
#ifdef SUPPORT_RESET_PARA_AFTER_STREAMON
    enum PARAM_RESET_FLAG {
        RESET_EXPOSURE = (1<<0),
        RESET_BRIGHTNESS = (1<<1),
        RESET_CONTRAST = (1<<2),
        RESET_SATURATION = (1<<3),
        RESET_SHARPNESS = (1<<4),
        RESET_HUE = (1<<5),
        RESET_SCENEMODE = (1<<6)
    };
    int             m_param_reset;
#endif
    int             m_flagCreate;
    int             m_preview_state;
    int             m_snapshot_state;
    int             m_camera_id;
    bool            m_camera_use_ISP;

    int             m_cam_fd;
    int             m_prev_fd;
    struct pollfd   m_events_c;

    int             m_cam_fd2;
    int             m_cap_fd;
    struct pollfd   m_events_c2;

    int             m_cam_fd3;
    int             m_rec_fd;
    struct pollfd   m_events_c3;
    int             m_flag_record_start;
#ifdef BURST_SHOT	
    int             m_flag_burst_start;
#endif
	bool 			mConFlag;
   struct pollfd    m_events_is;
   int          	m_fimc_is_fd;

#ifdef IS_FW_DEBUG
    int             m_mem_fd;
    off_t           m_debug_paddr;
#endif

    int             m_exynos_mem_fd_prev;
    int             m_exynos_mem_fd_rec;
    int             m_exynos_mem_fd_snap;
    void           *m_prev_mapped_addr;
    void           *m_rec_mapped_addr;
    void           *m_cap_mapped_addr;
	void           *m_fimc0_mapped_addr;
    unsigned int    m_snapshot_phys_addr;
    unsigned int m_burstshot_phys_addr[5];

    int             m_preview_v4lformat;
    int             m_preview_width;
    int             m_preview_height;
    int             m_preview_max_width;
    int             m_preview_max_height;
	
    int             m_preview_buf_max;
    int             m_preview_buf_cur;
    int             m_preview_buf_offset;
    mutable Mutex       mPrevFimcLock;

    int             m_snapshot_v4lformat;
    int             m_snapshot_width;
    int             m_snapshot_height;
    int             m_snapshot_max_width;
    int             m_snapshot_max_height;

    int             m_num_capbuf;
    int             m_videosnapshot_width;
    int             m_videosnapshot_height;

    int             m_angle;
    int             m_anti_banding;
    int             m_wdr;
    int             m_anti_shake;
    int             m_zoom_level;
    int             m_object_tracking;
    int             m_smart_auto;
    int             m_beauty_shot;
    int             m_vintage_mode;
    int             m_face_detect;
    int             m_object_tracking_start_stop;
    int             m_recording_en;
    bool            m_record_hint;
    int             m_recording_width;
    int             m_recording_height;
    int             m_recordshot_width;
    int             m_recordshot_height;
    long            m_gps_latitude;
    long            m_gps_longitude;
    long            m_gps_altitude;
    long            m_gps_timestamp;
    int             m_sensor_mode; /*Camcorder fix fps */
    int             m_shot_mode; /* Shot mode */
    int             m_exif_orientation;
    int             m_chk_dataline;
    int             m_video_gamma;
    int             m_slow_ae;
    int             m_camera_af_flag;
    int             m_auto_focus_state;
    bool            m_isTouchMetering;

    int             m_flag_camera_start;

    int             m_jpeg_fd;
    int             m_jpeg_thumbnail_width;
    int             m_jpeg_thumbnail_height;
    int             m_jpeg_thumbnail_quality;
    int             m_jpeg_quality;

    int 		m_postview_offset;
    int 		m_touchAFMode;
    int 		m_cap_index;
#ifdef ENABLE_ESD_PREVIEW_CHECK
    int             m_esd_check_count;
#endif // ENABLE_ESD_PREVIEW_CHECK

    exif_attribute_t mExifInfo;

    struct SecBuffer m_capture_buf[CAP_BUFFERS];
    struct SecBuffer m_buffers_share[MAX_BUFFERS];
    struct SecBuffer m_buffers_preview[MAX_BUFFERS];
    struct SecBuffer m_buffers_record[MAX_BUFFERS];
#ifdef BURST_SHOT	
    struct SecBuffer m_buffers_burst[BURST_CAP_BUFFERS];
#endif

    inline void     writeExifIfd(unsigned char **pCur,
                                 unsigned short tag,
                                 unsigned short type,
                                 unsigned int count,
                                 uint32_t value);
    inline void     writeExifIfd(unsigned char **pCur,
                                 unsigned short tag,
                                 unsigned short type,
                                 unsigned int count,
                                 unsigned char *pValue);
    inline void     writeExifIfd(unsigned char **pCur,
                                 unsigned short tag,
                                 unsigned short type,
                                 unsigned int count,
                                 rational_t *pValue,
                                 unsigned int *offset,
                                 unsigned char *start);
    inline void     writeExifIfd(unsigned char **pCur,
                                 unsigned short tag,
                                 unsigned short type,
                                 unsigned int count,
                                 unsigned char *pValue,
                                 unsigned int *offset,
                                 unsigned char *start);

    void            setExifChangedAttribute();
    void            setExifFixedAttribute();
    int             makeExif (unsigned char *exifOut,
                                        unsigned char *thumb_buf,
                                        unsigned int thumb_size,
                                        exif_attribute_t *exifInfo,
                                        unsigned int *size,
                                        bool useMainbufForThumb);
    void            resetCamera();


    static double   jpeg_ratio;
    static int      interleaveDataSize;
    static int      jpegLineLength;
};

extern unsigned long measure_time_camera(struct timeval *start, struct timeval *stop);

}; // namespace android

#endif // ANDROID_HARDWARE_CAMERA_SEC_H
