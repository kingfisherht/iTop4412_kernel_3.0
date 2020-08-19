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
//#define LOG_NDEBUG 0
#define LOG_TAG "CameraHardwareSec"
#include <utils/Log.h>

#include "SecCameraHWInterface_zoom.h"
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <camera/Camera.h>
#include <media/hardware/MetadataBufferType.h>
#include "snapshot_time_stamp.h"
#include "cutils/properties.h"

#define VIDEO_COMMENT_MARKER_H          0xFFBE
#define VIDEO_COMMENT_MARKER_L          0xFFBF
#define VIDEO_COMMENT_MARKER_LENGTH     4
#define JPEG_EOI_MARKER                 0xFFD9
#define HIBYTE(x) (((x) >> 8) & 0xFF)
#define LOBYTE(x) ((x) & 0xFF)

#define BACK_CAMERA_AUTO_FOCUS_DISTANCES_STR       "0.10,1.20,Infinity"
#define BACK_CAMERA_MACRO_FOCUS_DISTANCES_STR      "0.10,0.20,Infinity"
#define BACK_CAMERA_INFINITY_FOCUS_DISTANCES_STR   "0.10,1.20,Infinity"
#define FRONT_CAMERA_FOCUS_DISTANCES_STR           "0.20,0.25,Infinity"
#define USE_EGL

#define PICTURE_TIMESTAMP_LEN  19

const char KEY_PICTURE_TIME[]       = "picture-time";
const char VALUE_PICTURE_TIME_OFF[] = "offtime";

#define LIKELY( exp )       (__builtin_expect( (exp) != 0, true  ))
#define UNLIKELY( exp )     (__builtin_expect( (exp) != 0, false ))

// This hack does two things:
// -- it sets preview to NV21 (YUV420SP)
// -- it sets gralloc to YV12
//
// The reason being: the samsung encoder understands only yuv420sp, and gralloc
// does yv12 and rgb565.  So what we do is we break up the interleaved UV in
// separate V and U planes, which makes preview look good, and enabled the
// encoder as well.
//
// FIXME: Samsung needs to enable support for proper yv12 coming out of the
//        camera, and to fix their video encoder to work with yv12.
// FIXME: It also seems like either Samsung's YUV420SP (NV21) or img's YV12 has
//        the color planes switched.  We need to figure which side is doing it
//        wrong and have the respective party fix it.

namespace android {

struct addrs {
    uint32_t type;  // make sure that this is 4 byte.
    unsigned int addr_y;
    unsigned int addr_cbcr;
    unsigned int buf_index;
    unsigned int reserved;
};

struct addrs_cap {
    unsigned int addr_y;
    unsigned int width;
    unsigned int height;
};

static const int INITIAL_SKIP_FRAME = 4;   //mmkim 0612 -- 2. change initial frame drop count
static const int INITIAL_SKIP_FRAME_FRONT = 0; //bk front camera needn't skip 10 frames
static const int EFFECT_SKIP_FRAME = 1;

gralloc_module_t const* CameraHardwareSec::mGrallocHal;

CameraHardwareSec::CameraHardwareSec(int cameraId, camera_device_t *dev)
    :
      mCaptureInProgress(false),
      #ifdef BURST_SHOT
      mburstencodeflag(false),
      mburstbufferflag(false),
      mbursthdrflag(false),
      mBurststartIndex(-1),
      mBreakBurstFimcThread(false),
      #endif
      mParameters(),
      mFrameSizeDelta(0),
      mCameraSensorName(NULL),
      mUseInternalISP(false),
      mSkipFrame(0),
      mNotifyCb(0),
      mDataCb(0),
      mDataCbTimestamp(0),
      mCallbackCookie(0),
      mMsgEnabled(CAMERA_MSG_RAW_IMAGE),
      mRecordRunning(false),
      mPostViewWidth(0),
      mPostViewHeight(0),
      mPostViewSize(0),
      mCapIndex(0),
      mCurrentIndex(-1),
      mOldRecordIndex(-1),
      mOldPreviewIndex(-1),
      mOldBurstIndex(-1),
      mRecordHint(false),
      mRunningSetParam(0),
      mTouched(0),
      mFDcount(0),
      mRunningThread(0),
      mhdr_count(0),
      //add begin by suojp 2012-04-11
      #ifdef S3_CHECK_PREVIEW_LIGHT
      mCheckPreviewStep(30),
      mCheckPreviewCount(0),
      mPreviewCount(0),
      mCheckLightWorking(false),
      mPreviewLightValueSum(0.0f),
      mCheckPreviewFake(0),
      mPreviewFormat(V4L2_PIX_FMT_YVU420),
      mPreviewBackLightingValueSum(0.0f),
      mCheckLightLib({0}),
      #endif
      //add end by suojp 2012-04-11
      //add begin by suojp 2012-04-29
      #ifdef S3_CONTINOUS_SHOT
      mbufContinousSaveFile(NULL),
      mExitContinousSaveFileThread(false),
      mExitContinousShotThread(true),
      mContinousShotIndex(0),
      mbufContinousMaxCount(1000),
      mTimeIntervalMin(-1),
      #endif
      //add end by suojp 2012-04-29
      //add begin by suojp 2012-05-05
      #ifdef S3_BLINK_DETECTION
      mBlinkLevelMax(10),
      mBlinkCheckCount(30),
      #endif
      //add end by suojp 2012-05-05
      //add begin by suojp 2012-06-04
      #ifdef S3_LOSSFOCUS_NOTIFY
      mFocusSave(0),
      mExitLossFocusNotifyThread(false),
      #endif
      //add end by suojp 2012-06-04
      //add begin by suojp 2012-06-05
      #ifdef S3_FPS_DETECTION
      mCapture_Cnt(0),
      #endif

      #ifdef S3_STATUS_UPDATE
      mExitStatusThread(false),
      #endif
      //add end by suojp 2012-06-05
      //add begin by suojp 2012-06-18
      #ifdef S3_IGNORE_PREVIEW_WINDOW
      bIgnorePreviewWindow(false),
      #endif
      //add end by suojp 2012-06-18
      mHalDevice(dev)
{
    LOGD("%s :", __func__);
    memset(&mCapBuffer, 0, sizeof(struct SecBuffer));
    int ret = 0;

#ifdef BURST_SHOT
    for(int i = 0; i <BURST_CAP_BUFFERS; i++)
        mburst_index[i] = -1;
    for(int i = 0; i <3; i++)
        mhdr_index[i] = -1;
#endif   

    //add begin by suojp 2012-04-29
#ifdef S3_CONTINOUS_SHOT
    strcpy(mstrContinousSaveFilePath, "/sdcard/DCIM/Camera/dump_");
#endif
    //add end by suojp 2012-04-29

    mPreviewWindow = NULL;
    mSecCamera = SecCamera::createInstance();

    mRawHeap = NULL;
    mPreviewHeap = NULL;
    mFaceDataHeap = NULL;
    for(int i = 0; i < BUFFER_COUNT_FOR_ARRAY; i++)
        mRecordHeap[i] = NULL;

    if (!mGrallocHal) {
        ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&mGrallocHal);
        if (ret)
            LOGE("ERR(%s):Fail on loading gralloc HAL", __func__);
    }

    ret = mSecCamera->CreateCamera(cameraId);
    if (ret < 0) {
        LOGE("ERR(%s):Fail on mSecCamera init", __func__);
        mSecCamera->DestroyCamera();
    }

    initDefaultParameters(cameraId);

    mExitAutoFocusThread = false;
    mExitPreviewThread = false;
    /* whether the PreviewThread is active in preview or stopped.  we
     * create the thread but it is initially in stopped state.
     */
    mPreviewRunning = false;
    mPreviewStartDeferred = false;
    //add begin by suojp 2012-06-06
#ifdef SCALADO_SCF    
    mBurstCapture=0;
#endif
    //add end by suojp 2012-06-06
    
    mPreviewThread = new PreviewThread(this);
    mPreviewFimcThread = new PreviewFimcThread(this);
    mRecordFimcThread = new RecordFimcThread(this);
    mRecordThread = new RecordThread(this);
#ifdef BURST_SHOT	
    mBurstshotFimcThread = new BurstshotFimcThread(this);
    mBurstpictureThread = new BurstpictureThread(this);
#endif 
    mCallbackThread = new CallbackThread(this);
    mAutoFocusThread = new AutoFocusThread(this);
    mPictureThread = new PictureThread(this);
#ifdef IS_FW_DEBUG
    if (mUseInternalISP) {
        mPrevOffset = 0;
        mCurrOffset = 0;
        mPrevWp = 0;
        mCurrWp = 0;
        mDebugVaddr = 0;
        mStopDebugging = false;
        mDebugThread = new DebugThread(this);
    }
#endif
}

int CameraHardwareSec::getCameraId() const
{
    return mSecCamera->getCameraId();
}

void CameraHardwareSec::initDefaultParameters(int cameraId)
{
    if (mSecCamera == NULL) {
        LOGE("ERR(%s):mSecCamera object is NULL", __func__);
        return;
    }

    CameraParameters p;
    CameraParameters ip;

    mCameraSensorName = mSecCamera->getCameraSensorName();
    if (mCameraSensorName == NULL) {
        LOGE("ERR(%s):mCameraSensorName is NULL", __func__);
        return;
    }
    LOGI("CameraSensorName: %s", mCameraSensorName);

    int preview_max_width   = 0;
    int preview_max_height  = 0;
    int snapshot_max_width  = 0;
    int snapshot_max_height = 0;

    mCameraID = cameraId;
    //  mUseInternalISP = true;//mSecCamera->getUseInternalISP();

    mUseInternalISP = mSecCamera->getUseInternalISP(); //dg change for debug ov5640


    ///   if (cameraId == SecCamera::CAMERA_ID_FRONT) 	mUseInternalISP =true;//crystal.wang add
    ///
    ///
    /*
s3 request:
type    |      resolution          |   ratio (100*w/h)   |  videosnapshot resolution  |   fps
picture
               3232x2424              4  : 3   (133)               3232x2424             15fps
               3200x1800              16 : 9   (177)               3200x1800             22fps
               3040x1824              5  : 3   (166)               3040x1824             22fps
               2592x1944              4  : 3   (133)               2592x1944             22fps
               1920x1080              16 : 9   (177)               3072x1728             22fps
               1280x720               16 : 9   (177)               3200x1800             22fps
               800x480                5  : 3   (166)               2800x1680             22fps
               640x480                4  : 3   (133)               2240x1680             22fps
               176x144                11 : 9   (122)               1760x1440             22fps

video
               1920x1080              16 : 9   (177)               2560x1440             29fps
               1280x720               16 : 9   (177)               2560x1440             29fps
               800x480                5  : 3   (166)               2400x1440             29fps
               640x480                4  : 3   (133)               1920x1440             29fps
               176x144                11 : 9   (122)               1760x1440             29fps

preview resolution should be calculated by pic or video resolution with same ratio.
attention: preview width must be 16-aligned.
preview
               1280x720               16 : 9   (177)
               800x450                16 : 9   (177)
               640x360                16 : 9   (177)

               880x720                11 : 9   (122)
               352x288                11 : 9   (122)
               176x144                11 : 9   (122)

               960x720                4  : 3   (133)
               640x480                4  : 3   (133)
               320x240                4  : 3   (133)

               1200x720               5  : 3   (166)
               800x480                5  : 3   (166)
*/



      LOGI("mUseInternalISP=: %d", mUseInternalISP);



    //if (cameraId == SecCamera::CAMERA_ID_BACK) {
    if (!mUseInternalISP) {	//changed by jijun.yu
        //Others
    //    p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
          //    "720x480,640x384,640x360,640x480,320x240,528x432,176x144");

        //dg  change  for ov5640
        p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,"1280x960,1280x720,1024x768,640x480");


      //  p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
        //      "3248x2436,3216x2144,3200x1920,3072x1728,2592x1944,1920x1080,1440x1080,1280x720,1232x1008,800x480,720x480,640x480,176x144");


        //dg change for ov5640,support this size,but some can not work well.
   //     p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
      //        "2592x1944,2048x1536,1600x1200,1280x960,1280x720,1024x768,640x480");
           // this can work well on  ov5640
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "1280x960,1280x720,1024x768,640x480");


        //dg  change for ov5640
      //  p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,   "1280x960,1280x720,1024x768,640x480"); //support this size video add by dg 2015-05-31. we use meida_profile.xml file size default.
        //dg add for ov5640 debug record video
      //  p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "640x480");

    } else {
        //3H7
        //modify by charles.hu , 2012/10/16
        p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
              "3232x2424,2592x1944,1920x1080,1280x720,800x600,640x480,176x144");
        p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
              "1920x1080,1280x720,800x480,640x480,320x240,176x144");
        p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
              "1280x720,800x450,640x360,880x720,352x288,176x144,960x720,640x480,320x240,1200x720,800x480");
        p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "640x480");
    }
    //  }
#if 0
    else {
        /*** 	{String8(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES),String8("1280x720,640x480")},
        {String8(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES),String8("1600x1200,1280x960,640x480,320x240")},///,640x480
        {String8(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES),String8("1280x720,640x480,176x144")},
       {String8(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO),String8("1280x720")},
       */

        if (!mUseInternalISP) {
            //S5K8AAYX
            /***
            p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
                  "800x480,640x480,640x360,352x288,320x240,176x144,160x120");
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "1600x1200,800x480,640x480,640x360,352x288,320x240,176x144,160x120");
            p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
                  "800x480,640x480,640x360,352x288,320x240,176x144");
                        p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "640x480");
                        */
#if defined (SC1_DVT1)
            //for HI253 default size modify.lzy 2013.3.26
            p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
                  "1280x720,640x480,176x144");
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "1600x1200,1280x720,640x480,320x240");
            p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
                  "1280x720,640x480,176x144");
            p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "1280x720");
#else
            //for S5K8AAYX
            p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
                  "1280x720,640x480,320x240,176x144");
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "1280x960,1280x720,640x480,320x240");
            p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
                  "1280x720,640x480,176x144");
            p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "1280x720");
#endif
            //size:preview, picture, video
            /*{String8(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES),String8("1280x720,640x480,176x144")},
                {String8(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES),String8("1280x960,1280x720,640x480,320x240")},
                {String8(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES),String8("1280x720,640x480,176x144")},
            {String8(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO),String8("1280x720")},*/
        } else {
            //Others
            /***
            p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
                  "3264x2448,1920x1080,1280x720,800x480,720x480,640x480,320x240,528x432,176x144");
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "3264x2448,3264x1968,2048x1536,2048x1232,800x480,640x480");*/
            p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
                  "1280x720,640x480");
            p.set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
                  "1600x1200,1280x960,640x480,320x240");
            p.set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
                  "1280x720,640x480,176x144");
            p.set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "1280x720");
        }
    }
#endif
    p.getSupportedPreviewSizes(mSupportedPreviewSizes);

    String8 parameterString;

    // If these fail, then we are using an invalid cameraId and we'll leave the
    // sizes at zero to catch the error.
    if (mSecCamera->getPreviewMaxSize(&preview_max_width,
                                      &preview_max_height) < 0)
        LOGE("getPreviewMaxSize fail (%d / %d)",
             preview_max_width, preview_max_height);
    if (mSecCamera->getSnapshotMaxSize(&snapshot_max_width,
                                       &snapshot_max_height) < 0)
        LOGE("getSnapshotMaxSize fail (%d / %d)",
             snapshot_max_width, snapshot_max_height);

    parameterString = CameraParameters::PIXEL_FORMAT_YUV420P;
    parameterString.append(",");
    parameterString.append(CameraParameters::PIXEL_FORMAT_YUV420SP);
    parameterString.append(",");
    parameterString.append(CameraParameters::PIXEL_FORMAT_RGB565);
    parameterString.append(",");
    parameterString.append(CameraParameters::PIXEL_FORMAT_RGBA8888);

    p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, parameterString);
    //    p.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420P);
    //change jijun.yu for CTS
    p.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
    //    mFrameSizeDelta = 16;  //why???
    mFrameSizeDelta = 0;  //why???
    p.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT, CameraParameters::PIXEL_FORMAT_YUV420SP);
    p.setPreviewSize(preview_max_width, preview_max_height);

    p.setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);
    p.setPictureSize(snapshot_max_width, snapshot_max_height);
    p.set(CameraParameters::KEY_JPEG_QUALITY, "100"); // maximum quality
    p.set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS,
          CameraParameters::PIXEL_FORMAT_JPEG);


#ifdef USE_FACE_DETECTION
    if (mUseInternalISP) {
        //Cellon add begin , charles.hu 2012/10/25
        //add Facedetection OSD
        p.set("face-detection-values","on,off");
        p.set("face-detection","off");
        //Cellon add end   , charles.hu 2012/10/25
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, "5");
    } else {
        p.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW, "0");
    }
#endif

    if (cameraId == SecCamera::CAMERA_ID_BACK) {
        parameterString = CameraParameters::FOCUS_MODE_AUTO;
        parameterString.append(",");
        parameterString.append(CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);
        p.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES,
              parameterString.string());
        p.set(CameraParameters::KEY_FOCUS_MODE,
              CameraParameters::FOCUS_MODE_AUTO);//CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);//for cts by lzy
        p.set(CameraParameters::KEY_FOCUS_DISTANCES,
              BACK_CAMERA_AUTO_FOCUS_DISTANCES_STR);
#ifdef USE_TOUCH_AF
        if (mUseInternalISP)
            p.set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS, "1");
#endif
        p.set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES,
              "160x120,0x0");
        p.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, "160");
        p.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, "120");
        p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "7,15,30");
        p.setPreviewFrameRate(30);
    }

    else {
        //        p.set(CameraParameters::KEY_FOCUS_MODE, NULL);
        //add jijun.yu  ??? CTS
        p.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES,
              CameraParameters::FOCUS_MODE_AUTO);
        p.set(CameraParameters::KEY_FOCUS_MODE,
              CameraParameters::FOCUS_MODE_AUTO);

        p.set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES,
              "160x120,0x0");
        p.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, "160");
        p.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, "120");
        //p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES,
        // "7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,50,60");
        p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "7,15,30");
        p.setPreviewFrameRate(30);///crystal changed
        /// p.setPreviewFrameRate(15);
        ///p.setPreviewFrameRate(7);
    }


    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    parameterString = CameraParameters::EFFECT_NONE;
    parameterString.append(",");
    parameterString.append(CameraParameters::EFFECT_AQUA);
    parameterString.append(",");
    parameterString.append(CameraParameters::EFFECT_MONO);
    parameterString.append(",");
    parameterString.append(CameraParameters::EFFECT_NEGATIVE);
    parameterString.append(",");
    parameterString.append(CameraParameters::EFFECT_SEPIA);
    p.set(CameraParameters::KEY_SUPPORTED_EFFECTS, parameterString.string());
    p.set(CameraParameters::KEY_EFFECT, CameraParameters::EFFECT_NONE);
    //	}

    if (cameraId == SecCamera::CAMERA_ID_BACK) {
        parameterString = CameraParameters::FLASH_MODE_ON;
        parameterString.append(",");
        parameterString.append(CameraParameters::FLASH_MODE_OFF);
        parameterString.append(",");
        parameterString.append(CameraParameters::FLASH_MODE_AUTO);
        parameterString.append(",");
        parameterString.append(CameraParameters::FLASH_MODE_TORCH);
        p.set(CameraParameters::KEY_SUPPORTED_FLASH_MODES,
              parameterString.string());
        p.set(CameraParameters::KEY_FLASH_MODE,
              CameraParameters::FLASH_MODE_OFF);

        /* we have two ranges, 4-30fps for night mode and
         * 15-30fps for all others
         */


        //      p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(15000,30000)");
        //     p.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "15000,30000");
    }
    p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(1000,120000)");
    p.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "1000,120000");
    p.set(CameraParameters::KEY_FOCAL_LENGTH, "3.43");
    
#if 0
    else {
        p.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(7000,30000)");
        p.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "7000,30000");
#if defined (SC1_DVT1)
        p.set(CameraParameters::KEY_FOCAL_LENGTH, "1.55");
#else
        p.set(CameraParameters::KEY_FOCAL_LENGTH, "0.9");
#endif
    }
#endif

    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    parameterString = CameraParameters::SCENE_MODE_AUTO;
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_ACTION);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_PORTRAIT);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_LANDSCAPE);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_BEACH);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_SNOW);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_FIREWORKS);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_SPORTS);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_PARTY);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_CANDLELIGHT);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_NIGHT);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_SUNSET);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_FALL_COLOR);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_BACK_LIGHT);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_TEXT);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_INDOOR);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_DAWN);
    parameterString.append(",");
    parameterString.append(CameraParameters::SCENE_MODE_DUSK);
    p.set(CameraParameters::KEY_SUPPORTED_SCENE_MODES,
          parameterString.string());
    p.set(CameraParameters::KEY_SCENE_MODE,
          CameraParameters::SCENE_MODE_AUTO);
    //}




    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    parameterString = CameraParameters::WHITE_BALANCE_AUTO;
    parameterString.append(",");
    parameterString.append(CameraParameters::WHITE_BALANCE_INCANDESCENT);
    parameterString.append(",");
    parameterString.append(CameraParameters::WHITE_BALANCE_FLUORESCENT);
    parameterString.append(",");
    parameterString.append(CameraParameters::WHITE_BALANCE_DAYLIGHT);
    parameterString.append(",");
    parameterString.append(CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT);
    p.set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE,
          parameterString.string());

    p.set(CameraParameters::KEY_WHITE_BALANCE, CameraParameters::WHITE_BALANCE_AUTO);
    //	}

    p.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, "100");

    p.set(KEY_PICTURE_TIME, VALUE_PICTURE_TIME_OFF);
    p.set(CameraParameters::KEY_ROTATION, 0);
#if 0
    if (cameraId == SecCamera::CAMERA_ID_FRONT) {
        p.set(CameraParameters::KEY_ROTATION, 90);
    }
#endif

    
    

    
    // if (cameraId == SecCamera::CAMERA_ID_BACK) {
    p.set(CameraParameters::KEY_CONTRAST, 0);
    p.set(CameraParameters::KEY_MAX_CONTRAST, 3);
    p.set(CameraParameters::KEY_MIN_CONTRAST, -3);

    // }
    //if (cameraId == SecCamera::CAMERA_ID_BACK) {
    p.set(CameraParameters::KEY_ISO, "auto");
    p.set(CameraParameters::KEY_SUPPORTED_ISO_MODES,"auto,50,100,200,400,800,1200");
    //	}
    p.set(CameraParameters::KEY_METERING_MODE, "matrix");
    p.set(CameraParameters::KEY_SUPPORTED_METERING_MODES,"average,center,spot,matrix");

    //Cellon modify by charles.hu , begin ,
    if(!strncmp((const char*)mSecCamera->getCameraSensorName(), "M5MO", 10))
        p.set(CameraParameters::KEY_WDR, 0);
    //Cellon modify by charles.hu , end   ,

    ip.set("chk_dataline", 0);
#if 0
    if (cameraId == SecCamera::CAMERA_ID_FRONT) {
        ip.set("vtmode", 0);
        ip.set("blur", 0);
    }
#endif
    p.set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, "51.2");
    p.set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, "39.4");
    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, "0");
    p.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, "4");
    p.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, "-4");
    p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, "1");
    //	}
#if 0
    else
    {//for cts bikun
        p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, "0");
        p.set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, "0");
        p.set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, "0");
        p.set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, "0");
    }
#endif	
    //   if (cameraId == SecCamera::CAMERA_ID_BACK) {
    p.set(CameraParameters::KEY_BRIGHTNESS,"0");
    p.set(CameraParameters::KEY_MAX_BRIGHTNESS, "2");
    p.set(CameraParameters::KEY_MIN_BRIGHTNESS, "-2");
    p.set(CameraParameters::KEY_BRIGHTNESS_STEP, "1");
    //  }


    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    p.set(CameraParameters::KEY_SATURATION,"0");
    p.set(CameraParameters::KEY_MAX_SATURATION, "2");
    p.set(CameraParameters::KEY_MIN_SATURATION, "-2");
    p.set(CameraParameters::KEY_SATURATION_STEP, "1");
    //	}

    //   if (cameraId == SecCamera::CAMERA_ID_BACK) {			///front no this ,Crystal add if for ov2675
    p.set(CameraParameters::KEY_SHARPNESS,"0");
    p.set(CameraParameters::KEY_MAX_SHARPNESS, "2");
    p.set(CameraParameters::KEY_MIN_SHARPNESS, "-2");
    p.set(CameraParameters::KEY_SHARPNESS_STEP, "1");


    p.set("hue", 0);
    p.set("hue-max", 2);
    p.set("hue-min", -2);
    //	}
    //	if (cameraId == SecCamera::CAMERA_ID_BACK) {
    parameterString = CameraParameters::ANTIBANDING_AUTO;
    parameterString.append(",");
    parameterString.append(CameraParameters::ANTIBANDING_50HZ);
    parameterString.append(",");
    parameterString.append(CameraParameters::ANTIBANDING_60HZ);
    parameterString.append(",");
    parameterString.append(CameraParameters::ANTIBANDING_OFF);
    p.set(CameraParameters::KEY_SUPPORTED_ANTIBANDING,
          parameterString.string());

    p.set(CameraParameters::KEY_ANTIBANDING, CameraParameters::ANTIBANDING_AUTO);
    //	}
    if (mUseInternalISP) {
        p.set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK_SUPPORTED, "true");
        p.set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK, "false");
    }

    if (mUseInternalISP) {
        p.set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK_SUPPORTED, "true");
        p.set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK, "false");
    }

    p.set(CameraParameters::KEY_RECORDING_HINT, "false");

    p.set(CameraParameters::KEY_FOCUS_AREAS,NULL);//"0"

    // metering
    p.set(CameraParameters::KEY_MAX_NUM_METERING_AREAS, mSecCamera->getMaxNumMeteringAreas());

#ifdef VIDEO_SNAPSHOT
    if (mUseInternalISP)
        p.set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED, "true");
#endif

#ifndef BOARD_USE_V4L2_ION
    ///if (cameraId == SecCamera::CAMERA_ID_BACK) ///crystal del,for ov2675 zoom func ,2012-10-25
    {
        p.set(CameraParameters::KEY_ZOOM_SUPPORTED, "true");
        p.set(CameraParameters::KEY_MAX_ZOOM, ZOOM_LEVEL_MAX - 1);
        //    	p.set(CameraParameters::KEY_ZOOM_RATIOS, "31,4");
        p.set(CameraParameters::KEY_ZOOM_RATIOS, "100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300");
    }
#endif

    //if (cameraId == SecCamera::CAMERA_ID_BACK){

    p.set(CameraParameters::KEY_SHUTTER_MODE_SPPORTED,"1");//flag for camera apk whether sensor support the function  add by lzy

    //	}
    //add begin by suojp 2012-06-06
#ifdef SCALADO_SCF
    //add SCF support

    //zero shuttle lag
    p.set(CameraParameters::KEY_SCF_CAPTURE_ZSL_SUPPORT,"1");

    //brust shot
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_INTERVAL_SUPPORT,"0");//isBurstIntervalSupported
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_INTERVAL,"100");//isBurstIntervalSupported
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_RETROACTIVE,"0");//BurstRetroactive
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_TOTAL,"0");  //burst shot number
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_CAPTURES_SUPPORT,"5");//MAX burst shot number

    //bracketing capturing
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_EXPOSURES_SUPPORT,"true");
    //p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_EXPOSURES,"1,0,-1");
    p.set(CameraParameters::KEY_SCF_CAPTURE_BURST_EXPOSURES,"");
    mIsBracketEV = false;
#endif
    //add end by suojp 2012-06-06

    mPreviewRunning = false;
    mParameters = p;
    mInternalParameters = ip;

    /* make sure mSecCamera has all the settings we do.  applications
     * aren't required to call setParameters themselves (only if they
     * want to change something.
     */
    setParameters(p);
}

CameraHardwareSec::~CameraHardwareSec()
{
    LOGD("%s", __func__);
    release();
    //mSecCamera->DestroyCamera();
}

status_t CameraHardwareSec::setPreviewWindow(preview_stream_ops *w)
{
    int min_bufs;

    mPreviewWindow = w;
    LOGD("%s: mPreviewWindow %p", __func__, mPreviewWindow);

    if (!w) {
        LOGI("preview window is NULL!");
        return OK;
    }

    mPreviewLock.lock();

    if (mPreviewRunning && !mPreviewStartDeferred) {
        LOGI("stop preview (window change)");
        /*do not stop preview  even there are no preview window jijun.yu*/
        //        stopPreviewInternal();
    }

    if (w->get_min_undequeued_buffer_count(w, &min_bufs)) {
        LOGE("%s: could not retrieve min undequeued buffer count", __func__);
        return INVALID_OPERATION;
    }

    if (min_bufs >= BUFFER_COUNT_FOR_GRALLOC) {
        LOGE("%s: min undequeued buffer count %d is too high (expecting at most %d)", __func__,
             min_bufs, BUFFER_COUNT_FOR_GRALLOC - 1);
    }

    LOGD("%s: setting buffer count to %d", __func__, BUFFER_COUNT_FOR_GRALLOC);
    if (w->set_buffer_count(w, BUFFER_COUNT_FOR_GRALLOC)) {
        LOGE("%s: could not set buffer count", __func__);
        return INVALID_OPERATION;
    }

    int preview_width;
    int preview_height;
    mParameters.getPreviewSize(&preview_width, &preview_height);

    int hal_pixel_format;

    const char *str_preview_format = mParameters.getPreviewFormat();
    LOGD("%s: preview format %s", __func__, str_preview_format);
    //    mFrameSizeDelta = 16;
    mFrameSizeDelta = 0;

    hal_pixel_format = HAL_PIXEL_FORMAT_YV12; // default

    if (!strcmp(str_preview_format,
                CameraParameters::PIXEL_FORMAT_RGB565)) {
        hal_pixel_format = HAL_PIXEL_FORMAT_RGB_565;
        mFrameSizeDelta = 0;
    } else if (!strcmp(str_preview_format,
                       CameraParameters::PIXEL_FORMAT_RGBA8888)) {
        hal_pixel_format = HAL_PIXEL_FORMAT_RGBA_8888;
        mFrameSizeDelta = 0;
    } else if (!strcmp(str_preview_format,
                       CameraParameters::PIXEL_FORMAT_YUV420SP)) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
    } else if (!strcmp(str_preview_format,
                       CameraParameters::PIXEL_FORMAT_YUV420P))
        hal_pixel_format = HAL_PIXEL_FORMAT_YV12; // HACK

#ifdef USE_EGL
#ifdef BOARD_USE_V4L2_ION
    if (w->set_usage(w, GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_HW_ION)) {
#else
    if (w->set_usage(w, GRALLOC_USAGE_SW_WRITE_OFTEN)) {
#endif
        LOGE("%s: could not set usage on gralloc buffer", __func__);
        return INVALID_OPERATION;
    }
#else
#ifdef BOARD_USE_V4L2_ION
    if (w->set_usage(w, GRALLOC_USAGE_SW_WRITE_OFTEN
                     | GRALLOC_USAGE_HWC_HWOVERLAY | GRALLOC_USAGE_HW_ION)) {
#else
    if (w->set_usage(w, GRALLOC_USAGE_SW_WRITE_OFTEN
                     | GRALLOC_USAGE_HW_FIMC1 | GRALLOC_USAGE_HWC_HWOVERLAY)) {
#endif
        LOGE("%s: could not set usage on gralloc buffer", __func__);
        return INVALID_OPERATION;
    }
#endif

    LOGD("preview_width x preview_height = %dx%d, format = %d",
         preview_width, preview_height, hal_pixel_format);

    if (w->set_buffers_geometry(w,
                                preview_width, preview_height,
                                hal_pixel_format)) {
        LOGE("%s: could not set buffers geometry to %s",
             __func__, str_preview_format);
        return INVALID_OPERATION;
    }

#ifdef BOARD_USE_V4L2_ION
    for(int i = 0; i < BUFFER_COUNT_FOR_ARRAY; i++)
        if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &mBufferHandle[i], &mStride[i])) {
            LOGE("%s: Could not dequeue gralloc buffer[%d]!!", __func__, i);
            return INVALID_OPERATION;
        }
#endif

    if (mPreviewRunning && mPreviewStartDeferred) {
        LOGD("start/resume preview");
        status_t ret = startPreviewInternal();
        if (ret == OK) {
            mPreviewStartDeferred = false;
            mPreviewCondition.signal();
        }
    }
    mPreviewLock.unlock();

    return OK;
}

void CameraHardwareSec::setCallbacks(camera_notify_callback notify_cb,
                                     camera_data_callback data_cb,
                                     camera_data_timestamp_callback data_cb_timestamp,
                                     camera_request_memory get_memory,
                                     void *user)
{
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mGetMemoryCb = get_memory;
    mCallbackCookie = user;
}

void CameraHardwareSec::enableMsgType(int32_t msgType)
{
    LOGD("%s : msgType = 0x%x, mMsgEnabled before = 0x%x",
         __func__, msgType, mMsgEnabled);
    if(CAMERA_MSG_VIDEO_FRAME&msgType)
        LOGW("%s:CAMERA_MSG_VIDEO_FRAME is enable",__func__);
    mMsgEnabled |= msgType;

    mPreviewLock.lock();
    if ((msgType & (CAMERA_MSG_PREVIEW_FRAME | CAMERA_MSG_VIDEO_FRAME)) &&
            mPreviewRunning && mPreviewStartDeferred) {
        LOGD("%s: starting deferred preview", __func__);
        if (startPreviewInternal() == OK) {
            mPreviewStartDeferred = false;
            mPreviewCondition.signal();
        }
    }
    mPreviewLock.unlock();

    LOGD("%s : mMsgEnabled = 0x%x", __func__, mMsgEnabled);
}

void CameraHardwareSec::disableMsgType(int32_t msgType)
{
    LOGD("%s : msgType = 0x%x, mMsgEnabled before = 0x%x",
         __func__, msgType, mMsgEnabled);
    if(CAMERA_MSG_VIDEO_FRAME&msgType)
        LOGW("%s:CAMERA_MSG_VIDEO_FRAME is disable",__func__);
    mMsgEnabled &= ~msgType;
    LOGD("%s : mMsgEnabled = 0x%x", __func__, mMsgEnabled);
}

bool CameraHardwareSec::msgTypeEnabled(int32_t msgType)
{
    return (mMsgEnabled & msgType);
}

void CameraHardwareSec::setSkipFrame(int frame)
{
    Mutex::Autolock lock(mSkipFrameLock);
    if (frame < mSkipFrame)
        return;

    mSkipFrame = frame;
}

int CameraHardwareSec::previewThreadWrapper()
{
    LOGI("%s: starting", __func__);
    while (1) {
        mPreviewLock.lock();
        while (!mPreviewRunning) {
            mCurrentIndex = -1;
            mCapIndex = -1;
            mSecCamera->stopPreview();
            /* signal that we're stopping */
            mPreviewStoppedCondition.signal();
            LOGI("%s: calling mSecCamera->stopPreview() and waiting", __func__);
            mPreviewCondition.wait(mPreviewLock);
            LOGI("%s: return from wait", __func__);
        }
        mPreviewLock.unlock();

        if (mExitPreviewThread) {
            LOGI("%s: exiting", __func__);
            mCurrentIndex = -1;
            mCapIndex = -1;
            mSecCamera->stopPreview();
            return 0;
        }

        if (mUseInternalISP)
            previewThreadForZoom();
        else
            previewThread(); /// previewThreadForZoom();///previewThread(); ///crystal dbg for ov2675 zoom func ,2012-10-25
    }
}

LOG_TIME_DEFINE(0)
int CameraHardwareSec::previewThreadForZoom()
{
    LOGD("================previewThreadForZoom");
    int index;
    nsecs_t timestamp;
    SecBuffer previewAddr, recordAddr;
    static int numArray = 0;
    void *virAddr[3];

#ifdef BOARD_USE_V4L2_ION
    private_handle_t *hnd = NULL;
#else
    struct addrs *addrs;
#endif

#ifdef S3_FPS_DETECTION	
    if (msgTypeEnabled(CAMERA_MSG_FPS_DETECTION)
            && mPreviewRunning
            && (mCapture_Cnt == -1))
    {
#ifdef S3_STATUS_UPDATE	
        mStatusUpdateLock.lock();
        mCapture_Cnt = 0;
        LOG_TIME_START(0)
                mStatusUpdateLock.unlock();
#endif		
    }
#endif
    //add end by suojp 2012-05-10

    mFaceMetaData.faces = mMetaFaces;
    index = mSecCamera->getPreview(&mFaceMetaData);

    if (UNLIKELY(mCurrentIndex < 0)) {
        LOGD("%s: pass the preview thread", __func__);
    } else {
        if (mSecCamera->setPreviewFrame(mCurrentIndex) < 0) {
            LOGE("%s: Fail qbuf, index(%d)", __func__, index);
            return false;
        } else
            mCurrentIndex = -1;
    }

    if (UNLIKELY(index < 0)) {
        if(index == DEAD_OBJECT && mPreviewRunning) {
            LOGE("ERR(%s):Fail on SecCamera->getPreview(), reset device", __func__);
            if (mStateLock.tryLock() != NO_ERROR) {
                LOGE("ERR(%s): Failed on lock mStateLock", __func__);
                return false;
            }
            if (mPreviewLock.tryLock() != NO_ERROR) {
                LOGE("ERR(%s): Failed on lock mPreviewLock", __func__);
                mStateLock.unlock();
                return false;
            }
            mFimcLock.lock();

            mCurrentIndex = -1;
            mCapIndex = -1;
#ifdef SUPPORT_ISP_RESET
            mSecCamera->resetPreviewCamera();
#else
            mSecCamera->stopPreview();
            mSecCamera->startPreview();
#endif

            mFimcLock.unlock();
            mPreviewLock.unlock();
            mStateLock.unlock();

        }
        else
            LOGE("ERR(%s):Fail on SecCamera->getPreview(), break", __func__);
        return true;
    }

    if (UNLIKELY(mSkipFrame > 0)) {
        mSkipFrameLock.lock();
        mSkipFrame--;
        mSkipFrameLock.unlock();
        LOGD("%s: index %d skipping frame", __func__, index);
        if (mSecCamera->setPreviewFrame(index) < 0) {
            LOGE("%s: Could not qbuff[%d]!!", __func__, index);
            return false;
        }
        return true;
    }

    mCurrentIndex = index;
#ifdef BURST_SHOT
    if(mBurstCapture){
        if(!mburstencodeflag)
            mCapIndex = index;
    }
    else
#endif
    {
        if (!mCaptureInProgress)
            mCapIndex = index;
    }

    mFimcActionCondition.broadcast();

    //add begin by suojp 2012-05-10
#ifdef S3_FPS_DETECTION
    if (msgTypeEnabled(CAMERA_MSG_FPS_DETECTION)
            && mPreviewRunning && (mCapture_Cnt >= 0))
    {
#ifdef S3_STATUS_UPDATE
        Mutex::Autolock _l(mStatusUpdateLock);
#endif
        mCapture_Cnt ++;
        LOG_TIME_END(0)
    }
#endif
    //add end by suojp 2012-05-10

    return NO_ERROR;
}

int CameraHardwareSec::previewThread()
{
  //  LOGD("===========previewThread");
    int index;
    nsecs_t timestamp;
    SecBuffer previewAddr, recordAddr;
    static int numArray = 0;
    void *virAddr[3];

#ifdef BOARD_USE_V4L2_ION
    private_handle_t *hnd = NULL;
#else
    struct addrs *addrs;
#endif

    mFaceData.faces = mFaces;
    index = mSecCamera->getPreview(&mFaceData);

    if (index < 0) {
        LOGE("ERR(%s):Fail on SecCamera->getPreview(), reset front camera", __func__);
        if(index == DEAD_OBJECT && mPreviewRunning)
        {
            if (mPreviewLock.tryLock() != NO_ERROR) {
                LOGE("ERR(%s): Failed on lock mPreviewLock", __func__);
                return false;
            }
            mFimcLock.lock();

            mCurrentIndex = -1;
            mCapIndex = -1;
            mSecCamera->stopPreview();
            mSecCamera->startPreview();

            mFimcLock.unlock();
            mPreviewLock.unlock();
        }
        return UNKNOWN_ERROR;
    }

#ifdef ZERO_SHUTTER_LAG
    if (mUseInternalISP && !mRecordHint) {
        mCapIndex = mSecCamera->getSnapshot();

        if (mCapIndex >= 0) {
            if (mSecCamera->setSnapshotFrame(mCapIndex) < 0) {
                LOGE("%s: Fail qbuf, index(%d)", __func__, mCapIndex);
                return INVALID_OPERATION;
            }
        }
    }
#endif

    mSkipFrameLock.lock();
    if (mSkipFrame > 0) {
        mSkipFrame--;
        mSkipFrameLock.unlock();
        LOGD("%s: index %d skipping frame", __func__, index);
        if (mSecCamera->setPreviewFrame(index) < 0) {
            LOGE("%s: Could not qbuff[%d]!!", __func__, index);
            return UNKNOWN_ERROR;
        }
        return NO_ERROR;
    } else {
        /*For now HAL, the preview render by mPreviewWindow, so it should not callback the buffer
                 to App(only for Ov2659)*/
        if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME && mPreviewRunning)
            mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewHeap, index, NULL, mCallbackCookie);
    }
    
    mSkipFrameLock.unlock();

    timestamp = systemTime(SYSTEM_TIME_MONOTONIC);

    int width, height, frame_size, offset;
    const char *str_preview_format = NULL;
    for(int i=0;i<3;i++) {
        str_preview_format = mParameters.getPreviewFormat();
        if(str_preview_format != NULL) break;
        LOGW("previewThread:WARNNING:can't get previewFormat from mParameters.getPreviewFormat! try again");
        usleep(10000);
    }
    mSecCamera->getPreviewSize(&width, &height, &frame_size);
    /*add support RGB565 for Front Camera jijun.yu 20120723*/
    if (!mUseInternalISP &&!strcmp(str_preview_format,//mParameters.getPreviewFormat(), bk adds,it may conflict with setPreviewFormat in setParameter
                                   CameraParameters::PIXEL_FORMAT_RGB565))
        offset = ALIGN(frame_size,4096) * index;
    else
        offset = frame_size * index;
    if (mPreviewWindow && mGrallocHal && mPreviewRunning) {
#ifdef BOARD_USE_V4L2_ION
        hnd = (private_handle_t*)*mBufferHandle[index];

        if (mPreviewHeap) {
            mPreviewHeap->release(mPreviewHeap);
            mPreviewHeap = 0;
        }
        mPreviewHeap = mGetMemoryCb(hnd->fd, frame_size, 1, 0);

        hnd = NULL;

        mGrallocHal->unlock(mGrallocHal, *mBufferHandle[index]);
        if (0 != mPreviewWindow->enqueue_buffer(mPreviewWindow, mBufferHandle[index])) {
            LOGE("%s: Could not enqueue gralloc buffer[%d]!!", __func__, index);
            goto callbacks;
        } else {
            mBufferHandle[index] = NULL;
            mStride[index] = NULL;
        }

        numArray = index;
#endif

        if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &mBufferHandle[numArray], &mStride[numArray])) {
            LOGE("%s: Could not dequeue gralloc buffer[%d]!!", __func__, numArray);
            goto callbacks;
        }

        if (!mGrallocHal->lock(mGrallocHal,
                               *mBufferHandle[numArray],
                               GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_YUV_ADDR,
                               0, 0, width, height, virAddr)) {
#ifdef BOARD_USE_V4L2
            mSecCamera->getPreviewAddr(index, &previewAddr);
            char *frame = (char *)previewAddr.virt.extP[0];
#else
            char *frame = ((char *)mPreviewHeap->data) + offset;
#endif

#ifdef BOARD_USE_V4L2_ION
            mSecCamera->setUserBufferAddr(virAddr, index, PREVIEW_MODE);
#else
            int total = frame_size + mFrameSizeDelta;
            int h = 0;
            char *src = frame;
            /* TODO : Need to fix size of planes for supported color fmt.
                      Currnetly we support only YV12(3 plane) and NV21(2 plane)*/
            // Y

            if (!strcmp(str_preview_format,
                        CameraParameters::PIXEL_FORMAT_RGB565)) {
                /*add support RGB565 for Front Camera jijun.yu 20120723*/
                memcpy(virAddr[0],src,  ALIGN(frame_size,4096));
            } else if (!strcmp(str_preview_format,
                               CameraParameters::PIXEL_FORMAT_RGBA8888)) {
                memcpy(virAddr[0],src, width * height * 4);
            } else {
                memcpy(virAddr[0],src, width * height);
                src += width * height;

                if (mPreviewFmtPlane == PREVIEW_FMT_2_PLANE) {
                    memcpy(virAddr[1], src, width * height / 2);
                } else if (mPreviewFmtPlane == PREVIEW_FMT_3_PLANE) {
                    // U
                    memcpy(virAddr[1], src, width * height / 4);
                    src += width * height / 4;

                    // V
                    memcpy(virAddr[2], src, width * height / 4);
                }
            }

            mGrallocHal->unlock(mGrallocHal, **mBufferHandle);
#endif
        }
        else
            LOGE("%s: could not obtain gralloc buffer", __func__);

        if (mSecCamera->setPreviewFrame(index) < 0) {
            LOGE("%s: Fail qbuf, index(%d)", __func__, index);
            goto callbacks;
        }

        index = 0;
#ifndef BOARD_USE_V4L2_ION
        if (0 != mPreviewWindow->enqueue_buffer(mPreviewWindow, *mBufferHandle)) {
            LOGE("Could not enqueue gralloc buffer!");
            goto callbacks;
        }
#endif
    }

callbacks:
    // Notify the client of a new frame.
    //   if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME && mPreviewRunning)
    //       mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewHeap, index, NULL, mCallbackCookie);

    /*for OV there are not face detection function delete by jijun.yu  0803*/
#ifdef USE_FACE_DETECTION
    //   if (mUseInternalISP && (mMsgEnabled & CAMERA_MSG_PREVIEW_METADATA) && mPreviewRunning)
    //    mDataCb(CAMERA_MSG_PREVIEW_METADATA, mFaceDataHeap, 0, &mFaceData, mCallbackCookie);
#endif
    /*delete it ,for ov record use a single thread add by jijun.yu 0803*/
#if  0
    Mutex::Autolock lock(mRecordLock);
    if (mRecordRunning == true) {
        int recordingIndex = 0;

        index = mSecCamera->getRecordFrame();
        if (index < 0) {
            LOGE("ERR(%s):Fail on SecCamera->getRecordFrame()", __func__);
            return UNKNOWN_ERROR;
        }

#ifdef VIDEO_SNAPSHOT
        if (mUseInternalISP && mRecordHint) {
            mCapIndex = mSecCamera->getSnapshot();

            if (mSecCamera->setSnapshotFrame(mCapIndex) < 0) {
                LOGE("%s: Fail qbuf, index(%d)", __func__, mCapIndex);
                return INVALID_OPERATION;
            }
        }
#endif

#ifdef BOARD_USE_V4L2_ION
        numArray = index;
#else
        recordingIndex = index;
        mSecCamera->getRecordAddr(index, &recordAddr);

        LOGD("record PhyY(0x%08x) phyC(0x%08x) ", recordAddr.phys.extP[0], recordAddr.phys.extP[1]);

        if (recordAddr.phys.extP[0] == 0xffffffff || recordAddr.phys.extP[1] == 0xffffffff) {
            LOGE("ERR(%s):Fail on SecCamera getRectPhyAddr Y addr = %0x C addr = %0x", __func__,
                 recordAddr.phys.extP[0], recordAddr.phys.extP[1]);
            return UNKNOWN_ERROR;
        }

        addrs = (struct addrs *)(*mRecordHeap)->data;

        addrs[index].type   = kMetadataBufferTypeCameraSource;
        addrs[index].addr_y = recordAddr.phys.extP[0];
        addrs[index].addr_cbcr = recordAddr.phys.extP[1];
        addrs[index].buf_index = index;
#endif

        // Notify the client of a new frame.
        if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME)
            mDataCbTimestamp(timestamp, CAMERA_MSG_VIDEO_FRAME,
                             mRecordHeap[0], recordingIndex, mCallbackCookie);
        else
            mSecCamera->releaseRecordFrame(index);
    }
#endif
    return NO_ERROR;
}

/*Sometimes,add recorder thread for ov recording  0803 jijun.yu*/	
status_t CameraHardwareSec::recordThread()
{
    int index;
    nsecs_t timestamp;
    SecBuffer previewAddr, recordAddr;
    void *virAddr[3];
    unsigned int dst_addr;
    struct addrs *addrs;

    Mutex::Autolock lock(mOvrecStateLock);

    if (mRecordRunning == true) {
        int recordingIndex = 0;
        index = mSecCamera->getRecordFrame();
        if (index < 0) {
            LOGE("ERR(%s):Fail on SecCamera->getRecordFrame()", __func__);
            return UNKNOWN_ERROR;
        }

#ifdef VIDEO_SNAPSHOT
        if (mUseInternalISP && mRecordHint) {
            mCapIndex = mSecCamera->getSnapshot();

            if (mSecCamera->setSnapshotFrame(mCapIndex) < 0) {
                LOGE("%s: Fail qbuf, index(%d)", __func__, mCapIndex);
                return INVALID_OPERATION;
            }
        }
#endif

#ifdef BOARD_USE_V4L2_ION
        numArray = index;
#else
        timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
        recordingIndex = index;
        mSecCamera->getRecordAddr(index, &recordAddr);

        LOGD("record PhyY(0x%08x) phyC(0x%08x) ", recordAddr.phys.extP[0], recordAddr.phys.extP[1]);
        if (recordAddr.phys.extP[0] == 0xffffffff || recordAddr.phys.extP[1] == 0xffffffff) {
            LOGE("ERR(%s):Fail on SecCamera getRectPhyAddr Y addr = %0x C addr = %0x", __func__,
                 recordAddr.phys.extP[0], recordAddr.phys.extP[1]);
            return UNKNOWN_ERROR;
        }

        addrs = (struct addrs *)(*mRecordHeap)->data;

        addrs[index].type	= kMetadataBufferTypeCameraSource;
        addrs[index].addr_y = recordAddr.phys.extP[0];
        addrs[index].addr_cbcr = recordAddr.phys.extP[1];
        addrs[index].buf_index = index;
#endif

        // Notify the client of a new frame.
        if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME)  {
            mDataCbTimestamp(timestamp, CAMERA_MSG_VIDEO_FRAME,
                             mRecordHeap[0], recordingIndex, mCallbackCookie);

        }else{
            mSecCamera->releaseRecordFrame(index);
            LOGW("CAMERA_MSG_VIDEO_FRAME is disabled in recordThread\n");
        }
    }else {

        mFimcLock.lock();
        //				  mSecCamera->releaseRecordFrame(index);

        if (mRunningThread > 0) {
            mRunningThread--;
            LOGD(" mRunningthread-- : %d", mRunningThread);
        }
        mFimcLock.unlock();
        LOGW(" recordthread exit false-- : %d", mRunningThread);
        return false;
    }

    return true;
}



status_t CameraHardwareSec::previewFimcThread()
{
    LOGD("%s", __func__);

    int index = 0;
    nsecs_t timestamp;
    SecBuffer previewAddr, recordAddr;
    static int numArray = 0;
    void *virAddr[3];
    unsigned int dst_addr;

#ifdef BOARD_USE_V4L2_ION
    private_handle_t *hnd = NULL;
#else
    struct addrs *addrs;
#endif

    if (UNLIKELY(mPreviewRunning == false)) {
        mFimcLock.lock();
        LOGD("preview Fimc stop");
        if (mRunningThread > 0) {
            mRunningThread--;
            LOGD(" mRunningthread-- : %d", mRunningThread);
        }
        mFimcStoppedCondition.broadcast();

        mFimcLock.unlock();
        return false;
    }

    int width, height, frame_size;
    mSecCamera->getPreviewSize(&width, &height, &frame_size);

    memcpy(mFaces, mMetaFaces, sizeof(camera_face_t) * CAMERA_MAX_FACES);
    mFaceData.number_of_faces = mFaceMetaData.number_of_faces;
    mFaceData.faces = mFaces;

    if (UNLIKELY(mCurrentIndex < 0)) {
        LOGD("%s: doing nothing in preview fimc thread", __func__);
        return true;
    } else if (mOldPreviewIndex == mCurrentIndex) {
        /*
           Preview thread sync to FIMC0, need check performance with FIMC0.
           (condition broadcast ignored if preview thread is not in waiting). Jiangshanbin 2012/05/16
        */
        mFimcLock.lock();
        mFimcActionCondition.waitRelative(mFimcLock,200000000);
        mFimcLock.unlock();

        //Just return true, next runtime, thread will exit from the beginning. Jiangshanbin 2012/06/19
        if (mPreviewRunning == false || (mCurrentIndex < 0))
            return true;
    }

    mOldPreviewIndex = mCurrentIndex;

#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
    mPreviewFimcLock.lock();
#endif
    mSecCamera->runPreviewFimcOneshot((unsigned int)(mSecCamera->getShareBufferAddr(mCurrentIndex)), &mFaceData);
#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
    mPreviewFimcLock.unlock();
#endif

#ifdef USE_FACE_DETECTION
    mCallbackCondition.signal();
#endif

    //add begin by suojp 2012-06-18
#ifdef S3_IGNORE_PREVIEW_WINDOW
    if (bIgnorePreviewWindow) {
        // Notify the client of a new frame.
        if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME && mPreviewRunning)
            mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewHeap, index, NULL, mCallbackCookie);
        
        return true;
    }
#endif
    //add end by suojp 2012-06-18

    if (mPreviewWindow && mGrallocHal && mPreviewRunning) {
#ifdef BOARD_USE_V4L2_ION
        hnd = (private_handle_t*)*mBufferHandle[index];

        if (mPreviewHeap) {
            mPreviewHeap->release(mPreviewHeap);
            mPreviewHeap = 0;
        }

        mPreviewHeap = mGetMemoryCb(hnd->fd, frame_size, 1, 0);

        hnd = NULL;

        mGrallocHal->unlock(mGrallocHal, *mBufferHandle[index]);
        if (0 != mPreviewWindow->enqueue_buffer(mPreviewWindow, mBufferHandle[index])) {
            LOGE("%s: Could not enqueue gralloc buffer[%d]!!", __func__, index);
            return true;
        } else {
            mBufferHandle[index] = NULL;
            mStride[index] = NULL;
        }

        numArray = index;
#endif

        if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &mBufferHandle[numArray], &mStride[numArray])) {
            LOGE("%s: Could not dequeue gralloc buffer[%d]!!", __func__, numArray);
            return true;
        }

        if (!mGrallocHal->lock(mGrallocHal,
                               *mBufferHandle[numArray],
                               GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_YUV_ADDR,
                               0, 0, width, height, virAddr)) {
#ifdef BOARD_USE_V4L2
            mSecCamera->getPreviewAddr(index, &previewAddr);
            char *frame = (char *)previewAddr.virt.extP[0];
#else
            char *frame = mSecCamera->getMappedAddr();
#endif

#ifdef BOARD_USE_V4L2_ION
            mSecCamera->setUserBufferAddr(virAddr, index, PREVIEW_MODE);
#else
            int total = frame_size + mFrameSizeDelta;
            int h = 0;
            char *src = (char *)frame;

            /* TODO : Need to fix size of planes for supported color fmt.
                      Currnetly we support only YV12(3 plane) and NV21(2 plane)*/

            /*            if (src == NULL || src == (char *)0xffffffff)
                LOGE("++++ src buf %x", src);
            if (virAddr[0] == NULL || virAddr[0] == (char *)0xffffffff
                || virAddr[1] == NULL || virAddr[1] == (char *)0xffffffff
                || virAddr[2] == NULL || virAddr[2] == (char *)0xffffffff)
                LOGE("++++ virAddr[0] %x [1] %x [2] %x", virAddr[0], virAddr[1], virAddr[2]);
*/
            // Y
            const char *str_preview_format = NULL;
            for(int i=0;i<3;i++) {
                str_preview_format = mParameters.getPreviewFormat();;
                if(str_preview_format != NULL) break;
                LOGW("previewFimcThread:WARNNING:can't get previewFormat from mParameters.getPreviewFormat! try again");
                usleep(10000);
            }

            
            if (str_preview_format != NULL && !strcmp(str_preview_format,
                                                      CameraParameters::PIXEL_FORMAT_RGB565)) {
                memcpy(virAddr[0],src, width * height * 2);
            } else if (str_preview_format != NULL && !strcmp(str_preview_format,
                                                             CameraParameters::PIXEL_FORMAT_RGBA8888)) {
                memcpy(virAddr[0],src, width * height * 4);
            } else {
                memcpy(virAddr[0], src, width * height);
                src += width * height;

                if (mPreviewFmtPlane == PREVIEW_FMT_2_PLANE) {
                    memcpy(virAddr[1], src, width * height / 2);
                } else if (mPreviewFmtPlane == PREVIEW_FMT_3_PLANE) {
                    // U
                    memcpy(virAddr[1], src, width * height / 4);
                    src += width * height / 4;

                    // V
                    memcpy(virAddr[2], src, width * height / 4);
                }
            }
            mGrallocHal->unlock(mGrallocHal, **mBufferHandle);
#endif
        }
        else
            LOGE("%s: could not obtain gralloc buffer", __func__);

        index = 0;
#ifndef BOARD_USE_V4L2_ION
        if (0 != mPreviewWindow->enqueue_buffer(mPreviewWindow, *mBufferHandle)) {
            LOGE("Could not enqueue gralloc buffer!");
            return true;
        }
#endif
    }

    // Notify the client of a new frame.
    if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME && mPreviewRunning)
        mDataCb(CAMERA_MSG_PREVIEW_FRAME, mPreviewHeap, index, NULL, mCallbackCookie);

#ifdef S3_CHECK_PREVIEW_LIGHT
    //Check light done skip 1 frame.
    if(msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT) && mOldPreviewIndex%2)
        mCheckLightCondition.signal();
#endif

    return true;
}

status_t CameraHardwareSec::recordFimcThread()
{
    int index;
    nsecs_t timestamp;
    SecBuffer previewAddr, recordAddr;
    static int numArray = 0;
    void *virAddr[3];
    unsigned int dst_addr;
    struct addrs *addrs;

    Mutex::Autolock lock(mRecordLock);
    if (mRecordRunning == true) {
        if (mCurrentIndex < 0) {
            LOGE("%s: doing nothing mCurrent index < 0", __func__);
            usleep(10000);
            return true;
        }

        if (mOldRecordIndex == mCurrentIndex) {
            /*
               Record thread sync to FIMC0, need check performance with FIMC0.
               (condition broadcast ignored if Record thread is not in waiting). Jiangshanbin 2012/05/16
            */
            mFimcLock.lock();
            mFimcActionCondition.wait(mFimcLock);
            mFimcLock.unlock();
            if (!mRecordRunning) {
                mFimcLock.lock();
                if (mRunningThread > 0) {
                    mRunningThread--;
                    LOGW(" mRunningthread-- : %d", mRunningThread);
                }
                mFimcStoppedCondition.signal();
                mFimcLock.unlock();
                LOGW(" RecordFimcThread exit with false-- : %d", mRunningThread);
                return false;
            }
        } else {
            LOGW("recordFimcThread: we are late, cur index (%d), old index (%d)", mCurrentIndex, mOldRecordIndex);
        }
        mOldRecordIndex = mCurrentIndex;

        timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
        int recordingIndex = 0;

#ifdef BOARD_USE_V4L2_ION
        numArray = mOldRecordIndex;
#else
        recordingIndex = mOldRecordIndex;
        mSecCamera->runRecordFimcOneshot(recordingIndex, (unsigned int)(mSecCamera->getShareBufferAddr(recordingIndex)));
        mSecCamera->getRecordPhysAddr(recordingIndex, &recordAddr);

        LOGD("index (%d), record PhyY(0x%08x) phyC(0x%08x) ", recordingIndex, recordAddr.phys.extP[0], recordAddr.phys.extP[1]);

        if (recordAddr.phys.extP[0] == 0xffffffff || recordAddr.phys.extP[1] == 0xffffffff) {
            LOGE("ERR(%s):Fail on SecCamera getRectPhyAddr Y addr = %0x C addr = %0x", __func__,
                 recordAddr.phys.extP[0], recordAddr.phys.extP[1]);
            return UNKNOWN_ERROR;
        }

        addrs = (struct addrs *)(*mRecordHeap)->data;

        addrs[recordingIndex].type   = kMetadataBufferTypeCameraSource;
        addrs[recordingIndex].addr_y = recordAddr.phys.extP[0];
        addrs[recordingIndex].addr_cbcr = recordAddr.phys.extP[1];
        addrs[recordingIndex].buf_index = recordingIndex;
#endif

        // Notify the client of a new frame.
        if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) {
            mDataCbTimestamp(timestamp, CAMERA_MSG_VIDEO_FRAME,
                             mRecordHeap[0], recordingIndex, mCallbackCookie);
        } else {
            //jmq.not need to release in isp modes
            //mSecCamera->releaseRecordFrame(recordingIndex);
            LOGW("CAMERA_MSG_VIDEO_FRAME is disabled in recordFimcThread\n");
        }
    } else {
        LOGW("+++++++++++++false \n");
        mFimcLock.lock();
        if (mRunningThread > 0) {
            mRunningThread--;
            LOGD(" mRunningthread-- : %d", mRunningThread);
        }
        mFimcStoppedCondition.signal();
        mFimcLock.unlock();
        return false;
    }

    return true;
}

#ifdef BURST_SHOT
status_t CameraHardwareSec::burstshotFimcThread()
{
    SecBuffer capAddr;
    int i =0;
    int burstindex;

    LOGD("%s", __func__);

    if (mPreviewRunning == false) {
        LOGD("try burstshot Fimc stop");
        mFimcLock.lock();
        LOGD("burstshot Fimc stop");
        if (mRunningThread > 0) {
            mRunningThread--;
            LOGD(" mRunningthread-- : %d", mRunningThread);
        }
        mFimcStoppedCondition.signal();
        mFimcLock.unlock();
        mBreakBurstFimcThread=true;
        mBurstFirstCondition.signal();
        LOGD("burstshot Fimc return num 1 !");
        return false;
    }

    if(mburstencodeflag){
        if (mCurrentIndex < 0) {
            LOGD("%s: doing nothing mCurrnet index < 0", __func__);
            return true;
        }

        if (mOldBurstIndex == mCurrentIndex){
            mFimcLock.lock();
            mFimcActionCondition.wait(mFimcLock);
            mFimcLock.unlock();
            if (!mburstencodeflag) {
                mFimcLock.lock();
                if (mRunningThread > 0) {
                    mRunningThread--;
                    LOGD(" mRunningthread-- : %d", mRunningThread);
                }
                mFimcStoppedCondition.signal();
                mFimcLock.unlock();
                return false;
            }
        } else {
            LOGD("burstFimcThread: cur index (%d), old index (%d)", mCurrentIndex, mOldBurstIndex);
        }
        
        mOldBurstIndex = mCurrentIndex;
        burstindex = mOldBurstIndex;

        if (!mIsBracketEV)
        {
            for(i=0;i<BURST_CAP_BUFFERS;i++){
                if(mburst_index[i]==(-1)){
                    mburst_index[i] = mCurrentIndex;
                    mSecCamera->runBurstshotFimcOneshot(i,(unsigned int)(mSecCamera->getShareBufferAddr(burstindex)));
                    mSecCamera->getBurstPhysAddr(mCapIndex, &capAddr);
                    LOGD("(%s):i:%d-burstindex:%d", __func__,i,burstindex);
                    break;
                }
            }

            if (capAddr.phys.extP[0] == 0xffffffff)
            {
                LOGE("ERR(%s):Fail on SecCamera getBurstPhysAddr start addr = %0x", __func__, capAddr.phys.extP[0]);
                return UNKNOWN_ERROR;
            }

            if(mburst_index[BURST_CAP_BUFFERS-1] !=(-1))
            {
                LOGD("(%s):8 new YUV frames are refreshed after burstshot command", __func__);
                LOGD("burst In Progress-wait until encode finish");

                //mBurstPictureLock.unlock();

                if (mPreviewRunning == false) {
                    LOGD("try burstshot Fimc stop");
                    mFimcLock.lock();
                    LOGD("burstshot Fimc stop");
                    if (mRunningThread > 0) {
                        mRunningThread--;
                        LOGD(" mRunningthread-- : %d", mRunningThread);
                    }
                    mFimcStoppedCondition.signal();
                    mFimcLock.unlock();
                    mBreakBurstFimcThread=true;
                    mBurstFirstCondition.signal();
                    LOGD("burstshot Fimc return num 2 !");
                    return false;
                }
                mBurstFirstCondition.signal();
                mTakePictureLock.lock();
                mStopPictureCondition.wait(mTakePictureLock);
                mTakePictureLock.unlock();
            }

        }
        else
        {
            if(mburstbufferflag == true)
            {
                for(i=0;i<CAP_BUFFERS;i++)
                {
                    mburst_index[i] = (mCurrentIndex+i)%CAP_BUFFERS;
                    LOGD("mburst_index[%d] = %d==", i, mburst_index[i]);
                }

                mburstbufferflag = false;
                LOGD("(%s):burststartindex:%d", __func__,mburst_index[0]);
            }
            if (mExposureCounter < mBurstCapture -1)
            {
                if(mburst_index[mExposureCounter*EV_SKIP_FRAME]	== mCurrentIndex)
                {
                    mSecCamera->setExposure(mBracketingEx[mExposureCounter]);
                    mExposureCounter+=1;
                }
                else if(mburst_index[mExposureCounter*EV_SKIP_FRAME] == mCurrentIndex)
                {
                    mSecCamera->setExposure(mBracketingEx[mExposureCounter]);
                    mExposureCounter+=1;
                }
                //else if(mburst_index[4] == mCurrentIndex)
                //{
                //mSecCamera->setExposure(mBracketingEx[mExposureCounter]);
                //mExposureCounter+=1;
                //}
            }


            // LOGD("(%s):mCurrentIndex:%d,EV:%d", __func__,mCurrentIndex,mSecCamera->getEVFrame(mCurrentIndex));

            // wait for the valid 3 frame
            if((mSecCamera->getEVFrame(mCurrentIndex)==(mBracketingEx[0]))&& (mhdr_index[0]==(-1)))
            {
                for(i=0;i<CAP_BUFFERS;i++)
                    if(mburst_index[i] == mCurrentIndex )
                    {
                        mhdr_index[0] = i;
                        break;
                    }
                mhdr_count++;
                mSecCamera->runBurstshotFimcOneshot(0,(unsigned int)(mSecCamera->getShareBufferAddr(mCurrentIndex)));
                mSecCamera->getBurstPhysAddr(mCapIndex, &capAddr);
                LOGD("(%s):EV:minus 4-i:%d-mCurrentIndex:%d", __func__,i,mCurrentIndex);
            }
            else if((mSecCamera->getEVFrame(mCurrentIndex)==(mBracketingEx[1])) && (mhdr_index[1]==(-1)))
            {
                for(i=0;i<CAP_BUFFERS;i++)
                    if(mburst_index[i] == mCurrentIndex)
                    {
                        mhdr_index[1] = i;
                        break;
                    }
                mhdr_count++;
                mSecCamera->runBurstshotFimcOneshot(1,(unsigned int)(mSecCamera->getShareBufferAddr(mCurrentIndex)));
                mSecCamera->getBurstPhysAddr(mCapIndex, &capAddr);
                LOGD("(%s):EV:plus 4-i:%d-mCurrentIndex:%d", __func__,i,mCurrentIndex);
            }
            else if(/*(mSecCamera->getEVFrame(mCurrentIndex)== mBracketingEx[2]) &&*/ (mhdr_index[2]==(-1)))/*s3 0725 patch*/
            {
                for(i=0;i<CAP_BUFFERS;i++)
                    if(mburst_index[i] == mCurrentIndex)
                    {
                        mhdr_index[2] = i;
                        break;
                    }
                mhdr_count++;
                mSecCamera->runBurstshotFimcOneshot(2,(unsigned int)(mSecCamera->getShareBufferAddr(mCurrentIndex)));
                mSecCamera->getBurstPhysAddr(mCapIndex, &capAddr);
                LOGD("(%s):EV:default-i:%d-mCurrentIndex:%d", __func__,i,mCurrentIndex);
            }

            if(mhdr_count == 3)
            {
                // haoli 0720 for hdr interrupt
                for(int i = 0; i <3; i++)
                    mhdr_index[i] = -1;
                LOGD("(%s):8 new YUV frames are refreshed after burstshot command", __func__);
                LOGD("burst In Progress-wait until encode finish");

                //mBurstPictureLock.unlock();

                if (mPreviewRunning == false) {
                    LOGD("try burstshot Fimc stop");
                    mFimcLock.lock();
                    LOGD("burstshot Fimc stop");
                    if (mRunningThread > 0) {
                        mRunningThread--;
                        LOGD(" mRunningthread-- : %d", mRunningThread);
                    }
                    mFimcStoppedCondition.signal();
                    mFimcLock.unlock();
                    mBreakBurstFimcThread=true;
                    mBurstFirstCondition.signal();
                    LOGD("burstshot Fimc return num 3 !");
                    return false;
                }
                mBurstFirstCondition.signal();
                mTakePictureLock.lock();
                mStopPictureCondition.wait(mTakePictureLock);
                mTakePictureLock.unlock();
            }
        }

        //mTakePictureCondition.signal();
    }else{
        mFimcLock.lock();
        if (mRunningThread > 0) {
            mRunningThread--;
            LOGD(" mRunningthread-- : %d", mRunningThread);
        }
        mFimcStoppedCondition.signal();
        mFimcLock.unlock();
        LOGD("burstshot Fimc return num 4 !");
        return false;
    }


    return true;
}
#endif

status_t CameraHardwareSec::callbackThread()
{
    LOGD("%s", __func__);

    mCallbackLock.lock();

    mCallbackCondition.wait(mCallbackLock);

    if (mUseInternalISP && (mMsgEnabled & CAMERA_MSG_PREVIEW_METADATA) && mPreviewRunning && !mRecordRunning) {
        if((mFaceDataHeap != NULL)&&  (mDataCb!=NULL) )//jmq.sometimes callback dumped, avoid it. Debugging
            mDataCb(CAMERA_MSG_PREVIEW_METADATA, mFaceDataHeap, 0, &mFaceData, mCallbackCookie);
        else
            if(mFaceDataHeap == NULL)
                LOGE(" %s mFaceDataHeap  is NULL", __func__);
            else
                LOGE(" %s mDataCb  is NULL", __func__);
    }

    mCallbackLock.unlock();

    return true;
}

#ifdef IS_FW_DEBUG
bool CameraHardwareSec::debugThread()
{
    unsigned int LP, CP;

    mDebugLock.lock();
    mDebugCondition.waitRelative(mDebugLock, 2000000000);
    if (mStopDebugging) {
        mDebugLock.unlock();
        return false;
    }

    mCurrWp = mIs_debug_ctrl.write_point;
    mCurrOffset = mCurrWp - FIMC_IS_FW_DEBUG_REGION_ADDR;

    if (mCurrWp != mPrevWp) {
        *(char *)(mDebugVaddr + mCurrOffset - 1) = '\0';
        LP = CP = mDebugVaddr + mPrevOffset;

        if (mCurrWp < mPrevWp) {
            *(char *)(mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE - 1) = '\0';
            while (CP < (mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE) && *(char *)CP != NULL) {
                while(*(char *)CP != '\n' && *(char *)CP != NULL) {
                    CP++;
                }
                *(char *)CP = NULL;
                if (*(char *)LP != NULL)
                    LOGD_IS("%s", LP);
                LP = ++CP;
            }
            LP = CP = mDebugVaddr;
        }

        while (CP < (mDebugVaddr + mCurrOffset) && *(char *)CP != NULL) {
            while(*(char *)CP != '\n' && *(char *)CP != NULL) {
                CP++;
            }
            *(char *)CP = NULL;
            if (*(char *)LP != NULL)
                LOGD_IS("%s", LP);
            LP = ++CP;
        }
    }

    mPrevWp = mIs_debug_ctrl.write_point;
    mPrevOffset = mPrevWp - FIMC_IS_FW_DEBUG_REGION_ADDR;
    mDebugLock.unlock();

    return true;
}
#endif

status_t CameraHardwareSec::startPreview()
{
    int ret = 0;

    LOGD("%s :", __func__);

    Mutex::Autolock lock(mStateLock);
    if (mCaptureInProgress) {
        LOGE("%s : capture in progress, not allowed", __func__);
        return INVALID_OPERATION;
    }

    mPreviewLock.lock();
    if (mPreviewRunning) {
        // already running
        LOGE("%s : preview thread already running", __func__);
        mPreviewLock.unlock();
        return INVALID_OPERATION;
    }

    mPreviewRunning = true;
    mPreviewStartDeferred = false;

    if (!mPreviewWindow &&
            !(mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) &&
            !(mMsgEnabled & CAMERA_MSG_VIDEO_FRAME)) {
        LOGI("%s : deferring", __func__);
        mPreviewStartDeferred = true;
        mPreviewLock.unlock();
        return NO_ERROR;
    }

    ret = startPreviewInternal();
    if (ret == OK)
        mPreviewCondition.signal();

    mPreviewLock.unlock();
    return ret;
}

status_t CameraHardwareSec::startPreviewInternal()
{
    LOGD("%s", __func__);
    int width, height, frame_size;

    mSecCamera->getPreviewSize(&width, &height, &frame_size);
    LOGD("mPreviewHeap(fd(%d), size(%d), width(%d), height(%d))",
         mSecCamera->getCameraFd(SecCamera::PREVIEW), frame_size + mFrameSizeDelta, width, height);

#ifdef BOARD_USE_V4L2_ION
#ifdef ZERO_SHUTTER_LAG
    /*TODO*/
    int mPostViewWidth, mPostViewHeight, mPostViewSize;
    mSecCamera->getPostViewConfig(&mPostViewWidth, &mPostViewHeight, &mPostViewSize);
    for(int i = 0; i < CAP_BUFFERS; i++) {
        mPostviewHeap[i] = new MemoryHeapBaseIon(mPostViewSize);
        mSecCamera->setUserBufferAddr(mPostviewHeap[i]->base(), i, CAPTURE_MODE);
    }
#endif
    void *vaddr[3];

    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (mBufferHandle[i] == NULL) {
            if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &mBufferHandle[i], &mStride[i])) {
                LOGE("%s: Could not dequeue gralloc buffer[%d]!!", __func__, i);
                return INVALID_OPERATION;
            }
        }
        if (mGrallocHal->lock(mGrallocHal,
                              *mBufferHandle[i],
                              GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_YUV_ADDR,
                              0, 0, width, height, vaddr)) {
            LOGE("ERR(%s): Could not get virtual address!!, index = %d", __func__, i);
            return UNKNOWN_ERROR;
        }
        mSecCamera->setUserBufferAddr(vaddr, i, PREVIEW_MODE);
    }
#endif

    int ret  = mSecCamera->startPreview();
    LOGD("%s : mSecCamera->startPreview() returned %d", __func__, ret);

    if (ret < 0) {
        LOGE("ERR(%s):Fail on mSecCamera->startPreview()", __func__);
        return UNKNOWN_ERROR;
    }
    //if (mCameraID == SecCamera::CAMERA_ID_BACK)
    setSkipFrame(INITIAL_SKIP_FRAME);
    //else
    //	setSkipFrame(INITIAL_SKIP_FRAME_FRONT);

    if (mPreviewHeap) {
        mPreviewHeap->release(mPreviewHeap);
        mPreviewHeap = 0;
    }

    if (mFaceDataHeap) {
        mFaceDataHeap->release(mFaceDataHeap);
        mFaceDataHeap = 0;
    }

    for(int i=0; i<BUFFER_COUNT_FOR_ARRAY; i++){
        if (mRecordHeap[i] != NULL) {
            mRecordHeap[i]->release(mRecordHeap[i]);
            mRecordHeap[i] = 0;
        }
    }

#ifndef BOARD_USE_V4L2  
    /*add support RGB565 for Front Camera jijun.yu 20120723*/
    if (!mUseInternalISP &&!strcmp(mParameters.getPreviewFormat(),
                                   CameraParameters::PIXEL_FORMAT_RGB565))
        mPreviewHeap = mGetMemoryCb((int)mSecCamera->getCameraFd(SecCamera::PREVIEW),
                                    ALIGN(frame_size,4096) ,//+ mFrameSizeDelta, //for pass CTS test jijun.yu
                                    MAX_BUFFERS,
                                    0); // no cookie
    else
        mPreviewHeap = mGetMemoryCb((int)mSecCamera->getCameraFd(SecCamera::PREVIEW),
                                    frame_size,//+ mFrameSizeDelta, //for pass CTS test jijun.yu
                                    MAX_BUFFERS,
                                    0); // no cookie
#endif

    mFaceDataHeap = mGetMemoryCb(-1, 1, 1, 0);

    mSecCamera->getPostViewConfig(&mPostViewWidth, &mPostViewHeight, &mPostViewSize);
    LOGD("CameraHardwareSec: mPostViewWidth = %d mPostViewHeight = %d mPostViewSize = %d",
         mPostViewWidth,mPostViewHeight,mPostViewSize);

    if (mUseInternalISP) {
        LOGD("%s: run preview fimc", __func__);
        if (mPreviewFimcThread->run("CameraPreviewFimcThread", PRIORITY_URGENT_DISPLAY) != NO_ERROR) {
            LOGE("%s : couldn't run preview fimc thread", __func__);
            return INVALID_OPERATION;
        } else {
            mRunningThread++;
        }

        //Flash on when torch
        if(mCameraID == SecCamera::CAMERA_ID_BACK) {
            const char *new_flash_mode_str = mParameters.get(CameraParameters::KEY_FLASH_MODE);

            if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_TORCH))
            {
                setTorchScenario();
                mSecCamera->setFlashMode(FLASH_MODE_TORCH);
            }
        }

        //Create status thread only for back camera
#ifdef S3_STATUS_UPDATE
        if (mStatusUpdateThread == NULL) {
            mExitStatusThread = false;
            mStatusUpdateThread = new S3StatusThread(this);
        }
#endif

#ifdef S3_LOSSFOCUS_NOTIFY
        if (mLossFocusNotifyThread == NULL &&
                msgTypeEnabled( CAMERA_MSG_CHECK_LOSS_FOCUS)) {
            mExitLossFocusNotifyThread = false;
            mLossFocusNotifyThread = new LossFocusNotifyThread(this, mFocusSave);
        }
#endif
#ifdef IS_FW_DEBUG
        if (mSecCamera->getDebugAddr(&mDebugVaddr) < 0) {
            LOGE("ERR(%s):Fail on SecCamera->getDebugAddr()", __func__);
            return UNKNOWN_ERROR;
        }

        mIs_debug_ctrl.write_point = *((unsigned int *)(mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE));
        mIs_debug_ctrl.assert_flag = *((unsigned int *)(mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE + 4));
        mIs_debug_ctrl.pabort_flag = *((unsigned int *)(mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE + 8));
        mIs_debug_ctrl.dabort_flag = *((unsigned int *)(mDebugVaddr + FIMC_IS_FW_DEBUG_REGION_SIZE + 12));
        mDebugThread->run("debugThread", PRIORITY_DEFAULT);
#endif
    }

    return NO_ERROR;
}

void CameraHardwareSec::stopPreviewInternal()
{
    LOGD("%s :", __func__);

    /* request that the preview thread stop. */
    if (mPreviewRunning) {
        /* TODO : we have to waiting all of FIMC threads */
        mFimcLock.lock();
        mPreviewRunning = false;
        mStopPictureCondition.signal();
        while (mRunningThread > 0) {
            mFimcActionCondition.broadcast();
            LOGD("%s: wait for all thread stop, %d, mPreview running %d", __func__, mRunningThread, mPreviewRunning);
            mFimcStoppedCondition.wait(mFimcLock);
        }
        mFimcLock.unlock();

        mRunningThread = 0;

        if (!mPreviewStartDeferred) {
            mPreviewCondition.signal();
            /* wait until preview thread is stopped */
            mPreviewStoppedCondition.wait(mPreviewLock);

#ifdef BOARD_USE_V4L2_ION
            for (int i = 0; i < MAX_BUFFERS; i++) {
                if (mBufferHandle[i] != NULL) {
                    if (0 != mPreviewWindow->cancel_buffer(mPreviewWindow, mBufferHandle[i])) {
                        LOGE("%s: Fail to cancel buffer[%d]", __func__, i);
                    } else {
                        mBufferHandle[i] = NULL;
                        mStride[i] = NULL;
                    }
                }
            }
#endif
        }
        else
            LOGD("%s : preview running but deferred, doing nothing", __func__);

        //thread exit move to end to avoid block mPreviewStoppedCondition.wait(mPreviewLock)
#ifdef S3_LOSSFOCUS_NOTIFY
        if (mLossFocusNotifyThread!= NULL) {
            mLossFocusNotifyThread->requestExit();
            mExitLossFocusNotifyThread = true;
            mLossFocusNotifyThread->requestExitAndWait();
            mLossFocusNotifyThread.clear();
            mLossFocusNotifyThread = NULL;
        }
#endif

#ifdef S3_STATUS_UPDATE
        if (mStatusUpdateThread!= NULL) {
            mStatusUpdateThread->requestExit();
            mExitStatusThread = true;
            mCheckLightCondition.signal();
            mStatusUpdateThread->requestExitAndWait();
            mStatusUpdateThread.clear();
            mStatusUpdateThread = NULL;
        }
#endif
    } else
        LOGI("%s : preview not running, doing nothing", __func__);
}

void CameraHardwareSec::stopPreview()
{
    LOGD("%s :", __func__);
    Mutex::Autolock lock(mStateLock);

    /* request that the preview thread stop. */
    mPreviewLock.lock();
    stopPreviewInternal();
    mPreviewLock.unlock();
}

bool CameraHardwareSec::previewEnabled()
{
    Mutex::Autolock lock(mPreviewLock);
    LOGD("%s : preview running %d", __func__, mPreviewRunning);
    return mPreviewRunning;
}

status_t CameraHardwareSec::startRecording()
{
    LOGD("%s :", __func__);

    Mutex::Autolock lock(mRecordLock);

    for(int i = 0; i<BUFFER_COUNT_FOR_ARRAY; i++){
        if (mRecordHeap[i] != NULL) {
            mRecordHeap[i]->release(mRecordHeap[i]);
            mRecordHeap[i] = 0;
        }

#ifdef BOARD_USE_V4L2_ION
        int width, height, size;

        /* TODO : For V4L2_ION. This is temporary code. we need to modify.
         *        mSecCamera->getRecordingSize(&width, &height);
         */
        mSecCamera->getPreviewSize(&width, &height, &size);

        mRecordHeap[i] = mGetMemoryCb(-1, (ALIGN((ALIGN(width, 16) * ALIGN(height, 16)), 2048) + ALIGN((ALIGN(width, 16) * ALIGN(height >> 1, 8)), 2048)), 1, NULL);

        mSecCamera->setUserBufferAddr((void *)(mRecordHeap[i]->data), i, RECORD_MODE);
#else
        mRecordHeap[i] = mGetMemoryCb(-1, sizeof(struct addrs), MAX_BUFFERS, NULL);
#endif
        if (!mRecordHeap[i]) {
            LOGE("ERR(%s): Record heap[%d] creation fail", __func__, i);
            return UNKNOWN_ERROR;
        }
    }

    LOGD("mRecordHeaps alloc done");

    if (mRecordRunning == false) {
        if (mUseInternalISP) {
            mSecCamera->setFimcForRecord();
            LOGD("%s: run record fimc", __func__);
            if (mRecordFimcThread->run("CameraRecordFimcThread", PRIORITY_URGENT_DISPLAY) != NO_ERROR) {
                LOGE("%s : couldn't run preview fimc thread", __func__);
                return INVALID_OPERATION;
            } else {
                mRunningThread++;
            }
        } else {
            if (mSecCamera->startRecord(mRecordHint) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->startRecord()", __func__);
                return UNKNOWN_ERROR;
            }
            /*for ov Camera split preview and recorder thread add by jijun.yu 0803 */
            if (mRecordThread->run("CameraRecordThread", PRIORITY_URGENT_DISPLAY) != NO_ERROR) {
                LOGE("%s : couldn't run preview fimc thread", __func__);
                return INVALID_OPERATION;
            } else {
                mRunningThread++;
            }
        }
        mRecordRunning = true;
    }
    return NO_ERROR;
}

void CameraHardwareSec::stopRecording()
{
    LOGD("%s :", __func__);

    /*Sometimes, the record can not stop in time  0803 jijun.yu*/
    //   if (mRecordRunning == true && (!mUseInternalISP))
    //	 mRecordRunning = false;

    if (mRecordRunning == true) {
        if (!mUseInternalISP) {
            /*cancel the record thread for Ov 0806 jijun.yu*/
            if (mRecordThread.get()) {
                LOGD("%s: waiting for recorder thread to exit", __func__);
                mRecordThread->requestExitAndWait();
                mRunningThread --;
                LOGD("%s: recorder thread has exited", __func__);
            }
        } else {
            if (mRecordFimcThread.get()) {
                mRecordFimcThread->requestExitAndWait();
                if (mRunningThread > 0) {
                    mRunningThread--;
                }
                LOGD("%s: FIMC recorder thread has exited", __func__);
            }
        }

        Mutex::Autolock lock(mRecordLock);
        if (mSecCamera->stopRecord() < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->stopRecord()", __func__);
            return;
        }
        mRecordRunning = false;
    }

}

bool CameraHardwareSec::recordingEnabled()
{
    LOGD("%s :", __func__);

    return mRecordRunning;
}

void CameraHardwareSec::releaseRecordingFrame(const void *opaque)
{
    LOGD("%s :", __func__);
#ifdef BOARD_USE_V4L2_ION
    int i;
    for (i = 0; i < MAX_BUFFERS; i++)
        if ((char *)mRecordHeap[i]->data == (char *)opaque)
            break;

    mSecCamera->releaseRecordFrame(i);
#else
    if (!mUseInternalISP) {
        struct addrs *addrs = (struct addrs *)opaque;
        mSecCamera->releaseRecordFrame(addrs->buf_index);
    }
#endif
}

int CameraHardwareSec::autoFocusThread()
{
    int count =0;
    int af_status =0 ;
    int ret =0;
    int flash_mode =0;

    LOGD("%s : starting", __func__);



    /* block until we're told to start.  we don't want to use
     * a restartable thread and requestExitAndWait() in cancelAutoFocus()
     * because it would cause deadlock between our callbacks and the
     * caller of cancelAutoFocus() which both want to grab the same lock
     * in CameraServices layer.
     */
    mFocusLock.lock();
    /* check early exit request */
    if (mExitAutoFocusThread) {
        mFocusLock.unlock();
        LOGD("%s : exiting on request0", __func__);
        return NO_ERROR;
    }
    mFocusCondition.wait(mFocusLock);
    /* check early exit request */
    if (mExitAutoFocusThread) {
        mFocusLock.unlock();
        LOGD("%s : exiting on request1", __func__);
        return NO_ERROR;
    }
    mFocusLock.unlock();

    /* TODO : Currently only possible auto focus at BACK caemra
              We need to modify to check that sensor can support auto focus */
    if (mCameraID == SecCamera::CAMERA_ID_BACK) {
        /* Sometimes camera will call autofocus in continuous-focus mode(KEY_FOCUS_MODE: continuous-xxx),
            auto focus will not working. Set focus mode auto here, it will set back in next setParameter after finish
            the auto focus.  Jiangshanbin 2012/06/05*/
        //mSecCamera->setFocusMode(FOCUS_MODE_AUTO);
        LOGD("%s : calling setAutoFocus", __func__);
        if (mTouched == 0) {
            LOGD("%s : calling setAutoFocus", __func__);
            mSecCamera->setFocusMode(FOCUS_MODE_AUTO);
            if (mSecCamera->setAutofocus() < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setAutofocus()", __func__);
                af_status = 0;//indicate error
                goto notifycaller;
                //return UNKNOWN_ERROR;
            }
        } else {
            LOGD("%s : calling setTouchAF", __func__);

#ifdef TAF_CAF_FLASH
            const char *new_flash_mode_str = mParameters.get(CameraParameters::KEY_FLASH_MODE);

            if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_AUTO))
                flash_mode = FLASH_MODE_AUTO;
            else if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_ON))
                flash_mode = FLASH_MODE_ON;
            else
                flash_mode = FLASH_MODE_BASE;

            if (mSecCamera->setTouchAF(flash_mode) < 0) {
                mTouched = 0;
                LOGE("ERR(%s):Fail on mSecCamera->setAutofocus()", __func__);
                af_status = 0;//indicate error
                goto notifycaller;
                //return UNKNOWN_ERROR;
            }


#else
            if (mSecCamera->setTouchAF() < 0) {
                mTouched = 0;
                LOGE("ERR(%s):Fail on mSecCamera->setAutofocus()", __func__);
                af_status = 0;//indicate error
                goto notifycaller;
                //return UNKNOWN_ERROR;
            }

#endif
        }
    }

    /* TODO */
    /* This is temperary implementation.
       When camera support AF blocking mode, this code will be removed
       Continous AutoFocus is not need to success */
    //const char *focusModeStr = mParameters.get(CameraParameters::KEY_FOCUS_MODE);
    //int isContinousAF = !strncmp(focusModeStr, CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO, 7);
    //if (mUseInternalISP && !isContinousAF) {
    if (mUseInternalISP && mTouched&& (mCameraID == SecCamera::CAMERA_ID_BACK)) {
        int i, err = -1;			//too long time for CTS  !!!!!!!!!!!
        /*the time decrease from 400  if set to 400, the consume time is too long when the focus lost*/
        for (i = 0; i < 200; i++) {
            usleep(10000);

            af_status = mSecCamera->getAutoFocusResult();

            if ((af_status & 0x2)) {
                err = 0;
                break;
            }
        }
#ifdef TAF_CAF_FLASH
        if((flash_mode == FLASH_MODE_AUTO)||( flash_mode ==FLASH_MODE_ON))
            mSecCamera->setTouchAF(FLASH_MODE_OFF);
#endif
    } else {
        //af_status = mSecCamera->getAutoFocusResult();
        af_status = 0x02;//lisw if sensor do not support auto focus function
    }
    mTouched = 0;

notifycaller:
#if 1 //##mmkim -- test
    if (af_status == 0x02) {
        LOGD("%s : AF Success !!", __func__);
        if (mMsgEnabled & CAMERA_MSG_FOCUS) {
            /* CAMERA_MSG_FOCUS only takes a bool.  true for
             * finished and false for failure.  cancel is still
             * considered a true result.
             */
            mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
        }
    } else {
        LOGD("%s : AF Fail !!", __func__);
        LOGD("%s : mMsgEnabled = 0x%x", __func__, mMsgEnabled);
        if (mMsgEnabled & CAMERA_MSG_FOCUS)
            mNotifyCb(CAMERA_MSG_FOCUS, false, 0, mCallbackCookie);
    }
#else
    if (af_status == 0x01) {
        LOGD("%s : AF Cancelled !!", __func__);
        if (mMsgEnabled & CAMERA_MSG_FOCUS)
            mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
    } else if (af_status == 0x02) {
        LOGD("%s : AF Success !!", __func__);
        if (mMsgEnabled & CAMERA_MSG_FOCUS) {
            /* CAMERA_MSG_FOCUS only takes a bool.  true for
             * finished and false for failure.  cancel is still
             * considered a true result.
             */
            mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
        }
    } else {
        LOGD("%s : AF Fail !!", __func__);
        LOGD("%s : mMsgEnabled = 0x%x", __func__, mMsgEnabled);
        if (mMsgEnabled & CAMERA_MSG_FOCUS)
            mNotifyCb(CAMERA_MSG_FOCUS, false, 0, mCallbackCookie);
    }
#endif
    LOGD("%s : exiting with no error", __func__);
    return NO_ERROR;
}

status_t CameraHardwareSec::autoFocus()
{
    LOGD("%s :", __func__);
    /* signal autoFocusThread to run once */
    mFocusCondition.signal();
    return NO_ERROR;
}

status_t CameraHardwareSec::cancelAutoFocus()
{
    LOGD("%s :", __func__);

    if (mSecCamera->cancelAutofocus() < 0) {
        LOGE("ERR(%s):Fail on mSecCamera->cancelAutofocus()", __func__);
        return UNKNOWN_ERROR;
    }

    return NO_ERROR;
}

int CameraHardwareSec::save_jpeg( unsigned char *real_jpeg, int jpeg_size)
{
    FILE *yuv_fp = NULL;
    char filename[100], *buffer = NULL;

    /* file create/open, note to "wb" */
    yuv_fp = fopen("/data/camera_dump.jpeg", "wb");
    if (yuv_fp == NULL) {
        LOGE("Save jpeg file open error");
        return -1;
    }

    LOGD("[BestIQ]  real_jpeg size ========>  %d", jpeg_size);
    buffer = (char *) malloc(jpeg_size);
    if (buffer == NULL) {
        LOGE("Save YUV] buffer alloc failed");
        if (yuv_fp)
            fclose(yuv_fp);

        return -1;
    }

    memcpy(buffer, real_jpeg, jpeg_size);

    fflush(stdout);

    fwrite(buffer, 1, jpeg_size, yuv_fp);

    fflush(yuv_fp);

    if (yuv_fp)
        fclose(yuv_fp);
    if (buffer)
        free(buffer);

    return 0;
}

void CameraHardwareSec::save_postview(const char *fname, uint8_t *buf, uint32_t size)
{
    int nw;
    int cnt = 0;
    uint32_t written = 0;

    LOGD("opening file [%s]", fname);
    int fd = open(fname, O_RDWR | O_CREAT,S_IRUSR|S_IWUSR);
    if (fd < 0) {
        LOGE("failed to create file [%s]: %s", fname, strerror(errno));
        return;
    }

    LOGD("writing %d bytes to file [%s]", size, fname);
    while (written < size) {
        nw = ::write(fd, buf + written, size - written);
        if (nw < 0) {
            LOGE("failed to write to file %d [%s]: %s",written,fname, strerror(errno));
            break;
        }
        written += nw;
        cnt++;
    }
    LOGD("done writing %d bytes to file [%s] in %d passes",size, fname, cnt);
    ::close(fd);
}

#if 0
bool CameraHardwareSec::scaleDownYuv422(char *srcBuf, uint32_t srcWidth, uint32_t srcHeight,
                                        char *dstBuf, uint32_t dstWidth, uint32_t dstHeight)
{
    int32_t step_x, step_y;
    int32_t iXsrc, iXdst;
    int32_t x, y, src_y_start_pos, dst_pos, src_pos;

    if (dstWidth % 2 != 0 || dstHeight % 2 != 0) {
        LOGE("scale_down_yuv422: invalid width, height for scaling");
        return false;
    }

    step_x = srcWidth / dstWidth;
    step_y = srcHeight / dstHeight;

    dst_pos = 0;
    for (uint32_t y = 0; y < dstHeight; y++) {
        src_y_start_pos = (y * step_y * (srcWidth * 2));

        for (uint32_t x = 0; x < dstWidth; x += 2) {
            src_pos = src_y_start_pos + (x * (step_x * 2));

            dstBuf[dst_pos++] = srcBuf[src_pos    ];
            dstBuf[dst_pos++] = srcBuf[src_pos + 1];
            dstBuf[dst_pos++] = srcBuf[src_pos + 2];
            dstBuf[dst_pos++] = srcBuf[src_pos + 3];
        }
    }

    return true;
}
#else
//yqf, test
bool CameraHardwareSec::scaleDownYuv422(char *srcBuf, uint32_t srcWidth, uint32_t srcHeight,
                                        char *dstBuf, uint32_t dstWidth, uint32_t dstHeight)
{
    int32_t step_x, step_y;
    int32_t iXsrc, iXdst;
    int32_t x, y, src_y_start_pos, dst_pos, src_pos;
    int32_t  i,j, m,n,sum_y0,sum_y1,sum_cb0,sum_cr0,average_y0,average_y1,average_cb0,average_cr0,start;

    if (dstWidth % 2 != 0 || dstHeight % 2 != 0) {
        LOGE("scale_down_yuv422: invalid width, height for scaling");
        return false;
    }

    step_x = srcWidth  / dstWidth;
    step_y = srcHeight / dstHeight;

    dst_pos = 0;
    m = 0; //row m in src
    n = 0; //column n in src
    LOGD("yqf test, srcWidth:%d, srcHeight:%d, dstWidth:%d, dstHeight:%d \n", srcWidth, srcHeight, dstWidth, dstHeight);

    for( y = 0 ; y < dstHeight ; y++ )
    {

        for( x = 0 ; x < dstWidth ; x += 2 ){

            sum_y0  = 0;
            sum_cb0 = 0;
            sum_y1  = 0;
            sum_cr0 = 0;

            for( j = 0 ; j < step_y ; j++ ){

                start = (m + j)*srcWidth*2 + n*4;    //yqf, take 'y0cb0y1cr0' as unit to process

                for( i = 0 ; i < step_x ; i++ ){
                    sum_y0  += srcBuf[start+4*i];
                    sum_cb0 += srcBuf[start+4*i+1];
                    sum_y1  += srcBuf[start+4*i+2];
                    sum_cr0 += srcBuf[start+4*i+3];
                }

            }
            average_y0  = sum_y0 /(step_x*step_y);
            average_cb0 = sum_cb0/(step_x*step_y);
            average_y1  = sum_y1 /(step_x*step_y);
            average_cr0 = sum_cr0/(step_x*step_y);

            dstBuf[dst_pos++] = average_y0;
            dstBuf[dst_pos++] = average_cb0;
            dstBuf[dst_pos++] = average_y1;
            dstBuf[dst_pos++] = average_cr0;

            n += step_x;
        }
        m += step_y;
        n = 0;
    }

    return true;
}
#endif

bool CameraHardwareSec::scaleDownYuv420sp(struct SecBuffer *srcBuf, uint32_t srcWidth, uint32_t srcHeight,
                                          char *dstBuf, uint32_t dstWidth, uint32_t dstHeight)
{
    int32_t step_x, step_y;
    int32_t iXsrc, iXdst;
    int32_t x, y, src_y_start_pos, dst_pos, src_pos;
    int32_t src_Y_offset;
    char *src_buf;

    if (dstWidth % 2 != 0 || dstHeight % 2 != 0) {
        LOGE("scale_down_yuv422: invalid width, height for scaling");
        return false;
    }

    step_x = srcWidth / dstWidth;
    step_y = srcHeight / dstHeight;

    // Y scale down
    src_buf = srcBuf->virt.extP[0];
    dst_pos = 0;
    for (uint32_t y = 0; y < dstHeight; y++) {
        src_y_start_pos = y * step_y * srcWidth;

        for (uint32_t x = 0; x < dstWidth; x++) {
            src_pos = src_y_start_pos + (x * step_x);

            dstBuf[dst_pos++] = src_buf[src_pos];
        }
    }

    // UV scale down
    src_buf = srcBuf->virt.extP[1];
    for (uint32_t i = 0; i < (dstHeight / 2); i++) {
        src_y_start_pos = i * step_y * srcWidth;

        for (uint32_t j = 0; j < dstWidth; j += 2) {
            src_pos = src_y_start_pos + (j * step_x);

            dstBuf[dst_pos++] = src_buf[src_pos    ];
            dstBuf[dst_pos++] = src_buf[src_pos + 1];
        }
    }

    return true;
}

bool CameraHardwareSec::fileDump(char *filename, void *srcBuf, uint32_t size)
{
    FILE *yuv_fd = NULL;
    char *buffer = NULL;
    static int count = 0;

    yuv_fd = fopen(filename, "w+");

    if (yuv_fd == NULL) {
        LOGE("ERR file open fail: %s", filename);
        return 0;
    }

    buffer = (char *)malloc(size);

    if (buffer == NULL) {
        fclose(yuv_fd);
        LOGE("ERR malloc file");
        return 0;
    }

    memcpy(buffer, srcBuf, size);

    fflush(stdout);

    fwrite(buffer, 1, size, yuv_fd);

    fflush(yuv_fd);

    if (yuv_fd)
        fclose(yuv_fd);
    if (buffer)
        free(buffer);

    return true;
}

bool CameraHardwareSec::YUY2toNV21(void *srcBuf, void *dstBuf, uint32_t srcWidth, uint32_t srcHeight)
{
    int32_t        x, y, src_y_start_pos, dst_cbcr_pos, dst_pos, src_pos;
    unsigned char *srcBufPointer = (unsigned char *)srcBuf;
    unsigned char *dstBufPointer = (unsigned char *)dstBuf;

    dst_pos = 0;
    dst_cbcr_pos = srcWidth*srcHeight;
    for (uint32_t y = 0; y < srcHeight; y++) {
        src_y_start_pos = (y * (srcWidth * 2));

        for (uint32_t x = 0; x < (srcWidth * 2); x += 2) {
            src_pos = src_y_start_pos + x;

            dstBufPointer[dst_pos++] = srcBufPointer[src_pos];
        }
    }
    for (uint32_t y = 0; y < srcHeight; y += 2) {
        src_y_start_pos = (y * (srcWidth * 2));

        for (uint32_t x = 0; x < (srcWidth * 2); x += 4) {
            src_pos = src_y_start_pos + x;

            dstBufPointer[dst_cbcr_pos++] = srcBufPointer[src_pos + 3];
            dstBufPointer[dst_cbcr_pos++] = srcBufPointer[src_pos + 1];
        }
    }

    return true;
}

#ifdef BURST_SHOT
int CameraHardwareSec::burstpictureThread()
{
    LOGD("%s : ", __func__);

    int jpeg_size = 0;
    int ret = NO_ERROR;
    unsigned char *jpeg_data = NULL;
    int postview_offset = 0;
    unsigned char *postview_data = NULL;

    unsigned char *addr = NULL;
    int mPostViewWidth, mPostViewHeight, mPostViewSize;
    int mThumbWidth, mThumbHeight, mThumbSize;
    int cap_width, cap_height, cap_frame_size;

    int JpegImageSize = 0;
    int i=0;
    //int mBurstCapture=8;
    int burstJpeg_size[BURST_CAP_BUFFERS] = {0};
    int burststartindex=0;
    int burst_index[BURST_CAP_BUFFERS] = {0};
    int burststartflag =0;
    int hdroffset;
    SecBuffer capAddr;

#if 1
    if(mUseInternalISP){
        mBreakBurstFimcThread=false;
        mBurstPictureLock.lock();
        mBurstFirstCondition.wait(mBurstPictureLock);
        mBurstPictureLock.unlock();
        if(mBreakBurstFimcThread==true)
        {
            LOGD("mBreakBurstFimcThread return !");
            return ret;
        }


    }
#endif	
    //mBurstPictureLock.lock();

    LOG_TIME_DEFINE(0)

            mSecCamera->getPostViewConfig(&mPostViewWidth, &mPostViewHeight, &mPostViewSize);
    mSecCamera->getThumbnailConfig(&mThumbWidth, &mThumbHeight, &mThumbSize);
    int postviewHeapSize = mPostViewSize;
    if (!mRecordRunning)
        mSecCamera->getSnapshotSize(&cap_width, &cap_height, &cap_frame_size);
    else {
        mSecCamera->getRecordingSize(&cap_width, &cap_height);
        cap_frame_size = cap_width * cap_height * 3 / 2;
    }
    int mJpegHeapSize;
    if (!mUseInternalISP)
        mJpegHeapSize = cap_frame_size * SecCamera::getJpegRatio();
    else
        mJpegHeapSize = cap_frame_size;

    //mJpegHeapSize = mBurstCapture*cap_frame_size;
    mJpegHeapSize = 3*cap_frame_size;

    LOGD("[5B] mPostViewWidth = %d mPostViewHeight = %d\n",mPostViewWidth,mPostViewHeight);

    camera_memory_t *JpegHeap = mGetMemoryCb(-1, mJpegHeapSize, 1, 0);
#ifdef BOARD_USE_V4L2_ION
#ifdef ZERO_SHUTTER_LAG
    mThumbnailHeap = new MemoryHeapBaseIon(mThumbSize);
#else
    mPostviewHeap[mCapIndex] = new MemoryHeapBaseIon(mPostViewSize);
    mThumbnailHeap = new MemoryHeapBaseIon(mThumbSize);
#endif
#else
    mThumbnailHeap = new MemoryHeapBase(mThumbSize);
#endif

    if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE) {
        int picture_size, picture_width, picture_height;
        mSecCamera->getSnapshotSize(&picture_width, &picture_height, &picture_size);
        int picture_format = mSecCamera->getSnapshotPixelFormat();

        unsigned int thumb_addr, phyAddr;

        // Modified the shutter sound timing for Jpeg capture
        if (!mUseInternalISP) {
            //modify by charles.hu
            //if(!strncmp((const char*)mSecCamera->getCameraSensorName(), "S5K8AAYX", 10))
            if(1){// fix me:if want to support extISP Sensor which output JPEG data  driectly when take picture.
                LOGW("++++++++++++ take picture\n");
                if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                                   (unsigned char*)JpegHeap->data, &JpegImageSize) < 0) {
                    mStateLock.lock();
                    mCaptureInProgress = false;
                    mStateLock.unlock();
                    JpegHeap->release(JpegHeap);
                    return UNKNOWN_ERROR;
                }
                LOGI("snapshotandjpeg done");

            } else {
                mSecCamera->setSnapshotCmd();

                if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                    mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

                jpeg_data = mSecCamera->getJpeg(&JpegImageSize, &mThumbSize, &thumb_addr, &phyAddr);
                if (jpeg_data == NULL) {
                    LOGE("ERR(%s):Fail on SecCamera->getJpeg()", __func__);
                    ret = UNKNOWN_ERROR;
                }

                memcpy((unsigned char *)mThumbnailHeap->base(), (unsigned char *)thumb_addr, mThumbSize);
                memcpy(JpegHeap->data, jpeg_data, JpegImageSize);
            }
        } else {
            if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

#ifdef ZERO_SHUTTER_LAG
#if 0
            if (mSecCamera->getBurstPhysAddr(mCapIndex, &capAddr) < 0) {
                LOGE("ERR(%s):Fail on SecCamera getBurstPhysAddr = %0x ", __func__, capAddr.phys.extP[0]);
                return UNKNOWN_ERROR;
            }
#endif	   
            if (mSecCamera->getSnapshotAddr(mCapIndex, &mCapBuffer) < 0) {
                LOGE("ERR(%s):Fail on SecCamera getCaptureAddr = %0x ", __func__, mCapBuffer.virt.extP[0]);
                return UNKNOWN_ERROR;
            }

            if (!mRecordRunning) {
                scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                        (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
            } else {
                scaleDownYuv420sp(&mCapBuffer, cap_width, cap_height,
                                  (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
            }

#else
#ifdef BOARD_USE_V4L2_ION
            mCapBuffer.virt.extP[0] = (char *)mPostviewHeap[mCapIndex]->base();
#endif
#endif
            int exp_counter=-1;
            int frame_index = -1;
            unsigned char* pjepgbuffer=(unsigned char*)JpegHeap->data;
            LOGI("snapshotandjpeg start");
            LOG_TIME_START(0)
                    for(i=0;i<mBurstCapture;i++)
            {
                pjepgbuffer+=JpegImageSize;
                if (!mIsBracketEV)
                {
                    LOGD("scalado: common burst mode");
                    // frame_index = mburst_index[i];
                    frame_index = i;
                }
#ifdef BURST_HDR
                else
                {
                    LOGD("scalado: bracketing EV burst mode");
                    frame_index = i;// mhdr_index[i]
                }
#endif
                LOGD("scalado: burst pic index ==  %d",frame_index);
                if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, frame_index,
                                                   pjepgbuffer, &JpegImageSize) < 0) {
                    mStateLock.lock();
                    // mburstbufferflag = true;
                    mCaptureInProgress = false;
                    mburstencodeflag = false;
                    mStopPictureCondition.signal();
                    mStateLock.unlock();
                    JpegHeap->release(JpegHeap);
                    LOGE("ERR:getSnapshotAndJpeg",__func__);
                    return UNKNOWN_ERROR;
                }

                burstJpeg_size[i] = JpegImageSize;
                LOGD("(%s):mburst_index[hdroffset]:%d hdroffset:%d  burstJpeg_size[i]:%d", __func__,mburst_index[hdroffset],hdroffset,burstJpeg_size[i]);
            }


            LOG_TIME_END(0)
                    LOGD("(%s):time interval:%d", __func__,LOG_TIME(0));
            LOGI("snapshotandjpeg done");

#ifdef ZERO_SHUTTER_LAG
            // if (!mRecordRunning)
            // stopPreview();
            //            memset(&mCapBuffer, 0, sizeof(struct SecBuffer));
#else
            scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                    (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
#endif
        }
    }


#ifndef BOARD_USE_V4L2_ION
    int rawHeapSize = cap_frame_size;
    LOGD("mRawHeap : MemoryHeapBase(previewHeapSize(%d))", rawHeapSize);
#ifdef BOARD_USE_V4L2_ION
    mRawHeap = mGetMemoryCb(mPostviewHeap[mCapIndex]->getHeapID(), rawHeapSize, 1, 0);
#else
    mRawHeap = mGetMemoryCb((int)mSecCamera->getCameraFd(SecCamera::PICTURE), rawHeapSize, 1, 0);
#endif
    if (!mRawHeap)
        LOGE("ERR(%s): Raw heap creation fail", __func__);

    if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE)
        mDataCb(CAMERA_MSG_RAW_IMAGE, mRawHeap, 0, NULL, mCallbackCookie);
#endif
    mStateLock.lock();
    // mburstbufferflag = true;
    mCaptureInProgress = false;
    mburstencodeflag = false;
    mStopPictureCondition.signal();
    mStateLock.unlock();
    mhdr_count = 0;
    mExposureCounter = 0;
    mSecCamera->ClearBurstshotFlag();

    for(int i = 0; i <BURST_CAP_BUFFERS; i++)
        mburst_index[i] = -1;
    for(int i = 0; i <3; i++)
        mhdr_index[i] = -1;

    if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
        camera_memory_t *ExifHeap =
                mGetMemoryCb(-1, EXIF_FILE_SIZE + mThumbSize, 1, 0);

        int JpegExifSize = mSecCamera->getExif((unsigned char *)ExifHeap->data,
                                               (unsigned char *)mThumbnailHeap->base(),
                                               mThumbSize);
        LOGD("JpegExifSize=%d", JpegExifSize);

        if (JpegExifSize < 0) {
            ret = UNKNOWN_ERROR;
            goto out;
        }

        int mJpegHeapSize_out = 0;
        int offset = 0;
        unsigned char *ExifStart = NULL;
        unsigned char *ImageStart = NULL;

        camera_memory_t *JpegHeap_out[SCF_BURST_SHOT_MAX] = {NULL};
        for (int i=0;i<mBurstCapture;i++)
        {
            mJpegHeapSize_out = burstJpeg_size[i] + JpegExifSize;
            JpegHeap_out[i] = mGetMemoryCb(-1, mJpegHeapSize_out, 1, 0);

            ExifStart = (unsigned char *)JpegHeap_out[i]->data + 2;
            ImageStart = ExifStart + JpegExifSize;

            memcpy(JpegHeap_out[i]->data, JpegHeap->data + offset, 2);
            memcpy(ExifStart, ExifHeap->data, JpegExifSize);
            memcpy(ImageStart, JpegHeap->data + offset + 2, burstJpeg_size[i]  - 2);
            offset += burstJpeg_size[i];
            mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, JpegHeap_out[i], 0, NULL, mCallbackCookie);
        }

        if (ExifHeap) {
            ExifHeap->release(ExifHeap);
            ExifHeap = 0;
        }

        for(int i=0;i<mBurstCapture;i++)
        {
            if (JpegHeap_out[i])
            {
                JpegHeap_out[i]->release(JpegHeap_out[i]);
                JpegHeap_out[i] = 0;
            }
        }

    }
    mNotifyCb(CAMERA_MSG_BURST_SHOT, -1, 0, mCallbackCookie);//notify the app burst shot is done

    LOGD("%s : burstpictureThread end", __func__);

out:
    if (JpegHeap) {
        JpegHeap->release(JpegHeap);
        JpegHeap = 0;
    }

    if (mRawHeap) {
        mRawHeap->release(mRawHeap);
        mRawHeap = 0;
    }
    //modify by charles.hu
    //if(!strncmp((const char*)mSecCamera->getCameraSensorName(), "S5K8AAYX", 10))
    if(1){// fix me:if want to support extISP Sensor which output JPEG data  driectly when take picture.
        if (!mUseInternalISP && !mRecordRunning)
            mSecCamera->stopSnapshot();
    } else if (!mUseInternalISP && !mRecordRunning)
        mSecCamera->endSnapshot();

    return ret;
}
#endif

int CameraHardwareSec::pictureThread()
{
#ifdef BURST_SHOT
    if(mBurstCapture)
        return burstpictureThread();
#endif

    //add begin by suojp 2012-04-29
#ifdef DEF_WRITE_FILE_TEST
    return WriteFileTest();
#endif
#ifdef DEF_CONTINUOUS_SHOT_TEST
    return ContinuousShotTest();
#endif
#ifdef S3_CONTINOUS_SHOT
    if (msgTypeEnabled(CAMERA_MSG_CONTINOUS_SHOT))
    {
        return ContinuousShot();
    }
#endif	
    //add end by suojp 2012-04-29

    //add begin by suojp 2012-05-05
#ifdef S3_BLINK_DETECTION
    int face_number;
    bool bhaveBlink = false;
    if (!mRecordRunning && msgTypeEnabled(CAMERA_MSG_BLINK_DETECTION))
    {
        int blink_index[6] = {0};
        int arr_blink_level[16] = {0};
        int blink_level = 0;

        for(int trynum = 0; trynum < mBlinkCheckCount; trynum++)
        {
            bhaveBlink = false;
            
            for(int i = 0; i < CAP_BUFFERS; i++)
            {
                blink_index[i] = (mCapIndex+i)%CAP_BUFFERS;

                face_number = mSecCamera->getShareBufferFaceCount(blink_index[i]);
                LOGD("CAMERA_MSG_BLINK_DETECTION [%d] begin: mCapIndex=%d, face number=%d, CAP_BUFFERS=%d",
                     i, mCapIndex, face_number, CAP_BUFFERS);

                if (face_number < 1 || face_number > 16)
                {
                    LOGE("CAMERA_MSG_BLINK_DETECTION error face number=%d", face_number);
                    break;
                }

                mSecCamera->getShareBufferBlinkLevel(blink_index[i], arr_blink_level);

                for (int iface = 0; iface < face_number; iface++)
                {
                    blink_level = arr_blink_level[iface];
                    if (0 == blink_level || mBlinkLevelMax < blink_level)
                    {
                        LOGE("CAMERA_MSG_BLINK_DETECTION checked No.%d face eyes blink, "
                             "level=%d, retry %d...", i+1, blink_level, trynum+1);
                        bhaveBlink = true;
                        break;
                    }
                }

                LOGD("CAMERA_MSG_BLINK_DETECTION [%d] end: blink_index=%d, blinklevel=%d",
                     i, blink_index[i], blink_level);
                
                if (bhaveBlink)
                {
                    break;
                }
            }

            if (!bhaveBlink)
            {
                //not checked blink
                LOGD("CAMERA_MSG_BLINK_DETECTION no eyes blink, facenumber=%d, blink_level=%d,"
                     " try count=%d", face_number, blink_level, trynum);
                break;
            }

            //checked blink, retry
            LOGD("CAMERA_MSG_BLINK_DETECTION  wait begin...");
            mStateLock.lock();
            mCaptureInProgress = false;
            mStateLock.unlock();

            mFimcLock.lock();
            //mTakePictureLock.lock();
            //mCaptureInProgress = false;
            //mStopPictureCondition.signal();
            
            //mTakePictureCondition.wait(mTakePictureLock);
            //mCaptureInProgress = true;
            // mTakePictureLock.unlock();
            if (mPreviewRunning)
                mFimcActionCondition.waitRelative(mFimcLock,100000000);
            //mFimcActionCondition.wait(mFimcLock);
            mFimcLock.unlock();

            mSecCamera->setCapIndex(mCapIndex);
            
            mStateLock.lock();
            mCaptureInProgress = true;
            mStateLock.unlock();

            //face_number = mSecCamera->getShareBufferFaceCount(mCapIndex);

            LOGD("CAMERA_MSG_BLINK_DETECTION  wait end");
        }
        if (bhaveBlink)
        {
            LOGE("CAMERA_MSG_BLINK_DETECTION end, out of time ~ face_number(%d), blink_level(%d), bhaveBlink(%d)",
                 face_number, blink_level, bhaveBlink);
        }
        else
        {
            LOGD("CAMERA_MSG_BLINK_DETECTION end, face_number(%d), blink_level(%d), bhaveBlink(%d)",
                 face_number, blink_level, bhaveBlink);
        }
        //LOGE("CAMERA_MSG_BLINK_DETECTION end");
    }
#endif		
    //add end by suojp 2012-05-05

    //mLossFocusNotifyThread = new LossFocusNotifyThread(this, 4);  //limeng 0515:sample for polling af-lost


    LOGD("%s :", __func__);

    int jpeg_size = 0;
    int ret = NO_ERROR;
    unsigned char *jpeg_data = NULL;
    int postview_offset = 0;
    unsigned char *postview_data = NULL;

    unsigned char *addr = NULL;
    int mPostViewWidth, mPostViewHeight, mPostViewSize;
    int mThumbWidth, mThumbHeight, mThumbSize;
    int cap_width, cap_height, cap_frame_size;

    int JpegImageSize = 0;
    //==============
    unsigned int Src_phs_addr=0;
    char * Src_ptr=NULL;
    int Src_weight,Src_height,size,frame_size;
    //===============
    mSecCamera->getPostViewConfig(&mPostViewWidth, &mPostViewHeight, &mPostViewSize);
    mSecCamera->getThumbnailConfig(&mThumbWidth, &mThumbHeight, &mThumbSize);
    int postviewHeapSize = mPostViewSize;
    if (!mRecordRunning) {
        mSecCamera->getSnapshotSize(&cap_width, &cap_height, &cap_frame_size);
        if (mUseInternalISP) {
            if(((cap_width==3232&&cap_height==2424)||(cap_width==2592&&cap_height==1944)) && (0 >= mParameters.getInt(CameraParameters::KEY_ZOOM))){
                Src_phs_addr=(unsigned int)(mSecCamera->getShareBufferAddr(mCapIndex));
                Src_ptr=(char *)mSecCamera->getVirtualAddr(Src_phs_addr);
                mSecCamera->getPreviewSrcSize(&Src_weight,&Src_height,&size);
                frame_size=Src_weight*Src_height*2;
                LOGI("Src_phs_addr=%d,Src_ptr=%p",Src_phs_addr,Src_ptr);
                LOGI("Src_weight=%d~Src_height=%d cap_width=%d,cap_height=%d\n",Src_weight,Src_height,cap_width,cap_height);
            }else{
                LOGI("go runSnapshotFimcOneshot~");
                mSecCamera->getPreviewSrcSize(&Src_weight,&Src_height,&size);
                LOGI("Src_weight=%d~Src_height=%d cap_width=%d,cap_height=%d\n",Src_weight,Src_height,cap_width,cap_height);
                mSecCamera->runSnapshotFimcOneshot((unsigned int)(mSecCamera->getShareBufferAddr(mCapIndex)));
            }

        }
    }
    else {
        if (!mUseInternalISP) {
            mSecCamera->getRecordingSize(&cap_width, &cap_height);
            cap_frame_size = cap_width * cap_height * 3 / 2;
        } else {
            mSecCamera->getVideosnapshotSize(&cap_width, &cap_height, &cap_frame_size);
        }
    }

    //add begin by suojp 2012-05-05
#ifdef S3_BLINK_DETECTION
    if (!mRecordRunning && msgTypeEnabled(CAMERA_MSG_BLINK_DETECTION))
    {
        mNotifyCb(CAMERA_MSG_BLINK_DETECTION, 1, face_number, mCallbackCookie);
        mNotifyCb(CAMERA_MSG_BLINK_DETECTION, 0, bhaveBlink, mCallbackCookie);
    }
#endif		
    //add end by suojp 2012-05-05

    int mJpegHeapSize;
    if (!mUseInternalISP)
        mJpegHeapSize = cap_frame_size * SecCamera::getJpegRatio();
    else
        mJpegHeapSize = cap_frame_size;

    LOGD("[5B] mPostViewWidth = %d mPostViewHeight = %d\n",mPostViewWidth,mPostViewHeight);

    camera_memory_t *JpegHeap = mGetMemoryCb(-1, mJpegHeapSize, 1, 0);
#ifdef BOARD_USE_V4L2_ION
#ifdef ZERO_SHUTTER_LAG
    mThumbnailHeap = new MemoryHeapBaseIon(mThumbSize);
#else
    mPostviewHeap[mCapIndex] = new MemoryHeapBaseIon(mPostViewSize);
    mThumbnailHeap = new MemoryHeapBaseIon(mThumbSize);
#endif
#else
    if (mThumbSize > 0)
        mThumbnailHeap = new MemoryHeapBase(mThumbSize);
#endif

    // begin modify by shaking.wan for SMPCQ00015915 by samsung patch.
    //if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE)
    // end modify by shaking.wan for SMPCQ00015915 by samsung patch.
    {
        int picture_size, picture_width, picture_height;
        mSecCamera->getSnapshotSize(&picture_width, &picture_height, &picture_size);
        int picture_format = mSecCamera->getSnapshotPixelFormat();

        unsigned int thumb_addr, phyAddr;

        // Modified the shutter sound timing for Jpeg capture
        if (!mUseInternalISP) {
            //modify by charles.hu
            //if(!strncmp((const char*)mSecCamera->getCameraSensorName(), "S5K8AAYX", 10))

            if(1){// fix me:if want to support extISP Sensor which output JPEG data  driectly when take picture.
                if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                    mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);
                if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                                   (unsigned char*)JpegHeap->data, &JpegImageSize) < 0)
                {
                    mStateLock.lock();
                    mCaptureInProgress = false;
                    mStateLock.unlock();
                    JpegHeap->release(JpegHeap);
                    return UNKNOWN_ERROR;
                }
                
                if (!mRecordRunning && (mThumbSize > 0)) {
                    scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                            (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
                    snapshot_create_time_stamp((char *)mCapBuffer.virt.extP[0], cap_width,
                            cap_height, mParameters.get(KEY_PICTURE_TIME), true);
                }

                LOGI("snapshotandjpeg done");
            }
            else
            {
                mSecCamera->setSnapshotCmd();

                if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                    mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

                jpeg_data = mSecCamera->getJpeg(&JpegImageSize, &mThumbSize, &thumb_addr, &phyAddr);
                if (jpeg_data == NULL) {
                    LOGE("ERR(%s):Fail on SecCamera->getJpeg()", __func__);
                    ret = UNKNOWN_ERROR;
                }
                
                if (mThumbSize > 0)
                    memcpy((unsigned char *)mThumbnailHeap->base(), (unsigned char *)thumb_addr, mThumbSize);
                
                memcpy(JpegHeap->data, jpeg_data, JpegImageSize);
            }
        } else {
            if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

#ifdef ZERO_SHUTTER_LAG
            if (mSecCamera->getSnapshotAddr(mCapIndex, &mCapBuffer) < 0) {
                LOGE("ERR(%s):Fail on SecCamera getCaptureAddr = %0x ", __func__, mCapBuffer.virt.extP[0]);
                return UNKNOWN_ERROR;
            }
            if(!mRecordRunning&&((cap_width==3232&&cap_height==2424)||(cap_width==2592&&cap_height==1944)) && (0 >= mParameters.getInt(CameraParameters::KEY_ZOOM))){
                //scaleDownYuv422(Src_ptr,Src_weight,Src_height,(char *)mCapBuffer.virt.extP[0],cap_width,cap_height);
                memcpy(mCapBuffer.virt.extP[0],Src_ptr,frame_size);
                munmap((void *)Src_ptr,Src_weight * Src_height * 3 );
            }
            if (!mRecordRunning && (mThumbSize > 0)) {
                scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                        (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
                snapshot_create_time_stamp((char *)mCapBuffer.virt.extP[0], cap_width, cap_height, mParameters.get(KEY_PICTURE_TIME), true);
            }
            else {
#if 0
                if (mThumbSize > 0)
                    scaleDownYuv420sp(&mCapBuffer, cap_width, cap_height,
                                      (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
#else
#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
                mPreviewFimcLock.lock();
#endif
                if (mSecCamera->getVideoSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                                        (unsigned char*)JpegHeap->data, &JpegImageSize,
                                                        (unsigned char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight) < 0) {
#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
                    mPreviewFimcLock.unlock();
#endif
                    mStateLock.lock();
                    mCaptureInProgress = false;
                    mStopPictureCondition.signal();
                    mStateLock.unlock();
                    JpegHeap->release(JpegHeap);
                    return UNKNOWN_ERROR;
                }
#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
                mPreviewFimcLock.unlock();
#endif
#endif
            }
#else
#ifdef BOARD_USE_V4L2_ION
            mCapBuffer.virt.extP[0] = (char *)mPostviewHeap[mCapIndex]->base();
#endif
#endif

            if (!mRecordRunning) {
                if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                                   (unsigned char*)JpegHeap->data, &JpegImageSize) < 0) {
                    mStateLock.lock();
                    mCaptureInProgress = false;
                    mStopPictureCondition.signal();
                    mStateLock.unlock();
                    JpegHeap->release(JpegHeap);
                    return UNKNOWN_ERROR;
                }
            }
            LOGI("snapshotandjpeg done");

#ifdef ZERO_SHUTTER_LAG
            //if (!mRecordRunning) //limeng 0615:continuous viewfinder
            // stopPreview();
            //            memset(&mCapBuffer, 0, sizeof(struct SecBuffer));
#else
            if (mThumbSize > 0)
                scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                        (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
#endif
        }
    }

#ifndef BOARD_USE_V4L2_ION
    int rawHeapSize = cap_frame_size;
    LOGD("mRawHeap : MemoryHeapBase(previewHeapSize(%d))", rawHeapSize);
#ifdef BOARD_USE_V4L2_ION
    mRawHeap = mGetMemoryCb(mPostviewHeap[mCapIndex]->getHeapID(), rawHeapSize, 1, 0);
#else
    mRawHeap = mGetMemoryCb((int)mSecCamera->getCameraFd(SecCamera::PICTURE), rawHeapSize, 1, 0);
#endif
    if (!mRawHeap)
        LOGE("ERR(%s): Raw heap creation fail", __func__);

    if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE)
        mDataCb(CAMERA_MSG_RAW_IMAGE, mRawHeap, 0, NULL, mCallbackCookie);
#endif

    mStateLock.lock();
    mCaptureInProgress = false;
    mStopPictureCondition.signal();
    mStateLock.unlock();

    if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
        camera_memory_t *ExifHeap =
                mGetMemoryCb(-1, EXIF_FILE_SIZE + mThumbSize, 1, 0);

        int JpegExifSize = mSecCamera->getExif((unsigned char *)ExifHeap->data,
                                               (unsigned char *)mThumbnailHeap->base(),
                                               mThumbSize);
        LOGD("JpegExifSize=%d", JpegExifSize);

        if (JpegExifSize < 0) {
            ret = UNKNOWN_ERROR;
            JpegExifSize = mSecCamera->getMiniExif((unsigned char *)ExifHeap->data);
            //goto out;
        }

        int mJpegHeapSize_out = JpegImageSize + JpegExifSize;
        camera_memory_t *JpegHeap_out = mGetMemoryCb(-1, mJpegHeapSize_out, 1, 0);

        unsigned char *ExifStart = (unsigned char *)JpegHeap_out->data + 2;
        unsigned char *ImageStart = ExifStart + JpegExifSize;

        memcpy(JpegHeap_out->data, JpegHeap->data, 2);
        memcpy(ExifStart, ExifHeap->data, JpegExifSize);
        memcpy(ImageStart, JpegHeap->data + 2, JpegImageSize - 2);

        mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, JpegHeap_out, 0, NULL, mCallbackCookie);

        if (ExifHeap) {
            ExifHeap->release(ExifHeap);
            ExifHeap = 0;
        }

        if (JpegHeap_out) {
            JpegHeap_out->release(JpegHeap_out);
            JpegHeap_out = 0;
        }
    }

    LOGD("%s : pictureThread end", __func__);

out:
    if (JpegHeap) {
        JpegHeap->release(JpegHeap);
        JpegHeap = 0;
    }

    if (mRawHeap) {
        mRawHeap->release(mRawHeap);
        mRawHeap = 0;
    }
    //modify by charles.hu

    //if(!strncmp((const char*)mSecCamera->getCameraSensorName(), "S5K8AAYX", 10))
    if(1){// fix me:if want to support extISP Sensor which output JPEG data  driectly when take picture.
        if (!mUseInternalISP && !mRecordRunning)
            mSecCamera->stopSnapshot();
    } else if (!mUseInternalISP && !mRecordRunning)
        mSecCamera->endSnapshot();

    return ret;
}

status_t CameraHardwareSec::takePicture()
{

#ifdef S3_CONTINOUS_SHOT
    if (msgTypeEnabled(CAMERA_MSG_CONTINOUS_SHOT))
    {
        mSecCamera->EnableContinuesFlag();
    }
    else
    {
        mSecCamera->DisableContinuesFlag();
    }
#endif	

    LOGD("%s :", __func__);
    int  flash_mode = -1;
#ifdef ZERO_SHUTTER_LAG
    if (!mUseInternalISP) {
        stopPreview();
    } else {
        /*If preview is running but no frame out, wait signal to do capture.
            Jiangshabin 2012/07/24*/
        Mutex::Autolock lock(mStateLock);
        if (mPreviewRunning && (mCurrentIndex == -1 || mCapIndex == -1)) {
            mFimcLock.lock();
            mFimcActionCondition.waitRelative(mFimcLock,1000000000);
            mFimcLock.unlock();
        }
    }
#else
    stopPreview();
#endif

    Mutex::Autolock lock(mStateLock);
#ifdef BURST_SHOT
    if(mBurstCapture){
        if (mburstencodeflag) {
            LOGE("%s : burst already in progress", __func__);
            return INVALID_OPERATION;
        }
    }else
#endif
    {
        if (mCaptureInProgress) {
            LOGE("%s : capture already in progress", __func__);
            return INVALID_OPERATION;
        }
    }

    if (mUseInternalISP&&(mCameraID == SecCamera::CAMERA_ID_BACK)) {
        //Flash on when shutter
        const char *new_flash_mode_str = mParameters.get(CameraParameters::KEY_FLASH_MODE);

        if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_AUTO))
            flash_mode = FLASH_MODE_AUTO;
        else if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_ON))
            flash_mode = FLASH_MODE_ON;

        /*
           Change for new spec:
               - flash auto in protrait/partyindoor/text mode.
               - flash on in back light.
               - flash as manual settings in normal mode.
               - flash off in other scene mode.
        */
        int scene_mode = mSecCamera->getSceneMode();
        switch(scene_mode) {
        case SCENE_MODE_NONE:
            break;
        case SCENE_MODE_PORTRAIT:
        case SCENE_MODE_PARTY_INDOOR:
        case SCENE_MODE_TEXT:
            flash_mode = FLASH_MODE_AUTO;
            break;
        case SCENE_MODE_BACK_LIGHT:
            flash_mode = FLASH_MODE_ON;
            break;
        default:
            flash_mode = FLASH_MODE_OFF;
            break;
        }

        if (flash_mode == FLASH_MODE_AUTO || flash_mode == FLASH_MODE_ON) {
            mSecCamera->setFlashMode(flash_mode);
            //mmkim 0612 -- check flash led status, check  flashframe when flash led on.
            if(mSecCamera->getFlashResult()){
                int frame_skip = 0;

                //Set focus mode before flash, because auto focus will be done when flash on
                mSecCamera->setFocusMode(FOCUS_MODE_AUTO);

                //Wait flash frame
                do{
                    mFimcLock.lock();
                    mFimcActionCondition.waitRelative(mFimcLock,200000000);
                    mFimcLock.unlock();
                    frame_skip++;
                    if (mSecCamera->getFlashFrame(mCurrentIndex))
                        break;
                } while (frame_skip < 10);
                /*Cap previous frame, seems exif infor update later than frames got by FIMC.
                     So take this flags as for previous frame. Jiangshanbin 2012/06/07 */
                //mCapIndex = (mCurrentIndex + MAX_BUFFERS-1)%MAX_BUFFERS;
                mCapIndex = mCurrentIndex;
            }
        } else {
            mFimcLock.lock();
            /*fix the dead lock, the out time should be added . jijun.yu*/
            if (mPreviewRunning)
                mFimcActionCondition.waitRelative(mFimcLock,200000000);
            mFimcLock.unlock();
        }

        //##mmkim 0628-- for Sync Exif information.
        mSecCamera->setCapIndex(mCapIndex);
    }


#ifdef BURST_SHOT
    if(mBurstCapture){
        LOGD("%s: mBurstCapture:%d", __func__,mBurstCapture);
        //mBurstPictureLock.lock();
        mburstencodeflag = true;
        if (!mRecordHint) {
            mburstbufferflag = true;
            mSecCamera->setFimcForBurstshot();
            LOGD("%s: run burstshot fimc", __func__);
            if (mBurstshotFimcThread->run("CameraburstshotFimcThread", PRIORITY_URGENT_DISPLAY) != NO_ERROR) {
                LOGE("%s : couldn't run burstshot fimc thread", __func__);
                return INVALID_OPERATION;
            } else {
                mRunningThread++;
            }
        }

#if 0
        if (mBurstpictureThread->run("BurstPictureThread", PRIORITY_DEFAULT) != NO_ERROR) {
            LOGE("%s : couldn't run burst thread", __func__);
            return INVALID_OPERATION;
        }
#endif
    }else
#endif
    {
        if (mUseInternalISP&& !mRecordRunning)
            mSecCamera->setFimcForSnapshot();
    }

    mCaptureInProgress = true;
    if (mPictureThread->run("CameraPictureThread", PRIORITY_DEFAULT) != NO_ERROR) {
        LOGE("%s : couldn't run picture thread", __func__);
        return INVALID_OPERATION;
    }

    //Flash off after shutter
    if (flash_mode == FLASH_MODE_AUTO || flash_mode == FLASH_MODE_ON) {
        const char *focus_mode_str = mParameters.get(CameraParameters::KEY_FOCUS_MODE);
        int  focus_mode = -1;

        mSecCamera->setFlashMode(FLASH_MODE_OFF);

        //Reset the focus mode according  Camera Parameter
        if (!strcmp(focus_mode_str,
                    CameraParameters::FOCUS_MODE_AUTO)) {
            focus_mode = FOCUS_MODE_AUTO;
        } else if (!strcmp(focus_mode_str,
                           CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO) ||
                   !strcmp(focus_mode_str,
                           CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE)) {
            focus_mode = FOCUS_MODE_CONTINOUS;
        }

        if(focus_mode > 0)
            mSecCamera->setFocusMode(focus_mode);

    }

    return NO_ERROR;
}

status_t CameraHardwareSec::cancelPicture()
{
    LOGD("%s", __func__);

#ifdef BURST_SHOT
#if 0
    if(mBurstCapture){
        if (mBurstshotFimcThread.get()) {
            LOGD("%s: waiting for BurstshotFimc thread to exit", __func__);
            mBurstshotFimcThread->requestExitAndWait();
            LOGD("%s: BurstshotFimc thread has exited", __func__);
        }

#if 0
        if (mBurstpictureThread.get()) {
            LOGD("%s: waiting for picture thread to exit", __func__);
            mBurstpictureThread->requestExitAndWait();
            LOGD("%s: picture thread has exited", __func__);
        }
#endif
    }
#endif
#endif
    if (mPictureThread.get()) {
        LOGD("%s: waiting for picture thread to exit", __func__);
        mPictureThread->requestExitAndWait();
        LOGD("%s: picture thread has exited", __func__);
    }

    return NO_ERROR;
}

bool CameraHardwareSec::CheckVideoStartMarker(unsigned char *pBuf)
{
    if (!pBuf) {
        LOGE("CheckVideoStartMarker() => pBuf is NULL");
        return false;
    }

    if (HIBYTE(VIDEO_COMMENT_MARKER_H) == * pBuf      && LOBYTE(VIDEO_COMMENT_MARKER_H) == *(pBuf + 1) &&
            HIBYTE(VIDEO_COMMENT_MARKER_L) == *(pBuf + 2) && LOBYTE(VIDEO_COMMENT_MARKER_L) == *(pBuf + 3))
        return true;

    return false;
}

bool CameraHardwareSec::CheckEOIMarker(unsigned char *pBuf)
{
    if (!pBuf) {
        LOGE("CheckEOIMarker() => pBuf is NULL");
        return false;
    }

    // EOI marker [FF D9]
    if (HIBYTE(JPEG_EOI_MARKER) == *pBuf && LOBYTE(JPEG_EOI_MARKER) == *(pBuf + 1))
        return true;

    return false;
}

bool CameraHardwareSec::FindEOIMarkerInJPEG(unsigned char *pBuf, int dwBufSize, int *pnJPEGsize)
{
    if (NULL == pBuf || 0 >= dwBufSize) {
        LOGE("FindEOIMarkerInJPEG() => There is no contents.");
        return false;
    }

    unsigned char *pBufEnd = pBuf + dwBufSize;

    while (pBuf < pBufEnd) {
        if (CheckEOIMarker(pBuf++))
            return true;

        (*pnJPEGsize)++;
    }

    return false;
}

bool CameraHardwareSec::SplitFrame(unsigned char *pFrame, int dwSize,
                                   int dwJPEGLineLength, int dwVideoLineLength, int dwVideoHeight,
                                   void *pJPEG, int *pdwJPEGSize,
                                   void *pVideo, int *pdwVideoSize)
{
    LOGD("===========SplitFrame Start==============");

    if (NULL == pFrame || 0 >= dwSize) {
        LOGE("There is no contents (pFrame=%p, dwSize=%d", pFrame, dwSize);
        return false;
    }

    if (0 == dwJPEGLineLength || 0 == dwVideoLineLength) {
        LOGE("There in no input information for decoding interleaved jpeg");
        return false;
    }

    unsigned char *pSrc = pFrame;
    unsigned char *pSrcEnd = pFrame + dwSize;

    unsigned char *pJ = (unsigned char *)pJPEG;
    int dwJSize = 0;
    unsigned char *pV = (unsigned char *)pVideo;
    int dwVSize = 0;

    bool bRet = false;
    bool isFinishJpeg = false;

    while (pSrc < pSrcEnd) {
        // Check video start marker
        if (CheckVideoStartMarker(pSrc)) {
            int copyLength;

            if (pSrc + dwVideoLineLength <= pSrcEnd)
                copyLength = dwVideoLineLength;
            else
                copyLength = pSrcEnd - pSrc - VIDEO_COMMENT_MARKER_LENGTH;

            // Copy video data
            if (pV) {
                memcpy(pV, pSrc + VIDEO_COMMENT_MARKER_LENGTH, copyLength);
                pV += copyLength;
                dwVSize += copyLength;
            }

            pSrc += copyLength + VIDEO_COMMENT_MARKER_LENGTH;
        } else {
            // Copy pure JPEG data
            int size = 0;
            int dwCopyBufLen = dwJPEGLineLength <= pSrcEnd-pSrc ? dwJPEGLineLength : pSrcEnd - pSrc;

            if (FindEOIMarkerInJPEG((unsigned char *)pSrc, dwCopyBufLen, &size)) {
                isFinishJpeg = true;
                size += 2;  // to count EOF marker size
            } else {
                if ((dwCopyBufLen == 1) && (pJPEG < pJ)) {
                    unsigned char checkBuf[2] = { *(pJ - 1), *pSrc };

                    if (CheckEOIMarker(checkBuf))
                        isFinishJpeg = true;
                }
                size = dwCopyBufLen;
            }

            memcpy(pJ, pSrc, size);

            dwJSize += size;

            pJ += dwCopyBufLen;
            pSrc += dwCopyBufLen;
        }
        if (isFinishJpeg)
            break;
    }

    if (isFinishJpeg) {
        bRet = true;
        if (pdwJPEGSize)
            *pdwJPEGSize = dwJSize;
        if (pdwVideoSize)
            *pdwVideoSize = dwVSize;
    } else {
        LOGE("DecodeInterleaveJPEG_WithOutDT() => Can not find EOI");
        bRet = false;
        if (pdwJPEGSize)
            *pdwJPEGSize = 0;
        if (pdwVideoSize)
            *pdwVideoSize = 0;
    }
    LOGD("===========SplitFrame end==============");

    return bRet;
}

int CameraHardwareSec::decodeInterleaveData(unsigned char *pInterleaveData,
                                            int interleaveDataSize,
                                            int yuvWidth,
                                            int yuvHeight,
                                            int *pJpegSize,
                                            void *pJpegData,
                                            void *pYuvData)
{
    if (pInterleaveData == NULL)
        return false;

    bool ret = true;
    unsigned int *interleave_ptr = (unsigned int *)pInterleaveData;
    unsigned char *jpeg_ptr = (unsigned char *)pJpegData;
    unsigned char *yuv_ptr = (unsigned char *)pYuvData;
    unsigned char *p;
    int jpeg_size = 0;
    int yuv_size = 0;

    int i = 0;

    LOGD("decodeInterleaveData Start~~~");
    while (i < interleaveDataSize) {
        if ((*interleave_ptr == 0xFFFFFFFF) || (*interleave_ptr == 0x02FFFFFF) ||
                (*interleave_ptr == 0xFF02FFFF)) {
            // Padding Data
            interleave_ptr++;
            i += 4;
        } else if ((*interleave_ptr & 0xFFFF) == 0x05FF) {
            // Start-code of YUV Data
            p = (unsigned char *)interleave_ptr;
            p += 2;
            i += 2;

            // Extract YUV Data
            if (pYuvData != NULL) {
                memcpy(yuv_ptr, p, yuvWidth * 2);
                yuv_ptr += yuvWidth * 2;
                yuv_size += yuvWidth * 2;
            }
            p += yuvWidth * 2;
            i += yuvWidth * 2;

            // Check End-code of YUV Data
            if ((*p == 0xFF) && (*(p + 1) == 0x06)) {
                interleave_ptr = (unsigned int *)(p + 2);
                i += 2;
            } else {
                ret = false;
                break;
            }
        } else {
            // Extract JPEG Data
            if (pJpegData != NULL) {
                memcpy(jpeg_ptr, interleave_ptr, 4);
                jpeg_ptr += 4;
                jpeg_size += 4;
            }
            interleave_ptr++;
            i += 4;
        }
    }
    if (ret) {
        if (pJpegData != NULL) {
            // Remove Padding after EOI
            for (i = 0; i < 3; i++) {
                if (*(--jpeg_ptr) != 0xFF) {
                    break;
                }
                jpeg_size--;
            }
            *pJpegSize = jpeg_size;

        }
        // Check YUV Data Size
        if (pYuvData != NULL) {
            if (yuv_size != (yuvWidth * yuvHeight * 2)) {
                ret = false;
            }
        }
    }
    LOGD("decodeInterleaveData End~~~");

    return ret;
}

status_t CameraHardwareSec::dump(int fd) const
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    const Vector<String16> args;

    if (mSecCamera != 0) {
        mSecCamera->dump(fd);
        mParameters.dump(fd, args);
        mInternalParameters.dump(fd, args);
        memset(buffer,0,255);
        snprintf(buffer, 255, "SecCameraHwInterFace Variables:\n\tpreview running(%s)\n", mPreviewRunning?"true": "false");
        result.append(buffer);
        //TODO dump all the varialbe here for debug.
        memset(buffer,0,255);
        snprintf(buffer, 255, "\tmRunningThread(%d)\n"
                              "\tmCameraID(%d)\n"
                              "\tmCameraSensorName(%s)\n"
                              "\tmMsgEnabled(0x%x)\n"
                              "\tmUseInternalISP(%s)\n",
                 mRunningThread,
                 mCameraID,
                 mCameraSensorName?(char*)mCameraSensorName:"NULL",
                 mMsgEnabled,
                 mUseInternalISP?"true": "false");
        result.append(buffer);
        memset(buffer,0,255);
        snprintf(buffer, 255, "\n\tmRunningSetParam(%d)\n"
                              "\tmFDcount(%d)\n"
                              "\tmExitAutoFocusThread(%s)\n"
                              "\tmTouched(%d)\n"
                              "\tmFrameSizeDelta(%d)\n",
                 mRunningSetParam,
                 mFDcount,
                 mExitAutoFocusThread?"true": "false",
                 mTouched,
                 mFrameSizeDelta);
        result.append(buffer);
        memset(buffer,0,255);
        snprintf(buffer, 255, "\n\tmPreviewRunning(%s)\n"
                              "\tmExitPreviewThread(%s)\n"
                              "\tmPreviewStartDeferred(%s)\n"
                              "\tmCurrentIndex(%d)\n"
                              "\tmOldPreviewIndex(%d)\n"
                              "\tmPreviewFmtPlane(%d)\n"
                              "\tmSkipFrame(%d)\n",
                 mPreviewRunning?"true": "false",
                 mExitPreviewThread?"true": "false",
                 mPreviewStartDeferred?"true": "false",
                 mCurrentIndex,
                 mOldPreviewIndex,
                 mPreviewFmtPlane,
                 mSkipFrame);
        result.append(buffer);
        memset(buffer,0,255);
        snprintf(buffer, 255, "\n\tmCaptureInProgress(%s)\n"
                              "\tmCapIndex(%d)\n"
                              "\tmRecordRunning(%s)\n"
                              "\tmRecordHint(%s)\n"
                              "\tmOldRecordIndex(%d)\n"
                              "\tmPostViewWidth(%d)\n"
                              "\tmPostViewHeight(%d)\n"
                              "\tmPostViewSize(%d)\n",
                 mCaptureInProgress?"true": "false",
                 mCapIndex,
                 mRecordRunning?"true": "false",
                 mRecordHint?"true": "false",
                 mOldRecordIndex,
                 mPostViewWidth,
                 mPostViewHeight,
                 mPostViewSize);
        result.append(buffer);
        //TODO to dump burst related
        //	    bool        mburstencodeflag;
        //	    bool        mburstbufferflag;
        //	    bool        mbursthdrflag;
        //	    int         mburst_index[CAP_BUFFERS];
        //	    int         mBurststartIndex;
        //	    int         mhdr_index[3];
        //	    int         mOldBurstIndex;
        //	    int         mhdr_count;

    } else
        result.append("No camera client yet.\n");
    write(fd, result.string(), result.size());
    return NO_ERROR;
}

bool CameraHardwareSec::isSupportedPreviewSize(const int width,
                                               const int height) const
{
    unsigned int i;

    for (i = 0; i < mSupportedPreviewSizes.size(); i++) {
        if (mSupportedPreviewSizes[i].width == width &&
                mSupportedPreviewSizes[i].height == height)
            return true;
    }

    return false;
}
bool CameraHardwareSec::getPreviewSize(int pic_width, int pic_height, int *preview_width, int *preview_height)
{
    unsigned int i;
    Vector<Size>  previewSizes;

    int ratio = FRM_RATIO(pic_width, pic_height);
    mParameters.getSupportedPreviewSizes(previewSizes);

    if (ratio == (FRM_RATIO(*preview_width, *preview_height))) {
        for (i = 0; i < previewSizes.size(); i++) {
            if ((previewSizes[i].width==*preview_width) && (previewSizes[i].height==*preview_height)) {
                return true;
            }
        }
    }

    for (i = 0; i < previewSizes.size(); i++) {
        if (FRM_RATIO(previewSizes[i].width, previewSizes[i].height) == ratio) {
            *preview_width = previewSizes[i].width;
            *preview_height = previewSizes[i].height;
            return true;
        }
    }

    return false;
}

bool CameraHardwareSec::getVideosnapshotSize(int *width, int *height)
{
    bool ret = true;
    if ((*width > 1920) && (*height > 1080)) {
        //use original resolution
        *width = *width;
        *height = *height;
    } else if ((*width == 1920) && (*height == 1080)) {
        *width = 3072;
        *height = 1728;
        if (mRecordHint) {
            *width = 2560;
            *height = 1440;
        }
    } else if ((*width == 1280) && (*height == 720)) {
        *width = 3200;
        *height = 1800;
        if (mRecordHint) {
            *width = 2560;
            *height = 1440;
        }
    }
    /* else if ((*width == 800) && (*height == 480)) {
        *width = 2800;
        *height = 1680;
        if (mRecordHint) {
            *width = 2400;
            *height = 1440;
        }
    } */
    else if ((*width == 800) && (*height == 600)) {
        *width = 2240;
        *height = 1680;
        if (mRecordHint) {
            *width = 1920;
            *height = 1440;
        }
    } else if ((*width == 640) && (*height == 480)) {
        *width = 2240;
        *height = 1680;
        if (mRecordHint) {
            *width = 1920;
            *height = 1440;
        }
    } else if ((*width == 176) && (*height == 144)) {
        *width = 1760;
        *height = 1440;
    } else {
        ret = false;
    }

    return ret;
}

#if 1
/*for cts : the number of areas setting can support upto 64 , and if the number more than 64 ,it should be 
  notify a invaild message  jijun.yu */
int CameraHardwareSec::bracketsStr2Ints(char *str, int num, ExynosRect *rects, int *weights)
{
    char *curStr = str;
    char buf[128];
    char *bracketsOpen;
    char *bracketsClose;

    int tempArray[5];
    int validFocusedAreas = 0;
    while (strlen(curStr) > 4) {
        if (curStr == NULL)
            break;
        bracketsOpen = strchr(curStr, '(');
        if (bracketsOpen == NULL)
            break;
        bracketsClose = strchr(bracketsOpen, ')');
        if (bracketsClose == NULL)
            break;
        strncpy(buf, bracketsOpen, bracketsClose - bracketsOpen + 1);
        buf[bracketsClose - bracketsOpen + 1] = 0;

        if (subBracketsStr2Ints(5, buf, tempArray) == false) {
            LOGE("ERR(%s):subBracketsStr2Ints(%s) fail", __func__, buf);
            break;
        }
        validFocusedAreas++;

        if (validFocusedAreas > num) {
            return -101;
        }
#ifdef BOARD_USE_S5K4H5
        if(1==mCameraID){
#else
        if(0==mCameraID){
#endif		
            rects[validFocusedAreas-1].left   = tempArray[0];
            rects[validFocusedAreas-1].top    = tempArray[1];
            rects[validFocusedAreas-1].right  = tempArray[2];
            rects[validFocusedAreas-1].bottom = tempArray[3];
        }else{
#if 1

            //lzy the sensor is turn over 180 in HW and now turn the image over  through se
            //but the related	coordinate like as TF/ME/...should be recalculated.
            rects[validFocusedAreas-1].left   = -tempArray[2];
            rects[validFocusedAreas-1].top	  = -tempArray[3];
            rects[validFocusedAreas-1].right  = -tempArray[0];
            rects[validFocusedAreas-1].bottom = -tempArray[1];

#endif
        }
        weights[validFocusedAreas-1] = tempArray[4];
        curStr = bracketsClose+1;


    }

    return validFocusedAreas;

    for (int i = 0; i < num; i++) {
        if (curStr == NULL)
            break;

        bracketsOpen = strchr(curStr, '(');
        if (bracketsOpen == NULL)
            break;

        bracketsClose = strchr(bracketsOpen, ')');
        if (bracketsClose == NULL)
            break;

        strncpy(buf, bracketsOpen, bracketsClose - bracketsOpen + 1);
        buf[bracketsClose - bracketsOpen + 1] = 0;

        if (subBracketsStr2Ints(5, buf, tempArray) == false) {
            LOGE("ERR(%s):subBracketsStr2Ints(%s) fail", __func__, buf);
            break;
        }
#ifdef BOARD_USE_S5K4H5		
        if(1==mCameraID){
#else		
        if(0==mCameraID){
#endif		
            rects[i].left	= tempArray[0];
            rects[i].top	= tempArray[1];
            rects[i].right	= tempArray[2];
            rects[i].bottom = tempArray[3];
        }else{
#if 1

            //lzy the sensor is turn over 180 in HW and now turn the image over  through se
            //but the related	coordinate like as TF/ME/...should be recalculated.
            rects[i].left	= -tempArray[2];
            rects[i].top	= -tempArray[3];
            rects[i].right	= -tempArray[0];
            rects[i].bottom = -tempArray[1];

#endif
        }
        weights[i] = tempArray[4];

        validFocusedAreas++;

        curStr = bracketsClose;
    }
    return validFocusedAreas;
}
#else
int CameraHardwareSec::bracketsStr2Ints(char *str, int num, ExynosRect *rects, int *weights)
{
    char *curStr = str;
    char buf[128];
    char *bracketsOpen;
    char *bracketsClose;

    int tempArray[5];
    int validFocusedAreas = 0;

    for (int i = 0; i < num; i++) {
        if (curStr == NULL)
            break;

        bracketsOpen = strchr(curStr, '(');
        if (bracketsOpen == NULL)
            break;

        bracketsClose = strchr(bracketsOpen, ')');
        if (bracketsClose == NULL)
            break;

        strncpy(buf, bracketsOpen, bracketsClose - bracketsOpen + 1);
        buf[bracketsClose - bracketsOpen + 1] = 0;

        if (subBracketsStr2Ints(5, buf, tempArray) == false) {
            LOGE("ERR(%s):subBracketsStr2Ints(%s) fail", __func__, buf);
            break;
        }

        rects[i].left = tempArray[0];
        rects[i].top = tempArray[1];
        rects[i].right = tempArray[2];
        rects[i].bottom = tempArray[3];
        weights[i] = tempArray[4];

        validFocusedAreas++;

        curStr = bracketsClose;
    }
    return validFocusedAreas;
}
#endif
bool CameraHardwareSec::subBracketsStr2Ints(int num, char *str, int *arr)
{
    if (str == NULL || arr == NULL) {
        LOGE("ERR(%s):str or arr is NULL", __func__);
        return false;
    }

    // ex : (-10,-10,0,0,300)
    char buf[128];
    char *bracketsOpen;
    char *bracketsClose;
    char *tok;

    bracketsOpen = strchr(str, '(');
    if (bracketsOpen == NULL) {
        LOGE("ERR(%s):no '('", __func__);
        return false;
    }

    bracketsClose = strchr(bracketsOpen, ')');
    if (bracketsClose == NULL) {
        LOGE("ERR(%s):no ')'", __func__);
        return false;
    }

    strncpy(buf, bracketsOpen + 1, bracketsClose - bracketsOpen + 1);
    buf[bracketsClose - bracketsOpen + 1] = 0;

    tok = strtok(buf, ",");
    if (tok == NULL) {
        LOGE("ERR(%s):strtok(%s) fail", __func__, buf);
        return false;
    }

    arr[0] = atoi(tok);

    for (int i = 1; i < num; i++) {
        tok = strtok(NULL, ",");
        if (tok == NULL) {
            if (i < num - 1) {
                LOGE("ERR(%s):strtok() (index : %d, num : %d) fail", __func__, i, num);
                return false;
            }
            break;
        }

        arr[i] = atoi(tok);
    }

    return true;
}

bool CameraHardwareSec::meteringAxisTrans(ExynosRect *rects, ExynosRect2 *rect2s, int num)
{
    int width, height, frame_size;
    mSecCamera->getVideosnapshotSize(&width, &height, &frame_size);

    int zoom_level = mSecCamera->getZoom();
    int step = 51;
    float zoom[step];
    float inc = 0.1;
    zoom[0] = 1.0;
    for (int n = 0; n < (step - 1); n++) {
        zoom[n+1] = zoom[n] + inc;
    }

    int crop_width    = (int)((float)width / zoom[zoom_level]);
    crop_width = ALIGN(crop_width, 2);
    int crop_height   = (int)((float)height / zoom[zoom_level]);
    crop_height = ALIGN(crop_height, 2);

    int offset_width  = (width - crop_width) / 2;
    int offset_height = (height - crop_height) / 2;

    rect2s[num].x = offset_width + ((rects[num].left + 1000) * crop_width / 2000);
    rect2s[num].y = offset_height + ((rects[num].top  + 1000) * crop_height / 2000);
    rect2s[num].w = (rects[num].right  - rects[num].left) * crop_width / 2000;
    rect2s[num].h = (rects[num].bottom - rects[num].top) * crop_height / 2000;

    return true;
}


status_t CameraHardwareSec::setParameters(const CameraParameters& params)
{
    LOGD("%s :", __func__);

    status_t ret = NO_ERROR;

    mRunningSetParam = 1;
    const char *new_record_hint_str = params.get(CameraParameters::KEY_RECORDING_HINT);
    const char *curr_record_hint_str = mParameters.get(CameraParameters::KEY_RECORDING_HINT);
    LOGD("new_record_hint_str: %s", new_record_hint_str);

    if (new_record_hint_str) {
        if (strncmp(new_record_hint_str, curr_record_hint_str, 5)) {
            mRecordHint = !strncmp(new_record_hint_str, "true", 4);
            if (mSecCamera->setMode(mRecordHint) < 0) {
                LOGE("ERR(%s):fail on mSecCamera->setMode(%d)", __func__, mRecordHint);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_RECORDING_HINT, new_record_hint_str);
            }

            if (mUseInternalISP) {
                if (mSecCamera->initSetParams() < 0) {
                    LOGE("ERR(%s):fail on mSecCamera->initSetParams()", __func__);
                    ret = UNKNOWN_ERROR;
                }
            }
        }
    }

    /* if someone calls us while picture thread is running, it could screw
     * up the sensor quite a bit so return error.  we can't wait because
     * that would cause deadlock with the callbacks
     */
    mStateLock.lock();
#ifdef BURST_SHOT //limeng 0711
    if(mBurstCapture){
        if(mburstencodeflag){
            mStateLock.unlock();
            LOGE("%s : burst in progress, doing nothing", __func__);
            return UNKNOWN_ERROR;
        }
    }else
#endif
    {
        if (mCaptureInProgress) {
            mStateLock.unlock();
            LOGE("%s : capture in progress, not allowed", __func__);
            return UNKNOWN_ERROR;
        }
    }
    mStateLock.unlock();

    // picture size
    int new_picture_width  = 0;
    int new_picture_height = 0;

    params.getPictureSize(&new_picture_width, &new_picture_height);
    LOGD("%s : new_picture_width x new_picture_height = %dx%d", __func__, new_picture_width, new_picture_height);


    //dg add for debug

  //  new_picture_width=1600;
 //   new_picture_height=1200;

    LOGD("%s : change new_picture_width x new_picture_height = %dx%d", __func__, new_picture_width, new_picture_height);




    int videosnapshot_width = new_picture_width;
    int videosnapshot_height = new_picture_height;






    if (mUseInternalISP) {
        //when start preview for recording, the recording size is set in the picture size, only
        //before starting recording, the recorde size (video size) will be set into HAL.

        if (mRecordHint) {
            if ((videosnapshot_width > 1920) || (videosnapshot_height > 1080)) {
                LOGD ("Warning: record picture size is too large %d x %d", videosnapshot_width, videosnapshot_height);
                unsigned int i;
                Vector<Size>  videoSizes;

                int ratio = FRM_RATIO(videosnapshot_width, videosnapshot_height);
                mParameters.getSupportedVideoSizes(videoSizes);

                for (i = 0; i < videoSizes.size(); i++) {
                    if (FRM_RATIO(videoSizes[i].width, videoSizes[i].height) == ratio) {
                        videosnapshot_width = videoSizes[i].width;
                        videosnapshot_height = videoSizes[i].height;
                        break;
                    }
                }
                if ((videosnapshot_width > 1920) || (videosnapshot_height > 1080)) {
                    videosnapshot_width = 1280;
                    videosnapshot_height = 720;
                }
            }
        }

        if (!getVideosnapshotSize(&videosnapshot_width, &videosnapshot_height)) {
            LOGE("ERR(%s):fail on getVideosnapshotSize(width(%d), height(%d))",
                 __func__, videosnapshot_width, videosnapshot_height);
            ret = UNKNOWN_ERROR;
        }

        if (mSecCamera->setVideosnapshotSize(videosnapshot_width, videosnapshot_height) < 0) {
            LOGE("ERR(%s):fail on mSecCamera->setVideosnapshotSize(width(%d), height(%d))",
                 __func__, videosnapshot_width, videosnapshot_height);
            ret = UNKNOWN_ERROR;
        }
    }else{
        /*for Ov2659 camera  a temp for CTS , Pls check the exactly video snapshot size again
                 when the function added for Ov2659 jijun.yu*/
        if (mSecCamera->setVideosnapshotSize(videosnapshot_width, videosnapshot_height) < 0) {
            LOGE("ERR(%s):fail on mSecCamera->setVideosnapshotSize(width(%d), height(%d))",
                 __func__, videosnapshot_width, videosnapshot_height);
            ret = UNKNOWN_ERROR;
        }
    }

    // preview size
    int new_preview_width  = new_picture_width;
    int new_preview_height = new_picture_height;
    int new_preview_format = 0;

    params.getPreviewSize(&new_preview_width, &new_preview_height);
    if ((new_preview_width < 0) || (new_preview_height < 0) ) {
        return UNKNOWN_ERROR;
    }
    /*
    if (mUseInternalISP) {
        if (!getPreviewSize(new_picture_width, new_picture_height, &new_preview_width, &new_preview_height)) {
            LOGE("ERR(%s):fail on getPreviewSize", __func__);
            ret = UNKNOWN_ERROR;
        }
    } else {
        //for ov2659
        ;
    }
        */
    const char *new_str_preview_format = params.getPreviewFormat();
    LOGD("new_preview_width x new_preview_height = %dx%d, format = %s",
         new_preview_width, new_preview_height, new_str_preview_format);

    if (0 < new_preview_width && 0 < new_preview_height &&
            new_str_preview_format != NULL &&
            isSupportedPreviewSize(new_preview_width, new_preview_height)) {

        //      mFrameSizeDelta = 16;  //why???
        mFrameSizeDelta = 0;

        if (!strcmp(new_str_preview_format,
                    CameraParameters::PIXEL_FORMAT_RGB565)) {
            new_preview_format = V4L2_PIX_FMT_RGB565;
            mFrameSizeDelta = 0;
        }
        else if (!strcmp(new_str_preview_format,
                         CameraParameters::PIXEL_FORMAT_RGBA8888)) {
            new_preview_format = V4L2_PIX_FMT_RGB32;
            mFrameSizeDelta = 0;
        }
        else if (!strcmp(new_str_preview_format,
                         CameraParameters::PIXEL_FORMAT_YUV420SP)) {
            new_preview_format = V4L2_PIX_FMT_NV21;
            mPreviewFmtPlane = PREVIEW_FMT_2_PLANE;
        }
        else if (!strcmp(new_str_preview_format,
                         CameraParameters::PIXEL_FORMAT_YUV420P)) {
#ifdef BOARD_USE_V4L2_ION
            new_preview_format = V4L2_PIX_FMT_YVU420M;
#else
            new_preview_format = V4L2_PIX_FMT_YVU420;
#endif
            mPreviewFmtPlane = PREVIEW_FMT_3_PLANE;
        }
        else if (!strcmp(new_str_preview_format, "yuv420sp_custom"))
            new_preview_format = V4L2_PIX_FMT_NV12T;
        else if (!strcmp(new_str_preview_format, "yuv422i"))
            new_preview_format = V4L2_PIX_FMT_YUYV;
        else if (!strcmp(new_str_preview_format, "yuv422p"))
            new_preview_format = V4L2_PIX_FMT_YUV422P;
        else
            new_preview_format = V4L2_PIX_FMT_NV21; //for 3rd party

        int current_preview_width, current_preview_height, current_frame_size;
        mSecCamera->getPreviewSize(&current_preview_width,
                                   &current_preview_height,
                                   &current_frame_size);
        int current_pixel_format = mSecCamera->getPreviewPixelFormat();

        if (current_preview_width != new_preview_width ||
                current_preview_height != new_preview_height ||
                current_pixel_format != new_preview_format) {
            if (mSecCamera->setPreviewSize(new_preview_width, new_preview_height,
                                           new_preview_format) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setPreviewSize(width(%d), height(%d), format(%d))",
                     __func__, new_preview_width, new_preview_height, new_preview_format);
                ret = UNKNOWN_ERROR;
            } else {
                if (mPreviewWindow) {
                    if (mPreviewRunning && !mPreviewStartDeferred) {
                        LOGE("ERR(%s): preview is running, cannot change size and format!", __func__);
                        ret = INVALID_OPERATION;
                    }
                    LOGD("%s: mPreviewWindow (%p) set_buffers_geometry", __func__, mPreviewWindow);
                    LOGD("%s: mPreviewWindow->set_buffers_geometry (%p)", __func__,
                         mPreviewWindow->set_buffers_geometry);
                    mPreviewWindow->set_buffers_geometry(mPreviewWindow,
                                                         new_preview_width, new_preview_height,
                                                         V4L2_PIX_2_HAL_PIXEL_FORMAT(new_preview_format));
                    LOGD("%s: DONE mPreviewWindow (%p) set_buffers_geometry", __func__, mPreviewWindow);
                }
                mParameters.setPreviewSize(new_preview_width, new_preview_height);
                mParameters.setPreviewFormat(new_str_preview_format);
                //add begin by suojp 2012-05-21
#ifdef S3_CHECK_PREVIEW_LIGHT 				
                mPreviewFormat = new_preview_format;
#endif
                //add end by suojp 2012-05-21
            }
        }
    } else {
        LOGE("%s: Invalid preview size(%dx%d)",
             __func__, new_preview_width, new_preview_height);

        ret = INVALID_OPERATION;
    }

    int current_picture_width, current_picture_height, current_picture_size;
    mSecCamera->getSnapshotSize(&current_picture_width, &current_picture_height, &current_picture_size);

    if (new_picture_width != current_picture_width ||
            new_picture_height != current_picture_height) {
        if (mSecCamera->setSnapshotSize(new_picture_width, new_picture_height) < 0) {
            LOGE("ERR(%s):fail on mSecCamera->setSnapshotSize(width(%d), height(%d))",
                 __func__, new_picture_width, new_picture_height);
            ret = UNKNOWN_ERROR;
        } else {
            if (mUseInternalISP && !mRecordHint && mPreviewRunning) {
                stopPreview();

                if (mSecCamera->setFrameRate( mParameters.getPreviewFrameRate()) < 0){
                    LOGE("ERR(%s):Fail on mSecCamera->setFrameRate(%d)", __func__, mParameters.getPreviewFrameRate());
                    ret = UNKNOWN_ERROR;
                }

                startPreview();
            }

            mParameters.setPictureSize(new_picture_width, new_picture_height);
        }
    }

    // picture format
    const char *new_str_picture_format = params.getPictureFormat();
    LOGD("%s : new_str_picture_format %s", __func__, new_str_picture_format);
    if (new_str_picture_format != NULL) {
        int new_picture_format = 0;

        if (!strcmp(new_str_picture_format, CameraParameters::PIXEL_FORMAT_RGB565))
            new_picture_format = V4L2_PIX_FMT_RGB565;
        else if (!strcmp(new_str_picture_format, CameraParameters::PIXEL_FORMAT_RGBA8888))
            new_picture_format = V4L2_PIX_FMT_RGB32;
        else if (!strcmp(new_str_picture_format, CameraParameters::PIXEL_FORMAT_YUV420SP))
            new_picture_format = V4L2_PIX_FMT_NV21;
        else if (!strcmp(new_str_picture_format, "yuv420sp_custom"))
            new_picture_format = V4L2_PIX_FMT_NV12T;
        else if (!strcmp(new_str_picture_format, "yuv420p"))
            new_picture_format = V4L2_PIX_FMT_YUV420;
        else if (!strcmp(new_str_picture_format, "yuv422i"))
            new_picture_format = V4L2_PIX_FMT_YUYV;
        else if (!strcmp(new_str_picture_format, "uyv422i_custom")) //Zero copy UYVY format
            new_picture_format = V4L2_PIX_FMT_UYVY;
        else if (!strcmp(new_str_picture_format, "uyv422i")) //Non-zero copy UYVY format
            new_picture_format = V4L2_PIX_FMT_UYVY;
        else if (!strcmp(new_str_picture_format, CameraParameters::PIXEL_FORMAT_JPEG))
            new_picture_format = V4L2_PIX_FMT_YUYV;
        else if (!strcmp(new_str_picture_format, "yuv422p"))
            new_picture_format = V4L2_PIX_FMT_YUV422P;
        else
            new_picture_format = V4L2_PIX_FMT_NV21; //for 3rd party
        /*now we only support yuv422*/
        if (new_picture_format != V4L2_PIX_FMT_YUYV) {
            LOGE("only yuv422i picture format support new_str_picture_format(%s) \n ",new_str_picture_format);
            return UNKNOWN_ERROR;
        }

        /*Caution: ignore snapshot format setting during preview running.
           It should be blocked in app. Jiangshanbin 2012/06/28 */
        if (new_picture_format != mSecCamera->getSnapshotPixelFormat()) {
            if (mUseInternalISP && mPreviewRunning) {
                LOGE("ERR(%s):Ignore invalid setSnapshotPixelFormat(format(%d)) when preview is running", __func__, new_picture_format);
            } else if (mSecCamera->setSnapshotPixelFormat(new_picture_format) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setSnapshotPixelFormat(format(%d))", __func__, new_picture_format);
                ret = UNKNOWN_ERROR;
            } else
                mParameters.setPictureFormat(new_str_picture_format);
        }
    }

    // JPEG image quality
    int new_jpeg_quality = params.getInt(CameraParameters::KEY_JPEG_QUALITY);
    LOGD("%s : new_jpeg_quality %d", __func__, new_jpeg_quality);
    /* we ignore bad values */
    if (new_jpeg_quality >=1 && new_jpeg_quality <= 100) {
        if (mSecCamera->setJpegQuality(new_jpeg_quality) < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->setJpegQuality(quality(%d))", __func__, new_jpeg_quality);
            ret = UNKNOWN_ERROR;
        } else
            mParameters.set(CameraParameters::KEY_JPEG_QUALITY, new_jpeg_quality);
    }

    // JPEG thumbnail size
    int new_jpeg_thumbnail_width = params.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
    int new_jpeg_thumbnail_height= params.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);
    if (0 <= new_jpeg_thumbnail_width && 0 <= new_jpeg_thumbnail_height) {
        if (mSecCamera->setJpegThumbnailSize(new_jpeg_thumbnail_width, new_jpeg_thumbnail_height) < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->setJpegThumbnailSize(width(%d), height(%d))", __func__, new_jpeg_thumbnail_width, new_jpeg_thumbnail_height);
            ret = UNKNOWN_ERROR;
        } else {
            mParameters.set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, new_jpeg_thumbnail_width);
            mParameters.set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, new_jpeg_thumbnail_height);
        }
    }

    // JPEG thumbnail quality
    int new_jpeg_thumbnail_quality = params.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY);
    LOGD("%s : new_jpeg_thumbnail_quality %d", __func__, new_jpeg_thumbnail_quality);
    /* we ignore bad values */
    if (new_jpeg_thumbnail_quality >=1 && new_jpeg_thumbnail_quality <= 100) {
        if (mSecCamera->setJpegThumbnailQuality(new_jpeg_thumbnail_quality) < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->setJpegThumbnailQuality(quality(%d))",
                 __func__, new_jpeg_thumbnail_quality);
            ret = UNKNOWN_ERROR;
        } else
            mParameters.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, new_jpeg_thumbnail_quality);
    }

    // picture timestamp
    {
        const char *str = params.get(KEY_PICTURE_TIME);
        LOGD("(%s) : (KEY_PICTURE_TIME=%s)", __FUNCTION__, str);
        if (str != NULL && strlen(str) == PICTURE_TIMESTAMP_LEN) {
            mParameters.set(KEY_PICTURE_TIME, str);
            LOGD("(%s) : %s=%s LEN=%d ============#########\n", __FUNCTION__, KEY_PICTURE_TIME, str, strlen(str));
        } else {
            LOGD("(%s) : %s=%s LEN=%d ============\n", __FUNCTION__, KEY_PICTURE_TIME, str, strlen(str));
            mParameters.set(KEY_PICTURE_TIME, str);
        }
    }

    // frame rate
    int new_frame_rate = params.getPreviewFrameRate();
    /* ignore any fps request, we're determine fps automatically based
     * on scene mode.  don't return an error because it causes CTS failure.
     */
    // new_frame_rate = 15;
    if (mRecordHint) {
        if (new_frame_rate) {
            if (mUseInternalISP && (mSecCamera->setFrameRate(new_frame_rate) < 0)){
                LOGE("ERR(%s):Fail on mSecCamera->setFrameRate(%d)", __func__, new_frame_rate);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.setPreviewFrameRate(new_frame_rate);
            }
        }
    } else {
        if (mUseInternalISP && !mPreviewRunning) {
            if (mSecCamera->setFrameRate(new_frame_rate) < 0){
                LOGE("ERR(%s):Fail on mSecCamera->setFrameRate(%d)", __func__, new_frame_rate);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.setPreviewFrameRate(new_frame_rate);
            }
        }
    }
    mParameters.setPreviewFrameRate(new_frame_rate);

    // rotation
    int new_rotation = params.getInt(CameraParameters::KEY_ROTATION);
    LOGD("%s : new_rotation %d", __func__, new_rotation); ///V 2 D
    if (0 <= new_rotation) {
        LOGD("%s : set orientation:%d", __func__, new_rotation);
        if (mSecCamera->setExifOrientationInfo(new_rotation) < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->setExifOrientationInfo(%d)", __func__, new_rotation);
            ret = UNKNOWN_ERROR;
        } else
            mParameters.set(CameraParameters::KEY_ROTATION, new_rotation);
    }

    // zoom
    int new_zoom = params.getInt(CameraParameters::KEY_ZOOM);
    if (new_zoom > ZOOM_LEVEL_MAX - 1 )
        return UNKNOWN_ERROR;
    int current_zoom = mParameters.getInt(CameraParameters::KEY_ZOOM);
    LOGD("%s : new_zoom %d", __func__, new_zoom);
    LOGD("%s : new_zoom %d   current_zoom %d", __func__, new_zoom,current_zoom);
    if (0 <= new_zoom) {
        if (new_zoom != current_zoom) {
            if (mSecCamera->setZoom(new_zoom) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setZoom(zoom(%d))", __func__, new_zoom);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_ZOOM, new_zoom);
            }
        }
    }

    // brightness
    const char *new_brightness_str = params.get("brightness");//jmq.add
    if(new_brightness_str != NULL)
    {
        int new_brightness = params.getInt(CameraParameters::KEY_BRIGHTNESS);
        int max_brightness = params.getInt(CameraParameters::KEY_MAX_BRIGHTNESS);
        int min_brightness = params.getInt(CameraParameters::KEY_MIN_BRIGHTNESS);
        LOGD("%s : new_brightness %d", __func__, new_brightness);
        if ((min_brightness <= new_brightness) &&
                (max_brightness >= new_brightness)) {
            if (mSecCamera->setBrightness(new_brightness) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setBrightness(brightness(%d))", __func__, new_brightness);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set("brightness", new_brightness);
            }
        }
    }
    // saturation
    const char *new_saturation_str = params.get("saturation");//jmq.add
    if(new_saturation_str != NULL)
    {
        int new_saturation = params.getInt(CameraParameters::KEY_SATURATION);
        int max_saturation = params.getInt(CameraParameters::KEY_MAX_SATURATION);
        int min_saturation = params.getInt(CameraParameters::KEY_MIN_SATURATION);
        LOGD("%s : new_saturation %d", __func__, new_saturation);
        if ((min_saturation <= new_saturation) &&
                (max_saturation >= new_saturation)) {
            if (mSecCamera->setSaturation(new_saturation) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setSaturation(saturation(%d))", __func__, new_saturation);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set("saturation", new_saturation);
            }
        }
    }
    // sharpness
    const char *new_sharpness_str = params.get("sharpness");//jmq.add
    if(new_sharpness_str != NULL)
    {
        int new_sharpness = params.getInt(CameraParameters::KEY_SHARPNESS);
        int max_sharpness = params.getInt(CameraParameters::KEY_MAX_SHARPNESS);
        int min_sharpness = params.getInt(CameraParameters::KEY_MIN_SHARPNESS);
        LOGD("%s : new_sharpness %d", __func__, new_sharpness);
        if ((min_sharpness <= new_sharpness) &&
                (max_sharpness >= new_sharpness)) {
            if (mSecCamera->setSharpness(new_sharpness) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setSharpness(sharpness(%d))", __func__, new_sharpness);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set("sharpness", new_sharpness);
            }
        }
    }

    // hue
    const char *new_hue_str = params.get("hue");//jmq.add
    if(new_hue_str != NULL)
    {
        int new_hue = params.getInt("hue");
        int max_hue = params.getInt("hue-max");
        int min_hue = params.getInt("hue-min");
        LOGD("%s : new_hue %d", __func__, new_hue);
        if ((min_hue <= new_hue) &&
                (max_hue >= new_hue)) {
            if (mSecCamera->setHue(new_hue) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setHue(hue(%d))", __func__, new_hue);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set("hue", new_hue);
            }
        }
    }

    // exposure
    int new_exposure_compensation = params.getInt(CameraParameters::KEY_EXPOSURE_COMPENSATION);
    int max_exposure_compensation = params.getInt(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION);
    int min_exposure_compensation = params.getInt(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION);
    LOGD("%s : new_exposure_compensation %d", __func__, new_exposure_compensation);
    if ((min_exposure_compensation <= new_exposure_compensation) &&
            (max_exposure_compensation >= new_exposure_compensation)) {
        if (mSecCamera->setExposure(new_exposure_compensation) < 0) {
            LOGE("ERR(%s):Fail on mSecCamera->setExposure(exposure(%d))", __func__, new_exposure_compensation);
            ret = UNKNOWN_ERROR;
        } else {
            mParameters.set(CameraParameters::KEY_EXPOSURE_COMPENSATION, new_exposure_compensation);
        }
    }

    const char *new_AE_lock = params.get(CameraParameters::KEY_AUTO_EXPOSURE_LOCK);
    const char *old_AE_lock = mParameters.get(CameraParameters::KEY_AUTO_EXPOSURE_LOCK);
    if ((new_AE_lock != NULL) && mUseInternalISP /*&& mPreviewRunning*/) { //for CTS jijun.yu
        if (strncmp(new_AE_lock, old_AE_lock, 4)) {
            int ae_value = !strncmp(new_AE_lock, "true", 4);
            if (mSecCamera->setAutoExposureLock(ae_value) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setExposureLock", __func__);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK, new_AE_lock);
            }
        }
    }

    // ISO
    const char *new_iso_str = params.get(CameraParameters::KEY_ISO);
    LOGD("%s : new_iso_str %s", __func__, new_iso_str);
    if (new_iso_str != NULL) {
        int new_iso = -1;

        if (!strcmp(new_iso_str, "auto")) {
            new_iso = ISO_AUTO;
        } else if (!strcmp(new_iso_str, "50")) {
            new_iso = ISO_50;
        } else if (!strcmp(new_iso_str, "100")) {
            new_iso = ISO_100;
        } else if (!strcmp(new_iso_str, "200")) {
            new_iso = ISO_200;
        } else if (!strcmp(new_iso_str, "400")) {
            new_iso = ISO_400;
        } else if (!strcmp(new_iso_str, "800")) {
            new_iso = ISO_800;
        } else if (!strcmp(new_iso_str, "1200")) {
            new_iso = ISO_1200;
        } else if (!strcmp(new_iso_str, "1600")) {
            new_iso = ISO_1600;
        } else {
            LOGE("ERR(%s):Invalid iso value(%s)", __func__, new_iso_str);
            ret = UNKNOWN_ERROR;
        }

        if (0 <= new_iso) {
            if (mSecCamera->setISO(new_iso) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setISO(iso(%d))", __func__, new_iso);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set("iso", new_iso_str);
            }
        }
    }

    //add begin by suojp 2012-06-06
#ifdef SCALADO_SCF
    //tune on the bracketing capturing or burst shot
    const char *burst_shot_num = params.get(CameraParameters::KEY_SCF_CAPTURE_BURST_TOTAL);
    LOGD("%s : num-snaps-per-shutter %s", __func__, burst_shot_num);
    if ((!strcmp(burst_shot_num, "0"))||(!strcmp(burst_shot_num, "1")))
    {
        mBurstCapture=0;
        LOGD("%s : bBracketingCapture off", __func__);
    }else
    {
        LOGD("%s : bBracketingCapture on", __func__);
        mBurstCapture=atoi(burst_shot_num);
        memset(mBracketingEx,0,sizeof(int)*mBurstCapture);
    }

    mParameters.set(CameraParameters::KEY_SCF_CAPTURE_BURST_TOTAL,mBurstCapture);//add by lzy 2013.5.14
    //check for bracket capturing
    const char *bracketing_exp = params.get(CameraParameters::KEY_SCF_CAPTURE_BURST_EXPOSURES);

    if(bracketing_exp!=NULL)
    {
        if(strlen(bracketing_exp)>1)
        {
            LOGD("TEST:%s ", bracketing_exp);
            LOGD("%d : strlen(bracketing_exp)",strlen(bracketing_exp));
            mIsBracketEV = true;
            int counter_bracketing=0;
            char* pch = strtok ((char*)bracketing_exp,",");

            while(pch!=NULL)
            {
                mBracketingEx[counter_bracketing]=atoi(pch);
                LOGD("%s : mBracketingEx %d,%d", __func__,counter_bracketing,mBracketingEx[counter_bracketing]);
                //mBracketingEx 0,0
                counter_bracketing++;
                if(counter_bracketing==SCF_BURST_SHOT_MAX)
                    break;
                pch = strtok (NULL,",");
            }
            
            mBurstCapture=counter_bracketing;
        }
        else
            mIsBracketEV = false;
        mExposureCounter = 0;

    }
    /*   remove by lzy for burst shot 2013.5.14
    if (mBurstCapture > 0)
    {
        ret = ret | DEF_SCALADO_MODE_FLAG;
    }
    */
#endif
    //add end by suojp 2012-06-06
    // Metering
    const char *new_metering_str = params.get(CameraParameters::KEY_METERING_MODE);
    const char *cur_metering_str = mParameters.get(CameraParameters::KEY_METERING_MODE);
    LOGD("%s : new_metering_str %s", __func__, new_metering_str);
    if (new_metering_str != NULL) {
        if (cur_metering_str == NULL || strncmp(new_metering_str, cur_metering_str, 6)) {
            int new_metering = -1;

            if (!strcmp(new_metering_str, "average")) {
                if (mUseInternalISP)
                    new_metering = IS_METERING_AVERAGE;
                else
                    new_metering = METERING_MATRIX;
            } else if (!strcmp(new_metering_str, "center")) {
                if (mUseInternalISP)
                    new_metering = IS_METERING_CENTER;
                else
                    new_metering = METERING_CENTER;
            } else if (!strcmp(new_metering_str, "spot")) {
                if (mUseInternalISP)
                    new_metering = IS_METERING_SPOT;
                else
                    new_metering = METERING_SPOT;
            } else if (!strcmp(new_metering_str, "matrix")) {
                if (mUseInternalISP)
                    new_metering = IS_METERING_MATRIX;
                else
                    new_metering = METERING_MATRIX;
            } else {
                LOGE("ERR(%s):Invalid metering value(%s)", __func__, new_metering_str);
                ret = UNKNOWN_ERROR;
            }

            if (0 <= new_metering) {
                if (mSecCamera->setMetering(new_metering) < 0) {
                    LOGE("ERR(%s):Fail on mSecCamera->setMetering(metering(%d))", __func__, new_metering);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_METERING_MODE, new_metering_str);
                }
            }
        }
    }

#if 1  //##mmkim 0608  -- AE area coordinate transition from google to YUV.
    // metering areas
    const char *newMeteringAreas = params.get(CameraParameters::KEY_METERING_AREAS);
    const char *curMeteringAreas = mParameters.get(CameraParameters::KEY_METERING_AREAS);
    int maxNumMeteringAreas = mSecCamera->getMaxNumMeteringAreas();
    if (newMeteringAreas != NULL && maxNumMeteringAreas != 0) {
        //        if (curMeteringAreas == NULL || strncmp(newMeteringAreas, curMeteringAreas, 27))
        {
            // ex : (-10,-10,0,0,300),(0,0,10,10,700)
            ExynosRect *rects  = new ExynosRect[maxNumMeteringAreas];
            ExynosRect2 *rect2s  = new ExynosRect2[maxNumMeteringAreas];
            int         *weights = new int[maxNumMeteringAreas];
            int width, height, frame_size;
            mSecCamera->getVideosnapshotSize(&width, &height, &frame_size);

            int validMeteringAreas = bracketsStr2Ints((char *)newMeteringAreas, maxNumMeteringAreas,
                                                      rects, weights);
            /*If the number of areas more than the max number , the error message should be reported
                          jijun.yu*/
            if (validMeteringAreas == -101) {
                LOGE("setMeteringAreas value error \n");
                ret = UNKNOWN_ERROR;
                goto out_setmeter;
            }

            if (0 < validMeteringAreas) {
                if (validMeteringAreas == 1
                        && rects[0].left == 0 && rects[0].top == 0 && rects[0].right == 0 && rects[0].bottom == 0) {
                    rects[0].left = 0;
                    rects[0].top = 0;
                    rects[0].right = width;
                    rects[0].bottom = height;
                }
                else
                {
                    /*the meter area should be check to fit for the CTS
                             and if the area is invalid, the error message should be notified jijun.yu*/
                    for (int i = 0; i < validMeteringAreas; i++) {
                        if (!((rects[i].left == 0)&&(rects[i].right == 0)
                              &&(rects[i].top == 0)&&(rects[i].bottom == 0))) {

                            if ((rects[i].left < -1000)||(rects[i].top < -1000)
                                    ||(rects[i].right > 1000)||(rects[i].bottom > 1000)
                                    ||(*weights < 1)||(*weights > 1000)) {
                                LOGE("ERR(%s):setMeteringAreas(%s) fail", __func__, newMeteringAreas);
                                ret = UNKNOWN_ERROR;
                                goto out_setmeter;
                            }

                            if ((rects[i].left >= rects[i].right)
                                    ||(rects[i].top >= rects[i].bottom)) {
                                LOGE("ERR(%s):setMeteringAreas(%s) fail", __func__, newMeteringAreas);
                                ret = UNKNOWN_ERROR;
                                goto out_setmeter;
                            }
                        }
                        rects[i].left = (rects[i].left + 1000) * width / 2000;
                        rects[i].top = (rects[i].top + 1000) * height / 2000;
                        rects[i].right = (rects[i].right + 1000) * width / 2000;
                        rects[i].bottom = (rects[i].bottom + 1000) * height / 2000;
                    }
                }

                if (mSecCamera->setMeteringAreas(validMeteringAreas, rects, weights) == false) {
                    LOGE("ERR(%s):setMeteringAreas(%s) fail", __func__, newMeteringAreas);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_METERING_AREAS, newMeteringAreas);
                }
            }
out_setmeter:
            delete [] rect2s;
            delete [] rects;
            delete [] weights;
        }
    }
#else	
    // Metering areas
    const char *newMeteringAreas = params.get(CameraParameters::KEY_METERING_AREAS);
    const char *curMeteringAreas = mParameters.get(CameraParameters::KEY_METERING_AREAS);
    int maxNumMeteringAreas = mSecCamera->getMaxNumMeteringAreas();
    LOGD("%s: newMeteringAreas(%s)", __func__, newMeteringAreas);

    if (newMeteringAreas != NULL && maxNumMeteringAreas != 0) {
        //        if (curMeteringAreas == NULL /*|| strncmp(newMeteringAreas, curMeteringAreas, 27)*/)
        {
            // ex : (-10,-10,0,0,300),(0,0,10,10,700)
            ExynosRect *rects  = new ExynosRect[maxNumMeteringAreas];
            ExynosRect2 *rect2s  = new ExynosRect2[maxNumMeteringAreas];
            int        *weights = new int[maxNumMeteringAreas];

            int validMeteringAreas = bracketsStr2Ints((char *)newMeteringAreas, maxNumMeteringAreas, rects, weights);
            if (validMeteringAreas == -101) {
                ret = UNKNOWN_ERROR;
                goto out_meter;
            }
            if (!((rects->left == 0)&&(rects->right == 0)&&(rects->top == 0)&&(rects->bottom == 0))) {
                LOGW("rects->left(%d)rects->right(%d)rects->top(%d)rects->bottom(%d) ",rects->left,rects->right,rects->top,rects->bottom);
                if ((rects->left < -1000)||(rects->top < -1000)||(rects->right > 1000)||(rects->bottom > 1000)||(*weights < 1)||(*weights > 1000)) {
                    LOGE("meter para   value set error \n");
                    LOGW("rects->left(%d)rects->right(%d)rects->top(%d)rects->bottom(%d) ",rects->left,rects->right,rects->top,rects->bottom);
                    ret = UNKNOWN_ERROR;
                    goto out_meter;
                }
                if ((rects->left >= rects->right)||(rects->top >= rects->bottom)) {
                    LOGE("meter para  set error \n");
                    ret = UNKNOWN_ERROR;
                    goto out_meter;
                }
            }
            if (0 < validMeteringAreas) {
                for (int i = 0; i < validMeteringAreas; i++) {
                    meteringAxisTrans(rects, rect2s, i);
                }
                if (mSecCamera->setMeteringAreas(validMeteringAreas, rect2s, weights) == false) {
                    LOGE("ERR(%s):setMeteringAreas(%s) fail", __func__, newMeteringAreas);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_METERING_AREAS, newMeteringAreas);
                }
            }
out_meter:			
            delete [] rect2s;
            delete [] rects;
            delete [] weights;
        }
    }
#endif
    // AFC
    const char *new_antibanding_str = params.get(CameraParameters::KEY_ANTIBANDING);
    LOGD("%s : new_antibanding_str %s", __func__, new_antibanding_str);
    if (new_antibanding_str != NULL) {
        int new_antibanding = -1;

        if (!strcmp(new_antibanding_str, CameraParameters::ANTIBANDING_AUTO)) {
            new_antibanding = ANTI_BANDING_AUTO;
        } else if (!strcmp(new_antibanding_str, CameraParameters::ANTIBANDING_50HZ)) {
            new_antibanding = ANTI_BANDING_50HZ;
        } else if (!strcmp(new_antibanding_str, CameraParameters::ANTIBANDING_60HZ)) {
            new_antibanding = ANTI_BANDING_60HZ;
        } else if (!strcmp(new_antibanding_str, CameraParameters::ANTIBANDING_OFF)) {
            new_antibanding = ANTI_BANDING_OFF;
        } else {
            LOGE("ERR(%s):Invalid antibanding value(%s)", __func__, new_antibanding_str);
            ret = UNKNOWN_ERROR;
        }

        if (0 <= new_antibanding) {
            if (mSecCamera->setAntiBanding(new_antibanding) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setAntiBanding(antibanding(%d))", __func__, new_antibanding);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_ANTIBANDING, new_antibanding_str);
            }
        }
    }

    // scene mode
    const char *new_flash_mode_str = params.get(CameraParameters::KEY_FLASH_MODE);
    const char *new_focus_mode_str = params.get(CameraParameters::KEY_FOCUS_MODE);
    const char *new_white_str = params.get(CameraParameters::KEY_WHITE_BALANCE);
    const char *new_scene_mode_str = params.get(CameraParameters::KEY_SCENE_MODE);
    const char *current_scene_mode_str = mParameters.get(CameraParameters::KEY_SCENE_MODE);
    //	if (mCameraID == SecCamera::CAMERA_ID_BACK) {
    // fps range
    int new_min_fps = 0;
    int new_max_fps = 0;
    int current_min_fps, current_max_fps;
    params.getPreviewFpsRange(&new_min_fps, &new_max_fps);
    mParameters.getPreviewFpsRange(&current_min_fps, &current_max_fps);
    /* our fps range is determined by the sensor, reject any request
     * that isn't exactly what we're already at.
     * but the check is performed when requesting only changing fps range
     */
    if (new_scene_mode_str && current_scene_mode_str) {
        if (!strcmp(new_scene_mode_str, current_scene_mode_str)) {
            if ((new_min_fps != current_min_fps) || (new_max_fps != current_max_fps)) {
                LOGW("%s : requested new_min_fps = %d, new_max_fps = %d not allowed",
                     __func__, new_min_fps, new_max_fps);
                /* TODO : We need policy for fps. */
                LOGW("%s : current_min_fps = %d, current_max_fps = %d",
                     __func__, current_min_fps, current_max_fps);
                //ret = UNKNOWN_ERROR;  //who do this???
                if ((new_min_fps > new_max_fps) ||(new_min_fps < 0) || (new_max_fps < 0))
                    return UNKNOWN_ERROR;

            }
        }
    } else {
        /* Check basic validation if scene mode is different */
        if ((new_min_fps > new_max_fps) ||
                (new_min_fps < 0) || (new_max_fps < 0))
            ret = UNKNOWN_ERROR;
        return ret;
    }

    // const char *new_flash_mode_str = params.get(CameraParameters::KEY_FLASH_MODE);
    //const char *new_focus_mode_str = params.get(CameraParameters::KEY_FOCUS_MODE);
    //const char *new_white_str = params.get(CameraParameters::KEY_WHITE_BALANCE);

    // fps range is (15000,30000) by default.
    //   mParameters.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(15000,120000)");
    //    mParameters.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "15000,120000");

    if ((new_scene_mode_str != NULL) && (current_scene_mode_str != NULL) && strncmp(new_scene_mode_str, current_scene_mode_str, 5)) {
        int  new_scene_mode = -1;
        if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_AUTO)) {
            new_scene_mode = SCENE_MODE_NONE;
        } else {
            // defaults for non-auto scene modes
            /*          new_focus_mode_str = CameraParameters::FOCUS_MODE_AUTO;
            new_flash_mode_str = CameraParameters::FLASH_MODE_OFF;
            new_white_str = CameraParameters::WHITE_BALANCE_AUTO;
            mParameters.set(CameraParameters::KEY_WHITE_BALANCE, new_white_str);
*/
            if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_PORTRAIT)) {
                new_scene_mode = SCENE_MODE_PORTRAIT;
                if (mCameraID == SecCamera::CAMERA_ID_BACK)
                    new_flash_mode_str = CameraParameters::FLASH_MODE_AUTO;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_LANDSCAPE)) {
                new_scene_mode = SCENE_MODE_LANDSCAPE;
            } else if ((!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_SPORTS))||(!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_ACTION))) {
                new_scene_mode = SCENE_MODE_SPORTS;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_PARTY)
                       // add begin by suojp 2012-06-05
                       ||!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_INDOOR)) {
                // add end by suojp 2012-06-05
                new_scene_mode = SCENE_MODE_PARTY_INDOOR;
                if (mCameraID == SecCamera::CAMERA_ID_BACK)
                    new_flash_mode_str = CameraParameters::FLASH_MODE_AUTO;
            } else if ((!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_BEACH)) ||
                       (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_SNOW))) {
                LOGE("suojp set mode %s", new_scene_mode_str);
                new_scene_mode = SCENE_MODE_BEACH_SNOW;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_SUNSET)) {
                new_scene_mode = SCENE_MODE_SUNSET;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_NIGHT)) {
                new_scene_mode = SCENE_MODE_NIGHTSHOT;
                mParameters.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(4000,30000)");
                mParameters.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "4000,30000");
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_FIREWORKS)) {
                new_scene_mode = SCENE_MODE_FIREWORKS;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_CANDLELIGHT)) {
                new_scene_mode = SCENE_MODE_CANDLE_LIGHT;
                // add begin by suojp 2012-06-05
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_FALL_COLOR)) {
                new_scene_mode = SCENE_MODE_FALL_COLOR;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_BACK_LIGHT)) {
                new_scene_mode = SCENE_MODE_BACK_LIGHT;
            }else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_DAWN)
                      || !strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_DUSK)) {
                new_scene_mode = SCENE_MODE_DUSK_DAWN;
                //            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_CANDLELIGHT)) {
                //              new_scene_mode = SCENE_MODE_DUST_DAWN;
            } else if (!strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_TEXT)) {
                new_scene_mode = SCENE_MODE_TEXT;
                // add end by suojp 2012-06-05
            } else {
                LOGE("%s::unmatched scene_mode(%s)",
                     __func__, new_scene_mode_str); //action, night-portrait, theatre, steadyphoto
                ret = UNKNOWN_ERROR;
            }
        }

        if (0 <= new_scene_mode) {
            if (mSecCamera->setSceneMode(new_scene_mode) < 0) {
                LOGE("%s::mSecCamera->setSceneMode(%d) fail", __func__, new_scene_mode);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_SCENE_MODE, new_scene_mode_str);
            }
        }
    }
    //	}
#if 0
    else{//Front camera, for cts,copied from inc
        // fps range
        int new_min_fps = 0;
        int new_max_fps = 0;
        params.getPreviewFpsRange(&new_min_fps, &new_max_fps);
        if ((new_min_fps > new_max_fps) ||
                (new_min_fps < 0) || (new_max_fps < 0))
            ret = UNKNOWN_ERROR;
    }
#endif
    // focus mode
    /* TODO : currently only posible focus modes at BACK camera */
    if ((new_focus_mode_str != NULL) && (mCameraID == SecCamera::CAMERA_ID_BACK)) {
        int  new_focus_mode = -1;

        if (!strcmp(new_focus_mode_str,
                    CameraParameters::FOCUS_MODE_AUTO)) {
            new_focus_mode = FOCUS_MODE_AUTO;
            mParameters.set(CameraParameters::KEY_FOCUS_DISTANCES,
                            BACK_CAMERA_AUTO_FOCUS_DISTANCES_STR);
        } else if (!strcmp(new_focus_mode_str,
                           CameraParameters::FOCUS_MODE_MACRO)) {
            new_focus_mode = FOCUS_MODE_MACRO;
            mParameters.set(CameraParameters::KEY_FOCUS_DISTANCES,
                            BACK_CAMERA_MACRO_FOCUS_DISTANCES_STR);
        } else if (!strcmp(new_focus_mode_str,
                           CameraParameters::FOCUS_MODE_INFINITY)) {
            new_focus_mode = FOCUS_MODE_INFINITY;
            mParameters.set(CameraParameters::KEY_FOCUS_DISTANCES,
                            BACK_CAMERA_INFINITY_FOCUS_DISTANCES_STR);
        } else if (!strcmp(new_focus_mode_str,
                           CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO) ||
                   !strcmp(new_focus_mode_str,
                           CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE)) {
            new_focus_mode = FOCUS_MODE_CONTINOUS;
        } else {
            /* TODO */
            /* This is temperary implementation.
               When camera support all AF mode, this code will be changing */
            LOGE("%s::unmatched focus_mode(%s)", __func__, new_focus_mode_str);
            ret = UNKNOWN_ERROR;
        }

        if (0 <= new_focus_mode) {
            if (mSecCamera->setFocusMode(new_focus_mode) < 0) {
                LOGE("%s::mSecCamera->setFocusMode(%d) fail", __func__, new_focus_mode);
                ret = UNKNOWN_ERROR;
            } else {
                LOGD("%s::mSecCamera->setFocusMode(%d) success", __func__, new_focus_mode);
                mParameters.set(CameraParameters::KEY_FOCUS_MODE, new_focus_mode_str);
            }
        }
    }
    else {
        if (mCameraID == SecCamera::CAMERA_ID_FRONT) {
            int  new_focus_mode = -1;
            if (!strcmp(new_focus_mode_str,
                        CameraParameters::FOCUS_MODE_AUTO)) {
                new_focus_mode = FOCUS_MODE_AUTO;
                mParameters.set(CameraParameters::KEY_FOCUS_DISTANCES,
                                BACK_CAMERA_AUTO_FOCUS_DISTANCES_STR);
                if (mSecCamera->setFocusMode(new_focus_mode) < 0) {
                    LOGE("%s::mSecCamera->setFocusMode(%d) fail", __func__, new_focus_mode);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_FOCUS_MODE, new_focus_mode_str);
                }

            } else {
                ret = UNKNOWN_ERROR;
            }

        }

    }

    // flash..
    if (new_flash_mode_str != NULL) {
        int  new_flash_mode = -1;

        if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_OFF))
            new_flash_mode = FLASH_MODE_OFF;
        else if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_AUTO))
            new_flash_mode = FLASH_MODE_AUTO;
        else if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_ON))
            new_flash_mode = FLASH_MODE_ON;
        else if (!strcmp(new_flash_mode_str, CameraParameters::FLASH_MODE_TORCH))
            new_flash_mode = FLASH_MODE_TORCH;
        else {
            LOGE("%s::unmatched flash_mode(%s)", __func__, new_flash_mode_str); //red-eye
            ret = UNKNOWN_ERROR;
        }

#if 0
        if (0 <= new_flash_mode) {
            if (mSecCamera->setFlashMode(new_flash_mode) < 0) {
                LOGE("%s::mSecCamera->setFlashMode(%d) fail", __func__, new_flash_mode);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_FLASH_MODE, new_flash_mode_str);
            }
        }
#else
        if (0 <= new_flash_mode)
            mParameters.set(CameraParameters::KEY_FLASH_MODE, new_flash_mode_str);

        /*Set Flash off if not torch mode, and do the specific flash mode when take picture.
           If torch mode, set it torch if in preview running, otherwise, set it after start preview.
           Jiangshanbin 2012/05/29*/
        if (new_flash_mode != FLASH_MODE_TORCH)
            mSecCamera->setFlashMode(FLASH_MODE_OFF);
        else if (mPreviewRunning){
            setTorchScenario();
            mSecCamera->setFlashMode(FLASH_MODE_TORCH);
        }
        //		else if(new_flash_mode == FLASH_MODE_TORCH){
        //			LOGE("%s : new_flash_mode == %d......charles.hu\n",__func__,new_flash_mode);
        //			setTorchScenario();
        //			mSecCamera->setFlashMode(FLASH_MODE_TORCH);
        //		}

#endif
    }

    // whitebalance
    LOGD("%s :new_white_str %s", __func__, new_white_str);
    if ((new_scene_mode_str != NULL) && !strcmp(new_scene_mode_str, CameraParameters::SCENE_MODE_AUTO)) {
        if (new_white_str != NULL) {
            int new_white = -1;

            if (!strcmp(new_white_str, CameraParameters::WHITE_BALANCE_AUTO)) {
                new_white = WHITE_BALANCE_AUTO;
            } else if (!strcmp(new_white_str,
                               CameraParameters::WHITE_BALANCE_DAYLIGHT)) {
                new_white = WHITE_BALANCE_SUNNY;
            } else if (!strcmp(new_white_str,
                               CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT)) {
                new_white = WHITE_BALANCE_CLOUDY;
            } else if (!strcmp(new_white_str,
                               CameraParameters::WHITE_BALANCE_FLUORESCENT)) {
                new_white = WHITE_BALANCE_FLUORESCENT;
            } else if (!strcmp(new_white_str,
                               CameraParameters::WHITE_BALANCE_INCANDESCENT)) {
                new_white = WHITE_BALANCE_TUNGSTEN;
            } else {
                LOGE("ERR(%s):Invalid white balance(%s)", __func__, new_white_str); //twilight, shade, warm_flourescent
                ret = UNKNOWN_ERROR;
            }

            if (0 <= new_white) {
                if (mSecCamera->setWhiteBalance(new_white) < 0) {
                    LOGE("ERR(%s):Fail on mSecCamera->setWhiteBalance(white(%d))", __func__, new_white);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_WHITE_BALANCE, new_white_str);
                }
            }
        }
    }

    const char *new_AWB_lock = params.get(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK);
    const char *old_AWB_lock = mParameters.get(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK);
    if (new_AWB_lock != NULL && mUseInternalISP/* && mPreviewRunning*/) {  //for cts test
        if (strncmp(new_AWB_lock, old_AWB_lock, 4)) {
            int awb_value = !strncmp(new_AWB_lock, "true", 4);
            if (mSecCamera->setAutoWhiteBalanceLock(awb_value) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setoAutoWhiteBalanceLock()", __func__);
                ret = UNKNOWN_ERROR;
            } else {
                mParameters.set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK, new_AWB_lock);
            }
        }
    }
    LOGD("%s : new_AWB_lock %s", __func__, new_AWB_lock);

#if 1 //limeng 0605:new touch AF algorithm
    // focus areas
    const char *newFocusAreas = params.get(CameraParameters::KEY_FOCUS_AREAS);
    // modify start by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
    const char *currentFocusAreas = mParameters.get(CameraParameters::KEY_FOCUS_AREAS);
    // modify end by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
    int maxNumFocusAreas = params.getInt(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS);
    LOGD("newFocusAreas=%s maxNumFocusAreas =%d\n",newFocusAreas,maxNumFocusAreas);
    //if (newFocusAreas != NULL && maxNumFocusAreas != 0 && (mCameraID == SecCamera::CAMERA_ID_BACK)) {
    // modify start by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
    if (newFocusAreas != NULL && maxNumFocusAreas != 0 && (mCameraID == SecCamera::CAMERA_ID_BACK)&&(currentFocusAreas == NULL || strcmp(newFocusAreas,currentFocusAreas))) {
        // modify end by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
        ExynosRect *rects = new ExynosRect[maxNumFocusAreas];
        int		   *weights = new int[maxNumFocusAreas];

        int validFocusedAreas = bracketsStr2Ints((char *)newFocusAreas, maxNumFocusAreas, rects, weights);

        /*If the number of areas more than the max number , the error message should be reported*/
        if (validFocusedAreas == -101) {
            LOGE("Error in setting the focus AREA \n");
            ret = UNKNOWN_ERROR;
            goto out_setfocus_area;
        }
        if (0 < validFocusedAreas) {
            // CameraParameters.h
            // A special case of single focus area (0,0,0,0,0) means driver to decide
            // the focus area. For example, the driver may use more signals to decide
            // focus areas and change them dynamically. Apps can set (0,0,0,0,0) if they
            // want the driver to decide focus areas.
            //##mmkim 0608 -- Touch focus coordiante transition form google to YUV.
            int width, height, frame_size;
            mSecCamera->getVideosnapshotSize(&width, &height, &frame_size);

            if (validFocusedAreas == 1
                    && rects[0].left == 0 && rects[0].top == 0 && rects[0].right == 0
                    && rects[0].bottom == 0) {
                rects[0].left = 0;
                rects[0].top = 0;
                rects[0].right = width;
                rects[0].bottom = height;
                // modify start by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
                mParameters.set(CameraParameters::KEY_FOCUS_AREAS, newFocusAreas);
                // modify end by shaking.wan for HalfPressed focus by samsung patch.2012-11-07.
            } else {
                for (int i = 0; i < validFocusedAreas; i++) {
                    /*the meter area should be check to fit for the CTS  jijun.yu*/
                    if ((rects[i].left ==0)&&(rects[i].right == 0)&&(rects[i].top == 0)
                            &&(rects[i].bottom == 0)&&(weights[i] == 0)) {
                        //default value (0,0,0,0,0)			?????
                    } else {
                        if (((rects[i].left < -1000) || (rects[i].left >= rects[i].right))
                                || (rects[i].right > 1000)) {
                            LOGE("ERR(%s):setFocusAreas(%s) fail", __func__, newFocusAreas);
                            ret = UNKNOWN_ERROR;
                            goto out_setfocus_area;
                        }

                        if (((rects[i].top < -1000) || (rects[i].top >= rects[i].bottom))
                                || (rects[i].bottom > 1000)) {
                            LOGE("ERR(%s):setFocusAreas(%s) fail", __func__, newFocusAreas);
                            ret = UNKNOWN_ERROR;
                            goto out_setfocus_area;
                        }

                        if ((weights[i] < 1)||(weights[i] > 1000)) {
                            LOGE("ERR(%s):setFocusAreas(%s) fail", __func__, newFocusAreas);
                            ret = UNKNOWN_ERROR;
                            goto out_setfocus_area;
                        }
                    }
                    rects[i].left = (rects[i].left + 1000) * width / 2000;
                    rects[i].top = (rects[i].top + 1000) * height / 2000;
                    rects[i].right = (rects[i].right + 1000) * width / 2000;
                    rects[i].bottom = (rects[i].bottom + 1000) * height / 2000;
                }
                mTouched = weights[0];

                if (mSecCamera->setFocusAreas(validFocusedAreas, rects, weights) < 0) {
                    LOGE("ERR(%s):setFocusAreas(%s) fail", __func__, newFocusAreas);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_FOCUS_AREAS, newFocusAreas);
                }
            }
        }
out_setfocus_area:
        delete [] rects;
        delete [] weights;
    }
#else
    const char *new_touch_rect_str = params.get(CameraParameters::KEY_FOCUS_AREAS);
    LOGD("Touched rect is '%s'", new_touch_rect_str);
    if (new_touch_rect_str != NULL) {
        if (/*!strcmp(new_touch_rect_str,"0") &&*/ (strlen(new_touch_rect_str) > 10)) {
            int left = 0, top = 0, right = 0, bottom = 0, touched = 0;
            int objx, objy;
            int num_area = 0;
            char *end;
            char delim = ',';
            int max_facus_area =params.getInt(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS);
            while (1) {
                left = (int)strtol(new_touch_rect_str+1, &end, 10);
                if (*end != delim) {
                    LOGE("Cannot find '%c' in str=%s", delim, new_touch_rect_str);
                    return -1;
                }
                top = (int)strtol(end+1, &end, 10);
                if (*end != delim) {
                    LOGE("Cannot find '%c' in str=%s", delim, new_touch_rect_str);
                    return -1;
                }
                right = (int)strtol(end+1, &end, 10);
                if (*end != delim) {
                    LOGE("Cannot find '%c' in str=%s", delim, new_touch_rect_str);
                    return -1;
                }
                bottom = (int)strtol(end+1, &end, 10);
                if (*end != delim) {
                    LOGE("Cannot find '%c' in str=%s", delim, new_touch_rect_str);
                    return -1;
                }
                touched = (int)strtol(end+1, &end, 10);
                /*	        if (*end != ')') {
                    LOGE("Cannot find ')' in str=%s", new_touch_rect_str);
                    return -1;
                }*/
                if (strncmp(end,")",1)) {
                    LOGE("Cannot find ')' in str=%s", new_touch_rect_str);
                    return -1;
                }
                num_area = num_area+1;

                if ((left ==0)&&(right == 0)&&(top == 0)&&(bottom == 0)&&(touched == 0) ) {
                    //default value (0,0,0,0,0)			?????
                } else {

                    if (((left < -1000) || (left >= right)) || (right > 1000))
                        return UNKNOWN_ERROR;
                    if (((top < -1000) || (top >= bottom)) || (bottom > 1000))
                        return UNKNOWN_ERROR;
                    if ((touched < 1)||(touched > 1000))
                        return UNKNOWN_ERROR;
                }
                objx = (int)((right + left)/2);
                objy = (int)((bottom + top)/2);

                mTouched = touched;
                mSecCamera->setObjectPosition(objx, objy);
                if (num_area > max_facus_area)
                    return UNKNOWN_ERROR;
                if (*(end+1) == '\0') {
                    mParameters.set(CameraParameters::KEY_FOCUS_AREAS,new_touch_rect_str);
                    break;
                }
            }
        }else {
            mParameters.set(CameraParameters::KEY_FOCUS_AREAS,"0");
        }
#endif


        // image effect
        const char *new_image_effect_str = params.get(CameraParameters::KEY_EFFECT);
        LOGD("%s : new_image_effect_str %s", __func__, new_image_effect_str);

        if (new_image_effect_str != NULL) {

            int  new_image_effect = -1;

            if (!strcmp(new_image_effect_str, CameraParameters::EFFECT_NONE)) {
                new_image_effect = IMAGE_EFFECT_NONE;
            } else if (!strcmp(new_image_effect_str, CameraParameters::EFFECT_MONO)) {
                new_image_effect = IMAGE_EFFECT_BNW;
            } else if (!strcmp(new_image_effect_str, CameraParameters::EFFECT_SEPIA)) {
                new_image_effect = IMAGE_EFFECT_SEPIA;
            } else if (!strcmp(new_image_effect_str, CameraParameters::EFFECT_AQUA))
                new_image_effect = IMAGE_EFFECT_AQUA;
            else if (!strcmp(new_image_effect_str, CameraParameters::EFFECT_NEGATIVE)) {
                new_image_effect = IMAGE_EFFECT_NEGATIVE;
            } else {
                //posterize, whiteboard, blackboard, solarize
                LOGE("ERR(%s):Invalid effect(%s)", __func__, new_image_effect_str);
                ret = UNKNOWN_ERROR;
            }

            if (new_image_effect >= 0) {
                if (mSecCamera->setImageEffect(new_image_effect) < 0) {
                    LOGE("ERR(%s):Fail on mSecCamera->setImageEffect(effect(%d))", __func__, new_image_effect);
                    ret = UNKNOWN_ERROR;
                } else {
                    const char *old_image_effect_str = mParameters.get(CameraParameters::KEY_EFFECT);

                    if (old_image_effect_str) {
                        if (strcmp(old_image_effect_str, new_image_effect_str)) {
                            setSkipFrame(EFFECT_SKIP_FRAME);
                        }
                    }

                    mParameters.set(CameraParameters::KEY_EFFECT, new_image_effect_str);
                }
            }
        }

        //contrast
        const char *new_contrast_str = params.get(CameraParameters::KEY_CONTRAST);
        LOGD("%s : new_contrast_str %s", __func__, new_contrast_str);
        if (new_contrast_str != NULL) {
            int new_contrast = -1;

            if (!strcmp(new_contrast_str, "auto")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_DEFAULT;
                else
                    LOGW("WARN(%s):Invalid contrast value (%s)", __func__, new_contrast_str);
            } else if (!strcmp(new_contrast_str, "-2")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_MINUS_2;
                else
                    new_contrast = -2;
            } else if (!strcmp(new_contrast_str, "-1")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_MINUS_1;
                else
                    new_contrast = -1;
            } else if (!strcmp(new_contrast_str, "0")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_DEFAULT;
                else
                    new_contrast = 0;
            } else if (!strcmp(new_contrast_str, "1")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_PLUS_1;
                else
                    new_contrast = 1;
            } else if (!strcmp(new_contrast_str, "2")) {
                if (mUseInternalISP)
                    new_contrast = IS_CONTRAST_PLUS_2;
                else
                    new_contrast = 2;
            } else if (!strcmp(new_contrast_str, "-3")) {
                new_contrast = -3;
            } else if (!strcmp(new_contrast_str, "3")) {
                new_contrast = 3;
            } else {
                LOGE("ERR(%s):Invalid contrast value(%s)", __func__, new_contrast_str);
                ret = UNKNOWN_ERROR;
            }

            if (-9 < new_contrast) {
                if (mSecCamera->setContrast(new_contrast) < 0) {
                    LOGE("ERR(%s):Fail on mSecCamera->setContrast(contrast(%d))", __func__, new_contrast);
                    ret = UNKNOWN_ERROR;
                } else {
                    mParameters.set(CameraParameters::KEY_CONTRAST, new_contrast_str);
                }
            }
        }


        //WDR
        int new_wdr = params.getInt("wdr");
        LOGD("%s : new_wdr %d", __func__, new_wdr);

        if (0 <= new_wdr) {
            if (mSecCamera->setWDR(new_wdr) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setWDR(%d)", __func__, new_wdr);
                ret = UNKNOWN_ERROR;
            }
        }

        //anti shake
        int new_anti_shake = mInternalParameters.getInt("anti-shake");

        if (0 <= new_anti_shake) {
            if (mSecCamera->setAntiShake(new_anti_shake) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setWDR(%d)", __func__, new_anti_shake);
                ret = UNKNOWN_ERROR;
            }
        }

        // gps latitude
        const char *new_gps_latitude_str = params.get(CameraParameters::KEY_GPS_LATITUDE);
        if (mSecCamera->setGPSLatitude(new_gps_latitude_str) < 0) {
            LOGE("%s::mSecCamera->setGPSLatitude(%s) fail", __func__, new_gps_latitude_str);
            ret = UNKNOWN_ERROR;
        } else {
            if (new_gps_latitude_str) {
                mParameters.set(CameraParameters::KEY_GPS_LATITUDE, new_gps_latitude_str);
            } else {
                mParameters.remove(CameraParameters::KEY_GPS_LATITUDE);
            }
        }

        // gps longitude
        const char *new_gps_longitude_str = params.get(CameraParameters::KEY_GPS_LONGITUDE);

        if (mSecCamera->setGPSLongitude(new_gps_longitude_str) < 0) {
            LOGE("%s::mSecCamera->setGPSLongitude(%s) fail", __func__, new_gps_longitude_str);
            ret = UNKNOWN_ERROR;
        } else {
            if (new_gps_longitude_str) {
                mParameters.set(CameraParameters::KEY_GPS_LONGITUDE, new_gps_longitude_str);
            } else {
                mParameters.remove(CameraParameters::KEY_GPS_LONGITUDE);
            }
        }

        // gps altitude
        const char *new_gps_altitude_str = params.get(CameraParameters::KEY_GPS_ALTITUDE);

        if (mSecCamera->setGPSAltitude(new_gps_altitude_str) < 0) {
            LOGE("%s::mSecCamera->setGPSAltitude(%s) fail", __func__, new_gps_altitude_str);
            ret = UNKNOWN_ERROR;
        } else {
            if (new_gps_altitude_str) {
                mParameters.set(CameraParameters::KEY_GPS_ALTITUDE, new_gps_altitude_str);
            } else {
                mParameters.remove(CameraParameters::KEY_GPS_ALTITUDE);
            }
        }

        // gps timestamp
        const char *new_gps_timestamp_str = params.get(CameraParameters::KEY_GPS_TIMESTAMP);

        if (mSecCamera->setGPSTimeStamp(new_gps_timestamp_str) < 0) {
            LOGE("%s::mSecCamera->setGPSTimeStamp(%s) fail", __func__, new_gps_timestamp_str);
            ret = UNKNOWN_ERROR;
        } else {
            if (new_gps_timestamp_str) {
                mParameters.set(CameraParameters::KEY_GPS_TIMESTAMP, new_gps_timestamp_str);
            } else {
                mParameters.remove(CameraParameters::KEY_GPS_TIMESTAMP);
            }
        }

        // gps processing method
        const char *new_gps_processing_method_str = params.get(CameraParameters::KEY_GPS_PROCESSING_METHOD);

        if (mSecCamera->setGPSProcessingMethod(new_gps_processing_method_str) < 0) {
            LOGE("%s::mSecCamera->setGPSProcessingMethod(%s) fail", __func__, new_gps_processing_method_str);
            ret = UNKNOWN_ERROR;
        } else {
            if (new_gps_processing_method_str) {
                mParameters.set(CameraParameters::KEY_GPS_PROCESSING_METHOD, new_gps_processing_method_str);
            } else {
                mParameters.remove(CameraParameters::KEY_GPS_PROCESSING_METHOD);
            }
        }

        //add begin by suojp 2012-04-29
#ifdef S3_CONTINOUS_SHOT	
        const char *continous_shot_path_str = params.get(CameraParameters::KEY_CONTINOUS_SHOT_PATH);
        if (continous_shot_path_str){
            if (strlen(continous_shot_path_str) + 10 < DEF_FILE_PATH_MAX_SIZE)
            {
                strcpy(mstrContinousSaveFilePath, continous_shot_path_str);
                strcat(mstrContinousSaveFilePath, "%03d.jpg");
            }
            else
            {
                LOGE("ERR(%s):Fail on set continous shot path (%s)",
                     __func__, continous_shot_path_str);
            }
        }

        int continous_shot_max_count = params.getInt(CameraParameters::KEY_CONTINOUS_SHOT_MAX_COUNT);
        if (0 <= continous_shot_max_count) {
            mbufContinousMaxCount = continous_shot_max_count;
        }

        int continous_shot_min_interval = params.getInt(CameraParameters::KEY_CONTINOUS_SHOT_MIN_INTERVAL);
        if (0 <= continous_shot_min_interval) {
            mTimeIntervalMin = continous_shot_min_interval;
        }

#endif
        //add end by suojp 2012-04-29

        // Recording size
        /* TODO */
        /* GED application don't set different recording size before recording button is pushed */
        int new_recording_width  = 0;
        int new_recording_height = 0;
        params.getVideoSize(&new_recording_width, &new_recording_height);
        LOGD("new_recording_width (%d) new_recording_height (%d)",  new_recording_width, new_recording_height);

        int current_recording_width, current_recording_height;
        mParameters.getVideoSize(&current_recording_width, &current_recording_height);
        LOGD("current_recording_width (%d) current_recording_height (%d)",
             current_recording_width, current_recording_height);

        if (current_recording_width != new_recording_width ||
                current_recording_height != new_recording_height) {
            if (0 < new_recording_width && 0 < new_recording_height && !mRecordRunning) {
                if (mSecCamera->setRecordingSize(new_recording_width, new_recording_height) < 0) {
                    LOGE("ERR(%s):Fail on mSecCamera->setRecordingSize(width(%d), height(%d))",
                         __func__, new_recording_width, new_recording_height);
                    ret = UNKNOWN_ERROR;
                }
                if (mUseInternalISP && mPreviewRunning && !mRecordRunning){
                    // stopPreview();
                    //startPreview();
                }
                mParameters.setVideoSize(new_recording_width, new_recording_height);
            }
        }

        //gamma
        const char *new_gamma_str = mInternalParameters.get("video_recording_gamma");

        if (new_gamma_str != NULL) {
            int new_gamma = -1;
            if (!strcmp(new_gamma_str, "off"))
                new_gamma = GAMMA_OFF;
            else if (!strcmp(new_gamma_str, "on"))
                new_gamma = GAMMA_ON;
            else {
                LOGE("%s::unmatched gamma(%s)", __func__, new_gamma_str);
                ret = UNKNOWN_ERROR;
            }

            if (0 <= new_gamma) {
                if (mSecCamera->setGamma(new_gamma) < 0) {
                    LOGE("%s::mSecCamera->setGamma(%d) fail", __func__, new_gamma);
                    ret = UNKNOWN_ERROR;
                }
            }
        }

        //slow ae
        const char *new_slow_ae_str = mInternalParameters.get("slow_ae");

        if (new_slow_ae_str != NULL) {
            int new_slow_ae = -1;

            if (!strcmp(new_slow_ae_str, "off"))
                new_slow_ae = SLOW_AE_OFF;
            else if (!strcmp(new_slow_ae_str, "on"))
                new_slow_ae = SLOW_AE_ON;
            else {
                LOGE("%s::unmatched slow_ae(%s)", __func__, new_slow_ae_str);
                ret = UNKNOWN_ERROR;
            }

            if (0 <= new_slow_ae) {
                if (mSecCamera->setSlowAE(new_slow_ae) < 0) {
                    LOGE("%s::mSecCamera->setSlowAE(%d) fail", __func__, new_slow_ae);
                    ret = UNKNOWN_ERROR;
                }
            }
        }

        /*Camcorder fix fps*/
        int new_sensor_mode = mInternalParameters.getInt("cam_mode");

        if (0 <= new_sensor_mode) {
            if (mSecCamera->setSensorMode(new_sensor_mode) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setSensorMode(%d)", __func__, new_sensor_mode);
                ret = UNKNOWN_ERROR;
            }
        } else {
            new_sensor_mode=0;
        }

        /*Shot mode*/
        int new_shot_mode = mInternalParameters.getInt("shot_mode");

        if (0 <= new_shot_mode) {
            if (mSecCamera->setShotMode(new_shot_mode) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setShotMode(%d)", __func__, new_shot_mode);
                ret = UNKNOWN_ERROR;
            }
        } else {
            new_shot_mode=0;
        }

        // chk_dataline
        int new_dataline = mInternalParameters.getInt("chk_dataline");

        if (0 <= new_dataline) {
            if (mSecCamera->setDataLineCheck(new_dataline) < 0) {
                LOGE("ERR(%s):Fail on mSecCamera->setDataLineCheck(%d)", __func__, new_dataline);
                ret = UNKNOWN_ERROR;
            }
        }
        LOGD("%s return ret = %d", __func__, ret);

        mRunningSetParam = 0;
        return ret;
    }

    CameraParameters CameraHardwareSec::getParameters() const
    {
        LOGD("%s :", __func__);
        return mParameters;
    }

    status_t CameraHardwareSec::sendCommand(int32_t command, int32_t arg1, int32_t arg2)
    {
        /* TODO */
        /* CAMERA_CMD_START_FACE_DETECTION and CAMERA_CMD_STOP_FACE_DETECTION
       for Face Detection */
        if(command == CAMERA_CMD_START_FACE_DETECTION) {
            if (mParameters.getInt(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW) <= 0)
                return BAD_VALUE;

            if (mSecCamera->setFaceDetect(FACE_DETECTION_ON) < 0) {
                LOGE("ERR(%s): Fail on mSecCamera->startFaceDetection()", __func__);
                return BAD_VALUE;
            } else {
                return NO_ERROR;
            }
        }
        if(command == CAMERA_CMD_STOP_FACE_DETECTION) {
            if (mSecCamera->setFaceDetect(FACE_DETECTION_OFF) < 0) {
                LOGE("ERR(%s): Fail on mSecCamera->stopFaceDetection()", __func__);
                return BAD_VALUE;
            } else {
                return NO_ERROR;
            }
        }

        //add begin by suojp 2012-04-11
#ifdef S3_CHECK_PREVIEW_LIGHT
        if(command == CAMERA_CMD_START_CHECK_PREVIEW_LIGHT) {

            if (arg1 > 0xfff || arg1 < -0xfff)
            {
                LOGE("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_PREVIEW_LIGHT arg1(%d)", __func__, arg1);
                return BAD_VALUE;
            }

            if (arg1 < 1)
            {
                if (!msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT))
                {
                    LOGE("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_PREVIEW_LIGHT arg1(%d)~", __func__, arg1);
                    return BAD_VALUE;
                }

                mCheckPreviewFake = -arg1;
                LOGE("S3_CHECK_PREVIEW_LIGHT open fake mode mCheckPreviewFake(%d)", mCheckPreviewFake);
                return NO_ERROR;
            }

            mCheckPreviewStep = arg1;

            LOGE("S3_CHECK_PREVIEW_LIGHT start...");
            if (msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT))
            {
                if (mCheckPreviewFake > 0)
                {
                    mCheckPreviewFake = 0;
                    LOGE("S3_CHECK_PREVIEW_LIGHT close fake mode");
                }

                LOGE("S3_CHECK_PREVIEW_LIGHT restart, Set CheckPreviewStep=%d", mCheckPreviewStep);
                return NO_ERROR;
            }

            if (NULL == mCheckLightLib[0])
            {
                mCheckLightLib[0] = ILeImageLightnessEstmater::Create(LE_YUV420sp_SAMSUNG);
            }

            if (NULL == mCheckLightLib[1])
            {
                mCheckLightLib[1] = ILeImageLightnessEstmater::Create(LE_RGB565);
            }

            enableMsgType(CAMERA_MSG_CHECK_LIGHT);
            return NO_ERROR;
        }

        if(command == CAMERA_CMD_STOP_CHECK_PREVIEW_LIGHT) {
            LOGE("S3_CHECK_PREVIEW_LIGHT stop...");

            if (!msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT))
            {
                return NO_ERROR;
            }

            disableMsgType(CAMERA_MSG_CHECK_LIGHT);

            if (mCheckLightWorking)
            {
                for(int i = 0; i < 500; i++)
                {
                    usleep(2000);
                    if (!mCheckLightWorking)
                    {

                        LOGE("S3_CHECK_PREVIEW_LIGHT stop waiting check");
                        break;
                    }
                }
            }

            if (NULL != mCheckLightLib[0])
            {
                delete mCheckLightLib[0];
                mCheckLightLib[0] = NULL;
            }

            if (NULL != mCheckLightLib[1])
            {
                delete mCheckLightLib[1];
                mCheckLightLib[1] = NULL;
            }

            return NO_ERROR;
        }
#else
        if(command == CAMERA_CMD_START_CHECK_PREVIEW_LIGHT
                || command == CAMERA_CMD_STOP_CHECK_PREVIEW_LIGHT) {
            return NO_ERROR;
        }
#endif	
        //add end by suojp 2012-04-11

        //add begin by suojp 2012-04-29
#ifdef S3_CONTINOUS_SHOT
        if(command == CAMERA_CMD_BREAK_CONTINOUS_SHOT
                || CAMERA_CMD_ENABLE_CONTINOUS_SHOT == command) {
            LOGD("CAMERA_CMD_BREAK_CONTINOUS_SHOT ...");
            if (!mExitContinousShotThread)
            {
                LOGE("CAMERA_CMD_BREAK_CONTINOUS_SHOT mExitContinousShotThread = true");
                mExitContinousShotThread = true;
            }
            return NO_ERROR;
        }
#endif	
        //add end by suojp 2012-04-29
#ifdef BURST_SHOT
        if( CAMERA_CMD_ENABLE_BURST_SHOT == command) {
            LOGD("CAMERA_CMD_ENABLE_BURST_SHOT ...");
            if (!mExitContinousShotThread)
            {	//TODO SOMETHING   add by lzy
                LOGE("CAMERA_CMD_BREAK_CONTINOUS_SHOT mExitContinousShotThread = true");
                //mExitContinousShotThread = true;
            }
            return NO_ERROR;
        }


#endif
        //add begin by suojp 2012-05-11
#ifdef S3_LOSSFOCUS_NOTIFY
        if(command == CAMERA_CMD_START_CHECK_LOSS_FOCUS) {
            LOGI("CAMERA_CMD_START_CHECK_LOSS_FOCUS arg1(%d)...", arg1);
            if (arg1 > 4 || arg1 < 2)
            {
                LOGE("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_LOSS_FOCUS checkvalue(%d)", __func__, arg1);
                return BAD_VALUE;
            }

            if (!mUseInternalISP)
            {
                LOGI("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_LOSS_FOCUS not mUseInternalISP", __func__);
                return NO_ERROR;
            }

            if (NULL != mLossFocusNotifyThread.get())
            {
                LOGE("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_LOSS_FOCUS thread have started", __func__);
                return UNKNOWN_ERROR;
            }

            mExitLossFocusNotifyThread = false;
            mFocusSave = arg1;

            mLossFocusNotifyThread = new LossFocusNotifyThread(this, mFocusSave);
            if (NULL == mLossFocusNotifyThread.get()) {
                LOGE("ERR(%s): Fail on sendCommand CAMERA_CMD_START_CHECK_LOSS_FOCUS new thread", __func__);
                return UNKNOWN_ERROR;
            }

            return NO_ERROR;
        }

        if(command == CAMERA_CMD_STOP_CHECK_LOSS_FOCUS) {
            LOGI("CAMERA_CMD_STOP_CHECK_LOSS_FOCUS ...");
            if (NULL == mLossFocusNotifyThread.get())
            {
                return NO_ERROR;
            }

            mLossFocusNotifyThread->requestExit();
            mExitLossFocusNotifyThread = true;
            mLossFocusNotifyThread->requestExitAndWait();
            mLossFocusNotifyThread.clear();
            mLossFocusNotifyThread = NULL;
            return NO_ERROR;
        }
#else
        if(command == CAMERA_CMD_START_CHECK_LOSS_FOCUS
                || command == CAMERA_CMD_STOP_CHECK_LOSS_FOCUS ||command == CAMERA_CMD_ENABLE_FOCUS_MOVE_MSG) {
            return NO_ERROR;
        }
#endif
        //add end by suojp 2012-05-11
        //add begin by suojp 2012-05-26
#ifdef S3_BLINK_DETECTION
        if(command == CAMERA_CMD_START_BLINK_DETECTION) {
            LOGE("CAMERA_CMD_START_BLINK_DETECTION (%d, %d) ...",
                 mBlinkLevelMax, mBlinkCheckCount);
            if (1 > mBlinkLevelMax || 1 > mBlinkCheckCount
                    || mBlinkLevelMax > 100)
            {
                LOGE("ERR(%s): Fail on sendCommand "
                     "CAMERA_CMD_START_BLINK_DETECTION error parameter(%d, %d)",
                     __func__, mBlinkLevelMax, mBlinkCheckCount);
                return BAD_VALUE;
            }
            
            mBlinkLevelMax = arg1;
            mBlinkCheckCount = arg2;
            return NO_ERROR;
        }
#endif	
        //add end by suojp 2012-05-25
        //add begin by suojp 2012-06-18
#ifdef S3_IGNORE_PREVIEW_WINDOW
        if(command == CAMERA_CMD_IGNORE_PREVIEW_WINDOW) {
            bIgnorePreviewWindow = arg1 > 0;
            return NO_ERROR;
        }
#endif
        //add end by suojp 2012-06-18

        return BAD_VALUE;
    }

    void CameraHardwareSec::release()
    {
        LOGD("%s", __func__);

        if (mPreviewRunning)
            stopPreview();
        /* shut down any threads we have that might be running.  do it here
     * instead of the destructor.  we're guaranteed to be on another thread
     * than the ones below.  if we used the destructor, since the threads
     * have a reference to this object, we could wind up trying to wait
     * for ourself to exit, which is a deadlock.
     */
        if (mPreviewThread != NULL) {
            /* this thread is normally already in it's threadLoop but blocked
         * on the condition variable or running.  signal it so it wakes
         * up and can exit.
         */
            mPreviewThread->requestExit();
            mExitPreviewThread = true;
            mPreviewRunning = true; /* let it run so it can exit */
            mPreviewCondition.signal();
            mPreviewThread->requestExitAndWait();
            mPreviewThread.clear();
            mPreviewRunning = false; /* let it be actual value */
        }
        if (mAutoFocusThread != NULL) {
            /* this thread is normally already in it's threadLoop but blocked
         * on the condition variable.  signal it so it wakes up and can exit.
         */
            mFocusLock.lock();
            mAutoFocusThread->requestExit();
            mExitAutoFocusThread = true;
            mFocusCondition.signal();
            mFocusLock.unlock();
            mAutoFocusThread->requestExitAndWait();
            mAutoFocusThread.clear();
        }
        if (mPictureThread != NULL) {
            mPictureThread->requestExitAndWait();
            mPictureThread.clear();
        }

        //jmq.wait callback exit
        if (mCallbackThread != NULL) {
            mCallbackThread->requestExit();
            mCallbackCondition.signal();
            mCallbackThread->requestExitAndWait();
            mCallbackThread.clear();
        }
        if (mRecordThread!= NULL) {
            mRecordThread->requestExitAndWait();
            mRecordThread.clear();
        }

#if 0	
        if (mBurstpictureThread != NULL) {
            mBurstpictureThread->requestExitAndWait();
            mBurstpictureThread.clear();
        }
#endif	

#ifdef IS_FW_DEBUG
        mStopDebugging = true;
        mDebugCondition.signal();
        if (mDebugThread != NULL) {
            mDebugThread->requestExitAndWait();
            mDebugThread.clear();
        }
#endif
        //add begin by suojp 2012-04-19
#ifdef S3_CHECK_PREVIEW_LIGHT
        if (msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT))
        {
            sendCommand(CAMERA_CMD_STOP_CHECK_PREVIEW_LIGHT, 0, 0);
        }

#endif	
        //add end by suojp 2012-04-19
        //add begin by suojp 2012-04-29
        if (msgTypeEnabled( CAMERA_MSG_CONTINOUS_SHOT))
        {
            disableMsgType(CAMERA_MSG_CONTINOUS_SHOT);
            sendCommand(CAMERA_CMD_BREAK_CONTINOUS_SHOT, 0, 0);
        }
        //add end by suojp 2012-04-29
        //add begin by suojp 2012-07-06
#ifdef S3_LOSSFOCUS_NOTIFY
        if (msgTypeEnabled( CAMERA_MSG_CHECK_LOSS_FOCUS))
        {
            disableMsgType(CAMERA_MSG_CHECK_LOSS_FOCUS);
        }

        if (mLossFocusNotifyThread!= NULL) {
            mLossFocusNotifyThread->requestExit();
            mExitLossFocusNotifyThread = true;
            mLossFocusNotifyThread->requestExitAndWait();
            mLossFocusNotifyThread.clear();
            mLossFocusNotifyThread = NULL;
        }
#endif
        //add end by suojp 2012-07-06

#ifdef S3_STATUS_UPDATE
        if (mStatusUpdateThread!= NULL) {
            mStatusUpdateThread->requestExit();
            mExitStatusThread = true;
            mCheckLightCondition.signal();
            mStatusUpdateThread->requestExitAndWait();
            mStatusUpdateThread.clear();
            mStatusUpdateThread = NULL;
        }
#endif

        if (mRawHeap) {
            mRawHeap->release(mRawHeap);
            mRawHeap = 0;
        }
        if (mPreviewHeap) {
            mPreviewHeap->release(mPreviewHeap);
            mPreviewHeap = 0;
        }
        if (mFaceDataHeap) {
            mFaceDataHeap->release(mFaceDataHeap);
            mFaceDataHeap = 0;
        }
        for(int i = 0; i < BUFFER_COUNT_FOR_ARRAY; i++) {
            if (mRecordHeap[i]) {
                mRecordHeap[i]->release(mRecordHeap[i]);
                mRecordHeap[i] = 0;
            }
        }

        /* close after all the heaps are cleared since those
     * could have dup'd our file descriptor.
     */
        mSecCamera->DestroyCamera();
    }

    //add begin by suojp 2012-04-11
    int CameraHardwareSec::save_to_file(char *real_buf, int buf_size)
    {
        return save_to_file("/data/data/camera_dump.yuv", real_buf, buf_size);
    }

    int CameraHardwareSec::save_to_file(char *filename, char *real_buf, int buf_size)
    {
        LOGE("save_to_file begin");
        FILE *yuv_fp = NULL;
        char *buffer = NULL;

        yuv_fp = fopen(filename, "wb");
        if (yuv_fp == NULL) {
            LOGE("Save test file open error ");
            return -1;
        }

        LOGD("[BestIQ]  real_jpeg size ========>  %d", buf_size);
        buffer = (char *) malloc(buf_size);
        if (buffer == NULL) {
            LOGE("Save YUV] buffer alloc failed");
            if (yuv_fp)
                fclose(yuv_fp);

            return -2;
        }

        LOGE("save_to_file malloc ok");

        memcpy(buffer, real_buf, buf_size);

        fflush(stdout);

        fwrite(buffer, 1, buf_size, yuv_fp);

        fflush(yuv_fp);

        if (yuv_fp)
            fclose(yuv_fp);
        if (buffer)
            free(buffer);
        LOGE("save_to_file end");
        return 0;

    }

    int CameraHardwareSec::save_file(char *filename, char *real_buf, int buf_size)
    {
        LOGE("save_to_file begin %s", filename);
        FILE *yuv_fp = NULL;

        yuv_fp = fopen(filename, "wb");
        if (yuv_fp == NULL) {
            LOGE("Save test file open error ");
            return -1;
        }

        //    memcpy(buffer, real_buf, buf_size);

        //    fflush(stdout);

        fwrite(real_buf, 1, buf_size, yuv_fp);

        fflush(yuv_fp);

        if (yuv_fp)
            fclose(yuv_fp);

        LOGE("save_to_file end");
        return 0;
    }

#ifdef S3_CHECK_PREVIEW_LIGHT  
    bool CameraHardwareSec::checkLight(void)
    {
        mCheckLightWorking = true;

        float light = 0.0f;
        float BackLighting = 0.0f;
        char *srcFrame = mSecCamera->getMappedAddr();
        int width, height, frame_size;
        mSecCamera->getPreviewSize(&width, &height, &frame_size);

        if (UNLIKELY(V4L2_PIX_FMT_RGB565 == mPreviewFormat))
        {
            if (LIKELY(NULL != mCheckLightLib[1]))
            {
#if 0         
                char a, b;
                for (int i = 0; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        a = srcFrame[i*width*2+j*2];
                        b = srcFrame[i*width*2+j*2+1];
                    }
                }
#endif
                mCheckLightLib[1]->Init(srcFrame, width, height);
                light = mCheckLightLib[1]->GetLightness();
                BackLighting = mCheckLightLib[1]->GetBackLighting();

            }
        }
        else
        {
            if (LIKELY(NULL != mCheckLightLib[0]))
            {
                mCheckLightLib[0]->Init(srcFrame, width, height);
                light = mCheckLightLib[0]->GetLightness();
                BackLighting = mCheckLightLib[0]->GetBackLighting();
            }

        }

        mCheckLightLock.lock();
        mPreviewLightValueSum += light;
        mPreviewBackLightingValueSum += BackLighting;
        mCheckPreviewCount++;
        mCheckLightLock.unlock();

        mCheckLightWorking = false;
        LOGD("S3_CHECK_PREVIEW_LIGHT CameraHardwareSec::CheckLightThread end mPreviewLightValueSum(%f), mPreviewBackLightingValueSum(%f), mCheckPreviewCount(%d)",
             mPreviewLightValueSum, mPreviewBackLightingValueSum, mCheckPreviewCount);
        return true;
    }
#endif
    //add end by suojp 2012-04-11

#ifdef S3_STATUS_UPDATE
    bool CameraHardwareSec::S3StatusUpdate(void)
    {
        if (UNLIKELY(mExitStatusThread)) {
            return false;
        }
#if 0
        if (UNLIKELY(mCameraID != SecCamera::CAMERA_ID_BACK)) {
            LOGE("ERROR : Status update thread current only for back camera");
            return false;
        }
#endif
        //Sync with FIMC0 if preview is running
        if (mPreviewRunning) {
            mFimcLock.lock();
            mFimcActionCondition.waitRelative(mFimcLock,100000000);
            mFimcLock.unlock();
        }
        else
            usleep(10000);

        //Check Preview Light
#ifdef S3_CHECK_PREVIEW_LIGHT
        if (msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT) && mPreviewRunning)
        {
            int ret;
            //Sync with FIMC1, check light after preview FIMC one shot done
            mCheckLightLock.lock();
            ret = mCheckLightCondition.waitRelative(mCheckLightLock,100000000);
            mCheckLightLock.unlock();

            if(UNLIKELY(ret != 0))
                goto CHECK_LIGHT_END;

            if (!msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT) || !mPreviewRunning)
            {
                goto CHECK_LIGHT_END;
            }
            
            checkLight();

            if (!msgTypeEnabled(CAMERA_MSG_CHECK_LIGHT) || !mPreviewRunning)
            {
                goto CHECK_LIGHT_END;
            }

            mPreviewCount++;
            //if (!mCheckLightWorking)
            {
                if (mPreviewCount > mCheckPreviewStep)
                {
                    if (mCheckPreviewFake > 0)
                    {
                        //callback fake data
                        mNotifyCb(CAMERA_MSG_CHECK_LIGHT, mCheckPreviewFake, 0, mCallbackCookie);
                    }
                    else if (0 < mCheckPreviewCount)
                    {
                        //callback real data
                        double dLightValue = mPreviewLightValueSum / (float)mCheckPreviewCount;
                        double dBackLightValue = mPreviewBackLightingValueSum / (float)mCheckPreviewCount;

                        int light = (int)(dLightValue + 0.5f);
                        int lightBack = (int)(dBackLightValue + 0.5f);

                        LOGD("S3_CHECK_PREVIEW_LIGHT callback mPreviewLightValueSum(%f), mPreviewBackLightingValueSum(%f), "
                             "mCheckPreviewCount(%d), dLightValue(%f), dBackLightValue(%f)",
                             mPreviewLightValueSum, mPreviewBackLightingValueSum,
                             mCheckPreviewCount, dLightValue, dBackLightValue);

                        mNotifyCb(CAMERA_MSG_CHECK_LIGHT, lightBack, light, mCallbackCookie);
                    }

                    mCheckLightLock.lock();
                    mCheckPreviewCount = 0;
                    mPreviewLightValueSum = 0.0f;
                    mPreviewBackLightingValueSum = 0.0f;
                    mCheckLightLock.unlock();
                    mPreviewCount = 0;
                }
                LOGD("S3_CHECK_PREVIEW_LIGHT check one mPreviewCount(%d)", mPreviewCount);
                //mCheckLightCondition.signal();
            }
        }
CHECK_LIGHT_END:
#endif


        //Check FPS, >2 frames update once
#ifdef S3_FPS_DETECTION
        if (msgTypeEnabled(CAMERA_MSG_FPS_DETECTION) && mPreviewRunning
                && !mCaptureInProgress && (mCapture_Cnt > 1))
        {
            float lFps;
            lFps = LOG_TIME(0)/1000;
            mStatusUpdateLock.lock();
            int nFps = (int)(mCapture_Cnt*100000.0f / (float)(lFps));
            mCapture_Cnt = -1;
            mStatusUpdateLock.unlock();
            mNotifyCb(CAMERA_MSG_FPS_DETECTION, nFps, NULL, mCallbackCookie);
        }
#endif

        return true;
    }
#endif

    //add begin by suojp 2012-04-29
#ifdef DEF_WRITE_FILE_TEST
    int CameraHardwareSec::WriteFileTest()
    {
        const char arrtyDir[][20] = {
            "/data",
            "/data/data",
            "/sdcard",
            "/mnt/sdcard",
            "/mnt/extrasd_bin"
        };
        char tmp[30];
        for (int i = 0; i < sizeof(arrtyDir) / sizeof(arrtyDir[0]); i++)
        {
            strcpy(tmp, arrtyDir[i]);
            strcat(tmp, "/test.dat");
            save_file(tmp, "a", 1);
        }
        return 0;
    }
#endif
#ifdef DEF_CONTINUOUS_SHOT_TEST
    int CameraHardwareSec::ContinuousShotTest()
    {
        LOGE("CameraHardwareSec::ContinuousShotTest begin");
        int cap_width, cap_height, cap_frame_size;
        int JpegImageSize = 0;

        mSecCamera->getSnapshotSize(&cap_width, &cap_height, &cap_frame_size);

        int mJpegHeapSize= cap_frame_size;
        camera_memory_t *JpegHeap = mGetMemoryCb(-1, mJpegHeapSize, 1, 0);

        int picture_format = mSecCamera->getSnapshotPixelFormat();

        if (mSecCamera->getSnapshotAddr(mCapIndex, &mCapBuffer) < 0) {
            LOGE("ERR(%s):Fail on SecCamera getCaptureAddr = %0x ", __func__, mCapBuffer.virt.extP[0]);
            goto END;
        }

        for (int i = 1; i < 10001; i++)
        {
            LOGE("suojp loop mContinousShotIndex(%d), wait ...", i);
            if (i > 1)
            {
                mTakePictureLock.lock();
                mTakePictureCondition.wait(mTakePictureLock);
                mCaptureInProgress = true;
                mTakePictureLock.unlock();
            }

            LOGE("suojp wait end, snapshotandjpeg begin");
            usleep(100000);
#if 0		
            if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                               (unsigned char*)JpegHeap->data, &JpegImageSize) < 0) {
                LOGE("suojp getSnapshotAndJpeg error, retry...");
                i--;
                continue;
            }
#endif
            mTakePictureLock.lock();
            mCaptureInProgress = false;
            mStopPictureCondition.signal();
            mTakePictureLock.unlock();
            LOGE("suojp one pictrue end");

        }
END:	
        LOGE("suojp loop end");

        if (JpegHeap) {
            JpegHeap->release(JpegHeap);
            JpegHeap = 0;
        }

        LOGE("%s end", __func__);
        return 0;
    }
#endif

#ifdef S3_CONTINOUS_SHOT
    int CameraHardwareSec::ContinuousShot()
    {
        if (!mUseInternalISP)
        {
            LOGE("ERR(%s):Fail mUseInternalISP", __func__);
            return UNKNOWN_ERROR;
        }

        LOGD("%s begin", __func__);

        int jpeg_size = 0;
        int ret = NO_ERROR;
        unsigned char *jpeg_data = NULL;
        int postview_offset = 0;
        unsigned char *postview_data = NULL;

        unsigned char *addr = NULL;
        int mPostViewWidth, mPostViewHeight, mPostViewSize;
        int mThumbWidth, mThumbHeight, mThumbSize;
        int cap_width, cap_height, cap_frame_size;

        int JpegImageSize = 0;
        int JpegExifSize = 0;
        int mJpegHeapSize_out = 0;
        LOG_TIME_DEFINE(0)

                mExitContinousShotThread = false;

        mContinousSaveFileWorking = false;
        mExitContinousSaveFileThread = false;

        //memset(mstrContinousSaveFilePath, 0, DEF_FILE_PATH_MAX_SIZE);
        mbufContinousSaveFile = (char*) malloc(DEF_JPG_MAX_SIZE);
        mbufContinousSaveFileSize = 0;

        sp<ContinousSaveFileThread> threadSaveFile = new ContinousSaveFileThread(this);
        if (NULL == threadSaveFile.get()) {
            LOGE("ERR(%s):Fail on new  ContinousSaveFileThread", __func__);
            return UNKNOWN_ERROR;
        }

        LOG_TIME_START(0)
                LOGD("%s timer %d start", __func__, 0);

        mSecCamera->runSnapshotFimcOneshot((unsigned int)(mSecCamera->getShareBufferAddr(mCapIndex)));
        mSecCamera->getSnapshotSize(&cap_width, &cap_height, &cap_frame_size);

        int mJpegHeapSize= cap_frame_size;
        camera_memory_t *JpegHeap = mGetMemoryCb(-1, mJpegHeapSize, 1, 0);

        mSecCamera->getThumbnailConfig(&mThumbWidth, &mThumbHeight, &mThumbSize);
        if (mThumbSize > 0)
        {
            mThumbnailHeap = new MemoryHeapBase(mThumbSize);
        }

        camera_memory_t *ExifHeap = mGetMemoryCb(-1, EXIF_FILE_SIZE + mThumbSize, 1, 0);

        int picture_format = mSecCamera->getSnapshotPixelFormat();

        if (mSecCamera->getSnapshotAddr(mCapIndex, &mCapBuffer) < 0) {
            LOGE("ERR(%s):Fail on SecCamera getCaptureAddr = %0x ", __func__, mCapBuffer.virt.extP[0]);
            goto END;
        }

        if (mThumbSize > 0)
        {
            scaleDownYuv422((char *)mCapBuffer.virt.extP[0], cap_width, cap_height,
                    (char *)mThumbnailHeap->base(), mThumbWidth, mThumbHeight);
        }

        LOGD("%s loop begin", __func__);
        mContinousShotIndex = 1;
        LOGD("%s callback begin %d", __func__, mContinousShotIndex);
        mNotifyCb(CAMERA_MSG_CONTINOUS_SHOT, 1, mContinousShotIndex, mCallbackCookie);
        LOGD("%s callback begin %d ok", __func__, mContinousShotIndex);

        for (int i = 1; i < mbufContinousMaxCount+1; i++)
        {
            LOGD("%s loop mContinousShotIndex(%d), wait ...", __func__, i);

            if (i > 1)
            {
                if (mExitContinousShotThread)
                {
                    LOGD("%s break -1", __func__);
                    break;
                }
                if (ret != NO_ERROR)
                {
                    mTakePictureLock.lock();
                    LOGD("%s have error, mCaptureInProgress = false", __func__);
                    mCaptureInProgress = false;
                    //mStopPictureCondition.signal();
                    mTakePictureLock.unlock();
                    i--;

                    if (mExitContinousShotThread)
                    {
                        LOGD("%s break 0", __func__);
                        break;
                    }
                }
                else if (mTimeIntervalMin > 0)
                {
                    LOG_TIME_END(0)
                            long time = LOG_TIME(0);
                    LOGD("suojp timer inteval %d", time);
                    if (time < mTimeIntervalMin * 1000)
                    {
                        LOGD("suojp sleep %d", (mTimeIntervalMin * 1000-time));
                        usleep(mTimeIntervalMin * 1000 - time);
                    }

                    LOGD("%s timer restart, %d picture time = %d", __func__, mContinousShotIndex, time);
                    LOG_TIME_START(0)
                }

                if (mExitContinousShotThread)
                {
                    LOGD("%s break 1", __func__);
                    break;
                }

                mFimcLock.lock();
                LOGD("%s mFimcActionCondition wait", __func__);
                mFimcActionCondition.waitRelative(mFimcLock, 100000000);
                mFimcLock.unlock();

                if (mExitContinousShotThread)
                {
                    LOGD("%s break 2", __func__);
                    break;
                }

                mTakePictureLock.lock();
                LOGD("%s mCaptureInProgress = true", __func__);
                mCaptureInProgress = true;
                mTakePictureLock.unlock();

                LOGD("%s one pictrue begin", __func__);

                if (mContinousShotIndex != i)
                {
                    mContinuousShotLock.lock();
                    LOGD("%s mContinousShotIndex(%d)", __func__, i);
                    mContinousShotIndex = i;
                    mContinuousShotLock.unlock();

                    LOGD("%s callback begin %d", __func__, mContinousShotIndex);
                    mNotifyCb(CAMERA_MSG_CONTINOUS_SHOT, 1, mContinousShotIndex, mCallbackCookie);
                    LOGD("%s callback begin %d ok", __func__, mContinousShotIndex);
                }

                LOGD("%s runSnapshotFimcOneshot", __func__);
                mSecCamera->runSnapshotFimcOneshot((unsigned int)(mSecCamera->getShareBufferAddr(mCapIndex)));
#if 0
                if (mExitContinousShotThread)
                {
                    ret = UNKNOWN_ERROR;
                    LOGD("%s break 3", __func__);
                    mTakePictureLock.lock();
                    LOGD("%s mCaptureInProgress = false", __func__);
                    mCaptureInProgress = false;
                    mTakePictureLock.unlock();
                    break;
                }
#endif
                ret = NO_ERROR;

            }

            LOGD("%s wait end, snapshotandjpeg begin", __func__);
            if (mSecCamera->getSnapshotAndJpeg(&mCapBuffer, mCapIndex,
                                               (unsigned char*)JpegHeap->data, &JpegImageSize) < 0) {
                LOGE("ERR(%s):Fail on getSnapshotAndJpeg, retry...", __func__);
                ret = UNKNOWN_ERROR;
                continue;
            }

            LOGD("%s snapshotandjpeg done", __func__);

            JpegExifSize = mSecCamera->getExif((unsigned char *)ExifHeap->data,
                                               (unsigned char *)mThumbnailHeap->base(),
                                               mThumbSize);

            LOGD("%s getExif done, JpegExifSize=%d", __func__, JpegExifSize);

            if (JpegExifSize < 0) {
                LOGE("ERR(%s):Fail on getExif, retry...", __func__);
                ret = UNKNOWN_ERROR;
                continue;
            }

            LOGD("%s get picture done", __func__);
            if (mContinousSaveFileWorking)
            {
                LOGD("%s wait savefile thread", __func__);
                mContinuousShotLock.lock();
                mContinuousShotSaveCondition.waitRelative(mContinuousShotLock, 800000000);
                mContinuousShotLock.unlock();
                LOGD("%s wait savefile thread OK", __func__);

                if (mContinousSaveFileWorking)
                {
                    LOGE("ERR(%s):Fail on waiting write file thread, retry...", __func__);
                    ret = UNKNOWN_ERROR;
                    continue;
                }
            }

            mbufContinousSaveFileSize = JpegImageSize + JpegExifSize;
            if (mbufContinousSaveFileSize > DEF_JPG_MAX_SIZE)
            {
                LOGE("ERR(%s):Fail on mbufContinousSaveFileSize(%d), retry...", __func__, mbufContinousSaveFileSize);
                ret = UNKNOWN_ERROR;
                continue;
            }

            //memset(mbufContinousSaveFile, 0, DEF_JPG_MAX_SIZE);
            memcpy(mbufContinousSaveFile, JpegHeap->data, 2);
            memcpy(mbufContinousSaveFile+2, ExifHeap->data, JpegExifSize);
            memcpy(mbufContinousSaveFile+2+JpegExifSize, JpegHeap->data + 2, JpegImageSize - 2);

            //save_to_file((char *)JpegHeap_out->data, JpegHeap_out->size);

            mContinuousShotLock.lock();

            LOGD("%s mContinuousShotCondition signal", __func__);
            mContinuousShotCondition.signal();
            mContinuousShotLock.unlock();

            mTakePictureLock.lock();

            LOGD("%s mCaptureInProgress = false", __func__);
            mCaptureInProgress = false;
            mTakePictureLock.unlock();
            LOGD("%s one pictrue end", __func__);

            //sprintf(strFileName, "/data/data/pic/dump_%03d.jpg", i);
            //save_file(strFileName, bufTmp, mJpegHeapSize_out);
            //LOGE("suojp %s : i(%d) end ret = %d", __func__, i,  ret);
        }

        if (ret != NO_ERROR)
        {
            //call back error
            LOGD("%s callback error end %d", __func__, -mContinousShotIndex);
            mNotifyCb(CAMERA_MSG_CONTINOUS_SHOT, 0, -mContinousShotIndex, mCallbackCookie);
            LOGD("%s callback error end %d ok", __func__, -mContinousShotIndex);

        }
END:
        LOGD("%s loop end", __func__);

        mTakePictureLock.lock();
        mStopPictureCondition.signal();
        mTakePictureLock.unlock();
        mContinuousShotLock.lock();
        if (mContinousSaveFileWorking)
        {
            mContinuousShotSaveCondition.waitRelative(mContinuousShotLock, 1000000000);
        }

        mExitContinousSaveFileThread = true;
        mContinuousShotCondition.signal();
        threadSaveFile->requestExit();
        mContinuousShotLock.unlock();


        if (mbufContinousSaveFileSize > 0)
        {
            camera_memory_t *JpegHeap_out = mGetMemoryCb(-1, mbufContinousSaveFileSize, 1, 0);
            memcpy((char *)JpegHeap_out->data, mbufContinousSaveFile, mbufContinousSaveFileSize);

            LOGD("%s callback last frame data", __func__);
            mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, JpegHeap_out, 0, NULL, mCallbackCookie);
            if (mMsgEnabled & CAMERA_MSG_SHUTTER)
                //mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

                if (JpegHeap_out) {
                    JpegHeap_out->release(JpegHeap_out);
                    JpegHeap_out = 0;
                }
        }

        LOGD("%s callback end", __func__);


        if (ExifHeap) {
            ExifHeap->release(ExifHeap);
            ExifHeap = 0;
        }

        if (JpegHeap) {
            JpegHeap->release(JpegHeap);
            JpegHeap = 0;
        }
        /*
        mContinuousShotLock.lock();
        if (mContinousSaveFileWorking)
        {
                mContinuousShotSaveCondition.waitRelative(mContinuousShotLock, 1000000000);
        }

        mExitContinousSaveFileThread = true;
        mContinuousShotCondition.signal();
        threadSaveFile->requestExit();
        mContinuousShotLock.unlock();
        */
        if (NULL != mbufContinousSaveFile)
        {
            free(mbufContinousSaveFile);
            mbufContinousSaveFile = NULL;
        }

        threadSaveFile->requestExitAndWait();
        threadSaveFile.clear();

        mExitContinousShotThread = true;

        LOGD("%s : pictureThread exit", __func__);
        return ret;
    }

    void CameraHardwareSec::ContinousSaveFile()
    {
        LOGD("%s start", __func__);
        FILE *yuv_fp = NULL;
        char strFilePath[DEF_FILE_PATH_MAX_SIZE+10] = "";
        int nIndex = 0;
        while(true)
        {
            mContinuousShotLock.lock();
            if (mExitContinousSaveFileThread)
            {
                LOGD("%s, break 1", __func__);
                mContinuousShotLock.unlock();
                break;
            }

            LOGD("%s, mContinuousShotCondition wait", __func__);
            mContinuousShotCondition.waitRelative(mContinuousShotLock, 1000000000);
            if (mExitContinousSaveFileThread)
            {
                LOGD("%s, break 2", __func__);
                mContinuousShotLock.unlock();
                break;
            }

            LOGD("%s, mContinuousShotCondition wait ok, begin %d save", __func__, mContinousShotIndex);
            mContinousSaveFileWorking = true;
            nIndex = mContinousShotIndex;
            sprintf(strFilePath, mstrContinousSaveFilePath, nIndex);
            mContinuousShotLock.unlock();

            LOGD("%s write file %d begin", __func__, nIndex);
            yuv_fp = fopen(strFilePath, "wb");
            if (yuv_fp == NULL) {
                LOGE("%s ContinousSaveFileThread Save test file open failure~", __func__);
                mContinuousShotLock.lock();
                mContinousSaveFileWorking = false;
                mContinuousShotSaveCondition.signal();
                mContinuousShotLock.unlock();

                LOGD("%s callback error end %d", __func__, -nIndex);
                mNotifyCb(CAMERA_MSG_CONTINOUS_SHOT, 0, -nIndex, mCallbackCookie);
                LOGD("%s callback error end %d ok", __func__, -nIndex);

                continue;
            }

            fwrite(mbufContinousSaveFile, 1, mbufContinousSaveFileSize, yuv_fp);

            fflush(yuv_fp);

            if (yuv_fp)
            {
                fclose(yuv_fp);
            }

            yuv_fp = NULL;
            LOGD("%s write file %d end", __func__, nIndex);

            mContinuousShotLock.lock();
            mContinousSaveFileWorking = false;
            mContinuousShotSaveCondition.signal();
            mContinuousShotLock.unlock();

            LOGD("%s callback end %d", __func__, nIndex);
            mNotifyCb(CAMERA_MSG_CONTINOUS_SHOT, 0, nIndex, mCallbackCookie);
            LOGD("%s callback end %d ok", __func__, nIndex);
        }

        LOGD("%s, ContinousSaveFileThread exit", __func__);
    }
#endif
    //add end by suojp 2012-04-29

    //add begin by suojp 2012-05-11
#ifdef S3_LOSSFOCUS_NOTIFY
    bool CameraHardwareSec::LossFocusNotify(int focus)
    {
        LOGD("%s focus(%d) start...", __func__, focus);

        int ret = mFocusSave;
        int nStatus = 0;
        while (true)
        {
            if (UNLIKELY(mExitLossFocusNotifyThread))
            {
                LOGD("%s break 0, nStatus=%d ", __func__, nStatus);
                break;
            }

            if (NULL == mLossFocusNotifyThread.get())
            {
                LOGI("%s break 1, nStatus=%d ", __func__, nStatus);
                break;
            }

            if (!msgTypeEnabled(CAMERA_MSG_CHECK_LOSS_FOCUS))
            {
                LOGI("%s break 2, nStatus=%d ", __func__, nStatus);
                break;
            }
#if 0
            if (UNLIKELY(mCameraID != SecCamera::CAMERA_ID_BACK))
            {
                LOGI("%s break 3, nStatus=%d ", __func__, nStatus);
                break;
            }
#endif
            if (!mPreviewRunning)
            {
                LOGI("%s break 4, nStatus=%d ", __func__, nStatus);
                break;
            }

            switch(nStatus)
            {
            case 0:
                ret = mSecCamera->getAfLostState();
                if (ret > 1 && ret < 5)
                {
                    //prepare call back
                    nStatus = 1;
                }
                else
                {
                    LOGI("%s AfLostPoll ret=%d, retry...", __func__, ret);
                    //to sleep and retry
                    nStatus = 2;
                }
                break;
            case 1:
                if (mFocusSave != ret)
                {
                    mFocusSave = ret;
                    LOGD("%s call back to app ret=%d", __func__, ret);
                    mNotifyCb(CAMERA_MSG_CHECK_LOSS_FOCUS, ret, 0, mCallbackCookie);
                }

                // to sleep and retry
                nStatus = 2;
                break;
            case 2:
            {
                int i;
                for (i = 20; i>0;i--) {
                    //wait every 5ms to check exit.
                    if (mExitLossFocusNotifyThread
                            || NULL == mLossFocusNotifyThread.get()
                            || (!msgTypeEnabled(CAMERA_MSG_CHECK_LOSS_FOCUS)))
                        break;

                    usleep(5000);
                }
            }

                //retry
                nStatus = 0;
                break;
            default:
                break;
                
            }
            //LOGE("limeng %s AfLostPoll() ret(%d)", __func__, ret);
        }

        LOGD("%s focus(%d) exit..., nStatus=%d", __func__, focus, nStatus);

        return false;
    }
#endif
    //add end by suojp 2012-05-11

#ifdef S3_TORCH_SCENARIO //limeng 120727
    int CameraHardwareSec::setTorchScenario()
    {
        int torch_mode;
        int ret = NO_ERROR;

        if(mRecordRunning)
            torch_mode = TORCH_MODE_RECORD;
        else if(mCaptureInProgress)
            torch_mode = TORCH_MODE_PICTURE;
        else
            torch_mode = TORCH_MODE_FLASHLIGHT;

        ret = mSecCamera->setTorchMode(torch_mode);

        return 0;

    }
#endif


    static CameraInfo sCameraInfo[] = {
        {
            CAMERA_FACING_BACK,
            90,  /* orientation */
        },
        {
            CAMERA_FACING_FRONT,
            270,  /* orientation */
        }
    };

    status_t CameraHardwareSec::storeMetaDataInBuffers(bool enable)
    {
        // FIXME:
        // metadata buffer mode can be turned on or off.
        // Samsung needs to fix this.
        if (!enable) {
            LOGE("Non-metadata buffer mode is not supported!");
            return INVALID_OPERATION;
        }
        return OK;
    }

    /** Close this device */

    static camera_device_t *g_cam_device;

    static int HAL_camera_device_close(struct hw_device_t* device)
    {
        LOGI("%s", __func__);
        if (device) {
            camera_device_t *cam_device = (camera_device_t *)device;
            delete static_cast<CameraHardwareSec *>(cam_device->priv);
            free(cam_device);
            g_cam_device = 0;
        }
        return 0;
    }

    static inline CameraHardwareSec *obj(struct camera_device *dev)
    {
        return reinterpret_cast<CameraHardwareSec *>(dev->priv);
    }

    /** Set the preview_stream_ops to which preview frames are sent */
    static int HAL_camera_device_set_preview_window(struct camera_device *dev,
                                                    struct preview_stream_ops *buf)
    {
        LOGD("%s", __func__);
        return obj(dev)->setPreviewWindow(buf);
    }

    /** Set the notification and data callbacks */
    static void HAL_camera_device_set_callbacks(struct camera_device *dev,
                                                camera_notify_callback notify_cb,
                                                camera_data_callback data_cb,
                                                camera_data_timestamp_callback data_cb_timestamp,
                                                camera_request_memory get_memory,
                                                void* user)
    {
        LOGD("%s", __func__);
        obj(dev)->setCallbacks(notify_cb, data_cb, data_cb_timestamp,
                               get_memory,
                               user);
    }

    /**
 * The following three functions all take a msg_type, which is a bitmask of
 * the messages defined in include/ui/Camera.h
 */

    /**
 * Enable a message, or set of messages.
 */
    static void HAL_camera_device_enable_msg_type(struct camera_device *dev, int32_t msg_type)
    {
        LOGD("%s", __func__);
        obj(dev)->enableMsgType(msg_type);
    }

    /**
 * Disable a message, or a set of messages.
 *
 * Once received a call to disableMsgType(CAMERA_MSG_VIDEO_FRAME), camera
 * HAL should not rely on its client to call releaseRecordingFrame() to
 * release video recording frames sent out by the cameral HAL before and
 * after the disableMsgType(CAMERA_MSG_VIDEO_FRAME) call. Camera HAL
 * clients must not modify/access any video recording frame after calling
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME).
 */
    static void HAL_camera_device_disable_msg_type(struct camera_device *dev, int32_t msg_type)
    {
        LOGD("%s", __func__);
        obj(dev)->disableMsgType(msg_type);
    }

    /**
 * Query whether a message, or a set of messages, is enabled.  Note that
 * this is operates as an AND, if any of the messages queried are off, this
 * will return false.
 */
    static int HAL_camera_device_msg_type_enabled(struct camera_device *dev, int32_t msg_type)
    {
        LOGD("%s", __func__);
        return obj(dev)->msgTypeEnabled(msg_type);
    }

    /**
 * Start preview mode.
 */
    static int HAL_camera_device_start_preview(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->startPreview();
    }

    /**
 * Stop a previously started preview.
 */
    static void HAL_camera_device_stop_preview(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        obj(dev)->stopPreview();
    }

    /**
 * Returns true if preview is enabled.
 */
    static int HAL_camera_device_preview_enabled(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->previewEnabled();
    }

    /**
 * Request the camera HAL to store meta data or real YUV data in the video
 * buffers sent out via CAMERA_MSG_VIDEO_FRAME for a recording session. If
 * it is not called, the default camera HAL behavior is to store real YUV
 * data in the video buffers.
 *
 * This method should be called before startRecording() in order to be
 * effective.
 *
 * If meta data is stored in the video buffers, it is up to the receiver of
 * the video buffers to interpret the contents and to find the actual frame
 * data with the help of the meta data in the buffer. How this is done is
 * outside of the scope of this method.
 *
 * Some camera HALs may not support storing meta data in the video buffers,
 * but all camera HALs should support storing real YUV data in the video
 * buffers. If the camera HAL does not support storing the meta data in the
 * video buffers when it is requested to do do, INVALID_OPERATION must be
 * returned. It is very useful for the camera HAL to pass meta data rather
 * than the actual frame data directly to the video encoder, since the
 * amount of the uncompressed frame data can be very large if video size is
 * large.
 *
 * @param enable if true to instruct the camera HAL to store
 *      meta data in the video buffers; false to instruct
 *      the camera HAL to store real YUV data in the video
 *      buffers.
 *
 * @return OK on success.
 */
    static int HAL_camera_device_store_meta_data_in_buffers(struct camera_device *dev, int enable)
    {
        LOGD("%s", __func__);
        return obj(dev)->storeMetaDataInBuffers(enable);
    }

    /**
 * Start record mode. When a record image is available, a
 * CAMERA_MSG_VIDEO_FRAME message is sent with the corresponding
 * frame. Every record frame must be released by a camera HAL client via
 * releaseRecordingFrame() before the client calls
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME). After the client calls
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
 * responsibility to manage the life-cycle of the video recording frames,
 * and the client must not modify/access any video recording frames.
 */
    static int HAL_camera_device_start_recording(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->startRecording();
    }

    /**
 * Stop a previously started recording.
 */
    static void HAL_camera_device_stop_recording(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        obj(dev)->stopRecording();
    }

    /**
 * Returns true if recording is enabled.
 */
    static int HAL_camera_device_recording_enabled(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->recordingEnabled();
    }

    /**
 * Release a record frame previously returned by CAMERA_MSG_VIDEO_FRAME.
 *
 * It is camera HAL client's responsibility to release video recording
 * frames sent out by the camera HAL before the camera HAL receives a call
 * to disableMsgType(CAMERA_MSG_VIDEO_FRAME). After it receives the call to
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
 * responsibility to manage the life-cycle of the video recording frames.
 */
    static void HAL_camera_device_release_recording_frame(struct camera_device *dev,
                                                          const void *opaque)
    {
        LOGD("%s", __func__);
        obj(dev)->releaseRecordingFrame(opaque);
    }

    /**
 * Start auto focus, the notification callback routine is called with
 * CAMERA_MSG_FOCUS once when focusing is complete. autoFocus() will be
 * called again if another auto focus is needed.
 */
    static int HAL_camera_device_auto_focus(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->autoFocus();
    }

    /**
 * Cancels auto-focus function. If the auto-focus is still in progress,
 * this function will cancel it. Whether the auto-focus is in progress or
 * not, this function will return the focus position to the default.  If
 * the camera does not support auto-focus, this is a no-op.
 */
    static int HAL_camera_device_cancel_auto_focus(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->cancelAutoFocus();
    }

    /**
 * Take a picture.
 */
    static int HAL_camera_device_take_picture(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->takePicture();
    }

    /**
 * Cancel a picture that was started with takePicture. Calling this method
 * when no picture is being taken is a no-op.
 */
    static int HAL_camera_device_cancel_picture(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        return obj(dev)->cancelPicture();
    }

    /**
 * Set the camera parameters. This returns BAD_VALUE if any parameter is
 * invalid or not supported.
 */
    static int HAL_camera_device_set_parameters(struct camera_device *dev,
                                                const char *parms)
    {
        LOGD("%s", __func__);

        //add   by dg for debug ov5640
        LOGD("%s", parms);

        String8 str(parms);
        CameraParameters p(str);
        return obj(dev)->setParameters(p);
    }

    /** Return the camera parameters. */
    char *HAL_camera_device_get_parameters(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        String8 str;
        CameraParameters parms = obj(dev)->getParameters();
        str = parms.flatten();
        return strdup(str.string());
    }

    static void HAL_camera_device_put_parameters(struct camera_device *dev, char *parms)
    {
        LOGD("%s", __func__);
        free(parms);
    }

    /**
 * Send command to camera driver.
 */
    static int HAL_camera_device_send_command(struct camera_device *dev,
                                              int32_t cmd, int32_t arg1, int32_t arg2)
    {
        LOGD("%s", __func__);
        return obj(dev)->sendCommand(cmd, arg1, arg2);
    }

    /**
 * Release the hardware resources owned by this object.  Note that this is
 * *not* done in the destructor.
 */
    static void HAL_camera_device_release(struct camera_device *dev)
    {
        LOGD("%s", __func__);
        // add start by shaking.wan 2012-11-07.For LockScreen return Camera APP.Samsung patch
        property_set("debug.media.kgcamera", "false");
        // add start by shaking.wan 2012-11-07.For LockScreen return Camera APP.Samsung patch
        obj(dev)->release();
    }

    /**
 * Dump state of the camera hardware
 */
    static int HAL_camera_device_dump(struct camera_device *dev, int fd)
    {
        LOGD("%s", __func__);
        return obj(dev)->dump(fd);
    }

    static int HAL_getNumberOfCameras()
    {
        LOGD("%s", __func__);

        int cam_fd;
        static struct v4l2_input input;

        cam_fd = open(CAMERA_DEV_NAME, O_RDONLY);
        if (cam_fd < 0) {
            LOGE("ERR(%s):Cannot open %s (error : %s)", __func__, CAMERA_DEV_NAME, strerror(errno));
            return -1;
        }

        input.index = 0;
        while (ioctl(cam_fd, VIDIOC_ENUMINPUT, &input) == 0) {
            LOGI("Name of input channel[%d] is %s", input.index, input.name);
            input.index++;
        }

        close(cam_fd);

        return input.index-1;
    }

    static int HAL_getCameraInfo(int cameraId, struct camera_info *cameraInfo)
    {
        LOGD("%s", __func__);
        memcpy(cameraInfo, &sCameraInfo[cameraId], sizeof(CameraInfo));
        return 0;
    }

#define SET_METHOD(m) m : HAL_camera_device_##m

    static camera_device_ops_t camera_device_ops = {
        SET_METHOD(set_preview_window),
        SET_METHOD(set_callbacks),
        SET_METHOD(enable_msg_type),
        SET_METHOD(disable_msg_type),
        SET_METHOD(msg_type_enabled),
        SET_METHOD(start_preview),
        SET_METHOD(stop_preview),
        SET_METHOD(preview_enabled),
        SET_METHOD(store_meta_data_in_buffers),
        SET_METHOD(start_recording),
        SET_METHOD(stop_recording),
        SET_METHOD(recording_enabled),
        SET_METHOD(release_recording_frame),
        SET_METHOD(auto_focus),
        SET_METHOD(cancel_auto_focus),
        SET_METHOD(take_picture),
        SET_METHOD(cancel_picture),
        SET_METHOD(set_parameters),
        SET_METHOD(get_parameters),
        SET_METHOD(put_parameters),
        SET_METHOD(send_command),
        SET_METHOD(release),
        SET_METHOD(dump),
    };

#undef SET_METHOD

    static int HAL_camera_device_open(const struct hw_module_t* module,
                                      const char *id,
                                      struct hw_device_t** device)
    {
        LOGD("%s", __func__);

        // add start by shaking.wan 2012-11-07.For LockScreen return Camera APP.Samsung patch.
        property_set("debug.media.kgcamera", "true");
        // add end by shaking.wan 2012-11-07.For LockScreen return Camera APP.Samsung patch.

        int cameraId = atoi(id);
        if (cameraId < 0 || cameraId >= HAL_getNumberOfCameras()) {
            LOGE("Invalid camera ID %s", id);
            return -EINVAL;
        }

        if (g_cam_device) {
            if (obj(g_cam_device)->getCameraId() == cameraId) {
                LOGD("returning existing camera ID %s", id);
                goto done;
            } else {
                LOGE("Cannot open camera %d. camera %d is already running!",
                     cameraId, obj(g_cam_device)->getCameraId());
                return -ENOSYS;
            }
        }

        g_cam_device = (camera_device_t *)malloc(sizeof(camera_device_t));
        if (!g_cam_device)
            return -ENOMEM;

        g_cam_device->common.tag     = HARDWARE_DEVICE_TAG;
        g_cam_device->common.version = 1;
        g_cam_device->common.module  = const_cast<hw_module_t *>(module);
        g_cam_device->common.close   = HAL_camera_device_close;

        g_cam_device->ops = &camera_device_ops;

        LOGI("%s: open camera %s", __func__, id);

        g_cam_device->priv = new CameraHardwareSec(cameraId, g_cam_device);

done:
        *device = (hw_device_t *)g_cam_device;
        LOGI("%s: opened camera %s (%p)", __func__, id, *device);
        return 0;
    }

    static hw_module_methods_t camera_module_methods = {
        open : HAL_camera_device_open
    };

    extern "C" {
    struct camera_module HAL_MODULE_INFO_SYM = {
        common : {
        tag           : HARDWARE_MODULE_TAG,
                version_major : 1,
            version_minor : 0,
            id            : CAMERA_HARDWARE_MODULE_ID,
            name          : "orion camera HAL",
            author        : "Samsung Corporation",
            methods       : &camera_module_methods,
    },
            get_number_of_cameras : HAL_getNumberOfCameras,
            get_camera_info       : HAL_getCameraInfo
    };
    }

    }; // namespace android
