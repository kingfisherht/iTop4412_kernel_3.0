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

#ifndef ANDROID_HARDWARE_CAMERA_HARDWARE_SEC_H  
#define ANDROID_HARDWARE_CAMERA_HARDWARE_SEC_H

#include "SecCamera_zoom.h"
#include <utils/threads.h>
#include <utils/RefBase.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <hardware/camera.h>
#include <hardware/gralloc.h>
#include <camera/CameraParameters.h>
#ifdef BOARD_USE_V4L2_ION
#include <binder/MemoryHeapBaseIon.h>
#include "gralloc_priv.h"

#define  BUFFER_COUNT_FOR_GRALLOC (MAX_BUFFERS + 4)
#define  BUFFER_COUNT_FOR_ARRAY (MAX_BUFFERS)
#else
#define  BUFFER_COUNT_FOR_GRALLOC (MAX_BUFFERS)
#define  BUFFER_COUNT_FOR_ARRAY (1)
#endif

#define S3_TORCH_SCENARIO //limeng 120727

//#define S3_CHECK_PREVIEW_LIGHT //add by suojp 2012-04-14
#define S3_CONTINOUS_SHOT	  //add by suojp 2012-04-29
//#define DEF_CONTINUOUS_SHOT_TEST	  //add by suojp 2012-04-29
//#define DEF_WRITE_FILE_TEST		  //add by suojp 2012-04-29
#define S3_BLINK_DETECTION	  //add by suojp 2012-05-05
//#define S3_FPS_DETECTION	  //add by suojp 2012-05-10
//#define S3_LOSSFOCUS_NOTIFY	  //add by suojp 2012-05-11
#define S3_IGNORE_PREVIEW_WINDOW //add by suojp 2012-06-18
#if defined(S3_CHECK_PREVIEW_LIGHT) || defined(S3_FPS_DETECTION)
/*Use status update thread to callback the fps light infor. Jiangshanbin 2012/06/19*/
#define S3_STATUS_UPDATE
#endif

#ifdef S3_CHECK_PREVIEW_LIGHT
#define	DEF_PREVIEW_MAX_SIZE	(1280*72*20)
#include "LeImageLightness.h"
#endif

#ifdef S3_CONTINOUS_SHOT
#define DEF_FILE_PATH_MAX_SIZE 	150
#define DEF_JPG_MAX_SIZE		(1024*1024*5)	//max size 5M

#endif

#ifdef SCALADO_SCF
#define  SCF_BURST_SHOT_MAX  (5)                      //the max number of burst shot
#endif

namespace android {
    class CameraHardwareSec : public virtual RefBase {
public:
    virtual void        setCallbacks(camera_notify_callback notify_cb,
                                     camera_data_callback data_cb,
                                     camera_data_timestamp_callback data_cb_timestamp,
                                     camera_request_memory get_memory,
                                     void *user);

    virtual void        enableMsgType(int32_t msgType);
    virtual void        disableMsgType(int32_t msgType);
    virtual bool        msgTypeEnabled(int32_t msgType);

    virtual status_t    startPreview();
    virtual void        stopPreview();
    virtual bool        previewEnabled();

    virtual status_t    startRecording();
    virtual void        stopRecording();
    virtual bool        recordingEnabled();
    virtual void        releaseRecordingFrame(const void *opaque);

    virtual status_t    autoFocus();
    virtual status_t    cancelAutoFocus();
    virtual status_t    takePicture();
    virtual status_t    cancelPicture();
    virtual status_t    dump(int fd) const;
    virtual status_t    setParameters(const CameraParameters& params);
    virtual CameraParameters  getParameters() const;
    virtual status_t    sendCommand(int32_t command, int32_t arg1, int32_t arg2);
    virtual status_t    setPreviewWindow(preview_stream_ops *w);
    virtual status_t    storeMetaDataInBuffers(bool enable);
    virtual void        release();

    inline  int         getCameraId() const;

    CameraHardwareSec(int cameraId, camera_device_t *dev);
    virtual             ~CameraHardwareSec();
private:
    status_t    startPreviewInternal();
    void stopPreviewInternal();

    class PreviewThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        PreviewThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);
        }
        virtual bool threadLoop() {
            mHardware->previewThreadWrapper();
            return false;
        }
    };

    class PreviewFimcThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        PreviewFimcThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
           // run("CameraPreviewThread", PRIORITY_DISPLAY);
        }
        virtual bool threadLoop() {
            return mHardware->previewFimcThread();
        }
    };

    class RecordFimcThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        RecordFimcThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
           // run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);
        }
        virtual bool threadLoop() {
            return mHardware->recordFimcThread();
        }
    };
/*for Ov record thread add by jijun.yu 0803*/
    class RecordThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        RecordThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);////CameraPreviewThread //CameraRecordThread
        }
        virtual bool threadLoop() {
            return mHardware->recordThread();
        }
    };	

    class BurstshotFimcThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        BurstshotFimcThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);
        }
        virtual bool threadLoop() {
            return mHardware->burstshotFimcThread();
        }
    };
	

    class CallbackThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        CallbackThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("CameraPreviewThread", PRIORITY_URGENT_DISPLAY);
        }
        virtual bool threadLoop() {
            return mHardware->callbackThread();
        }
    };

    class PictureThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        PictureThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual bool threadLoop() {
            mHardware->pictureThread();
            return false;
        }
    };

     class BurstpictureThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        BurstpictureThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual bool threadLoop() {
            mHardware->burstpictureThread();
            return false;
        }
    };
	
    class AutoFocusThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        AutoFocusThread(CameraHardwareSec *hw): Thread(false), mHardware(hw) { }
        virtual void onFirstRef() {
            run("CameraAutoFocusThread", PRIORITY_DEFAULT);
        }
        virtual bool threadLoop() {
            mHardware->autoFocusThread();
            return true;
        }
    };

#ifdef IS_FW_DEBUG
    class DebugThread : public Thread {
        CameraHardwareSec *mHardware;
    public:
        DebugThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual bool threadLoop() {
            return mHardware->debugThread();
        }
    };
#endif

            void        initDefaultParameters(int cameraId);
            void        initHeapLocked();

            int         mRunningThread;
    sp<PreviewThread>   mPreviewThread;
            int         previewThread();
            int         previewThreadForZoom();
            int         previewThreadWrapper();

    sp<PreviewFimcThread>   mPreviewFimcThread;
            int         previewFimcThread();

    sp<RecordFimcThread>   mRecordFimcThread;
            int         recordFimcThread();
/*for ov camera record  thread add by jijun 0803*/
    sp<RecordThread>   mRecordThread;
            int         recordThread();			

    sp<BurstshotFimcThread>   mBurstshotFimcThread ;
            int         burstshotFimcThread ();			

    sp<CallbackThread>   mCallbackThread;
            int         callbackThread();

    sp<AutoFocusThread> mAutoFocusThread;
            int         autoFocusThread();

    sp<PictureThread>   mPictureThread;
            int         pictureThread();
			
   sp<BurstpictureThread>   mBurstpictureThread;		
	    int         burstpictureThread();
		
            bool        mCaptureInProgress;
	    bool        mburstencodeflag;
	    bool        mburstbufferflag;
	    bool        mbursthdrflag;

#ifdef IS_FW_DEBUG
    sp<DebugThread>     mDebugThread;
            bool        debugThread();
            bool        mDebugRunning;
            int         mPrevOffset;
            int         mCurrOffset;
            int         mPrevWp;
            int         mCurrWp;
   unsigned int         mDebugVaddr;
            bool        mStopDebugging;
    mutable Mutex       mDebugLock;
    mutable Condition   mDebugCondition;
#endif

            int         save_jpeg(unsigned char *real_jpeg, int jpeg_size);
            void        save_postview(const char *fname, uint8_t *buf,
                                        uint32_t size);
            int         decodeInterleaveData(unsigned char *pInterleaveData,
                                                int interleaveDataSize,
                                                int yuvWidth,
                                                int yuvHeight,
                                                int *pJpegSize,
                                                void *pJpegData,
                                                void *pYuvData);
            bool        YUY2toNV21(void *srcBuf, void *dstBuf, uint32_t srcWidth, uint32_t srcHeight);
            bool        scaleDownYuv422(char *srcBuf, uint32_t srcWidth,
                                        uint32_t srcHight, char *dstBuf,
                                        uint32_t dstWidth, uint32_t dstHight);
            bool        scaleDownYuv420sp(struct SecBuffer *srcBuf, uint32_t srcWidth,
                                        uint32_t srcHeight, char *dstBuf,
                                        uint32_t dstWidth, uint32_t dstHeight);

            bool        fileDump(char *filename, void *srcBuf, uint32_t size);
            bool        CheckVideoStartMarker(unsigned char *pBuf);
            bool        CheckEOIMarker(unsigned char *pBuf);
            bool        FindEOIMarkerInJPEG(unsigned char *pBuf,
                                            int dwBufSize, int *pnJPEGsize);
            bool        SplitFrame(unsigned char *pFrame, int dwSize,
                                   int dwJPEGLineLength, int dwVideoLineLength,
                                   int dwVideoHeight, void *pJPEG,
                                   int *pdwJPEGSize, void *pVideo,
                                   int *pdwVideoSize);
            void        setSkipFrame(int frame);
            bool        isSupportedPreviewSize(const int width,
                                               const int height) const;
            bool        getVideosnapshotSize(int *width, int *height);
            bool  	     getPreviewSize(int pic_width, int pic_height, int *preview_width, int *preview_height);
            int         bracketsStr2Ints(char *str, int num, ExynosRect *rects, int *weights);
			int         m_bracketsStr2Ints(char *str, int num, ExynosRect *rects, int *weights);
            bool        subBracketsStr2Ints(int num, char *str, int *arr);
            bool        meteringAxisTrans(ExynosRect *rects, ExynosRect2 *rect2s, int num);
    /* used by auto focus thread to block until it's told to run */
    mutable Mutex       mFocusLock;
    mutable Condition   mFocusCondition;
            bool        mExitAutoFocusThread;
            int         mTouched;

    /* used by preview thread to block until it's told to run */
    mutable Mutex       mPreviewLock;
    mutable Condition   mPreviewCondition;
    mutable Condition   mPreviewStoppedCondition;
    bool        mPreviewRunning;
    bool        mPreviewStartDeferred;
    bool        mExitPreviewThread;
//add begin by suojp 2012-06-06
#ifdef SCALADO_SCF			
    int mBurstCapture;
    int mBracketingEx[SCF_BURST_SHOT_MAX];	
    int mExposureCounter;
    bool mIsBracketEV;
    bool    mBreakBurstFimcThread;	
#endif
//add end by suojp 2012-06-06			
    mutable Mutex       mFimcLock;
    mutable Condition   mFimcStoppedCondition;
    mutable Condition   mFimcActionCondition;
    mutable Mutex       mCallbackLock;
    mutable Condition   mCallbackCondition;
    mutable Mutex       mTakePictureLock;
    mutable Condition   mTakePictureCondition;
    mutable Condition   mStopPictureCondition;
    mutable Mutex       mBurstPictureLock;
    mutable Condition   mBurstPictureCondition;
    mutable Condition   mStopBurstCondition;	
    mutable Condition   mBurstFirstCondition;

#ifdef USE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
    mutable Mutex       mPreviewFimcLock;
#endif

            preview_stream_ops *mPreviewWindow;

    /* used to guard threading state */
    mutable Mutex       mStateLock;

    mutable Mutex       mOvrecStateLock;
	

    enum PREVIEW_FMT {
        PREVIEW_FMT_1_PLANE = 0,
        PREVIEW_FMT_2_PLANE,
        PREVIEW_FMT_3_PLANE,
    };

            int         mPreviewFmtPlane;

    CameraParameters    mParameters;
    CameraParameters    mInternalParameters;

    int                 mFrameSizeDelta;
    camera_memory_t     *mPreviewHeap;
    camera_memory_t     *mRawHeap;
    sp<MemoryHeapBase>  mPostviewHeap[CAP_BUFFERS];
    sp<MemoryHeapBase>  mThumbnailHeap;
    camera_memory_t     *mRecordHeap[BUFFER_COUNT_FOR_ARRAY];

    camera_frame_metadata_t     mFaceData;
    camera_face_t               mFaces[CAMERA_MAX_FACES];
    camera_frame_metadata_t     mFaceMetaData;
    camera_face_t               mMetaFaces[CAMERA_MAX_FACES];
    camera_memory_t     *mFaceDataHeap;

    buffer_handle_t *mBufferHandle[BUFFER_COUNT_FOR_ARRAY];
    int mStride[BUFFER_COUNT_FOR_ARRAY];


    SecCamera           *mSecCamera;
            const __u8  *mCameraSensorName;
            bool        mUseInternalISP;

    mutable Mutex       mSkipFrameLock;
            int         mSkipFrame;

    camera_notify_callback     mNotifyCb;
    camera_data_callback       mDataCb;
    camera_data_timestamp_callback mDataCbTimestamp;
    camera_request_memory      mGetMemoryCb;
    void   *mCallbackCookie;

            int32_t     mMsgEnabled;

            int         mRunningSetParam;
            int         mFDcount;
            bool        mRecordRunning;
            bool        mRecordHint;
    mutable Mutex       mRecordLock;
            int         mPostViewWidth;
            int         mPostViewHeight;
            int         mPostViewSize;
     struct SecBuffer   mCapBuffer;
            int         mCapIndex;
            int         mCurrentIndex;
	    int         mBurststartIndex;
            int         mOldRecordIndex;
            int         mOldPreviewIndex;
	    int         mburst_index[CAP_BUFFERS];
	    int         mhdr_index[3];
	    int         mOldBurstIndex;
	    int         mhdr_count;
            int         mCameraID;
            Vector<Size> mSupportedPreviewSizes;

    struct is_debug_control {
        unsigned int write_point;        /* 0 ~ 500KB boundary */
        unsigned int assert_flag;        /* 0: Not invoked, 1: Invoked */
        unsigned int pabort_flag;        /* 0: Not invoked, 1: Invoked */
        unsigned int dabort_flag;        /* 0: Not invoked, 1: Invoked */
    };
    struct is_debug_control mIs_debug_ctrl;

    camera_device_t *mHalDevice;
    static gralloc_module_t const* mGrallocHal;
	//add begin by suojp 2012-04-11
#ifdef S3_CHECK_PREVIEW_LIGHT	
	unsigned int mCheckPreviewStep;	
	unsigned int mPreviewCount;
	float mPreviewLightValueSum;
	unsigned int mCheckPreviewCount;
	unsigned int mCheckPreviewFake;
	bool checkLight(void);	
	mutable Mutex       mCheckLightLock;
	mutable Condition   mCheckLightCondition;	
	bool        		mCheckLightWorking;
	ILeImageLightnessEstmater *mCheckLightLib[2];
	int					mPreviewFormat;
	float 				mPreviewBackLightingValueSum;	
#endif	

#ifdef S3_STATUS_UPDATE	
    class S3StatusThread : public Thread {
		CameraHardwareSec *mHardware;
    public:
        S3StatusThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("StatusUpdateThread", PRIORITY_LESS_FAVORABLE);
        }
        virtual bool threadLoop() {
            return mHardware->S3StatusUpdate();
        }
    };	
    sp<S3StatusThread>   mStatusUpdateThread;
    bool S3StatusUpdate(void);
    bool        		mExitStatusThread;
    mutable Mutex       mStatusUpdateLock;
#endif 
	int save_to_file(char *real_buf, int buf_size);
	int save_to_file(char *filename, char *real_buf, int buf_size);
	int save_file(char *filename, char *real_buf, int buf_size);
	//add end by suojp 2012-04-11

	//add begin by suojp 2012-04-29
#ifdef S3_CONTINOUS_SHOT
	int ContinuousShot();
	class ContinousSaveFileThread : public Thread {
		CameraHardwareSec *mHardware;
    public:
        ContinousSaveFileThread(CameraHardwareSec *hw):
        Thread(false),
        mHardware(hw) { }
        virtual void onFirstRef() {
            run("ContinousSaveFileThread", PRIORITY_URGENT_DISPLAY);
        }
        virtual bool threadLoop() {
			mHardware->ContinousSaveFile();
            return false;
        }
    };	

	void ContinousSaveFile();
	char 				mstrContinousSaveFilePath[DEF_FILE_PATH_MAX_SIZE];
	int 				mContinousShotIndex;	
	char 				*mbufContinousSaveFile;
	int 				mbufContinousSaveFileSize;	
	bool        		mExitContinousSaveFileThread;	
	bool        		mExitContinousShotThread;	
	bool        		mContinousSaveFileWorking;	
    mutable Condition   mContinuousShotSaveCondition;
    mutable Condition   mContinuousShotCondition;
	mutable Mutex       mContinuousShotLock;
	int 				mbufContinousMaxCount;	
	int 				mTimeIntervalMin;
#endif	

#ifdef DEF_CONTINUOUS_SHOT_TEST
	int ContinuousShotTest();
#endif


#ifdef DEF_WRITE_FILE_TEST
	int WriteFileTest();
#endif
	//add end by suojp 2012-04-29
	
	//add begin by suojp 2012-05-05
#ifdef S3_BLINK_DETECTION
	int 				mBlinkLevelMax;
	int 				mBlinkCheckCount;
#endif
	//add end by suojp 2012-05-05
	//add begin by suojp 2012-05-11
#ifdef S3_LOSSFOCUS_NOTIFY
	bool LossFocusNotify(int focus);

	class LossFocusNotifyThread : public Thread {
		CameraHardwareSec *mHardware;
		int nFocus;
	public:
		LossFocusNotifyThread(CameraHardwareSec *hw, int focus):
		Thread(false),
		nFocus(focus),
		mHardware(hw) { }
		virtual void onFirstRef() {
			run("LossFocusNotifyThread", PRIORITY_DEFAULT);
		}
		virtual bool threadLoop() {
			return mHardware->LossFocusNotify(nFocus);
		}
	};	
    sp<LossFocusNotifyThread>   mLossFocusNotifyThread;
	int mFocusSave;
    bool        		mExitLossFocusNotifyThread;

#endif

	//add end by suojp 2012-05-11
#ifdef S3_FPS_DETECTION
	int mCapture_Cnt;
#endif

	//add begin by suojp 2012-06-18
#ifdef S3_IGNORE_PREVIEW_WINDOW
	bool bIgnorePreviewWindow;
#endif
	//add end by suojp 2012-06-18

#ifdef S3_TORCH_SCENARIO //limeng 120727
	int setTorchScenario();
#endif
};

}; // namespace android

#endif
