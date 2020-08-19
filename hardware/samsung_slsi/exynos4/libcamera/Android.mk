LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

# HAL module implemenation stored in
# hw/<COPYPIX_HARDWARE_MODULE_ID>.<ro.product.board>.so
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

#LOCAL_C_INCLUDES += $(LOCAL_PATH)/../include
LOCAL_C_INCLUDES += hardware/samsung_slsi/exynos4/include/

LOCAL_C_INCLUDES += hardware/samsung_slsi/exynos4/libfimg4x \
	external/skia/include/core


LOCAL_SRC_FILES:= \
	SecCamera_zoom.cpp SecCameraHWInterface_zoom.cpp


LOCAL_SHARED_LIBRARIES:= libutils libcutils libbinder liblog libcamera_client libhardware libswscaler

ifeq ($(TARGET_SOC), exynos4210)
LOCAL_SHARED_LIBRARIES += libs5pjpeg
LOCAL_CFLAGS += -DSAMSUNG_EXYNOS4210
endif

ifeq ($(TARGET_SOC), exynos4x12)
LOCAL_SHARED_LIBRARIES += libhwjpeg
ifeq ($(BOARD_USES_FIMGAPI),true)
LOCAL_SHARED_LIBRARIES += libfimg
endif
LOCAL_CFLAGS += -DSAMSUNG_EXYNOS4x12
endif

ifeq ($(BOARD_USE_V4L2), true)
LOCAL_CFLAGS += -DBOARD_USE_V4L2
endif

ifeq ($(BOARD_USE_V4L2_ION), true)
LOCAL_CFLAGS += -DBOARD_USE_V4L2
LOCAL_CFLAGS += -DBOARD_USE_V4L2_ION
endif

ifeq ($(TARGET_DEVICE),sc1_dvt1)
LOCAL_CFLAGS += -DSC1_DVT1
else
endif

LOCAL_CFLAGS += -DUSE_PREVIEWFIMC_FOR_VIDEOSNAPSHOT
#LOCAL_CFLAGS += -DUSE_G2D_FOR_VIDEOSNAPSHOT

LOCAL_MODULE := camera.$(TARGET_DEVICE)

LOCAL_MODULE_TAGS := optional


# close camera temp. for build base line of KK, wjj.
include $(BUILD_SHARED_LIBRARY)
