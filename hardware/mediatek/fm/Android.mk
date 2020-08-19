ifeq ($(strip $(BOARD_MEDIATEK_USES_FM)),true)
ifeq ($(strip $(BOARD_CONNECTIVITY_MODULE)),combo_mt66xx)

LOCAL_PATH:=$(call my-dir)

$(warning [FM]starting to build fm native part...)  
include $(call all-subdir-makefiles)

endif
endif
