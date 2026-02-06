# SPDX-License-Identifier: Apache-2.0

###################################################################################################
#
#   FileName : ruls.mk
#
#   Copyright (c) Telechips Inc.
#
#   Description :
#
#
###################################################################################################

MCU_BSP_APP_SAMPLE_BASE_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_BASE=1

# Paths
VPATH += $(MCU_BSP_APP_SAMPLE_BASE_PATH)
#VPATH += $(MCU_BSP_APP_SAMPLE_BASE_PATH)/../../BabyDuck/Pure_Pursuit

# Includes
INCLUDES += -I$(MCU_BSP_APP_SAMPLE_BASE_PATH)
INCLUDES += -I$(MCU_BSP_APP_SAMPLE_BASE_PATH)/$(MCU_BSP_CHIPSET_FAMILY_NAME)
#INCLUDES += -I$(MCU_BSP_APP_SAMPLE_BASE_PATH)/../../BabyDuck/Pure_Pursuit

# Sources
SRCS += main.c

# Team2 file
TEAM2_ROOT_PATH := $(MCU_BSP_APP_SAMPLE_BASE_PATH)/Team2
TEAM2_INC_PATH  := $(TEAM2_ROOT_PATH)/include
TEAM2_SRC_PATH  := $(TEAM2_ROOT_PATH)/source

INT_PATH := ../test.app.gpio/

VPATH += $(TEAM2_SRC_PATH)

INCLUDES += -I$(TEAM2_ROOT_PATH)
INCLUDES += -I$(TEAM2_INC_PATH)
# INCLUDES += -I$(INT_PATH)          # 향후 include 폴더에 넣을 헤더 대비

SRCS += speed.c
SRCS += interrupt_example.c
SRCS += encoder.c
SRCS += spi.c

SRCS += ultrasonic.c

SRCS += ipc.c
SRCS += steer.c
SRCS += mpu_driver.c
SRCS += imu_module.c

SRCS += pose.c
# pose don't use task
#SRCS += pose_task.c 
SRCS += sensor_task.c

SRCS += Pure_Pursuit.c
SRCS += follow_steer_module.c

SRCS += parsing.c
SRCS += Vision_steer.c
