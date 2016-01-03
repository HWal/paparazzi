# Hey Emacs, this is a -*- makefile -*-

IMU_SEN10121_CFLAGS  = -DUSE_IMU
IMU_SEN10121_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_sen10121.h\"

IMU_SEN10121_SRCS  = $(SRC_SUBSYSTEMS)/imu.c
IMU_SEN10121_SRCS += $(SRC_SUBSYSTEMS)/imu/imu_sen10121.c
IMU_SEN10121_SRCS += peripherals/adxl345_i2c.c
IMU_SEN10121_SRCS += peripherals/itg3200.c

IMU_SEN10121_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_SEN10121_CFLAGS += -DUSE_I2C2
	IMU_SEN10121_CFLAGS += -DIMU_SEN10121_I2C_DEV=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_SEN10121_CFLAGS += -DUSE_I2C0
	IMU_SEN10121_CFLAGS += -DIMU_SEN10121_I2C_DEV=i2c0
endif


# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_SEN10121_CFLAGS)
$(TARGET).srcs += $(IMU_SEN10121_SRCS)
endif
