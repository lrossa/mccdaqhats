# SPDX-License-Identifier: EPICS
# Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH 2023
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build the IOC application

PROD_IOC += mccdaqhats

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install mccdaqhats.dbd into <top>/dbd
DBD += mccdaqhats.dbd

mccdaqhats_DBD += base.dbd
mccdaqhats_DBD += asyn.dbd
mccdaqhats_DBD += drvAsynIPPort.dbd
mccdaqhats_DBD += drvAsynSerialPort.dbd
mccdaqhats_DBD += mccdaqhatsSupport.dbd

mccdaqhats_LIBS += asyn
USR_INCLUDES += -I/usr/local/include -I$(HOME)/Projekte/mccdaqhats/include -I$(HOME)/mccdaqhats/include
USR_LDFLAGS += -L/usr/local/lib -L$(HOME)/Projekte/mccdaqhats/lib/build -L$(HOME)/mccdaqhats/lib/build
mccdaqhats_SYS_LIBS += daqhats

# specify all source files to be compiled and added to the library
mccdaqhats_SRCS += mccdaqhats.cpp
mccdaqhats_INC += mccdaqhats.h

# mccdaqhats_registerRecordDeviceDriver.cpp derives from mccdaqhats.dbd
mccdaqhats_SRCS += mccdaqhats_registerRecordDeviceDriver.cpp
mccdaqhats_SRCS_DEFAULT += mccdaqhatsMain.cpp
mccdaqhats_SRCS_vxWorks += -nil-

mccdaqhats_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
