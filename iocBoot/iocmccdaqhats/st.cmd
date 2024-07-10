#!../../bin/linux-arm/mccdaqhats
# SPDX-License-Identifier: EPICS
# Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH 2023

< envPaths

## Register all support components
dbLoadDatabase "$(TOP)/dbd/mccdaqhats.dbd"
mccdaqhats_registerRecordDeviceDriver(pdbbase)

## Load record instances
dbLoadRecords("$(TOP)/db/mccdaqhats_param.db","P=pi,PORT=MYPORT,ADDR=0,TIMEOUT=1,PINI=1")

# initialize controller
# portname, timeout
mccdaqhatsInitialize("MYPORT", 1)

#asynSetTraceFile("MYPORT", -1, "MYPORT.log")
#asynSetTraceMask("MYPORT", -1, 0x2F) # error, device, filter, driver, warning, !flow
#asynSetTraceIOMask("MYPORT", -1, 2)
#asynSetTraceInfoMask("MYPORT", -1, 0x0F)
asynSetTraceMask("MYPORT", -1, 0x0F)
asynSetTraceIOMask("MYPORT", -1, 0x02)

iocInit()
