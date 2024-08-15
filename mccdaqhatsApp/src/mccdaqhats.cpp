/*
 * SPDX-License-Identifier: EPICS
 * Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH 2023-2024
 * Lutz Rossa <rossa@helmholtz-berlin.de>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <epicsExport.h>
#include <epicsStdlib.h>
#include <epicsThread.h>
#include <epicsMath.h>
#include <iocsh.h>
#include <asynPortClient.h>
#include <daqhats/daqhats.h>
#include "mccdaqhats.h"
#include <limits>

#ifndef ARRAY_SIZE
/// preprocessor helper macro to return the length of a static array
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/**
 * \brief preprocessor helper macros to show line numbers as strings inside warnings
 * \code{.cpp}
 *    #pragma message(CPP_WARNINGPREFIX "TODO: this a warning")
 * \endcode
 */
#define CPP_NUMBER2STRING2(x) #x
#define CPP_NUMBER2STRING(x) CPP_NUMBER2STRING2(x)
#ifdef _MSC_VER
#define CPP_WARNINGPREFIX __FILE__ "(" CPP_NUMBER2STRING(__LINE__) "): warning C0000: "
#else
#define CPP_WARNINGPREFIX __FILE__ "(" CPP_NUMBER2STRING(__LINE__) "): "
#endif

/* ========================================================================
 * parameters
 * ======================================================================== */

/**
 * @brief The ParameterId enumeration defines internal parameter meanings.
 */
enum ParameterId
{
    MCCDAQHAT_C0, // 1st channel value
    MCCDAQHAT_C1,
    MCCDAQHAT_C2,
    MCCDAQHAT_C3,
    MCCDAQHAT_C4,
    MCCDAQHAT_C5,
    MCCDAQHAT_C6,
    MCCDAQHAT_C7,
    MCCDAQHAT_SLOPE0,      // 1st channel scale factor
    MCCDAQHAT_SLOPE1,
    MCCDAQHAT_SLOPE2,
    MCCDAQHAT_SLOPE3,
    MCCDAQHAT_SLOPE4,
    MCCDAQHAT_SLOPE5,
    MCCDAQHAT_SLOPE6,
    MCCDAQHAT_SLOPE7,
    MCCDAQHAT_OFFSET0,     // 1st channel offset
    MCCDAQHAT_OFFSET1,
    MCCDAQHAT_OFFSET2,
    MCCDAQHAT_OFFSET3,
    MCCDAQHAT_OFFSET4,
    MCCDAQHAT_OFFSET5,
    MCCDAQHAT_OFFSET6,
    MCCDAQHAT_OFFSET7,
    MCCDAQHAT_TCTYPE0,     // 1st channel thermo couple type
    MCCDAQHAT_TCTYPE1,
    MCCDAQHAT_TCTYPE2,
    MCCDAQHAT_TCTYPE3,
    MCCDAQHAT_CJC0,        // 1st cold junction channel value
    MCCDAQHAT_CJC1,
    MCCDAQHAT_CJC2,
    MCCDAQHAT_CJC3,
    MCCDAQHAT_IEPE0,       // sensor power
    MCCDAQHAT_IEPE1,
    MCCDAQHAT_DI,          // MCC152 DIO digital inputs
    MCCDAQHAT_DO,          // MCC152 DIO digital outputs
    MCCDAQHAT_MASK,        // channel bit mask
    MCCDAQHAT_START,       // acquisition status
    MCCDAQHAT_RATE,        // clock rate
    MCCDAQHAT_TRIG,        // trigger configuration
    MCCDAQHAT_CLKSRC,      // clock pin configuration
    MCCDAQHAT_RANGE,       // MCC128 analog range
    MCCDAQHAT_MODE,        // MCC128 single-ended/differential mode
    MCCDAQHAT_DIR,         // MCC152 direction
    MCCDAQHAT_IN_PULL_EN,  // MCC152 pullup enable
    MCCDAQHAT_IN_PULL_CFG, // MCC152 pullup configuration
    MCCDAQHAT_IN_INV,      // MCC152 data inversion
    MCCDAQHAT_IN_LATCH,    // MCC152 data latch
    MCCDAQHAT_OUT_TYPE     // MCC152 output type
};

/**
 * @brief The paramMccDaqHats struct defines asyn parameter mappings.
 */
struct paramMccDaqHats
{
    int         iAsynReason;  ///< for later: asyn reason/index for registered parameter
    epicsUInt8  byAddress;    ///< HAT address
    epicsUInt16 wHatID;       ///< HAT id -> hardware type
    ParameterId iHatParam;    ///< parameter id
    bool        bWritable;    ///< data direction for generated DB file
    std::string sDescription; ///< description for generated DB file
    std::vector<std::string> asEnum;  ///< list of allowed enumerations
    std::vector<double>      adCache; ///< cache of last read data
};

/* ========================================================================
 * mccdaqhats controller
 * ======================================================================== */
std::map<std::string, mccdaqhatsCtrl*> mccdaqhatsCtrl::m_mapControllers;

/**
 * @brief constructor of controller;
 *        this class is a singleton and created by "initialize", if one of the supported devices was detected
 * @param[in] szAsynPortName  name of this controller
 * @param[in] dTimeout        communication timeout
 */
mccdaqhatsCtrl::mccdaqhatsCtrl(const char* szAsynPortName, double dTimeout)
    : asynPortDriver(szAsynPortName,
                     1, // maximum address
#if ASYN_VERSION < 4 || (ASYN_VERSION == 4 && ASYN_REVISION < 32)
                     32 * MAX_NUMBER_HATS, // maximum parameters: 32 per HAT
#endif
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask, // additional interfaces
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask, // additional callback interfaces
                     ASYN_CANBLOCK, // asynFlags
                     1, // autoConnect
                     0, // default priority
                     0) // default stackSize
    , m_dTimeout(dTimeout)
    , m_hThread(static_cast<epicsThreadId>(0))
{
    m_mapControllers[szAsynPortName] = this;
}

/// destructor
mccdaqhatsCtrl::~mccdaqhatsCtrl()
{
    auto hThread(m_hThread);
    epicsUInt16 awTypes[MAX_NUMBER_HATS];
    m_hThread = static_cast<epicsThreadId>(0);
    if (hThread != m_hThread)
        epicsThreadMustJoin(hThread);
    memset(awTypes, 0, sizeof(awTypes));
    for (auto it = m_mapParameters.begin(); it != m_mapParameters.end(); ++it)
    {
        paramMccDaqHats* p(it->second);
        if (p)
        {
            if (p->byAddress < MAX_NUMBER_HATS)
                awTypes[p->byAddress] = p->wHatID;
            delete p;
        }
    }
    for (uint8_t i = 0; i < MAX_NUMBER_HATS; ++i)
    {
        switch (awTypes[i])
        {
            case HAT_ID_MCC_118: mcc118_close(i); break;
            case HAT_ID_MCC_128: mcc128_close(i); break;
            case HAT_ID_MCC_134: mcc134_close(i); break;
            case HAT_ID_MCC_152: mcc152_close(i); break;
            case HAT_ID_MCC_172: mcc172_close(i); break;
        }
    }
    auto it(m_mapControllers.find(portName));
    if (it != m_mapControllers.end())
        m_mapControllers.erase(it);
}

/**
 * @brief mccdaqhatsCtrl::backgroundthread is called in background for fetching data from hardware
 */
void mccdaqhatsCtrl::backgroundthread()
{
    m_hThread = epicsThreadGetIdSelf();
    epicsThreadSleep(0.1);
    while (m_hThread != static_cast<epicsThreadId>(0))
    {
        epicsUInt16 awTypes[MAX_NUMBER_HATS];
        double adData[80000];
        memset(awTypes, 0, sizeof(awTypes));
        for (auto it = m_mapParameters.begin(); it != m_mapParameters.end(); ++it)
        {
            paramMccDaqHats* p(it->second);
            if (p)
            {
                if (p->byAddress < MAX_NUMBER_HATS)
                    awTypes[p->byAddress] = p->wHatID;
            }
        }
        for (uint8_t i = 0; i < m_abyChannelMask.size() && i < MAX_NUMBER_HATS; ++i)
        {
            uint16_t wStatus(0);
            uint8_t byMask(m_abyChannelMask[i]), byChannelCount(0);
            uint32_t dwDataCount(0);
            if (!byMask) continue;
            for (uint8_t j = 0; j < 8; ++j)
                if ((byMask >> j) & 1)
                    ++byChannelCount;
            switch (awTypes[i])
            {
                case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
                {
                    if (mcc118_a_in_scan_read(i, &wStatus, -1, -1., &adData[0], ARRAY_SIZE(adData), &dwDataCount) != RESULT_SUCCESS
                        || !(wStatus & STATUS_RUNNING))
                        dwDataCount = 0;
                    break;
                }
                case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
                    if (mcc128_a_in_scan_read(i, &wStatus, -1, -1., &adData[0], ARRAY_SIZE(adData), &dwDataCount) != RESULT_SUCCESS
                        || !(wStatus & STATUS_RUNNING))
                        dwDataCount = 0;
                    break;
                case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
                    if (mcc172_a_in_scan_read(i, &wStatus, -1, -1., &adData[0], ARRAY_SIZE(adData), &dwDataCount) != RESULT_SUCCESS
                        || !(wStatus & STATUS_RUNNING))
                        dwDataCount = 0;
                    break;
            }
            if (!dwDataCount) continue;
            for (auto it = m_mapParameters.begin(); it != m_mapParameters.end(); ++it)
            {
                paramMccDaqHats* p(it->second);
                if (!p || p->byAddress != i) continue;
                switch (p->iHatParam)
                {
                    case MCCDAQHAT_C0:
                    case MCCDAQHAT_C1:
                    case MCCDAQHAT_C2:
                    case MCCDAQHAT_C3:
                    case MCCDAQHAT_C4:
                    case MCCDAQHAT_C5:
                    case MCCDAQHAT_C6:
                    case MCCDAQHAT_C7:
                    {
                        int iChannel(static_cast<int>(p->iHatParam - MCCDAQHAT_C0));
                        size_t uOffset(0);
                        asynParamType iType(asynParamNotDefined);
                        for (int j = 0; j < 8 && j < iChannel; ++j)
                            if ((byMask >> j) & 1)
                                ++uOffset;
                        getParamType(p->iHatParam, &iType);
                        if (iType == asynParamFloat64Array)
                        {
                            std::vector<double> adChannel;
                            adChannel.resize(dwDataCount, static_cast<double>(0.));
                            if ((byMask >> iChannel) & 1)
                            {
                                for (size_t j = 0; j < dwDataCount; ++j)
                                    adChannel[j] = adData[static_cast<size_t>(byChannelCount) * j + uOffset];
                            }
                            std::swap(p->adCache, adChannel);
                            doCallbacksFloat64Array(&p->adCache[0], p->adCache.size(), p->iHatParam, 0);
                        }
                        break;
                    }
                    default:
                        break;
                } // switch (p->iHatParam)
            } // for (auto it = m_mapParameters.begin(); it != m_mapParameters.end(); ++it)
        } // for (uint8_t i = 0; i < m_abyChannelMask.size() && i < MAX_NUMBER_HATS; ++i)
        epicsThreadSleep(0.001);
    } // while (m_hThread != static_cast<epicsThreadId>(0))
}

/**
 * @brief mccdaqhatsCtrl::interrupt is called in background from hardware
 */
void mccdaqhatsCtrl::interrupt()
{
    for (auto it1 = m_mapControllers.begin(); it1 != m_mapControllers.end(); ++it1)
    {
        mccdaqhatsCtrl* pCtrl((*it1).second);
        bool bChange(false);
        if (!pCtrl)
            continue;
        pCtrl->lock();
        for (auto it2 = pCtrl->m_mapParameters.begin(); it2 != pCtrl->m_mapParameters.end(); ++it2)
        {
            struct paramMccDaqHats* pParam((*it2).second);
            uint8_t byValue(0);
            if (pParam->wHatID != HAT_ID_MCC_152 || pParam->iHatParam != MCCDAQHAT_DI)
                continue;
            mcc152_dio_input_read_port(pParam->byAddress, &byValue);
            pCtrl->setIntegerParam(pParam->iAsynReason, byValue);
            mcc152_dio_int_status_read_port(pParam->byAddress, &byValue);
            bChange = true;
        }
        if (bChange)
            pCtrl->callParamCallbacks();
        pCtrl->unlock();
    }
}

/**
 * @brief mccdaqhatsCtrl::initialize is an iocsh wrapper function called for "mccdaqhatsInitialize";
 *        it searches for supported hardware, creates a singleton, parameters for every device
 * @param[in] (pArgs)            arguments to this wrapper
 * @param[in] szAsynPortName     [0]asyn port name of this motor controller
 * @param[in] dTimeout           [1]communication timeout in ms
 */
void mccdaqhatsCtrl::initialize(const iocshArgBuf* pArgs)
{
    const char* szAsynPort(pArgs[0].sval);
    double dTimeout(pArgs[1].dval);
    std::string sTmp;
    mccdaqhatsCtrl* pC(nullptr);

    if (!szAsynPort || !szAsynPort[0])
    {
        fprintf(stderr, "empty asyn port name not allowed\n");
        return;
    }
    if (dTimeout <= 0. || dTimeout > 10.)
    {
        fprintf(stderr, "invalid time specified (0 < timeout <= 10s)\n");
        return;
    }

    std::vector<struct HatInfo> hi;
    hi.resize(hat_list(HAT_ID_ANY, nullptr));
    if (hi.empty())
    {
        fprintf(stderr, "no hats found\n");
        return;
    }
    for (auto it = hi.begin(); it != hi.end(); ++it)
    {
        struct HatInfo* p(&(*it));
        memset(p, 0, sizeof(*p));
    }
    hat_list(HAT_ID_ANY, &hi[0]);
    printf("found %llu hats:\n", static_cast<unsigned long long>(hi.size()));
    for (auto it = hi.begin(); it != hi.end(); ++it)
    {
        struct HatInfo* pInfo(&(*it));
        char szPrefix[32], szSerial[1024];
        struct MCCAsynParam { const char* szSuffix; asynParamType iAsynType; ParameterId iHatParam; bool bWriteable; const char* szDesc; const char* szEnum; } *pParamList;
        // MCC 118 create parameters (8-ch 12 bit single-ended analog input)
                //    MCC_A<n>C0…MCC_A<n>C7   (floatarray)
                //    MCC_A<n>MASK (uint8 0xFF, 1…255 channel selection bit mask)
                //    MCC_A<n>RATE (float 100000, <=0: external clock with frequency hint)
                //    MCC_A<n>TRIG (enum 0, none=0, rising=1, falling=2, high=3, low=4)
                //    MCC_A<n>SLOPE0…7 (float 1)
                //    MCC_A<n>OFFSET0…7 (float 0)
                //    MCC_A<n>START (enum 0, STOP=0, START=1)
        struct MCCAsynParam aMCC118Params[] =
            { { "C",      asynParamFloat64Array, MCCDAQHAT_C0,      false, "channel value(s)", nullptr },
              { "SLOPE",  asynParamFloat64,      MCCDAQHAT_SLOPE0,  false, "EEPROM correction factor", nullptr },
              { "OFFSET", asynParamFloat64,      MCCDAQHAT_OFFSET0, false, "EEPROM correction offset", nullptr },
              { "START",  asynParamInt32,        MCCDAQHAT_START,   true,  "acquisition state", "stop|start" },
              { "MASK",   asynParamInt32,        MCCDAQHAT_MASK,    true,  "channel selection bit mask", nullptr },
              { "TRIG",   asynParamInt32,        MCCDAQHAT_TRIG,    true,  "trigger mode", "none|rising|falling|high|low" },
              { "RATE",   asynParamFloat64,      MCCDAQHAT_RATE,    true,  "ADC clock (<=0 ext. clock, freq. hint)", nullptr } };
        // MCC 128 create parameters (4-ch differential / 8-ch single-ended 16 bit analog input)
                //    MCC_A<n>C0…MCC_A<n>C7   (floatarray)
                //    MCC_A<n>MASK      (uint8 0xFF, 1…255 channel selection bit mask)
                //    MCC_A<n>RATE      (float 100000, <=0: external clock with frequency hint)
                //    MCC_A<n>TRIG      (enum 0, none=0, rising=1, falling=2, high=3, low=4)
                //    MCC_A<n>RANGE     (enum 0, ±10V=0, ±5V=1, ±2V=2, ±1V=3)
                //    MCC_A<n>MODE      (enum 0, 0=single-ended, 1=differential)
                //    MCC_A<n>SLOPE0…3  (float 1)
                //    MCC_A<n>OFFSET0…3 (float 0)
                //    MCC_A<n>START     (enum 0, STOP=0, START=1)
        struct MCCAsynParam aMCC128Params[] =
            { { "C",       asynParamFloat64Array, MCCDAQHAT_C0,      false, "channel value(s)", nullptr },
              { "SLOPE0",  asynParamFloat64,      MCCDAQHAT_SLOPE0,  false, "EEPROM correction factor", nullptr },
              { "SLOPE1",  asynParamFloat64,      MCCDAQHAT_SLOPE1,  false, "EEPROM correction factor", nullptr },
              { "SLOPE2",  asynParamFloat64,      MCCDAQHAT_SLOPE2,  false, "EEPROM correction factor", nullptr },
              { "SLOPE3",  asynParamFloat64,      MCCDAQHAT_SLOPE3,  false, "EEPROM correction factor", nullptr },
              { "OFFSET0", asynParamFloat64,      MCCDAQHAT_OFFSET0, false, "EEPROM correction offset", nullptr },
              { "OFFSET1", asynParamFloat64,      MCCDAQHAT_OFFSET1, false, "EEPROM correction offset", nullptr },
              { "OFFSET2", asynParamFloat64,      MCCDAQHAT_OFFSET2, false, "EEPROM correction offset", nullptr },
              { "OFFSET3", asynParamFloat64,      MCCDAQHAT_OFFSET3, false, "EEPROM correction offset", nullptr },
              { "START",   asynParamInt32,        MCCDAQHAT_START,   true,  "acquisition state", "stop|start" },
              { "MASK",    asynParamInt32,        MCCDAQHAT_MASK,    true,  "channel selection bit mask", nullptr },
              { "TRIG",    asynParamInt32,        MCCDAQHAT_TRIG,    true,  "trigger mode", "none|rising|falling|high|low" },
              { "RATE",    asynParamFloat64,      MCCDAQHAT_RATE,    true,  "ADC clock (<=0 ext. clock, freq. hint)", nullptr },
              { "RANGE",   asynParamInt32,        MCCDAQHAT_RANGE,   true,  "analog input range", "10V|5V|2V|1V" },
              { "MODE",    asynParamInt32,        MCCDAQHAT_MODE,    true,  "input mode", "single-ended|differential" } };
        // MCC 134 create parameters (4-ch 24 bit thermocouple input)
                //    MCC_A<n>C0…MCC_A<n>C3 (float)
                //    MCC_A<n>CJC0…3    (float cold junction compensation)
                //    MCC_A<n>TCTYPE0…3 (uint8 0, thermocouple type 0=disabled, 1=J, 2=K, 3=T, 4=E, 5=R, 6=S, 7=B, 8=N)
                //    MCC_A<n>RATE      (uint8 1, update interval in seconds, 1…255)
                //    MCC_A<n>SLOPE0…3  (float 1)
                //    MCC_A<n>OFFSET0…3 (float 0)
        struct MCCAsynParam aMCC134Params[] =
            { { "C",      asynParamFloat64, MCCDAQHAT_C0,      false, "channel value", nullptr },
              { "CJC",    asynParamFloat64, MCCDAQHAT_CJC0,    false, "cold junction compensation value", nullptr },
              { "TCTYPE", asynParamInt32,   MCCDAQHAT_TCTYPE0, true,  "thermocouple type", "disabled|J|K|T|E|R|S|B|N" },
              { "RATE",   asynParamInt32,   MCCDAQHAT_RATE,    true,  "update interval", nullptr },
              { "SLOPE",  asynParamFloat64, MCCDAQHAT_SLOPE0,  false, "EEPROM correction factor", nullptr },
              { "OFFSET", asynParamFloat64, MCCDAQHAT_OFFSET0, false, "EEPROM correction offset", nullptr } };
        // MCC 152 create parameters (2-ch 12 bit analog output, 8-ch digital I/O)
                //    MCC_A<n>C0   (uint16)
                //    MCC_A<n>C1   (uint16)
                //    MCC_A<n>DI   (uint8)
                //    MCC_A<n>DO   (uint8)
                //    MCC_A<n>DIR         (uint8 0xFF, digital direction bits: 0=out, 1=in)
                //    MCC_A<n>IN_PULL_EN  (uint8 0xFF, pull-resistor enable bits: 0=disable, 1=enable)
                //    MCC_A<n>IN_PULL_CFG (uint8 0xFF, pull-resistor config bits: 0=pull-down, 1=pull-up)
                //    MCC_A<n>IN_INV      (uint8 0x00, input invertion bits: 0=non-inverted, 1=inverted)
                //    MCC_A<n>IN_LATCH    (uint8 0x00, input latch bits: 0=non-latched, 1=latched)
                //    MCC_A<n>OUT_TYPE    (uint8 0x00, output config bits: 0=push/pull, 1=open-drain)
        struct MCCAsynParam aMCC152Params[] =
            { { "C0",          asynParamFloat64, MCCDAQHAT_C0,          true, "channel value", nullptr },
              { "C1",          asynParamFloat64, MCCDAQHAT_C1,          true, "channel value", nullptr },
              { "DI",          asynParamInt32,   MCCDAQHAT_DI,          true, "digital input bitmask", nullptr },
              { "DO",          asynParamInt32,   MCCDAQHAT_DO,          true, "digital output bitmask", nullptr },
              { "DIR",         asynParamInt32,   MCCDAQHAT_DIR,         true, "direction bit mask (0=output, 1=input)", nullptr },
              { "IN_PULL_EN",  asynParamInt32,   MCCDAQHAT_IN_PULL_EN,  true, "pull-resistor enable bit mask", nullptr },
              { "IN_PULL_CFG", asynParamInt32,   MCCDAQHAT_IN_PULL_CFG, true, "pull-resistor direction bit mask (1=up)", nullptr },
              { "IN_INV",      asynParamInt32,   MCCDAQHAT_IN_INV,      true, "input invertion mask", nullptr },
              { "IN_LATCH",    asynParamInt32,   MCCDAQHAT_IN_LATCH,    true, "input latch bit mask", nullptr },
              { "OUT_TYPE",    asynParamInt32,   MCCDAQHAT_OUT_TYPE,    true, "out conf bit mask (0=push,1=open-drain)", nullptr } };
        // MCC 172 create parameters (2-ch 24 bit differential analog input)
                //    MCC_A<n>C0…MCC_A<n>C1   (floatarray)
                //    MCC_A<n>MASK (uint8 0x03, 1…3 channel selection bit mask)
                //    MCC_A<n>RATE (float 200, sample rate 200…51200 Hz)
                //    MCC_A<n>TRIG (enum 0, none=0, rising=1, falling=2, high=3, low=4)
                //    MCC_A<n>CLKSRC (enum 0, local=0, master=1, slave=2)
                //    MCC_A<n>SLOPE0…1 (float 1)
                //    MCC_A<n>OFFSET0…1 (float 0)
                //    MCC_A<n>START (enum 0, STOP=0, START=1)
                //    MCC_A<n>IEPE0…1 (enum 0, OFF=0, ON=1)
        struct MCCAsynParam aMCC172Params[] =
            { { "C",      asynParamFloat64Array, MCCDAQHAT_C0,      false, "channel value", nullptr },
              { "SLOPE",  asynParamFloat64,      MCCDAQHAT_SLOPE0,  false, "EEPROM correction factor", nullptr },
              { "OFFSET", asynParamFloat64,      MCCDAQHAT_OFFSET0, false, "EEPROM correction offset", nullptr },
              { "IEPE",   asynParamInt32,        MCCDAQHAT_IEPE0,   true,  "IEPE power", "OFF|ON" },
              { "START",  asynParamInt32,        MCCDAQHAT_START,   true,  "acquisition state", "stop|start" },
              { "MASK",   asynParamInt32,        MCCDAQHAT_MASK,    true,  "channel selection bit mask", nullptr },
              { "TRIG",   asynParamInt32,        MCCDAQHAT_TRIG,    true,  "trigger mode", "none|rising|falling|high|low" },
              { "CLKSRC", asynParamInt32,        MCCDAQHAT_CLKSRC,  true,  "clock source", "local|master|slave" },
              { "RATE",   asynParamFloat64,      MCCDAQHAT_RATE,    true,  "sample rate", nullptr } };
        int iChannels(0), iListCount(0), iNoSuffixList(0);
        uint16_t wFW(0), wBoot(0);

        memset(szSerial, 0, sizeof(szSerial));
        printf("  a[%u]: id=0x%04x v=0x%04x %.*s\n", pInfo->address, pInfo->id,
               pInfo->version, static_cast<int>(sizeof(pInfo->product_name)), pInfo->product_name);
        snprintf(szPrefix, ARRAY_SIZE(szPrefix), "MCC_A%u", pInfo->address);
        szPrefix[ARRAY_SIZE(szPrefix) - 1] = '\0';
        switch (pInfo->id)
        {
            case HAT_ID_MCC_118_BOOTLOADER:
                pInfo->id = HAT_ID_MCC_118;
                goto handleMCC118;
            case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
handleMCC118:
                if (mcc118_open(pInfo->address) != RESULT_SUCCESS)
                {
                    printf("    cannot open MCC 118\n");
                    mcc118_close(pInfo->address);
                    break;
                }
                mcc118_firmware_version(pInfo->address, &wFW, &wBoot);
                mcc118_serial(pInfo->address, &szSerial[0]);
                printf("    MCC 118: 8-ch single-ended analog input (12 bit)\n");
                printf("    fw=0x%04x boot=0x%04x\n", wFW, wBoot);
                iChannels  = 8;
                pParamList = &aMCC118Params[0];
                iListCount = ARRAY_SIZE(aMCC118Params);
                iNoSuffixList = 3;
                break;
            case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
                if (mcc128_open(pInfo->address) != RESULT_SUCCESS)
                {
                    printf("    cannot open MCC 128\n");
                    mcc128_close(pInfo->address);
                    break;
                }
                mcc128_firmware_version(pInfo->address, &wFW);
                mcc128_serial(pInfo->address, &szSerial[0]);
                printf("    MCC 128: 4-ch differential / 8-ch single-ended analog input (16 bit)\n");
                printf("    fw=0x%04x\n", wFW);
                iChannels  = 8;
                pParamList = &aMCC128Params[0];
                iListCount = ARRAY_SIZE(aMCC128Params);
                iNoSuffixList = 1;
                break;
            case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
                if (mcc134_open(pInfo->address) != RESULT_SUCCESS)
                {
                    printf("    cannot open MCC 134\n");
                    mcc134_close(pInfo->address);
                    break;
                }
                mcc134_serial(pInfo->address, &szSerial[0]);
                printf("    MCC 134: 4-ch thermocouple input (24 bit)\n");
                iChannels  = 4;
                pParamList = &aMCC134Params[0];
                iListCount = ARRAY_SIZE(aMCC134Params);
                iNoSuffixList = 4;
                break;
            case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
                if (mcc152_open(pInfo->address) != RESULT_SUCCESS)
                {
                    printf("    cannot open MCC 152\n");
                    mcc152_close(pInfo->address);
                    break;
                }
                mcc152_serial(pInfo->address, &szSerial[0]);
                printf("    MCC 152: 2-ch analog output (12 bit), 8-ch digital I/O\n");
                iChannels  = 0;
                pParamList = &aMCC152Params[0];
                iListCount = ARRAY_SIZE(aMCC152Params);
                iNoSuffixList = 0;
                break;
            case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
                if (mcc172_open(pInfo->address) != RESULT_SUCCESS)
                {
                    printf("    cannot open MCC 172\n");
                    mcc172_close(pInfo->address);
                    break;
                }
                mcc172_firmware_version(pInfo->address, &wFW);
                mcc172_serial(pInfo->address, &szSerial[0]);
                printf("    MCC 172: 2-ch differential analog input (24 bit)\n");
                printf("    fw=0x%04x\n", wFW);
                iChannels  = 2;
                pParamList = &aMCC172Params[0];
                iListCount = ARRAY_SIZE(aMCC172Params);
                iNoSuffixList = 4;
                break;
            default:
                printf("    unknown ID\n");
                break;
        }
        if (szSerial[0])
        {
            szSerial[ARRAY_SIZE(szSerial) - 1] = '\0';
            for (size_t i = 0; i < ARRAY_SIZE(szSerial); ++i)
            {
                if (!szSerial[i]) break;
                if (szSerial[i] < 32 || szSerial[i] > 126) szSerial[i] = '.';
            }
            printf("    serial=%s\n", szSerial);
        }
        if (!pC && iListCount > 0)
        {
            // create new controller instance
            pC = new mccdaqhatsCtrl(szAsynPort, dTimeout);
            if (!pC)
            {
                fprintf(stderr, "cannot create new controller\n");
                break;
            }
            pC->lock();
        }
        hat_interrupt_callback_enable(&mccdaqhatsCtrl::interruptfunc, static_cast<void*>(pC));
        if (pInfo->address >= pC->m_abyChannelMask.size())
            pC->m_abyChannelMask.resize(static_cast<size_t>(pInfo->address) + 1, 0);
        pC->m_abyChannelMask[pInfo->address] = static_cast<uint8_t>((1 << iChannels) - 1);
        for (int i = 0; i < iListCount; ++i)
        {
            std::string szName;
            struct paramMccDaqHats p;
            const char* szEnum(pParamList[i].szEnum);
            p.iAsynReason  = -1;
            p.byAddress    = pInfo->address;
            p.wHatID       = pInfo->id;
            p.iHatParam    = pParamList[i].iHatParam;
            p.bWritable    = pParamList[i].bWriteable;
            p.sDescription = pParamList[i].szDesc;
            p.asEnum.clear();
            p.adCache.clear();
            while (szEnum && *szEnum)
            {
              size_t iLen(strlen(szEnum));
              const char* pSep(strchr(szEnum, '|'));
              if (pSep) iLen = pSep - szEnum;
              p.asEnum.push_back(std::string(szEnum, iLen));
              if (!pSep) break;
              szEnum = pSep + 1;
            }
            if (p.asEnum.size() < 2)
               p.asEnum.clear();
            for (int j = 0; j < iChannels; ++j)
            {
                szName = std::string(szPrefix) + std::string("_") + std::string(pParamList[i].szSuffix);
                if (i < iNoSuffixList && iChannels > 0)
                {
                    p.iHatParam = static_cast<ParameterId>(static_cast<int>(pParamList[i].iHatParam) + j);
                    szName += std::to_string(j);
                }
                else if (j > 0) break;
                pC->createParam(szName.c_str(), pParamList[i].iAsynType, &p.iAsynReason);
                pC->m_mapParameters[p.iAsynReason] = new paramMccDaqHats(p);
                pC->m_mapDev2Asyn[GetMapHash(pInfo->address, p.iHatParam)] = p.iAsynReason;
                switch (pInfo->id)
                {
                    case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
                        switch (pParamList[i].iHatParam)
                        {
                            case MCCDAQHAT_SLOPE0:
                            {
                                double s(1.), o;
                                if (mcc118_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    s = 1.;
                                pC->setDoubleParam(p.iAsynReason, s);
                                break;
                            }
                            case MCCDAQHAT_OFFSET0:
                            {
                                double s, o(0.);
                                if (mcc118_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    o = 0.;
                                pC->setDoubleParam(p.iAsynReason, o);
                                break;
                            }
                            case MCCDAQHAT_MASK:
                                pC->setIntegerParam(p.iAsynReason, pC->m_abyChannelMask[p.byAddress]);
                                break;
                            case MCCDAQHAT_TRIG:
                                pC->setIntegerParam(p.iAsynReason, 0); // no trigger
                                break;
                            case MCCDAQHAT_RATE:
                                pC->setDoubleParam(p.iAsynReason, 100000); // max 100kHz
                                break;
                            case MCCDAQHAT_START: // stopped
                                pC->setIntegerParam(p.iAsynReason, 0);
                                break;
                            default: break;
                        }
                        break;
                    case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
                        switch (pParamList[i].iHatParam)
                        {
                            case MCCDAQHAT_SLOPE0:
                            case MCCDAQHAT_SLOPE1:
                            case MCCDAQHAT_SLOPE2:
                            case MCCDAQHAT_SLOPE3:
                            {
                                double s(1.), o;
                                if (mcc128_calibration_coefficient_read(pInfo->address, static_cast<int>(pParamList[i].iHatParam - MCCDAQHAT_SLOPE0), &s, &o) != RESULT_SUCCESS)
                                    s = 1.;
                                pC->setDoubleParam(p.iAsynReason, s);
                                break;
                            }
                            case MCCDAQHAT_OFFSET0:
                            case MCCDAQHAT_OFFSET1:
                            case MCCDAQHAT_OFFSET2:
                            case MCCDAQHAT_OFFSET3:
                            {
                                double s, o(0.);
                                if (mcc128_calibration_coefficient_read(pInfo->address, static_cast<int>(pParamList[i].iHatParam - MCCDAQHAT_OFFSET0), &s, &o) != RESULT_SUCCESS)
                                    o = 0.;
                                pC->setDoubleParam(p.iAsynReason, o);
                                break;
                            }
                            case MCCDAQHAT_MASK:
                                pC->setIntegerParam(p.iAsynReason, pC->m_abyChannelMask[p.byAddress]);
                                break;
                            case MCCDAQHAT_TRIG:
                                pC->setIntegerParam(p.iAsynReason, 0); // no trigger
                                break;
                            case MCCDAQHAT_RATE:
                                pC->setDoubleParam(p.iAsynReason, 100000); // max 100kHz
                                break;
                            case MCCDAQHAT_RANGE:
                                pC->setIntegerParam(p.iAsynReason, 0); // ±10V
                                break;
                            case MCCDAQHAT_MODE:
                                pC->setIntegerParam(p.iAsynReason, 0); // single-ended
                                break;
                            case MCCDAQHAT_START: // stopped
                                pC->setIntegerParam(p.iAsynReason, 0);
                                break;
                            default: break;
                        }
                        break;
                    case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
                        switch (pParamList[i].iHatParam)
                        {
                            case MCCDAQHAT_SLOPE0:
                            {
                                double s(1.), o;
                                if (mcc134_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    s = 1.;
                                pC->setDoubleParam(p.iAsynReason, s);
                                break;
                            }
                            case MCCDAQHAT_OFFSET0:
                            {
                                double s, o(0.);
                                if (mcc134_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    o = 0.;
                                pC->setDoubleParam(p.iAsynReason, o);
                                break;
                            }
                            case MCCDAQHAT_TCTYPE0: // disabled
                                mcc134_tc_type_write(pInfo->address, static_cast<uint8_t>(p.iHatParam - MCCDAQHAT_TCTYPE0), 0);
                                pC->setIntegerParam(p.iAsynReason, 0);
                                break;
                            case MCCDAQHAT_MASK:
                                pC->setIntegerParam(p.iAsynReason, pC->m_abyChannelMask[p.byAddress]);
                                break;
                            case MCCDAQHAT_RATE: // update every second
                                mcc134_update_interval_write(pInfo->address, 1);
                                pC->setIntegerParam(p.iAsynReason, 1);
                                break;
                            default: break;
                        }
                        break;
                    case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
                        switch (pParamList[i].iHatParam)
                        {
                            case MCCDAQHAT_MASK:
                                pC->setIntegerParam(p.iAsynReason, pC->m_abyChannelMask[p.byAddress]);
                                break;

#define HANDLE_MCC152_DATA(param,func)   \
                            case MCCDAQHAT_##param:     \
                            {                           \
                                uint8_t byValue(0);     \
                                if (mcc152_dio_##func##_read_port(pInfo->address, &byValue) == RESULT_SUCCESS) \
                                    pC->setIntegerParam(p.iAsynReason, byValue); \
                                break;                  \
                            }
#define HANDLE_MCC152_CONF(param,item)   \
                            case MCCDAQHAT_##param:     \
                            {                           \
                                uint8_t byValue(0);     \
                                if (mcc152_dio_config_read_port(pInfo->address, DIO_##item, &byValue) == RESULT_SUCCESS) \
                                    pC->setIntegerParam(p.iAsynReason, byValue); \
                                break;                  \
                            }
                            HANDLE_MCC152_DATA(DI,input)
                            HANDLE_MCC152_DATA(DO,output)
                            HANDLE_MCC152_CONF(DIR,DIRECTION)
                            HANDLE_MCC152_CONF(IN_PULL_EN,PULL_ENABLE)
                            HANDLE_MCC152_CONF(IN_PULL_CFG,PULL_CONFIG)
                            HANDLE_MCC152_CONF(IN_INV,INPUT_INVERT)
                            HANDLE_MCC152_CONF(IN_LATCH,INPUT_LATCH)
                            HANDLE_MCC152_CONF(OUT_TYPE,OUTPUT_TYPE)
#undef HANDLE_MCC152_CONF
#undef HANDLE_MCC152_DATA
                            default: break;
                        }
                        if (pParamList[i].iHatParam == MCCDAQHAT_DI)
                        {
                            uint8_t byValue(0);
                            mcc152_dio_config_write_port(pInfo->address, DIO_INT_MASK, 0); // enable interrupts
                            mcc152_dio_int_status_read_port(pInfo->address, &byValue);
                            mcc152_dio_input_read_port(pInfo->address, &byValue); // clear previous interrupts
                        }
                        break;
                    case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
                        switch (pParamList[i].iHatParam)
                        {
                            case MCCDAQHAT_SLOPE0:
                            {
                                double s(1.), o;
                                if (mcc172_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    s = 1.;
                                pC->setDoubleParam(p.iAsynReason, s);
                                break;
                            }
                            case MCCDAQHAT_OFFSET0:
                            {
                                double s, o(0.);
                                if (mcc172_calibration_coefficient_read(pInfo->address, j, &s, &o) != RESULT_SUCCESS)
                                    o = 0.;
                                pC->setDoubleParam(p.iAsynReason, o);
                                break;
                            }
                            case MCCDAQHAT_IEPE0:
                            {
                                uint8_t c(0);
                                if (mcc172_iepe_config_read(pInfo->address, j, &c) != RESULT_SUCCESS)
                                    c = 0;
                                pC->setIntegerParam(p.iAsynReason, c);
                                break;
                            }
                            case MCCDAQHAT_CLKSRC:
                            {
                                uint8_t bySrc(0), bySync(0);
                                double dRate(static_cast<double>(epicsNAN));
                                if (mcc172_a_in_clock_config_read(pInfo->address, &bySrc, &dRate, &bySync) != RESULT_SUCCESS)
                                    bySrc = 0;
                                pC->setIntegerParam(p.iAsynReason, bySrc);
                            }
                            case MCCDAQHAT_RATE:
                            {
                                uint8_t bySrc(0), bySync(0);
                                double dRate(static_cast<double>(epicsNAN));
                                if (mcc172_a_in_clock_config_read(pInfo->address, &bySrc, &dRate, &bySync) != RESULT_SUCCESS)
                                    dRate = 0.;
                                pC->setDoubleParam(p.iAsynReason, dRate);
                            }
                            case MCCDAQHAT_MASK:
                                pC->setIntegerParam(p.iAsynReason, pC->m_abyChannelMask[p.byAddress]);
                                break;
                            case MCCDAQHAT_START: // stopped
                                pC->setIntegerParam(p.iAsynReason, 0);
                                break;
                            default: break;
                        }
                        break;
                    default:
                        break;
                } // switch (pInfo->id)
            } // for (int j = 0; j < iChannels; ++j)
        } // for (int i = 0; i < iListCount; ++i)
    } // for (auto it = hi.begin(); it != hi.end(); ++it)
    if (pC)
    {
        epicsThreadOpts opt(EPICS_THREAD_OPTS_INIT);
        opt.priority  = epicsThreadPriorityHigh;
        opt.stackSize = epicsThreadGetStackSize(epicsThreadStackBig);
        opt.joinable  = 1;
        epicsThreadCreateOpt("mccdaqhats", &mccdaqhatsCtrl::backgroundthreadfunc, static_cast<void*>(pC), &opt);
        while (pC->m_hThread == static_cast<epicsThreadId>(0))
            epicsThreadSleep(0.1);
        pC->callParamCallbacks();
        pC->unlock();
    }
}

/**
 * @brief report internal information on user request
 * @param[in] fp      output file descriptor
 * @param[in] iLevel  user report level
 */
void mccdaqhatsCtrl::report(FILE* fp, int iLevel)
{
    fprintf(fp, "mccdaqhats controller driver, %s timeout=%g\n\n",
            portName, m_dTimeout);
    if (iLevel > 3)
    {
        int iNumParams(0);
        getNumParams(&iNumParams);
        for (int i = 0; i < iNumParams; ++i)
        {
            const char* szName(nullptr);
            asynParamType iType(asynParamNotDefined);
            getParamName(i, &szName);
            getParamType(i, &iType);
            fprintf(fp, "  param%d:\t%s\ttype=%d", i, szName, iType);
            switch (iType)
            {
#define HANDLETYPE(as,ep,fun,var1,fmt,var2) case as:{ep v;fun(i,var1);fprintf(fp,fmt,var2);break;}
                HANDLETYPE(asynParamInt32,epicsInt32,getIntegerParam,&v,"\tint32=%d",v)
                HANDLETYPE(asynParamInt64,epicsInt64,getInteger64Param,&v,"\tint64=%lld",static_cast<long long>(v))
                HANDLETYPE(asynParamFloat64,double,getDoubleParam,&v,"\tdouble=%g",v)
                HANDLETYPE(asynParamOctet,std::string,getStringParam,v,"\tstring=%s",v.c_str())
#undef HANDLETYPE
                case asynParamFloat64Array:
                    fprintf(fp, "\tdouble-array");
                    break;
                default:
                    break;
            }
            if (m_mapParameters.find(i) != m_mapParameters.end())
            {
                struct paramMccDaqHats* p(m_mapParameters[i]);
                if (p)
                {
                    std::string sEnum;
                    if (!p->sDescription.empty())
                        fprintf(fp, " \"%s\"", p->sDescription.c_str());
                    for (size_t j = 0; j < p->asEnum.size(); ++j)
                    {
                        if (j)
                            sEnum += std::string("|");
                        sEnum += p->asEnum[j];
                    }
                    if (!sEnum.empty())
                        fprintf(fp, " enum %s", sEnum.c_str());
                }
            }
            fprintf(fp, "\n");
        }
    }
}

/**
 * @brief Called when asyn clients call pasynInt32->read().
 *        For some parameters, it reads the hardware and in other cases,
 *        this will call the base class, which simply returns the stored value.
 * @param[in]  pasynUser  pasynUser structure that encodes the reason and address.
 * @param[out] piValue    address of the value to read.
 * @return asyn result code
 */
asynStatus mccdaqhatsCtrl::readInt32(asynUser* pasynUser, epicsInt32* piValue)
{
    asynStatus iResult(asynError);
    struct paramMccDaqHats* pParam(nullptr);
    if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
        pParam = m_mapParameters[pasynUser->reason];
    iResult = asynPortDriver::readInt32(pasynUser, piValue); // default handler: read cache
    if (pParam)
    {
        if (pParam->byAddress >= m_abyChannelMask.size())
            m_abyChannelMask.resize(static_cast<size_t>(pParam->byAddress) + 1, 0);
        switch (pParam->wHatID)
        {
            case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
                switch (pParam->iHatParam)
                {
                    case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    {
                        uint16_t wStatus(0);
                        uint32_t dwSamplesPerChannel(0);
                        *piValue = (mcc118_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                                    && (wStatus & STATUS_RUNNING) != 0) ? 1 : 0;
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    }
                    case MCCDAQHAT_MASK: // uint8 0xFF, 1…255 channel selection bit mask
                        *piValue = m_abyChannelMask[pParam->byAddress];
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    default:
                        break;
                }
                break;
            case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
                switch (pParam->iHatParam)
                {
                    case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    {
                        uint16_t wStatus(0);
                        uint32_t dwSamplesPerChannel(0);
                        *piValue = (mcc128_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                                    && (wStatus & STATUS_RUNNING) != 0) ? 1 : 0;
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    }
                    case MCCDAQHAT_MASK: // uint8 0xFF, 1…255 channel selection bit mask
                        *piValue = m_abyChannelMask[pParam->byAddress];
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    default:
                        break;
                }
                break;
            case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
                switch (pParam->iHatParam)
                {
                    case MCCDAQHAT_C0:
                    case MCCDAQHAT_C1:
                    case MCCDAQHAT_C2:
                    case MCCDAQHAT_C3:
                    {
                        uint8_t byChannel(static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_C0));
                        double dValue(static_cast<double>(epicsNAN));
                        if (mcc134_a_in_read(pParam->byAddress, byChannel, OPTS_DEFAULT, &dValue) == RESULT_SUCCESS
                            && isfinite(dValue))
                        {
                            if (dValue >= std::numeric_limits<epicsInt32>::max())
                                *piValue = std::numeric_limits<epicsInt32>::max();
                            else if (dValue <= std::numeric_limits<epicsInt32>::min())
                                *piValue = std::numeric_limits<epicsInt32>::min();
                            else
                                *piValue = static_cast<epicsInt32>(dValue);
                            iResult = setIntegerParam(pasynUser->reason, *piValue);
                        }
                        else
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC134: cannot read channel value\n");
                            iResult = asynError;
                        }
                        break;
                    }
                    case MCCDAQHAT_CJC0: // float, cold junction compensation
                    case MCCDAQHAT_CJC1:
                    case MCCDAQHAT_CJC2:
                    case MCCDAQHAT_CJC3:
                    {
                        uint8_t byChannel(static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_CJC0));
                        double dValue(static_cast<double>(epicsNAN));
                        if (mcc134_cjc_read(pParam->byAddress, byChannel, &dValue) == RESULT_SUCCESS
                            && isfinite(dValue))
                        {
                            if (dValue >= std::numeric_limits<epicsInt32>::max())
                                *piValue = std::numeric_limits<epicsInt32>::max();
                            else if (dValue <= std::numeric_limits<epicsInt32>::min())
                                *piValue = std::numeric_limits<epicsInt32>::min();
                            else
                                *piValue = static_cast<epicsInt32>(dValue);
                            iResult = setIntegerParam(pasynUser->reason, *piValue);
                        }
                        else
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC134: cannot read cold junction channel value\n");
                            iResult = asynError;
                        }
                        break;
                    }
                    case MCCDAQHAT_RATE: // uint8 update interval, 1…255
                    {
                        uint8_t byInterval(0);
                        if (mcc134_update_interval_read(pParam->byAddress, &byInterval) == RESULT_SUCCESS)
                        {
                            *piValue = byInterval;
                            iResult = setIntegerParam(pasynUser->reason, *piValue);
                        }
                        else
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC134: cannot read update interval\n");
                            iResult = asynError;
                        }
                        break;
                    }
                    default:
                        break;
                }
                break;
            case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
                switch (pParam->iHatParam)
                {
#define HANDLE_MCC152_DATA(param,func,errmsg)   \
                    case MCCDAQHAT_##param:     \
                    {                           \
                        uint8_t byValue(0);     \
                        if (mcc152_dio_##func##_read_port(pParam->byAddress, &byValue) == RESULT_SUCCESS) \
                        {                       \
                            *piValue = byValue; \
                            iResult = setIntegerParam(pasynUser->reason, *piValue); \
                        }                       \
                        else                    \
                        {                       \
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC152: cannot read " errmsg "\n"); \
                            return asynError;   \
                        }                       \
                        break;                  \
                    }
#define HANDLE_MCC152_CONF(param,item,errmsg)   \
                    case MCCDAQHAT_##param:     \
                    {                           \
                        uint8_t byValue(0);     \
                        if (mcc152_dio_config_read_port(pParam->byAddress, DIO_##item, &byValue) == RESULT_SUCCESS) \
                        {                       \
                            *piValue = byValue; \
                            iResult = setIntegerParam(pasynUser->reason, *piValue); \
                        }                       \
                        else                    \
                        {                       \
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC152: cannot read " errmsg "\n"); \
                            return asynError;   \
                        }                       \
                        break;                  \
                    }
                    HANDLE_MCC152_DATA(DI,input,"digital inputs")
                    HANDLE_MCC152_DATA(DO,output,"digital outputs")
                    HANDLE_MCC152_CONF(DIR,DIRECTION,"direction")
                    HANDLE_MCC152_CONF(IN_PULL_EN,PULL_ENABLE,"pull-up direction")
                    HANDLE_MCC152_CONF(IN_PULL_CFG,PULL_CONFIG,"pull-up configuration")
                    HANDLE_MCC152_CONF(IN_INV,INPUT_INVERT,"data invertion")
                    HANDLE_MCC152_CONF(IN_LATCH,INPUT_LATCH,"latch configuration")
                    HANDLE_MCC152_CONF(OUT_TYPE,OUTPUT_TYPE,"output configuration")
#undef HANDLE_MCC152_CONF
#undef HANDLE_MCC152_DATA
                    default:
                        break;
                }
                break;
            case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
                switch (pParam->iHatParam)
                {
                    case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    {
                        uint16_t wStatus(0);
                        uint32_t dwSamplesPerChannel(0);
                        *piValue = (mcc172_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                                    && (wStatus & STATUS_RUNNING) != 0) ? 1 : 0;
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    }
                    case MCCDAQHAT_MASK: // uint8 0x03, 1…3 channel selection bit mask
                        *piValue = m_abyChannelMask[pParam->byAddress];
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    case MCCDAQHAT_IEPE0: // uint8 , 0=OFF, 1=ON
                    case MCCDAQHAT_IEPE1:
                    {
                        uint8_t c(0);
                        if (mcc172_iepe_config_read(pParam->byAddress,
                                static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_IEPE0), &c) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC134: cannot read IEPE config\n");
                            return asynError;
                        }
                        *piValue = c;
                        iResult = setIntegerParam(pasynUser->reason, *piValue);
                        break;
                    }
                    default:
                        break;
                }
                break;
            default: // unsupported device
                break;
        }
    }
    return iResult;
}

/**
 * @brief Called when asyn clients call pasynInt32->write().
 *        For some parameters, it writes to the hardware and in other cases,
 *        this will call the base class, which simply stores the value.
 * @param[in] pasynUser  pasynUser structure that encodes the reason and address.
 * @param[in] iValue     value to write.
 * @return asyn result code
 */
asynStatus mccdaqhatsCtrl::writeInt32(asynUser* pasynUser, epicsInt32 iValue)
{
    asynStatus iResult(asynSuccess);
    struct paramMccDaqHats* pParam(nullptr);
    if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
        pParam = m_mapParameters[pasynUser->reason];
    if (!pParam) // default handler for other asyn parameters
        goto handleWrite;
    if (pParam->byAddress >= m_abyChannelMask.size())
        m_abyChannelMask.resize(static_cast<size_t>(pParam->byAddress) + 1, 0);
    switch (pParam->wHatID)
    {
        case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
        {
            uint16_t wStatus(0);
            uint32_t dwSamplesPerChannel(0);
            bool bStarted(mcc118_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                          && (wStatus & STATUS_RUNNING) != 0);
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    if (!iValue)
                    {
                        // no data acquisition
                        if (mcc118_a_in_scan_stop(pParam->byAddress) != RESULT_SUCCESS && bStarted)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - cannot stop MCC118\n");
                            iResult = asynError;
                        }
                        mcc118_a_in_scan_cleanup(pParam->byAddress);
                    }
                    else if (!bStarted && m_abyChannelMask[pParam->byAddress])
                    {
                        // prepare data acquisition
                        int iAsynRate(m_mapDev2Asyn[GetMapHash(pParam->byAddress, MCCDAQHAT_RATE)]);
                        double dRate(static_cast<double>(epicsNAN)), dTmp(dRate);
                        uint8_t byMask(m_abyChannelMask[pParam->byAddress]), byChannels(0);
                        epicsInt32 iTrig(GetDevParamInt(pParam->byAddress, MCCDAQHAT_TRIG, static_cast<epicsInt32>(-1)));
                        uint32_t dwOptions(OPTS_CONTINUOUS);

                        if (iTrig < 0 || !byMask)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC118: invalid channel mask or trigger configured\n");
                            return asynError;
                        }
                        if (getDoubleParam(iAsynRate, &dRate) != asynSuccess)
                            dRate = static_cast<double>(epicsNAN);
                        for (int i = 0; i < 8; ++i, byMask >>= 1) if (byMask & 1) ++byChannels;
                        if (!isfinite(dRate) || fabs(dRate) < 1. || floor(byChannels * fabs(dRate)) > 100000.)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC118: invalid rate configured\n");
                            return asynError;
                        }
                        if (mcc118_a_in_scan_actual_rate(byChannels, fabs(dRate), &dTmp) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC118: invalid rate configured\n");
                            return asynError;
                        }
                        mcc118_a_in_scan_stop(pParam->byAddress);
                        mcc118_a_in_scan_cleanup(pParam->byAddress);
                        if (dRate < 0.)
                        {
                            dTmp = -dTmp;
                            dwOptions |= OPTS_EXTCLOCK;
                        }
                        if (iTrig > 0)
                        {
                            mcc118_trigger_mode(pParam->byAddress, static_cast<uint8_t>(iTrig - 1));
                            dwOptions |= OPTS_EXTTRIGGER;
                        }
                        if (fabs(dRate - dTmp) > 0.)
                        {
                            setDoubleParam(iAsynRate, dRate = dTmp);
                            callParamCallbacks();
                        }
                        // start data acquisition
                        if (mcc118_a_in_scan_start(pParam->byAddress, m_abyChannelMask[pParam->byAddress], 0, fabs(dRate), dwOptions) != RESULT_SUCCESS)
                        {
                            mcc118_a_in_scan_stop(pParam->byAddress);
                            mcc118_a_in_scan_cleanup(pParam->byAddress);
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - cannot start MCC118\n");
                            iResult = asynError;
                        }
                    }
                    break;
                case MCCDAQHAT_MASK: // uint8 0xFF, 1…255 channel selection bit mask
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC118 is active\n");
                        return asynError;
                    }
                    if (iValue > 0 && iValue < 256)
                        m_abyChannelMask[pParam->byAddress] = static_cast<uint8_t>(iValue);
                    else
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - invalid channel mask\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_RATE: // float 100000, <=0: external clock with frequency hint
                    return writeFloat64(pasynUser, static_cast<epicsFloat64>(iValue));
                case MCCDAQHAT_TRIG: // enum 0, none=0, rising=1, falling=2, high=3, low=4
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC118 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 4) iResult = asynError;
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC118 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_118
        case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
        {
            uint16_t wStatus(0);
            uint32_t dwSamplesPerChannel(0);
            bool bStarted(mcc128_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                          && (wStatus & STATUS_RUNNING) != 0);
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    if (!iValue)
                    {
                        // no data acquisition
                        if (mcc128_a_in_scan_stop(pParam->byAddress) != RESULT_SUCCESS && bStarted)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - cannot stop MCC128\n");
                            iResult = asynError;
                        }
                        mcc128_a_in_scan_cleanup(pParam->byAddress);
                    }
                    else if (!bStarted && m_abyChannelMask[pParam->byAddress])
                    {
                        // prepare data acquisition
                        int iAsynRate(m_mapDev2Asyn[GetMapHash(pParam->byAddress, MCCDAQHAT_RATE)]);
                        double dRate(static_cast<double>(epicsNAN)), dTmp(dRate);
                        uint8_t byMask(m_abyChannelMask[pParam->byAddress]), byChannels(0);
                        epicsInt32 iTrig(GetDevParamInt(pParam->byAddress, MCCDAQHAT_TRIG, static_cast<epicsInt32>(-1)));
                        epicsInt32 iRange(GetDevParamInt(pParam->byAddress, MCCDAQHAT_RANGE, static_cast<epicsInt32>(-1)));
                        epicsInt32 iMode(GetDevParamInt(pParam->byAddress, MCCDAQHAT_MODE, static_cast<epicsInt32>(-1)));
                        uint32_t dwOptions(OPTS_CONTINUOUS);

                        if (iTrig < 0 || iRange < 0 || iMode < 0 || !byMask)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC128: invalid channel mask, trigger, range or mode configured\n");
                            return asynError;
                        }
                        if (iMode && byMask > 0x0F) // 4-channels in differential mode
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC128: invalid channel mask for differential mode\n");
                            return asynError;
                        }
                        if (getDoubleParam(iAsynRate, &dRate) != asynSuccess)
                            dRate = static_cast<double>(epicsNAN);
                        for (int i = 0; i < 8; ++i, byMask >>= 1) if (byMask & 1) ++byChannels;
                        if (!isfinite(dRate) || fabs(dRate) < 1. || floor(byChannels * fabs(dRate)) > 100000.)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC128: invalid rate configured\n");
                            return asynError;
                        }
                        if (mcc128_a_in_scan_actual_rate(byChannels, fabs(dRate), &dTmp) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC128: invalid rate configured\n");
                            return asynError;
                        }
                        mcc128_a_in_scan_stop(pParam->byAddress);
                        mcc128_a_in_scan_cleanup(pParam->byAddress);
                        if (dRate < 0.)
                        {
                            dTmp = -dTmp;
                            dwOptions |= OPTS_EXTCLOCK;
                        }
                        if (iTrig > 0)
                        {
                            mcc128_trigger_mode(pParam->byAddress, static_cast<uint8_t>(iTrig - 1));
                            dwOptions |= OPTS_EXTTRIGGER;
                        }
                        mcc128_a_in_range_write(pParam->byAddress, static_cast<uint8_t>(iRange));
                        mcc128_a_in_mode_write(pParam->byAddress, static_cast<uint8_t>(iMode ? 1 : 0));
                        if (fabs(dRate - dTmp) > 0.)
                        {
                            setDoubleParam(iAsynRate, dRate = dTmp);
                            callParamCallbacks();
                        }
                        // start data acquisition
                        if (mcc128_a_in_scan_start(pParam->byAddress, m_abyChannelMask[pParam->byAddress], 0, fabs(dRate), dwOptions) != RESULT_SUCCESS)
                        {
                            mcc128_a_in_scan_stop(pParam->byAddress);
                            mcc128_a_in_scan_cleanup(pParam->byAddress);
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC128: cannot start\n");
                            iResult = asynError;
                        }
                    }
                    break;
                case MCCDAQHAT_MASK: // uint8 0xFF, 1…255 channel selection bit mask
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 is active\n");
                        return asynError;
                    }
                    if (iValue > 0 && iValue < 256)
                        m_abyChannelMask[pParam->byAddress] = static_cast<uint8_t>(iValue);
                    else
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - invalid channel mask\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_RATE: // float 100000, <=0: external clock with frequency hint
                    return writeFloat64(pasynUser, static_cast<epicsFloat64>(iValue));
                case MCCDAQHAT_TRIG: // enum 0, none=0, rising=1, falling=2, high=3, low=4
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 4)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 invalid trigger mode\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_RANGE: // enum 0, ±10V=0, ±5V=1, ±2V=2, ±1V=3
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 3)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 invalid analog range\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_MODE: // enum 0, 0=single-ended, 1=differential
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 1)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 invalid input mode\n");
                        iResult = asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC128 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_128
        case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
        {
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_RATE: // uint8, 1…255
                    if (iValue < 1 || iValue > 255)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC134: invalid update interval\n");
                        return asynError;
                    }
                    if (mcc134_update_interval_write(pParam->byAddress, static_cast<uint8_t>(iValue)) != RESULT_SUCCESS)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC134: cannot set update interval\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_TCTYPE0: // uint8, 0…8
                case MCCDAQHAT_TCTYPE1:
                case MCCDAQHAT_TCTYPE2:
                case MCCDAQHAT_TCTYPE3:
                    if (iValue < 0 || iValue > 8)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC134: invalid thermo couple type\n");
                        return asynError;
                    }
                    if (mcc134_tc_type_write(pParam->byAddress, static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_TCTYPE0),
                                             static_cast<uint8_t>(iValue)) != RESULT_SUCCESS)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC134: cannot write thermo couple type\n");
                        iResult = asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC134 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_134
        case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_DO:
                {
                    if (iValue < 0 || iValue > 255)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC152: invalid output value\n");
                        return asynError;
                    }
                    if (mcc152_dio_output_write_port(pParam->byAddress, static_cast<uint8_t>(iValue)) != RESULT_SUCCESS)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC152: cannot write digital outputs\n");
                        return asynError;
                    }
                    break;
                }
#define HANDLE_MCC152_CONF(param,item,errmsg)       \
                case MCCDAQHAT_##param:             \
                {                                   \
                    if (iValue < 0 || iValue > 255) \
                    {                               \
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC152: invalid output value\n"); \
                        return asynError;           \
                    }                               \
                    if (mcc152_dio_config_write_port(pParam->byAddress, DIO_##item, static_cast<uint8_t>(iValue)) != RESULT_SUCCESS) \
                    {                               \
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readInt32 - MCC152: cannot read " errmsg "\n"); \
                        return asynError;           \
                    }                               \
                    break;                          \
                }
                HANDLE_MCC152_CONF(DIR,DIRECTION,"direction")
                HANDLE_MCC152_CONF(IN_PULL_EN,PULL_ENABLE,"pull-up direction")
                HANDLE_MCC152_CONF(IN_PULL_CFG,PULL_CONFIG,"pull-up configuration")
                HANDLE_MCC152_CONF(IN_INV,INPUT_INVERT,"data invertion")
                HANDLE_MCC152_CONF(IN_LATCH,INPUT_LATCH,"latch configuration")
                HANDLE_MCC152_CONF(OUT_TYPE,OUTPUT_TYPE,"output configuration")
#undef HANDLE_MCC152_CONF
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC152 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
        {
            uint16_t wStatus(0);
            uint32_t dwSamplesPerChannel(0);
            bool bStarted(mcc172_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                          && (wStatus & STATUS_RUNNING) != 0);
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_START: // enum 0, STOP=0, START=1
                    if (!iValue)
                    {
                        // no data acquisition
                        if (mcc172_a_in_scan_stop(pParam->byAddress) != RESULT_SUCCESS && bStarted)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - cannot stop MCC172\n");
                            iResult = asynError;
                        }
                        mcc172_a_in_scan_cleanup(pParam->byAddress);
                    }
                    else if (!bStarted && m_abyChannelMask[pParam->byAddress])
                    {
                        // prepare data acquisition
                        int iAsynRate(m_mapDev2Asyn[GetMapHash(pParam->byAddress, MCCDAQHAT_RATE)]);
                        double dRate(static_cast<double>(epicsNAN)), dTmp(dRate);
                        uint8_t byMask(m_abyChannelMask[pParam->byAddress]), byTmp(0);
                        epicsInt32 iTrig(GetDevParamInt(pParam->byAddress, MCCDAQHAT_TRIG, static_cast<epicsInt32>(-1)));
                        epicsInt32 iClkSrc(GetDevParamInt(pParam->byAddress, MCCDAQHAT_CLKSRC, static_cast<epicsInt32>(-1)));
                        uint32_t dwOptions(OPTS_CONTINUOUS);

                        if (iTrig < 0 || iClkSrc < 0 || !byMask)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC172: invalid channel mask, trigger, clock source configured\n");
                            return asynError;
                        }
                        if (getDoubleParam(iAsynRate, &dRate) != asynSuccess)
                            dRate = static_cast<double>(epicsNAN);
                        if (!isfinite(dRate) || fabs(dRate) < 1. || floor(fabs(dRate)) > 51200.)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC172: invalid rate configured\n");
                            return asynError;
                        }
                        mcc172_a_in_scan_stop(pParam->byAddress);
                        mcc172_a_in_scan_cleanup(pParam->byAddress);
                        if (iTrig > 0)
                            dwOptions |= OPTS_EXTTRIGGER;
                        if (mcc172_a_in_clock_config_write(pParam->byAddress, static_cast<uint8_t>(iClkSrc), dRate) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC172: invalid clock source or rate configured\n");
                            return asynError;
                        }
                        if (mcc172_a_in_clock_config_read(pParam->byAddress, &byTmp, &dTmp, &byTmp) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC172: cannot read actual rate\n");
                            return asynError;
                        }
                        if (fabs(dTmp - dRate) > 0.)
                        {
                            setDoubleParam(iAsynRate, dRate = dTmp);
                            callParamCallbacks();
                        }
                        if (mcc172_trigger_config(pParam->byAddress, static_cast<uint8_t>(iClkSrc), static_cast<uint8_t>(iTrig - 1)) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - start MCC172: invalid clock source or trigger mode\n");
                            return asynError;
                        }
                        // start data acquisition
                        if (mcc172_a_in_scan_start(pParam->byAddress, m_abyChannelMask[pParam->byAddress], 0, dwOptions) != RESULT_SUCCESS)
                        {
                            mcc172_a_in_scan_stop(pParam->byAddress);
                            mcc172_a_in_scan_cleanup(pParam->byAddress);
                            iResult = asynError;
                        }
                    }
                    break;
                case MCCDAQHAT_MASK: // uint8 0xFF, 1…3 channel selection bit mask
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 is active\n");
                        return asynError;
                    }
                    if (iValue > 0 && iValue < 4)
                        m_abyChannelMask[pParam->byAddress] = static_cast<uint8_t>(iValue);
                    else
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 channel mask\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_RATE: // float 100000, <=0: external clock with frequency hint
                    return writeFloat64(pasynUser, static_cast<epicsFloat64>(iValue));
                case MCCDAQHAT_TRIG: // enum 0, none=0, rising=1, falling=2, high=3, low=4
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 4)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 invalid trigger mode\n");
                        iResult = asynError;
                    }
                    break;
                case MCCDAQHAT_CLKSRC: // enum 0, local=0, master=1, slave=2
                    if (bStarted)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 is active\n");
                        return asynError;
                    }
                    if (iValue < 0 || iValue > 2)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 invalid clock source\n");
                        return asynError;
                    }
                    if (iValue != 1) // disable clock output, real trigger will be configured later
                        mcc172_trigger_config(pParam->byAddress, static_cast<uint8_t>(iValue), TRIG_RISING_EDGE);
                    break;
                case MCCDAQHAT_IEPE0: // uint8 , 0=OFF, 1=ON
                case MCCDAQHAT_IEPE1:
                    if (iValue < 0 || iValue > 1)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 invalid IEPE config\n");
                        return asynError;
                    }
                    if (mcc172_iepe_config_write(pParam->byAddress,
                                                 static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_IEPE0),
                                                 static_cast<uint8_t>(iValue)) != RESULT_SUCCESS)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 cannot write IEPE config\n");
                        return asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeInt32 - MCC172 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_172
        default: // unsupported device
            iResult = asynError; // not writeable
            break;
    }

handleWrite:
    if (iResult == asynSuccess)
        iResult = asynPortDriver::writeInt32(pasynUser, iValue);
    return iResult;
}

/**
 * @brief Called when asyn clients call pasynFloat64->read().
 *        For some parameters, it reads the hardware and in other cases,
 *        this will call the base class, which simply returns the stored value.
 * @param[in]  pasynUser  pasynUser structure that encodes the reason and address.
 * @param[out] pdValue    address of the value to read.
 * @return asyn result code
 */
asynStatus mccdaqhatsCtrl::readFloat64(asynUser* pasynUser, epicsFloat64* pdValue)
{
    asynStatus iResult(asynError);
    struct paramMccDaqHats* pParam(nullptr);
    if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
        pParam = m_mapParameters[pasynUser->reason];
    iResult = asynPortDriver::readFloat64(pasynUser, pdValue); // default handler: read cache
    if (pParam)
    {
        if (pParam->byAddress >= m_abyChannelMask.size())
            m_abyChannelMask.resize(static_cast<size_t>(pParam->byAddress) + 1, 0);
        switch (pParam->wHatID)
        {
            case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
                break; // nothing to do here
            case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
                break; // nothing to do here
            case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
                switch (pParam->iHatParam)
                {
                    case MCCDAQHAT_C0:
                    case MCCDAQHAT_C1:
                    case MCCDAQHAT_C2:
                    case MCCDAQHAT_C3:
                    {
                        uint8_t byChannel(static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_C0));
                        if (mcc134_a_in_read(pParam->byAddress, byChannel, OPTS_DEFAULT, pdValue) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readFloat64 - MCC134: cannot read channel value\n");
                            iResult = asynError;
                        }
                        iResult = setDoubleParam(pasynUser->reason, *pdValue);
                        break;
                    }
                    case MCCDAQHAT_CJC0: // float, cold junction compensation
                    case MCCDAQHAT_CJC1:
                    case MCCDAQHAT_CJC2:
                    case MCCDAQHAT_CJC3:
                    {
                        uint8_t byChannel(static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_CJC0));
                        if (mcc134_cjc_read(pParam->byAddress, byChannel, pdValue) != RESULT_SUCCESS)
                        {
                            asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::readFloat64 - MCC134: cannot read cold junction channel value\n");
                            iResult = asynError;
                        }
                        iResult = setDoubleParam(pasynUser->reason, *pdValue);
                        break;
                    }
                    default:
                        break;
                }
                break;
            case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
                break; // nothing to do here
            case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
                break; // nothing to do here
            default: // unsupported device
                break;
        }
    }
    return iResult;
}

/**
 * @brief Called when asyn clients call pasynFloat64->write().
 *        For some parameters, it writes to the hardware and in other cases,
 *        this will call the base class, which simply stores the value.
 * @param[in] pasynUser  pasynUser structure that encodes the reason and address.
 * @param[in] dValue     value to write.
 * @return asyn result code
 */
asynStatus mccdaqhatsCtrl::writeFloat64(asynUser* pasynUser, epicsFloat64 dValue)
{
    asynStatus iResult(asynSuccess);
    struct paramMccDaqHats* pParam(nullptr);
    if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
        pParam = m_mapParameters[pasynUser->reason];
    if (!pParam) // default handler for other asyn parameters
        goto handleWrite;
    if (pParam->byAddress >= m_abyChannelMask.size())
        m_abyChannelMask.resize(static_cast<size_t>(pParam->byAddress) + 1, 0);
    switch (pParam->wHatID)
    {
        case HAT_ID_MCC_118: // 8-ch 12 bit single-ended analog input
        {
            uint16_t wStatus(0);
            uint32_t dwSamplesPerChannel(0);
            bool bStarted(mcc118_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                          && (wStatus & STATUS_RUNNING) != 0);
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_RATE: // float 100000, <=0: external clock with frequency hint
                    if (bStarted || !isfinite(dValue) || fabs(dValue) > 100000.)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC118 is active or invalid clock rate\n");
                        iResult = asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC118 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_118
        case HAT_ID_MCC_128: // 4-ch differential / 8-ch single-ended 16 bit analog input
        {
            uint16_t wStatus(0);
            uint32_t dwSamplesPerChannel(0);
            bool bStarted(mcc128_a_in_scan_status(pParam->byAddress, &wStatus, &dwSamplesPerChannel) == RESULT_SUCCESS
                          && (wStatus & STATUS_RUNNING) != 0);
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_RATE: // float 100000, <=0: external clock with frequency hint
                    if (bStarted || !isfinite(dValue) || fabs(dValue) > 100000.)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC128 is active or invalid clock rate\n");
                        iResult = asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC128 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        } // case HAT_ID_MCC_128
        case HAT_ID_MCC_134: // 4-ch 24 bit thermocouple input
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_RATE: // uint8 1, update interval in seconds, 1…255
                    if (isfinite(dValue) && dValue >= 1. && dValue < 256)
                        return writeInt32(pasynUser, static_cast<epicsInt32>(dValue));
                    else
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC134 invalid update interval\n");
                        iResult = asynError;
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC134 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        case HAT_ID_MCC_152: // 2-ch 12 bit analog output, 8-ch digital I/O
            switch (pParam->iHatParam)
            {
                case MCCDAQHAT_C0:
                case MCCDAQHAT_C1:
                    if (!isfinite(dValue) || dValue < 0. || dValue > 5.)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC152 invalid output value\n");
                        iResult = asynError; // not writeable
                    }
                    else if (mcc152_a_out_write(pParam->byAddress, static_cast<uint8_t>(pParam->iHatParam - MCCDAQHAT_C0), OPTS_DEFAULT, dValue) != RESULT_SUCCESS)
                    {
                        asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC152 cannot write output value\n");
                        iResult = asynError; // not writeable
                    }
                    break;
                default:
                    asynPrint(pasynUser, ASYN_TRACE_ERROR, "mccdaqhats::writeFloat64 - MCC134 read only parameter\n");
                    iResult = asynError; // not writeable
                    break;
            }
            break;
        case HAT_ID_MCC_172: // 2-ch 24 bit differential analog input
            break; // nothing to do here
        default: // unsupported device
            iResult = asynError; // not writeable
            break;
    }
handleWrite:
    if (iResult == asynSuccess)
        iResult = asynPortDriver::writeFloat64(pasynUser, dValue);
    return iResult;
}

/**
 * @brief Called when asyn clients call pasynFloat64Array->read().
 *        For some parameters, it reads the hardware and in other cases,
 *        this will call the base class, which simply returns an error.
 * @param[in]  pasynUser    pasynUser structure that encodes the reason and address.
 * @param[out] pdValue      address of buffer for read values.
 * @param[in]  uElements    number of elements to read.
 * @param[out] puIn         elements actually read.
 * @return asyn result code
 */
asynStatus mccdaqhatsCtrl::readFloat64Array(asynUser *pasynUser, epicsFloat64* pdValue, size_t uElements, size_t* puIn)
{
    asynStatus iResult(asynError);
    struct paramMccDaqHats* pParam(nullptr);
    if (pasynUser->reason >= 0 && m_mapParameters.count(pasynUser->reason))
        pParam = m_mapParameters[pasynUser->reason];
    if (!pParam) // default handler for other asyn parameters
        iResult = asynPortDriver::readFloat64Array(pasynUser, pdValue, uElements, puIn);
    else if (!pParam->adCache.empty())
    {
        if (uElements > pParam->adCache.size())
            uElements = pParam->adCache.size();
        memmove(pdValue, &pParam->adCache[0], uElements * sizeof(*pdValue));
        *puIn = uElements;
        iResult = asynSuccess;
    }
    return iResult;
}

/**
 * @brief get index for "m_mapDev2Asyn" mapping
 * @param[in] byAddress device address (0…MAX_NUMBER_HATS-1)
 * @param[in] iParam    parameter index (enum \ref ParameterId)
 * @return index for "m_mapDev2Asyn" mapping
 */
int mccdaqhatsCtrl::GetMapHash(uint8_t byAddress, int iParam)
{
    return static_cast<int>(MAX_NUMBER_HATS * iParam + byAddress);
}

/**
 * @brief Get asyn cached parameter value of device parameter.
 * @param[in] byAddress      device address (0…MAX_NUMBER_HATS-1)
 * @param[in] iParam         enum \ref ParameterId
 * @param[in] iDefaultValue  default value in case of error
 * @return cached value or for errors default value
 */
epicsInt32 mccdaqhatsCtrl::GetDevParamInt(uint8_t byAddress, int iParam, epicsInt32 iDefaultValue)
{
    epicsInt32 iValue(iDefaultValue);
    int iIndex(GetMapHash(byAddress, iParam));
    if (m_mapDev2Asyn.find(iIndex) == m_mapDev2Asyn.end() ||
        getIntegerParam(m_mapDev2Asyn[iIndex], &iValue) != asynSuccess)
        iValue = iDefaultValue;
    return iValue;
}

/**
 * @brief Get asyn cached parameter value of device parameter.
 * @param[in] byAddress      device address (0…MAX_NUMBER_HATS-1)
 * @param[in] iParam         enum \ref ParameterId
 * @param[in] dDefaultValue  default value in case of error
 * @return cached value or for errors default value
 */
epicsFloat64 mccdaqhatsCtrl::GetDevParamDouble(uint8_t byAddress, int iParam, epicsFloat64 dDefaultValue)
{
    epicsFloat64 dValue(dDefaultValue);
    int iIndex(GetMapHash(byAddress, iParam));
    if (m_mapDev2Asyn.find(iIndex) == m_mapDev2Asyn.end() ||
        getDoubleParam(m_mapDev2Asyn[iIndex], &dValue) != asynSuccess)
        dValue = dDefaultValue;
    return dValue;
}

/**
 * @brief mccdaqhatsCtrl::writeDB is an iocsh wrapper function called for "mccdaqhatswriteDB";
 *        write example EPICS DB file for what the mccdaqhatsInitialize function found here
 * @param[in] (pArgs)            arguments to this wrapper
 * @param[in] szAsynPortName     [0]asyn port name of this motor controller
 * @param[in] szFileName         [1](over)write this file
 */
void mccdaqhatsCtrl::writeDB(const iocshArgBuf* pArgs)
{
    const char* szAsynPort(pArgs[0].sval);
    const char* szFilename(pArgs[1].sval);
    mccdaqhatsCtrl* pInstance(nullptr);
    size_t uCount(0);
    FILE* pOutfile(nullptr);
    bool bFirstCtrl(true);
    if (szAsynPort && !*szAsynPort) szAsynPort = nullptr;
    if (szFilename && !*szFilename) szFilename = nullptr;
    if (szAsynPort)
    {
        if (m_mapControllers.empty())
            uCount = 0;
        else
        {
            for (auto it = m_mapControllers.begin(); it != m_mapControllers.end(); ++it)
            {
                mccdaqhatsCtrl* pCtrl((*it).second);
                if (!pCtrl)
                    continue;
                if (!strcmp(szAsynPort, pCtrl->portName))
                {
                    pInstance = pCtrl;
                    uCount = 1;
                    break;
                }
            }
        }
    }
    else
    {
        for (auto it = m_mapControllers.begin(); it != m_mapControllers.end(); ++it)
            if ((*it).second)
                ++uCount;
    }
    if (!uCount)
    {
        fprintf(stderr, "no MCC HAT support was found\n");
        return;
    }
    if (!szFilename)
    {
        fprintf(stderr, "missing file name for writing\n");
        return;
    }
    pOutfile = fopen(szFilename, "w");
    if (!pOutfile)
    {
        fprintf(stderr, "cannot open file %s for writing\n", szFilename);
        return;
    }

    for (auto it = m_mapControllers.begin(); it != m_mapControllers.end(); ++it)
    {
        mccdaqhatsCtrl* pCtrl((*it).second);
        bool bFirstParam(true);
        if (!pCtrl || (pInstance && pCtrl != pInstance))
            continue;
        pCtrl->lock();
        if (bFirstCtrl)
            bFirstCtrl = false;
        else
            fprintf(pOutfile, "\n");
        fprintf(pOutfile, "########################################\n");
        fprintf(pOutfile, "# asynport %s\n", pCtrl->portName);
        fprintf(pOutfile, "########################################\n");
        for (auto it2 = pCtrl->m_mapParameters.begin(); it2 != pCtrl->m_mapParameters.end(); ++it2)
        {
            struct paramMccDaqHats* pParam((*it2).second);
            const char *szHatType(nullptr), *szRecordType(nullptr), *szParamName(nullptr), *szDTYP(nullptr), *szAdditional(nullptr);
            asynParamType iParamType(asynParamNotDefined);
            pCtrl->getParamType(pParam->iAsynReason, &iParamType);
            switch (iParamType)
            {
                case asynParamInt32:         szDTYP = "asynInt32";         break;
                case asynParamInt64:         szDTYP = "asynInt64";         break;
                case asynParamUInt32Digital: szDTYP = "asynUInt32Digital"; break;
                case asynParamFloat64:       szDTYP = "asynFloat64";       break;
                case asynParamOctet:         szDTYP = pParam->bWritable ? "asynOctetWriteRead"  : "asynOctetRead";      break;
                case asynParamInt8Array:     szDTYP = pParam->bWritable ? "asynInt8ArrayOut"    : "asynInt8ArrayIn";    break;
                case asynParamInt16Array:    szDTYP = pParam->bWritable ? "asynInt16ArrayOut"   : "asynInt16ArrayIn";   break;
                case asynParamInt32Array:    szDTYP = pParam->bWritable ? "asynInt32ArrayOut"   : "asynInt32ArrayIn";   break;
                case asynParamInt64Array:    szDTYP = pParam->bWritable ? "asynInt64ArrayOut"   : "asynInt64ArrayIn";   break;
                case asynParamFloat32Array:  szDTYP = pParam->bWritable ? "asynFloat32ArrayOut" : "asynFloat32ArrayIn"; break;
                case asynParamFloat64Array:  szDTYP = pParam->bWritable ? "asynFloat64ArrayOut" : "asynFloat64ArrayIn"; break;
                default:
                    continue;
            }
            pCtrl->getParamName(pParam->iAsynReason, &szParamName);
            if (!szParamName || !*szParamName)
                continue;
            switch (pParam->wHatID)
            {
                case HAT_ID_MCC_118: szHatType = "MCC 118 (8-ch 12 bit single-ended analog input)";                     break;
                case HAT_ID_MCC_128: szHatType = "MCC 128 (4-ch differential / 8-ch single-ended 16 bit analog input)"; break;
                case HAT_ID_MCC_134: szHatType = "MCC 134 (4-ch 24 bit thermocouple input)";                            break;
                case HAT_ID_MCC_152: szHatType = "MCC 152 (2-ch 12 bit analog output, 8-ch digital I/O)";               break;
                case HAT_ID_MCC_172: szHatType = "MCC 172 (2-ch 24 bit differential analog input)";                     break;
                default:             szHatType = "?unknown?"; break;
            }
            switch (pParam->asEnum.size())
            {
                case 0:
                case 1:
                    break;
                case 2:
                    szRecordType = pParam->bWritable ? "bo" : "bi";
                    break;
                default:
                    szRecordType = pParam->bWritable ? "mbbo" : "mbbi";
                    break;
            }
            if (!szRecordType)
            {
                switch (iParamType)
                {
                    case asynParamInt32:
                    case asynParamInt64:
                    case asynParamUInt32Digital: szRecordType = pParam->bWritable ? "longout"   : "longin";   break;
                    case asynParamFloat64:       szRecordType = pParam->bWritable ? "ao"        : "ai";       break;
                    case asynParamOctet:         szRecordType = pParam->bWritable ? "stringout" : "stringin"; break;
                    case asynParamInt8Array:     szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"CHAR\")\nfield (NELM, \"$(NELM=10000)\")\n";  break;
                    case asynParamInt16Array:    szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"SHORT\")\nfield (NELM, \"$(NELM=10000)\")\n"; break;
                    case asynParamInt32Array:    szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"LONG\")\nfield (NELM, \"$(NELM=10000)\")\n"; break;
                    case asynParamInt64Array:    szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"INT64\")\nfield (NELM, \"$(NELM=10000)\")\n"; break;
                    case asynParamFloat32Array:  szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"FLOAT\")\nfield (NELM, \"$(NELM=10000)\")\n"; break;
                    case asynParamFloat64Array:  szRecordType = pParam->bWritable ? "aao" : "aai"; szAdditional = "field(FTVL, \"DOUBLE\")\nfield (NELM, \"$(NELM=10000)\")\n"; break;
                    default:
                        break;
                }
            }
            if (!szRecordType || !szDTYP)
                continue;
            if (bFirstParam)
                bFirstParam = false;
            else
                fprintf(pOutfile, "\n");
            fprintf(pOutfile, "# %u %s: 0x%x %s\n", pParam->byAddress, szParamName, pParam->wHatID, szHatType);
            fprintf(pOutfile, "record(%s, \"$(P):%s\")\n{\n", szRecordType, szParamName);
            fprintf(pOutfile, "  field(DTYP, \"%s\")\n", szDTYP);
            fprintf(pOutfile, "  field(%s, \"@asyn($(PORT=%s),0,1)%s\")\n", pParam->bWritable ? "OUT" : "INP", pCtrl->portName, szParamName);
            if (!pParam->sDescription.empty())
                fprintf(pOutfile, "  field(DESC, \"%s\")\n", pParam->sDescription.c_str());
            if (pParam->bWritable)
            {
                fprintf(pOutfile, "  info(asyn:FIFO, \"$(FIFO=100)\")\n");
                fprintf(pOutfile, "  info(asyn:READBACK, \"1\")\n");
            }
            else
            {
                fprintf(pOutfile, "  field(SCAN, \"I/O Intr\")\n");
                fprintf(pOutfile, "  info(asyn:FIFO, \"$(FIFO=100)\")\n");
            }
            if (pParam->asEnum.size() == 2)
            {
                fprintf(pOutfile, "  field(ZNAM, \"%s\")\n", pParam->asEnum[0].c_str());
                fprintf(pOutfile, "  field(ONAM, \"%s\")\n", pParam->asEnum[1].c_str());
            }
            else if (pParam->asEnum.size() > 2)
            {
                const char* aszPrefix[16] = {"ZR","ON","TW","TH","FR","FV","SX","SV","EI","NI","TE","EL","TV","TT","FT","FF"};
                for (size_t i = 0; i < 16 && i < pParam->asEnum.size(); ++i)
                {
                    fprintf(pOutfile, "  field(%sVL, \"%d\")\n", aszPrefix[i], static_cast<int>(i));
                    fprintf(pOutfile, "  field(%sST, \"%s\")\n", aszPrefix[i], pParam->asEnum[i].c_str());
                }
            }
            if (szAdditional && *szAdditional)
            {
                std::string s(szAdditional);
                while (!s.empty())
                {
                    auto i(s.find('\n'));
                    if (i == std::string::npos)
                        i = s.size();
                    ++i;
                    fprintf(pOutfile, "  %s", s.substr(0, i).c_str());
                    s.erase(0, i);
                }
            }
            fprintf(pOutfile, "}\n");
        }
        pCtrl->unlock();
    }

    fflush(pOutfile);
    fclose(pOutfile);
}

/* ========================================================================
 * iocsh registration
 * ======================================================================== */

static const iocshArg mccdaqhatsInitializeArg0 = { "asyn-port-name", iocshArgString };
static const iocshArg mccdaqhatsInitializeArg1 = { "comm-timeout",   iocshArgDouble };
static const iocshArg* mccdaqhatsInitializeArgs[] = { &mccdaqhatsInitializeArg0, &mccdaqhatsInitializeArg1 };
static const iocshFuncDef mccdaqhatsInitializeDef =
    { "mccdaqhatsInitialize", ARRAY_SIZE(mccdaqhatsInitializeArgs),
      mccdaqhatsInitializeArgs
#if defined(EPICS_VERSION) && EPICS_VERSION >= 7
#if EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3
      ,"register a mccdaqhats controller\n\n"
      "  asyn-port-name  asyn port name of the controller\n"
      "  comm-timeout    PLC communication timeout in sec\n"
#endif
#endif
    };

static const iocshArg mccdaqhatsWriteDBArg0 = { "asyn-port-name", iocshArgString };
static const iocshArg mccdaqhatsWriteDBArg1 = { "filename",   iocshArgStringPath };
static const iocshArg* mccdaqhatsWriteDBArgs[] = { &mccdaqhatsWriteDBArg0, &mccdaqhatsWriteDBArg1 };
static const iocshFuncDef mccdaqhatsWriteDBDef =
    { "mccdaqhatsWriteDB", ARRAY_SIZE(mccdaqhatsWriteDBArgs),
      mccdaqhatsWriteDBArgs
#if defined(EPICS_VERSION) && EPICS_VERSION >= 7
#if EPICS_REVISION > 0 || EPICS_MODIFICATION >= 3
      ,"write example EPICS DB file for what the mccdaqhatsInitialize function found here\n\n"
      "  asyn-port-name  asyn port name of the controller\n"
      "  filename        (over)write this file\n"
#endif
#endif
    };

/// helper function to register iocsh commands
static void mccdaqhatsRegister()
{
    static bool bFirst(true);
    if (bFirst)
    {
        bFirst = false;
        iocshRegister(&mccdaqhatsInitializeDef, &mccdaqhatsCtrl::initialize);
        iocshRegister(&mccdaqhatsWriteDBDef, &mccdaqhatsCtrl::writeDB);
    }
}

/// helpers helper function to call "mccdaqhatsRegister" on startup
epicsExportRegistrar(mccdaqhatsRegister);
