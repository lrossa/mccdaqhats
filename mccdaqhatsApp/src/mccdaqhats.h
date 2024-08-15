/*
 * SPDX-License-Identifier: EPICS
 * Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH 2023-2024
 * Lutz Rossa <rossa@helmholtz-berlin.de>
 */
#ifndef MCCDAQHATS_INCLUDED
#define MCCDAQHATS_INCLUDED

#include <asynPortDriver.h>
#include <iocsh.h>
#include <map>
#include <vector>

// forward declarations
struct iocshArgs;
struct parammccdaqhats;

/// mccdaqhats controller
class mccdaqhatsCtrl : public asynPortDriver
{
protected:
    /*
     * @brief constructor of controller
     * @param[in] szAsynPortName  name of this controller
     * @param[in] dTimeout        communication timeout
     */
    explicit mccdaqhatsCtrl(const char* szAsynPortName, double dTimeout);
public:
    virtual ~mccdaqhatsCtrl();

    // iocsh function called for "mccdaqhatsInitialize"
    static void initialize(const iocshArgBuf* pArgs);
    // iocsh function called for "mccdaqhatsWriteDB"
    static void writeDB(const iocshArgBuf* pArgs);

    // asyn functions for parameter handling
    asynStatus readInt32   (asynUser* pasynUser, epicsInt32* piValue);
    asynStatus writeInt32  (asynUser* pasynUser, epicsInt32  iValue);
    asynStatus readFloat64 (asynUser* pasynUser, epicsFloat64* pdValue);
    asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64  dValue);
    asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64* pdValue, size_t uElements, size_t* puIn);

    // report internal information on user request
    void report(FILE* fp, int iLevel);

    virtual void backgroundthread();
    virtual void interrupt();

protected:
    static std::map<std::string, mccdaqhatsCtrl*> m_mapControllers; ///< global mapping of all controllers
    std::map<int, struct paramMccDaqHats*> m_mapParameters;  ///< mapping of asyn reasons to parameter
    std::map<int, int>                     m_mapDev2Asyn;    ///< mapping of device/parameter to asyn reason
    double                                 m_dTimeout;       ///< communication timeout
    std::vector<uint8_t>                   m_abyChannelMask; ///< channel mask for every module
    epicsThreadId                          m_hThread;        ///< background update thread

    static int   GetMapHash(uint8_t byAddress, int iParam);
    epicsInt32   GetDevParamInt(uint8_t byAddress, int iParam, epicsInt32 iDefaultValue);
    epicsFloat64 GetDevParamDouble(uint8_t byAddress, int iParam, epicsFloat64 dDefaultValue);

private:
    /// @brief this is a wrapper for real implementation "mccdaqhatsCtrl::backgroundthread"
    static void backgroundthreadfunc(void* pParameter)
        { mccdaqhatsCtrl* pMeMyselfAndI(reinterpret_cast<mccdaqhatsCtrl*>(pParameter)); if (pMeMyselfAndI) pMeMyselfAndI->backgroundthread(); }

    /// @brief this is a wrapper for real implementation "mccdaqhatsCtrl::interrupt"
    static void interruptfunc(void* pParameter)
        { mccdaqhatsCtrl* pMeMyselfAndI(reinterpret_cast<mccdaqhatsCtrl*>(pParameter)); if (pMeMyselfAndI) pMeMyselfAndI->interrupt(); }
};

#endif /*MCCDAQHATS_INCLUDED*/
