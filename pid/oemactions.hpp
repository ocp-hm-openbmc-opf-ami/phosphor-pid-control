#include "actioncontroller.hpp"

#include <iostream>

namespace pid_control
{
/** @brief OemMinfailAspeedfan defined action
 *  @param[in] sensorName sensor name
 *  @param[in] readFailureCnt  count number of fan having reading 0
 *  @param[in] Minimum number  of fan should have reading 0
 *  @return The setpoint to set pwm value
 */
inline double ActionOem::oemMinFailAspeedFan(std::string sensorName,
                                             uint8_t* readFailureCnt,
                                             uint8_t* minimumNumFailedFan)
{
    double setpoint = 0;

    if (((*readFailureCnt == *minimumNumFailedFan) &&
         !(std::strncmp(sensorName.c_str(), aspeedFan.c_str(),
                        strlen(aspeedFan.c_str())))))
    {
        setpoint = getoemMaxPwm();
        return setpoint;
    }
    return setpoint;
}
/** @brief OemMinfailPSUFan defined action
 *  @param[in] sensorName sensor name
 *  @param[in] readFailureCnt  count number of PSU fan having reading 0
 *  @param[in] Minimum number  of PSU fan should have reading 0
 *  @return The setpoint to set pwm value
 */

inline double ActionOem::oemMinFailPsuFan(std::string sensorName,
                                          uint8_t* readFailureCnt,
                                          uint8_t* minimumNumFailedFan)
{
    double setpoint = 0;
    if (((*readFailureCnt == *minimumNumFailedFan) &&
         !(std::strncmp(sensorName.c_str(), psuFan.c_str(),
                        strlen(psuFan.c_str())))))
    {
        setpoint = getoemMaxPwm();
        return setpoint;
    }
    return setpoint;
}

/** @brief MaxfailAspeedFan defined action
 *  @param[in] sensorName sensor name
 *  @param[in] readFailureCnt  count number of fan having reading 0
 *  @param[in] Maximum  number  of fan should have reading 0
 */

inline void ActionOem::maxFailAspeedFan(
    std::string sensorName, uint8_t* readFailureCnt, uint8_t* maxFanFailure)
{
    if ((*readFailureCnt == *maxFanFailure) &&
        !(std::strncmp(sensorName.c_str(), aspeedFan.c_str(),
                       strlen(aspeedFan.c_str()))))
    {
        boost::asio::io_context io;
        auto conn = std::make_shared<sdbusplus::asio::connection>(io);
        if (getPowerStatus(conn))
        {
            poweroff(conn);
        }
    }
}

/** @brief MaxfailPSUFan defined action
 *  @param[in] sensorName sensor name
 *  @param[in] readFailureCnt  count number of PSU fan having reading 0
 *  @param[in] Maximum number  of PSU fan should have reading 0
 */

inline void ActionOem::maxFailPsuFan(
    std::string sensorName, uint8_t* readFailureCnt, uint8_t* maxFanFailure)
{
    if (((*readFailureCnt == *maxFanFailure) &&
         !(std::strncmp(sensorName.c_str(), psuFan.c_str(),
                        strlen(psuFan.c_str())))))
    {
        boost::asio::io_context io;
        auto conn = std::make_shared<sdbusplus::asio::connection>(io);
        if (getPowerStatus(conn))
        {
            poweroff(conn);
        }
    }
}
/** @brief monitorSensorUNC defined action
 *  @param[in] sensorName sensor name
 *  @return setpoint to set pwm value when sensor triggers to unc event
 */

inline double ActionOem::monitorSensorUnc(std::string sensorName)
{
    std::string objectPath = sensorObjectPath;
    double setpoint = 0;
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    objectPath = objectPath + sensorName;
    std::string service;

    // warning interface
    try
    {
        service = getService(warningInterface, objectPath.c_str());
        propertyMap warningMap;
        auto method = conn->new_method_call(service.c_str(), objectPath.c_str(),
                                            PROP_INTF, METHOD_GET_ALL);
        method.append(warningInterface);
        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Failed to get all properties");
        }
        reply.read(warningMap);
        auto findWarningHigh = warningMap.find("WarningAlarmHigh");
        if (findWarningHigh != warningMap.end())
        {
            if (std::get<bool>(warningMap.at("WarningAlarmHigh")))
            {
                setpoint = getOemSetMaxPwm();
            }
        }
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to fetch",
            phosphor::logging::entry("EXCEPTION=%s", e.what()));
    }
    return setpoint;
}
/** @brief monitorSensorUNC defined action
 *  @param[in] sensorName sensor name
 */

inline void ActionOem::monitorSensorUcr(std::string sensorName)
{
    std::string objectPath = sensorObjectPath;
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    objectPath = objectPath + sensorName;
    std::string service;
    // critical interface
    try
    {
        service = getService(criticalInterface, objectPath.c_str());
        propertyMap criticalMap;
        auto method = conn->new_method_call(service.c_str(), objectPath.c_str(),
                                            PROP_INTF, METHOD_GET_ALL);
        method.append(criticalInterface);
        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Failed to get all properties");
        }
        reply.read(criticalMap);
        auto findCriticalHigh = criticalMap.find("CriticalAlarmHigh");

        if (findCriticalHigh != criticalMap.end())
        {
            if (std::get<bool>(criticalMap.at("CriticalAlarmHigh")))
            {
                if (getPowerStatus(conn))
                {
                    poweroff(conn);
                }
            }
        }
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to fetch",
            phosphor::logging::entry("EXCEPTION=%s", e.what()));
    }
}

} // namespace pid_control
