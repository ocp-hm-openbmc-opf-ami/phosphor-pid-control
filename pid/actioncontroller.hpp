#pragma once
#include "conf.hpp"
#include "dbus/dbusconfiguration.hpp"
#include "dbus/dbushelper.hpp"
#include "util.hpp"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <variant>

constexpr auto PROP_INTF = "org.freedesktop.DBus.Properties";
constexpr auto METHOD_GET = "Get";
constexpr auto METHOD_GET_ALL = "GetAll";
constexpr auto METHOD_SET = "Set";

/*power*/
static constexpr const char* pwrService = "xyz.openbmc_project.State.Chassis";
static constexpr const char* pwrStateObjPath =
    "/xyz/openbmc_project/state/chassis0";
static constexpr const char* pwrStateIface =
    "xyz.openbmc_project.State.Chassis";
static constexpr const char* pwrCtlOff =
    "xyz.openbmc_project.State.Chassis.Transition.Off";

static constexpr const char* pwrHostStateObjPath =
    "/xyz/openbmc_project/state/host0";

static constexpr const char* pwrHostStateIface =
    "xyz.openbmc_project.State.Host";

static constexpr const char* pwrCtlOn =
    "xyz.openbmc_project.State.Chassis.Transition.On";
/*sensor*/
constexpr const char* warningInterface =
    "xyz.openbmc_project.Sensor.Threshold.Warning";
constexpr const char* criticalInterface =
    "xyz.openbmc_project.Sensor.Threshold.Critical";
constexpr auto sensorObjectPath = "/xyz/openbmc_project/sensors/temperature/";

static const std::string psuFan = "PSU";
static const std::string aspeedFan = "Fan_";
const std::string tempSensor = "_Temp";

/*Oem conditions*/
constexpr std::string sensorUnc = "sensor UNC";
const std::string minFailFanZoneLeft = "Fan Fail Left Zone";
const std::string minFailFanZoneRight = "Fan Fail Right Zone";
constexpr std::string minFailFanPsu = "PSU Fan Fail";
const std::string maxFailFanZoneLeft = "Max Fans Fail LeftZone";
const std::string maxFailFanZoneRight = "Max Fans Fail RightZone";
constexpr std::string maxFailFanPsu = "Max Fans Fail";
constexpr std::string sensorUcr = "sensor UCR";

using property = std::string;
using sensorValue = std::variant<int64_t, double, std::string, bool>;
using propertyMap = std::map<property, sensorValue>;

using value = std::variant<uint8_t, uint16_t, std::string>;
using namespace phosphor::logging;
using namespace sdbusplus;

namespace pid_control
{
class ActionOem
{
  private:
    const conf::OemConfig _oemconfig;

  protected:
    double oem_setpoint = 0;

  public:
    ActionOem(conf::OemConfig oemconfig) : _oemconfig(oemconfig), oem_setpoint()
    {}

    double processFanAction(double reading, std::string sensorName,
                            uint8_t* readFailureCnt);
    void processThermalAction(std::string sensorName, double* setpoint);

    double monitorThreshold(std::string sensorName);

    double oemMinFailAspeedFan(std::string sensorName, uint8_t* readFailureCnt,
                               uint8_t* minimumNumFailedFan);
    double oemMinFailPsuFan(std::string sensorName, uint8_t* readFailureCnt,
                            uint8_t* minimumNumFailedFan);
    void maxFailAspeedFan(std::string sensorName, uint8_t* readFailureCnt,
                          uint8_t* maxFanFailure);
    void maxFailPsuFan(std::string sensorName, uint8_t* readFailureCnt,
                       uint8_t* maxFanFailure);
    double monitorSensorUnc(std::string sensorName);
    void monitorSensorUcr(std::string sensorName);

    uint64_t getoemMaxPwm(void) const
    {
        return _oemconfig.maxPwm;
    }
    uint64_t getOemMinFailFan(void) const
    {
        return _oemconfig.minNumberFailedFans;
    }
    uint64_t getOemMaxFailFan(void) const
    {
        return _oemconfig.maxnumberFans;
    }

    uint64_t getOemSetMaxPwm(void) const
    {
        return _oemconfig.setMaxPwm;
    }
    std::vector<std::string> getOemName(void) const
    {
        return _oemconfig.name;
    }
};

inline bool getPowerStatus(std::shared_ptr<sdbusplus::asio::connection> conn)
{
    bool pwrGood = false;
    std::string pwrStatus;
    value variant;
    try
    {
        auto method = conn->new_method_call(pwrService, pwrStateObjPath,
                                            PROP_INTF, METHOD_GET);
        method.append(pwrStateIface, "CurrentPowerState");
        auto reply = conn->call(method);
        reply.read(variant);
        pwrStatus = std::get<std::string>(variant);
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to get getPowerStatus Value",
            phosphor::logging::entry("EXCEPTION=%s", e.what()));
        return pwrGood;
    }
    if (pwrStatus == "xyz.openbmc_project.State.Chassis.PowerState.On")
    {
        pwrGood = true;
    }
    return pwrGood;
}

inline void poweroff(std::shared_ptr<sdbusplus::asio::connection> conn)
{
    std::cerr << " poweroff \n";
    auto method = conn->new_method_call(pwrService, pwrStateObjPath, PROP_INTF,
                                        METHOD_SET);
    method.append(pwrStateIface, "RequestedPowerTransition");
    method.append(std::variant<std::string>(pwrCtlOff));

    auto reply = conn->call(method);

    if (reply.is_method_error())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ailed to set RequestedPowerTransition poweroff");
    }
}
inline std::string getService(const std::string& intf, const std::string& path)
{
    auto bus = bus::new_default_system();
    auto mapper =
        bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetObject");

    mapper.append(path);
    mapper.append(std::vector<std::string>({intf}));

    std::map<std::string, std::vector<std::string>> response;

    try
    {
        auto responseMsg = bus.call(mapper);

        responseMsg.read(response);
    }
    catch (const sdbusplus::exception::exception& ex)
    {
        log<level::ERR>("ObjectMapper call failure",
                        entry("WHAT=%s", ex.what()));
        throw;
    }

    if (response.begin() == response.end())
    {
        throw std::runtime_error("Unable to find Object: " + path);
    }

    return response.begin()->first;
}
} // namespace pid_control
