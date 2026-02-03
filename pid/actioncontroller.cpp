#include "actioncontroller.hpp"

#include "oemactions.hpp"

#include <string.h>

#include <iostream>

/** @brief Process Fan defined action
 *  @param[in] reading  sensor reading
 *  @param[in] sensorName sensor name
 *  @param[in] readFailureCnt  count number of fan having reading 0
 *  @return The setpoint to set pwm value
 */
namespace pid_control
{
double ActionOem::processFanAction(double reading, std::string sensorName,
                                   uint8_t* readFailureCnt)
{
    double setpoint = 0;

    if (!(reading > 0))
    {
        (*readFailureCnt)++;
    }
    std::vector<std::string> oemConditionName = getOemName();

    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  minFailFanZoneLeft) != oemConditionName.end() ||
        std::find(oemConditionName.begin(), oemConditionName.end(),
                  minFailFanZoneRight) != oemConditionName.end())
    {
        uint8_t minimumNumFailedFan = getOemMinFailFan();
        setpoint = oemMinFailAspeedFan(sensorName, readFailureCnt,
                                       &minimumNumFailedFan);
    }
    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  minFailFanPsu) != oemConditionName.end())
    {
        uint8_t minimumNumFailedFan = getOemMinFailFan();
        setpoint =
            oemMinFailPsuFan(sensorName, readFailureCnt, &minimumNumFailedFan);
    }
    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  maxFailFanZoneLeft) != oemConditionName.end() ||
        std::find(oemConditionName.begin(), oemConditionName.end(),
                  maxFailFanZoneRight) != oemConditionName.end())
    {
        uint8_t maxFanFailure = getOemMaxFailFan();
        maxFailAspeedFan(sensorName, readFailureCnt, &maxFanFailure);
    }
    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  maxFailFanPsu) != oemConditionName.end())
    {
        uint8_t maxFanFailure = getOemMaxFailFan();
        maxFailPsuFan(sensorName, readFailureCnt, &maxFanFailure);
    }
    return setpoint;
}

/** @brief Process Temp sensor defined action
 *  @param[in] sensorName sensor name
 *  @param[in] The setpoint to set pwm value
 */

void ActionOem::processThermalAction(std::string sensorName, double* setpoint)
{
    std::vector<std::string> oemConditionName = getOemName();
    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  sensorUnc) != oemConditionName.end())
    {
        *setpoint = monitorSensorUnc(sensorName);
    }
    if (std::find(oemConditionName.begin(), oemConditionName.end(),
                  sensorUcr) != oemConditionName.end())
    {
        monitorSensorUcr(sensorName);
    }
}
} // namespace pid_control
