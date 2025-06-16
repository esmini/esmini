/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "OSCEnvironment.hpp"

using namespace scenarioengine;

void OSCEnvironment::SetAtmosphericPressure(double atmosphericpressure)
{
    if (atmosphericpressure < OSCAtmosphericMin || atmosphericpressure > OSCAtmosphericMax)
    {
        LOG_WARN("Atmospheric pressure clamped between 80000.0 and 120000.0 Pa");
    }
    // Clamp the atmospheric pressure to the defined range
    atmosphericpressure_ = CLAMP(atmosphericpressure, OSCAtmosphericMin, OSCAtmosphericMax);
}

double OSCEnvironment::GetAtmosphericPressure() const
{
    return atmosphericpressure_.value();
}

bool OSCEnvironment::IsAtmosphericPressureSet() const
{
    return atmosphericpressure_.has_value();
}

void OSCEnvironment::SetTemperature(double temperature)
{
    if (temperature < OSCTemperatureMin || temperature > OSCTemperatureMax)
    {
        LOG_WARN("Temperature clamped between 170.0 and 340.0 K");
    }
    // Clamp the temperature to the defined range
    temperature_ = CLAMP(temperature, OSCTemperatureMin, OSCTemperatureMax);
}

double OSCEnvironment::GetTemperature() const
{
    return temperature_.value();
}

bool OSCEnvironment::IsTemperatureSet() const
{
    return temperature_.has_value();
}

void OSCEnvironment::SetFractionalCloudState(const std::string& fractionalcloudStateStr)
{
    fractionalcloudstate_ = fractionalcloudStateStr;
}

void OSCEnvironment::SetCloudState(CloudState cloudstate)
{
    std::map<CloudState, std::string> stateMap{
        {CloudState::FREE, "zeroOktas"},
        {CloudState::CLOUDY, "fourOktas"},
        {CloudState::RAINY, "sixOktas"},
        {CloudState::OVERCAST, "eightOktas"},
        {CloudState::SKYOFF, "nineOktas"},
    };
    auto it = stateMap.find(cloudstate);
    if (it == stateMap.end())
    {
        fractionalcloudstate_ = "other";
    }
    else
    {
        fractionalcloudstate_ = stateMap[cloudstate];
    }
}

std::string scenarioengine::OSCEnvironment::GetFractionalCloudState() const
{
    return fractionalcloudstate_.value();
}

bool OSCEnvironment::IsFractionalCloudStateSet() const
{
    return fractionalcloudstate_.has_value();
}

double scenarioengine::OSCEnvironment::GetFractionalCloudStateFactor() const
{
    if (IsFractionalCloudStateSet())
    {
        std::map<std::string, double> stateFactorMap{{"zeroOktas", 0.0},
                                                     {"oneOktas", 0.125},
                                                     {"twoOktas", 0.25},
                                                     {"threeOktas", 0.375},
                                                     {"fourOktas", 0.5},
                                                     {"fiveOktas", 0.625},
                                                     {"sixOktas", 0.75},
                                                     {"sevenOktas", 0.875},
                                                     {"eightOktas", 0.95},
                                                     {"nineOktas", 1.0}};

        auto it = stateFactorMap.find(fractionalcloudstate_.value());
        if (it != stateFactorMap.end())
        {
            return it->second;
        }
        else
        {
            return 0.0;  // Default to 0.0 if the state is unrecognized
        }
    }
    else
    {
        return 0.0;  // Default to 0.0 if no fractional cloud state is set
    }
}

void OSCEnvironment::SetFog(const Fog& fog)
{
    fog_ = fog;
}

void OSCEnvironment::SetFog(const double visualrange)
{
    fog_ = Fog{visualrange, std::nullopt};
}

Fog OSCEnvironment::GetFog() const
{
    return fog_.value();
}

double OSCEnvironment::GetFogVisibilityRangeFactor() const
{
    if (IsFogSet())
    {
        return (1 / (GetFog().visibility_range + 1));
    }
    else
    {
        return 0.0;
    }
}

bool OSCEnvironment::IsFogSet() const
{
    return fog_.has_value();
}

bool scenarioengine::OSCEnvironment::IsFogBoundingBoxSet() const
{
    return fog_.has_value() && fog_->boundingbox.has_value();
}

void OSCEnvironment::SetWind(const Wind& wind)
{
    if (wind.direction < OSCWindDirectionMin || wind.direction > OSCWindDirectionMax)
    {
        LOG_WARN("Wind direction clamped between 0.0 and 360.0 degrees");
    }
    // Clamp the wind direction to the defined range
    Wind windNew;
    windNew.direction = CLAMP(wind.direction, OSCWindDirectionMin, OSCWindDirectionMax);
    windNew.speed     = wind.speed;
    wind_             = windNew;
}

Wind OSCEnvironment::GetWind() const
{
    return wind_.value();
}

bool OSCEnvironment::IsWindSet() const
{
    return wind_.has_value();
}

bool scenarioengine::OSCEnvironment::IsWeatherSet() const
{
    return IsTemperatureSet() || IsAtmosphericPressureSet() || IsFractionalCloudStateSet() || IsFogSet() || IsWindSet() || IsPrecipitationSet() ||
           IsSunSet();
}

void OSCEnvironment::SetPrecipitation(const Precipitation& precipitation)
{
    precipitation_ = precipitation;
}

Precipitation OSCEnvironment::GetPrecipitation() const
{
    return precipitation_.value();
}

bool OSCEnvironment::IsPrecipitationSet() const
{
    return precipitation_.has_value();
}

bool scenarioengine::OSCEnvironment::IsPrecipitationIntensitySet() const
{
    return IsPrecipitationSet() && GetPrecipitation().precipitationintensity.has_value();
}

double scenarioengine::OSCEnvironment::GetPrecipitationIntensity() const
{
    return GetPrecipitation().precipitationintensity.value();
}

void OSCEnvironment::SetSun(const Sun& sun)
{
    if (sun.elevation < OSCSunElevationMin || sun.elevation > OSCSunElevationMax)
    {
        LOG_WARN("Sun elevation clamped between -90.0 and 90.0 degrees");
    }
    if (sun.azimuth < OSCSunAzimuthMin || sun.azimuth > OSCSunAzimuthMax)
    {
        LOG_WARN("Sun azimuth clamped between 0.0 and 360.0 degrees");
    }
    Sun sunNew;
    // Clamp the sun elevation and azimuth to the defined ranges
    sunNew.elevation = CLAMP(sun.elevation, OSCSunElevationMin, OSCSunElevationMax);
    sunNew.azimuth   = CLAMP(sun.azimuth, OSCSunAzimuthMin, OSCSunAzimuthMax);
    sunNew.intensity = sun.intensity;
    sun_             = sunNew;
}

Sun OSCEnvironment::GetSun() const
{
    return sun_.value();
}

bool OSCEnvironment::IsSunSet() const
{
    return sun_.has_value();
}

bool scenarioengine::OSCEnvironment::IsSunIntensitySet() const
{
    return IsSunSet() && GetSun().intensity.has_value();
}

double scenarioengine::OSCEnvironment::GetSunIntensity() const
{
    double intensity = GetSun().intensity.value_or(0.0);
    if (intensity < 0.0)
    {
        intensity = 0.0;
        LOG_WARN("Sun illuminance can't be less than 0, setting it to 0.");
    }
    return intensity;
}

double scenarioengine::OSCEnvironment::GetSunIntensityFactor() const
{
    if (IsSunIntensitySet())
    {
        double intensity = CLAMP(GetSunIntensity(), OSCSunIntensityMin, OSCSunIntensityMax);
        // Normalize intensity to a factor between 0 and 1
        return (intensity - OSCSunIntensityMin) / (OSCSunIntensityMax - OSCSunIntensityMin);
    }
    else if (IsTimeOfDaySet())
    {
        return GetSecondsToFactor(static_cast<int>(GetSecondsSinceMidnight(GetTimeOfDay().datetime)));
    }
    else
    {
        return 1.0;  // Default to 1.0 if no sun intensity is set
    }
}

void OSCEnvironment::SetTimeOfDay(const TimeOfDay& timeofday)
{
    timeofday_ = timeofday;
}

TimeOfDay OSCEnvironment::GetTimeOfDay() const
{
    return timeofday_.value();
}

bool OSCEnvironment::IsTimeOfDaySet() const
{
    return timeofday_.has_value();
}

void OSCEnvironment::SetRoadCondition(const RoadCondition& roadcondition)
{
    roadcondition_ = roadcondition;
}

void OSCEnvironment::SetRoadCondition(const double friction)
{
    roadcondition_ = RoadCondition{friction, std::nullopt, std::nullopt};
}

RoadCondition OSCEnvironment::GetRoadCondition() const
{
    return roadcondition_.value();
}

bool OSCEnvironment::IsRoadConditionSet() const
{
    return roadcondition_.has_value();
}

void OSCEnvironment::UpdateEnvironment(const OSCEnvironment& new_environment)
{
    if (new_environment.IsAtmosphericPressureSet())
    {
        SetAtmosphericPressure(new_environment.GetAtmosphericPressure());
    }
    if (new_environment.IsTemperatureSet())
    {
        SetTemperature(new_environment.GetTemperature());
    }
    if (new_environment.IsFractionalCloudStateSet())
    {
        SetFractionalCloudState(new_environment.GetFractionalCloudState());
    }
    if (new_environment.IsFogSet())
    {
        SetFog(new_environment.GetFog());
    }
    if (new_environment.IsWindSet())
    {
        SetWind(new_environment.GetWind());
    }
    if (new_environment.IsPrecipitationSet())
    {
        SetPrecipitation(new_environment.GetPrecipitation());
    }
    if (new_environment.IsSunSet())
    {
        SetSun(new_environment.GetSun());
    }
    if (new_environment.IsRoadConditionSet())
    {
        SetRoadCondition(new_environment.GetRoadCondition());
    }
    if (new_environment.IsTimeOfDaySet())
    {
        SetTimeOfDay(new_environment.GetTimeOfDay());
    }
    SetEnvironmentUpdatedInViewer(new_environment.IsEnvironmentUpdatedInViewer());
}

bool OSCEnvironment::IsEnvironment() const
{
    return IsTemperatureSet() || IsAtmosphericPressureSet() || IsFractionalCloudStateSet() || IsFogSet() || IsWindSet() || IsPrecipitationSet() ||
           IsSunSet() || IsRoadConditionSet() || IsTimeOfDaySet();
}

bool OSCEnvironment::IsEnvironmentUpdatedInViewer() const
{
    return isEnvironmentUpdatedInViewer_;
}

void OSCEnvironment::SetEnvironmentUpdatedInViewer(const bool isEnvironmentUpdatedInViewer)

{
    isEnvironmentUpdatedInViewer_ = isEnvironmentUpdatedInViewer;
}
