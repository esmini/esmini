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

OSCEnvironment::OSCEnvironment()
{
    bool pressureset_ = false;
    bool temperatureset_ = false;
    bool cloudstateset_ = false;
    bool fogset_ = false;
    bool windset_ = false;
    bool precipitationset_ = false;
    bool sunset_ = false;
    bool roadconditionset_ = false;
    bool timeofdayset_ = false;
}
OSCEnvironment::~OSCEnvironment(){}


double OSCEnvironment::GetAtmosphericPressure()
{
    if (pressureset_)
    {
        return atmosphericpressure_;
    }
    else
    {
        return -1;
    }
}


double OSCEnvironment::GetTemperature()
{
    if (temperatureset_)
    {
        return temperature_;
    }
    else
    {
        return -1;
    }
}


CloudState OSCEnvironment::GetCloudState()
{
    if (cloudstateset_)
    {
        return cloudstate_;
    }
    else
    {
        return CloudState::SKY_NOT_VISIBLE;
    }
}


Fog *OSCEnvironment::GetFog()
{
    if (fogset_)
    {
        return &fog_;
    } 
    else
    {
        return nullptr;
    }
}


Wind *OSCEnvironment::GetWind()
{
    if (windset_)
    {
        return &wind_;
    }
    else
    {
        return nullptr;
    }
}


Precipitation *OSCEnvironment::GetPrecipitation()
{
    if (precipitationset_)
    {
        return &precipitation_;
    } 
    else
    {
        return nullptr;
    }
}

Sun *OSCEnvironment::GetSun()
{
    if (sunset_)
    {
        return &sun_;
    } 
    else
    {
        return nullptr;
    }
}
        


TimeOfDay *OSCEnvironment::GetTimeOfDay()
{
    if (timeofdayset_)
    {
        return &timeofday_;
    } 
    else
    {
        return nullptr;
    }
}

RoadCondition *OSCEnvironment::GetRoadCondition()
{
    if (roadconditionset_)
    {
        return &roadcondition_;
    } 
    else
    {
        return nullptr;
    }
}

void OSCEnvironment::UpdateEnvironment(OSCEnvironment* new_environment)
{
    if (new_environment != nullptr)
    {
        if (new_environment->IsAtmosphericPressure())
        {
            SetAtmosphericPressure(new_environment->GetAtmosphericPressure());
        }
        if (new_environment->IsTemperature())
        {
            SetTemperature(new_environment->GetTemperature());
        }
        if (new_environment->IsCloudState())
        {
            SetCloudState(new_environment->GetCloudState());
        }
        if (new_environment->IsFog())
        {
            SetFog(new_environment->GetFog());
        }
        if (new_environment->IsWind())
        {
            SetWind(new_environment->GetWind());
        }
        if (new_environment->IsPrecipitation())
        {
            SetPrecipitation(new_environment->GetPrecipitation());
        }
        if (new_environment->IsSun())
        {
            SetSun(new_environment->GetSun());
        }
        if (new_environment->IsRoadCondition())
        {
            SetRoadCondition(new_environment->GetRoadCondition());
        }
        if (new_environment->IsTimeOfDay())
        {
            SetTimeOfDay(new_environment->GetTimeOfDay());
        }

    }

}
bool OSCEnvironment::IsEnvironment()
{
    return pressureset_ ||
    temperatureset_ ||
    cloudstateset_ ||
    fogset_ ||
    windset_ ||
    precipitationset_ ||
    sunset_ ||
    roadconditionset_ ||
    timeofdayset_;
}
