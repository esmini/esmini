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
#pragma once
#include <optional>
#include "OSCBoundingBox.hpp"
#include "OSCProperties.hpp"
#include "logger.hpp"

namespace scenarioengine
{
    /*
    constants
    */
    double const OSCAtmosphericMin   = 80000.0;   // Pa
    double const OSCAtmosphericMax   = 120000.0;  // Pa
    double const OSCTemperatureMin   = 170.0;     // kelvin
    double const OSCTemperatureMax   = 340.0;     // kelvin
    double const OSCSunAzimuthMin    = 0.0;       // rad
    double const OSCSunAzimuthMax    = 2 * M_PI;  // rad
    double const OSCSunElevationMin  = -M_PI;     // rad
    double const OSCSunElevationMax  = M_PI;      // rad
    double const OSCWindDirectionMin = 0.0;       // rad
    double const OSCWindDirectionMax = 2 * M_PI;  // rad
    double const OSCSunIntensityMax  = 100000.0;  // lux
    double const OSCSunIntensityMin  = 0.0;       // lux

    /*
    Class PrecipitationType
    Represents the different types of precipitations
    */
    enum class PrecipitationType
    {
        DRY,
        RAIN,
        SNOW
    };

    /*
    Class CloudState
    Represents the different types of cloud states, note: implemented for future 1.2 enumerations with 1.1 included
    */
    enum class CloudState
    {
        CLOUDY,
        FREE,
        OVERCAST,
        RAINY,
        SKYOFF,
        OTHER
    };

    enum class WetnessType
    {
        DRY,
        MOSIT,
        WETWITHPUDDLES,
        LOWFLOODED,
        HIGHFLOODED
    };

    /*
    Class Fog
    Represents a fog entry of openscenario

    visibility_range 	double 	 	visibility range in the fog
    boundingbox_     	double 	 	The boundingbox of the fog (optional)
    */
    typedef struct
    {
        float visibility_range;
        std::optional<OSCBoundingBox> boundingbox;
    } Fog;

    /*
    Class Sun
    Represents the state of the sun

    azimuth 	    double 	 	azimuth of the sun, 0=north, PI/2 = east, PI=south, 3/2 PI=west.
    elevation     	double 	 	elevation angle, 0 to PI/2
    intensity       double      Illuminance of the sun (daylight about 100,00 lux)
    */
    typedef struct
    {
        double                azimuth;
        double                elevation;
        std::optional<double> intensity;
    } Sun;

    /*
    Class Precipitation
    Represents the state of rain/snow

    precipitationintensity 	    double 	 	        intensity of precipitation, NOT IMPLEMENTED YET
    precipitationtype     	    PrecipitationType   type of precipitation, NOT IMPLEMENTED YET

    */
    typedef struct
    {
        std::optional<double> precipitationintensity;
        PrecipitationType     precipitationtype;
    } Precipitation;

    /*
    Class Wind
    Represents the state of the wind

    direction 	    double      direction of the wind in the x/y plane, NOT IMPLEMENTED YET
    speed           double     	speed of the wind, NOT IMPLEMENTED YET, NOT IMPLEMENTED YET

    */
    typedef struct
    {
        double direction;
        double speed;
    } Wind;

    /*
    Class TimeOfDay
    Represents the state of the wind

    animation 	    True      if an animation should be used, NOT IMPLEMENTED YET
    speed           dateTime  NOT IMPLEMENTED YET

    */
    typedef struct
    {
        bool        animation;
        std::string datetime;
    } TimeOfDay;

    /*
    Class RoadCondition
    Represents the state of the wind

    frictionscalefactor 	    double      the friction scale factor
    wetness                     wetness      the wetness of the road (optional) NOT IMPLEMENTED YET
    properties                  Properties  additional properties of the RoadCondition (optional) NOT IMPLEMENTED YET

    */
    typedef struct
    {
        double                       frictionscalefactor;
        std::optional<WetnessType>   wetness;
        std::optional<OSCProperties> properties;
    } RoadCondition;

    /*
    Class OSCEnvironment
    Defines the full environment of OpenSCENARIO

    */

    class OSCEnvironment
    {
    public:
        OSCEnvironment()  = default;
        ~OSCEnvironment() = default;

        void      SetTimeOfDay(const TimeOfDay& timeofday);
        TimeOfDay GetTimeOfDay() const;
        bool      IsTimeOfDaySet() const;

        void   SetAtmosphericPressure(double atmosphericpressure);
        double GetAtmosphericPressure() const;
        bool   IsAtmosphericPressureSet() const;

        void   SetTemperature(double temperature);
        double GetTemperature() const;
        bool   IsTemperatureSet() const;

        void        SetCloudState(CloudState cloudstate);
        void        SetFractionalCloudState(const std::string& fractionalcloudStateStr);
        std::string GetFractionalCloudState() const;
        bool        IsFractionalCloudStateSet() const;
        double      GetFractionalCloudStateFactor() const;

        void   SetSun(const Sun& sun);
        Sun    GetSun() const;
        bool   IsSunSet() const;
        bool   IsSunIntensitySet() const;
        double GetSunIntensity() const;
        double GetSunIntensityFactor() const;

        void   SetFog(const Fog& fog);
        void   SetFog(const double visualrange);
        Fog    GetFog() const;
        double GetFogVisibilityRangeFactor() const;
        bool   IsFogSet() const;
        bool   IsFogBoundingBoxSet() const;

        void          SetPrecipitation(const Precipitation& precipitation);
        Precipitation GetPrecipitation() const;
        bool          IsPrecipitationSet() const;
        bool          IsPrecipitationIntensitySet() const;
        double        GetPrecipitationIntensity() const;

        void SetWind(const Wind& wind);
        Wind GetWind() const;
        bool IsWindSet() const;

        bool IsWeatherSet() const;

        void          SetRoadCondition(const RoadCondition& roadcondition);
        void          SetRoadCondition(const double friction);
        RoadCondition GetRoadCondition() const;
        bool          IsRoadConditionSet() const;

        void UpdateEnvironment(const OSCEnvironment& new_environment);
        bool IsEnvironment() const;
        bool IsEnvironmentUpdatedInViewer() const;
        void SetEnvironmentUpdatedInViewer(const bool isEnvironmentUpdatedInViewer);

    private:
        std::optional<TimeOfDay>     timeofday_;
        std::optional<double>        atmosphericpressure_;
        std::optional<double>        temperature_;
        std::optional<std::string>   fractionalcloudstate_;
        std::optional<Sun>           sun_;
        std::optional<Fog>           fog_;
        std::optional<Precipitation> precipitation_;
        std::optional<Wind>          wind_;
        std::optional<RoadCondition> roadcondition_;
        bool                         isEnvironmentUpdatedInViewer_ = false;
    };

}  // namespace scenarioengine
