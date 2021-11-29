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
#include "OSCBoundingBox.hpp"
#include "OSCProperties.hpp"

namespace scenarioengine
{

    /*
    Class PrecipitationType
    Represents the different types of precipitations
    */
    typedef enum
    {
        DRY,
        RAIN,
        SNOW
    } PrecipitationType;

    /*
    Class CloudState
    Represents the different types of cloud states, note: implemented for future 1.2 enumerations with 1.1 included
    */
    typedef enum
    {
        CLOUDLESS, // osc <=1.1 free
        SUNNE,
        SERENE,
        SLIGHTLY_CLOUDY,
        LIGHT_CLOUDY, // osc <=1.1 cloudy
        CLOUDY,
        HEAVILY_CLOUDY, // osc <=1.1 rainy
        ALMOST_OVERCAST,
        OVERCAST, // osc <=1.1 overcast
        SKY_NOT_VISIBLE // osc <=1.1 sky off
    } CloudState;

    /*
    Class Fog
    Represents a fog entry of openscenario

    visibility_range 	double 	 	visibility range in the fog
    boundingbox_     	double 	 	The boundingbox of the fog (optional)
    */
    typedef struct
    {
        float visibility_range;
        OSCBoundingBox boundingbox;
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
        double azimuth;
        double elevation;
        double intensity;
    } Sun;
    
    /*
    Class Precipitation
    Represents the state of rain/snow

    precipitationintensity 	    double 	 	        intensity of precipitation
    precipitationtype     	    PrecipitationType   type of precipitation
    
    */
    typedef struct 
    {
        double precipitationintensity;
        PrecipitationType precipitationtype;
    } Precipitation;

    /*
    Class Wind
    Represents the state of the wind

    direction 	    double      direction of the wind in the x/y plane
    speed           double     	speed of the wind
    
    */
    typedef struct 
    {
        double direction;
        double speed;
    } Wind;

    /*
    Class TimeOfDay
    Represents the state of the wind

    animation 	    double      if an animation should be used
    speed           dateTime  	NOT IMPLEMENTED YET
    
    */
    typedef struct 
    {
        bool animation;
        std::string datetime;
    } TimeOfDay;


    /*
    Class RoadCondition
    Represents the state of the wind

    frictionscalefactor 	    double      the friction scale factor
    properties                  Properties  additional properties of the RoadCondition
    
    */
    typedef struct 
    {
        double frictionscalefactor;
        OSCProperties properties;
    } RoadCondition;


    /*
    Class OSCEnvironment
    Defines the full environment of OpenSCENARIO
    
    */
    class OSCEnvironment
    {
        public:
            OSCEnvironment();
            ~OSCEnvironment();

            void SetAtmosphericPressure(double atmosphericpressure) {atmosphericpressure_ = atmosphericpressure; pressureset_ = true;}
            bool IsAtmosphericPressure() {return pressureset_;}
            double GetAtmosphericPressure();
            
            void SetTemperature(double temperature) {temperature_ = temperature; temperatureset_ = true;}
            bool IsTemperature() {return temperatureset_;}
            double GetTemperature();

            void SetCloudState(CloudState cloudstate) {cloudstate_ = cloudstate; cloudstateset_ = true;}
            void SetCloudState(CloudState* new_cloudstate) {cloudstate_ = *new_cloudstate;}
            bool IsCloudState() {return cloudstateset_;}
            CloudState GetCloudState();

            void SetFog(double visualrange) {fog_.visibility_range = visualrange; fogset_ = true;}
            void SetFog(double visualrange, OSCBoundingBox bb) {fog_.visibility_range = visualrange; fog_.boundingbox = bb; fogset_ = true;}
            void SetFog(Fog* new_fog) {fog_ = *new_fog;}
            bool IsFog() {return fogset_;}
            Fog *GetFog();

            void SetWind(double direction, double speed) {wind_.direction = direction; wind_.speed = speed; windset_ = true; }
            void SetWind(Wind* new_wind) {wind_ = *new_wind;}
            bool IsWind() {return windset_;}
            Wind *GetWind();

            void SetPrecipitation(double intensity, PrecipitationType type) {precipitation_.precipitationintensity = intensity; precipitation_.precipitationtype = type; precipitationset_ = true;}
            void SetPrecipitation(Precipitation *new_precipitation) {precipitation_ = *new_precipitation;}
            bool IsPrecipitation() {return precipitationset_;}
            Precipitation *GetPrecipitation();

            void SetSun(double azimuth, double elevation, double intensity) {sun_.azimuth = azimuth; sun_.elevation = elevation; sun_.intensity = intensity; sunset_ = true;}
            void SetSun(Sun *new_sun) {sun_ = *new_sun; sunset_ = true;}
            bool IsSun() {return sunset_;}
            Sun *GetSun();
    
            void SetTimeOfDay(double animation, std::string tod) {timeofday_.animation = animation; timeofday_.datetime = tod; timeofdayset_ = true;}
            void SetTimeOfDay(TimeOfDay *new_timeofday) {timeofday_ = *new_timeofday;timeofdayset_ = true;}
            bool IsTimeOfDay() {return timeofdayset_;}
            TimeOfDay *GetTimeOfDay();

            void SetRoadCondition(double friction) {roadcondition_.frictionscalefactor = friction; roadconditionset_ = true;}
            void SetRoadCondition(RoadCondition *new_roadcondition) {roadcondition_ = *new_roadcondition; roadconditionset_ = true;}
            bool IsRoadCondition() {return roadconditionset_;}
            RoadCondition *GetRoadCondition();

            void UpdateEnvironment(OSCEnvironment *new_environment);

            
        private:
            double atmosphericpressure_;
            bool pressureset_;
            double temperature_;
            bool temperatureset_;
            CloudState cloudstate_;
            bool cloudstateset_;
            Fog fog_;
            bool fogset_;
            Wind wind_;
            bool windset_;
            Precipitation precipitation_;
            bool precipitationset_;
            Sun sun_;
            bool sunset_;
            RoadCondition roadcondition_;
            bool roadconditionset_;
            TimeOfDay timeofday_;
            bool timeofdayset_;

    };
    
};
