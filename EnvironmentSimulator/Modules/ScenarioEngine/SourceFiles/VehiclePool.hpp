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

#include <map>

namespace scenarioengine
{

    class ScenarioReader;
    class Vehicle;

    struct VehicleGroup
    {
        std::vector<Vehicle*> vehicles;
        double                weight = 1.0;
    };

    class VehiclePool
    {
    public:
        VehiclePool() = default;
        VehiclePool(ScenarioReader* reader, const std::vector<std::pair<int, double>>* categories, bool accept_cars_with_trailer);
        ~VehiclePool();

        int  Initialize(ScenarioReader* reader, const std::vector<std::pair<int, double>>* categories, bool accept_cars_with_trailer);
        int  AddVehicle(Vehicle* vehicle);
        void DeleteVehicleAndTrailers(Vehicle* vehicle);
        const std::vector<Vehicle*>& GetVehicles(std::string category) const;
        Vehicle*                     GetVehicle(unsigned int index, std::string category = "");
        Vehicle*                     GetRandomVehicle(std::string category = "");
        bool                         Empty() const
        {
            return vehicle_pool_.empty();
        }

    private:
        std::map<std::string, VehicleGroup> vehicle_pool_;
        double                              total_weight_ = 0.0;
    };

}  // namespace scenarioengine