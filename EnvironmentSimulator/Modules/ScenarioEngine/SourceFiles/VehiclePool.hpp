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

namespace scenarioengine
{

    class ScenarioReader;
    class Vehicle;

    class VehiclePool
    {
    public:
        VehiclePool() = default;
        VehiclePool(ScenarioReader* reader, const std::vector<unsigned int>* categories, bool accept_trailers);
        ~VehiclePool();

        int                          Initialize(ScenarioReader* reader, const std::vector<unsigned int>* categories, bool accept_trailers);
        int                          AddVehicle(Vehicle* vehicle);
        const std::vector<Vehicle*>& GetVehicles() const
        {
            return vehicle_pool_;
        }
        Vehicle* GetVehicle(unsigned int index);
        Vehicle* GetRandomVehicle();

    private:
        std::vector<Vehicle*> vehicle_pool_;
    };

}  // namespace scenarioengine