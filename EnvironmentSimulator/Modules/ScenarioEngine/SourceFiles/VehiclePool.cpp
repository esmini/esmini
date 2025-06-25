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

#include <random>
#include "VehiclePool.hpp"
#include "ScenarioReader.hpp"

using namespace scenarioengine;

VehiclePool::VehiclePool(ScenarioReader* reader, const std::vector<unsigned int>* categories, bool accept_trailers)
{
    Initialize(reader, categories, accept_trailers);
}

void VehiclePool::DeleteVehicleAndTrailers(Vehicle* vehicle)
{
    auto RecursiveDeleteTrailers = [](Vehicle* v, auto& recurseLambda) -> void
    {
        if (v->trailer_hitch_ && v->trailer_hitch_->trailer_vehicle_)
        {
            recurseLambda(static_cast<Vehicle*>(v->trailer_hitch_->trailer_vehicle_), recurseLambda);
        }

        delete v;
    };

    if (vehicle->trailer_hitch_ && vehicle->trailer_hitch_->trailer_vehicle_)
    {
        RecursiveDeleteTrailers(static_cast<Vehicle*>(vehicle->trailer_hitch_->trailer_vehicle_), RecursiveDeleteTrailers);
    }

    delete vehicle;
}

VehiclePool::~VehiclePool()
{
    for (auto* entry : vehicle_pool_)
    {
        DeleteVehicleAndTrailers(entry);
    }

    vehicle_pool_.clear();
}

int VehiclePool::Initialize(ScenarioReader* reader, const std::vector<unsigned int>* categories, bool accept_trailers)
{
    Catalogs* catalogs = reader->GetCatalogs();
    // Register model filesnames from first vehicle catalog
    // if no catalog loaded, use same model as central object
    for (size_t i = 0; i < catalogs->catalog_.size(); i++)
    {
        const Catalog* catalog = catalogs->catalog_[i];
        if (catalog->GetType() == CatalogType::CATALOG_VEHICLE)
        {
            for (size_t j = 0; j < catalog->entry_.size(); j++)
            {
                Vehicle* vehicle = reader->parseOSCVehicle(catalog->entry_[j]->GetNode());
                if (!(accept_trailers == false && vehicle->TrailerVehicle() != nullptr) &&
                    (((categories == nullptr || categories->empty()) &&
                      // default categories
                      (vehicle->category_ == Vehicle::Category::CAR || vehicle->category_ == Vehicle::Category::BUS ||
                       vehicle->category_ == Vehicle::Category::TRUCK || vehicle->category_ == Vehicle::Category::VAN ||
                       vehicle->category_ == Vehicle::Category::MOTORBIKE)) ||
                     // specified categories
                     (categories != nullptr && std::find(categories->begin(), categories->end(), vehicle->category_) != categories->end())))
                {
                    vehicle_pool_.push_back(vehicle);
                }
                else
                {
                    DeleteVehicleAndTrailers(vehicle);
                }
            }
        }
    }

    return 0;
}

int VehiclePool::AddVehicle(Vehicle* vehicle)
{
    if (vehicle != nullptr)
    {
        vehicle_pool_.push_back(vehicle);
        return static_cast<int>(vehicle_pool_.size() - 1);
    }
    return -1;
}

Vehicle* VehiclePool::GetVehicle(unsigned int index)
{
    if (index < vehicle_pool_.size())
    {
        return vehicle_pool_[index];
    }

    return nullptr;
}

Vehicle* VehiclePool::GetRandomVehicle()
{
    std::uniform_int_distribution<int> dist(0, static_cast<int>(vehicle_pool_.size() - 1));
    unsigned int                       number = static_cast<unsigned int>(dist(SE_Env::Inst().GetRand().GetGenerator()));

    return GetVehicle(number);
}
