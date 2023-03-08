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

#include <iostream>
#include <string>
#include <vector>

#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "Entities.hpp"

namespace scenarioengine
{
    typedef enum
    {
        CATALOG_UNDEFINED,
        CATALOG_VEHICLE,
        CATALOG_DRIVER,
        CATALOG_PEDESTRIAN,
        CATALOG_PEDESTRIAN_CONTROLLER,
        CATALOG_MISC_OBJECT,
        CATALOG_ENVIRONMENT,
        CATALOG_MANEUVER,
        CATALOG_TRAJECTORY,
        CATALOG_ROUTE,
        CATALOG_CONTROLLER
    } CatalogType;

    class Entry
    {
    public:
        std::string        name_;
        pugi::xml_document root_;
        CatalogType        type_;

        Entry(std::string name, pugi::xml_document root);
        pugi::xml_node GetNode()
        {
            return root_.first_child();
        }

        static std::string GetTypeAsStr_(CatalogType type);
        std::string        GetTypeAsStr()
        {
            return GetTypeAsStr_(type_);
        }
        CatalogType GetTypeByNodeName(pugi::xml_node node);
    };

    class Catalog
    {
    public:
        std::string          name_;
        CatalogType          type_;
        std::vector<Entry *> entry_;

        ~Catalog()
        {
            for (auto *entry : entry_)
            {
                delete entry;
            }
        }

        CatalogType GetType()
        {
            return type_;
        }

        void AddEntry(Entry *entry)
        {
            entry_.push_back(entry);
        }

        Entry *FindEntryByName(std::string name)
        {
            for (size_t i = 0; i < entry_.size(); i++)
            {
                if (entry_[i]->name_ == name)
                {
                    return entry_[i];
                }
            }
            return 0;
        }

        std::string GetTypeAsStr()
        {
            return Entry::GetTypeAsStr_(type_);
        }
    };

    class Catalogs
    {
    public:
        typedef struct
        {
            CatalogType type_;
            std::string dir_name_;
        } CatalogDirEntry;

        std::vector<CatalogDirEntry> catalog_dirs_;
        std::vector<Catalog *>       catalog_;

        Catalogs()
        {
        }
        ~Catalogs()
        {
            for (auto *entry : catalog_)
            {
                delete entry;
            }
        }

        int RegisterCatalogDirectory(std::string type, std::string directory);

        Catalog *FindCatalogByName(std::string name)
        {
            for (size_t i = 0; i < catalog_.size(); i++)
            {
                if (catalog_[i]->name_ == name)
                {
                    return catalog_[i];
                }
            }

            return 0;
        }

        void AddCatalog(Catalog *catalog)
        {
            catalog_.push_back(catalog);
        }

        Entry *FindCatalogEntry(std::string catalog_name, std::string entry_name)
        {
            Entry   *entry   = 0;
            Catalog *catalog = FindCatalogByName(catalog_name);

            if (catalog == 0)
            {
                LOG("Couldn't find catalog %s", catalog_name.c_str());
                return 0;
            }
            else
            {
                entry = catalog->FindEntryByName(entry_name);
                if (entry == 0)
                {
                    LOG("Couldn't find entry %s in catalog %s", entry_name.c_str(), catalog_name.c_str());
                }
            }
            return entry;
        }

        pugi::xml_node FindCatalogNode(std::string catalog_name, std::string entry_name)
        {
            Entry         *entry = FindCatalogEntry(catalog_name, entry_name);
            pugi::xml_node node;

            if (entry != 0)
            {
                node = entry->GetNode();
            }
            else
            {
                LOG("Couldn't get element in entry %s in catalog %s", entry_name.c_str(), catalog_name.c_str());
            }

            return node;
        }
    };

}  // namespace scenarioengine
