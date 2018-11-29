#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "CommonMini.hpp"
#include "roadmanager.hpp"
#include "Entities.hpp"

class Entry
{
public:
	typedef enum
	{
		VEHICLE,
		DRIVER,
		PEDESTRIAN,
		PEDESTRIAN_CONTROLLER,
		MISC_OBJECT,
		ENVIRONMENT,
		MANEUVER,
		TRAJECTORY,
		ROUTE,
	} Type;

	std::string Type2Str(Type type)
	{
		if (type == VEHICLE) return "VEHICLE";
		else if (type == DRIVER) return "DRIVER";
		else if (type == PEDESTRIAN) return "PEDESTRIAN";
		else if (type == PEDESTRIAN_CONTROLLER) return "PEDESTRIAN_CONTROLLER";
		else if (type == MISC_OBJECT) return "MISC_OBJECT";
		else if (type == ENVIRONMENT) return "ENVIRONMENT";
		else if (type == MANEUVER) return "MANEUVER";
		else if (type == TRAJECTORY) return "TRAJECTORY";
		else if (type == ROUTE) return "ROUTE";
		else LOG("Type %d not recognized", type);

		return "";
	}

	void *element_;
	Type type_;
	std::string name_;

	Entry(Type type, std::string name, void *element) : type_(type), name_(name), element_(element) {}
	void* GetElement() { return element_; }
};


class Catalog
{
public:
	std::string name_;

	std::vector<Entry*> entry_;

	void AddEntry(Entry *entry)
	{
		entry_.push_back(entry);
	}

	Entry* FindEntryByName(std::string name)
	{
		for (size_t i = 0; i < entry_.size(); i++)
		{
			if(entry_[i]->name_ == name)
			{
				return  entry_[i];
			}
		}
		return 0;
	}
};

class Catalogs
{
public:

	std::vector<Catalog*> catalog_;

	Catalogs()
	{

	}

	Catalog* FindCatalogByName(std::string name)
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
		Entry *entry = 0;
		Catalog *catalog = FindCatalogByName(catalog_name);

		if (catalog == 0)
		{
			LOG("Couldn't find catalog %s", catalog_name.c_str());
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

	void *FindCatalogElement(std::string catalog_name, std::string entry_name)
	{
		Entry *entry = FindCatalogEntry(catalog_name, entry_name);
		void *element = 0;

		if (entry != 0)
		{
			element = entry->GetElement();
		}
		else
		{
			LOG("Couldn't get element in entry %s in catalog %s", entry_name.c_str(), catalog_name.c_str());
		}

		return element;
	}
};
