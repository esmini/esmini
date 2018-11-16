#pragma once

#include "Entities.hpp"


class OSCAction
{
public:
	typedef enum
	{
		GLOBAL,
		USER_DEFINED,
		PRIVATE,
	} BaseType;

	BaseType base_type_;
	bool active_;

	OSCAction(BaseType type) : base_type_(type), active_(false)
	{
		LOG("");
	}

	virtual void Step(double dt, Object *object)
	{
		LOG("Virutal, should be overridden!");
	};
};
