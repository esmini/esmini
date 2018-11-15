#pragma once

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

	OSCAction(BaseType type) : base_type_(type)
	{
		LOG("");
	}

	virtual void Step(double dt)
	{
		LOG("Virutal, should be overridden!");
	};
};
