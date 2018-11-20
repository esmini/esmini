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

	typedef enum
	{
		NOT_TRIGGED,
		ACTIVE,
		DONE
	} State;

	BaseType base_type_;
	State state_;

	OSCAction(BaseType type) : base_type_(type), state_(State::NOT_TRIGGED)
	{
		LOG("");
	}

	virtual void Trig()
	{
		state_ = State::ACTIVE;
		LOG("Trigged %d", base_type_);
	}

	virtual void Step(double dt)
	{
		LOG("Virutal, should be overridden!");
	};
};
