#pragma once

#include "Entities.hpp"

namespace scenarioengine
{

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
			INACTIVE,
			TRIGGED,
			ACTIVATED,      // Just activated - this state last for one step
			ACTIVE,
			DEACTIVATED,    // Just done/deactivated - this state last for one step
		} State;

		BaseType base_type_;
		State state_;
		std::string name_;

		OSCAction(BaseType type) : base_type_(type), state_(State::INACTIVE)
		{
			LOG("");
		}

		std::string basetype2str(BaseType type);

		bool IsActive()
		{
			return state_ == State::TRIGGED || state_ == State::ACTIVATED || state_ == State::ACTIVE;
		}

		virtual void Trig()
		{
			state_ = State::TRIGGED;
			LOG("Action %s (%s) trigged", name_.c_str(), basetype2str(base_type_).c_str());
		}

		void Stop()
		{
			if (IsActive())
			{
				LOG("Action %s (%s) stopped", name_.c_str(), basetype2str(base_type_).c_str());
				state_ = State::DEACTIVATED;
			}
		}

		virtual void Step(double dt)
		{
			(void)dt;
			LOG("Virutal, should be overridden!");
			
			if (state_ == State::ACTIVATED)
			{
				state_ = State::ACTIVE;
			}
			else if (state_ == State::DEACTIVATED)
			{
				state_ = INACTIVE;
			}
		}
	};

}
