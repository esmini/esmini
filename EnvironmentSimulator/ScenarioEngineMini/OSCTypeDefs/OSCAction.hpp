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
			NOT_TRIGGED,
			ACTIVE,
			DONE
		} State;

		BaseType base_type_;
		State state_;
		std::string name_;

		OSCAction(BaseType type) : base_type_(type), state_(State::NOT_TRIGGED)
		{
			LOG("");
		}

		std::string basetype2str(BaseType type);

		virtual void Trig()
		{
			state_ = State::ACTIVE;
			LOG("Trigged action %s of type %d", name_.c_str(), base_type_);
		}

		void Stop()
		{
			if (state_ = State::ACTIVE)
			{
				LOG("Action %s, base type: %s, stopped", name_.c_str(), basetype2str(base_type_).c_str());
				state_ = DONE;
			}
		}

		void Done()
		{
			if (state_ = State::ACTIVE)
			{
				LOG("Action %s, base type: %s, done", name_.c_str(), basetype2str(base_type_).c_str());
				state_ = DONE;
			}
		}

		virtual void Step(double dt)
		{
			LOG("Virutal, should be overridden!");
		};
	};

}