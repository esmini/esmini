#include "Story.hpp"
#include "CommonMini.hpp"


Story::Story()
{
	LOG("Story: New Story created");
}

void Story::printStory()
{

	LOG("---------------------------------------");
	LOG("Print of Story: %s", name.c_str());

	LOG("\tOwner = %s", owner.c_str());

	// Act
	for (size_t i = 0; i < Act.size(); i++)
	{
		LOG("Story - Act");
		LOG("\tname = %s", Act[i].name.c_str());

		// Sequence
		for (size_t j = 0; j < Act[i].Sequence.size(); j++)
		{
			LOG("Story - Act - Sequence");
			LOG("\tname = %s", Act[i].Sequence[j].name.c_str());
			LOG("\tnumberOfExecutions = %d", Act[i].Sequence[j].numberOfExecutions);
						
			LOG("Story - Act - Sequence - Actors - Entity");

			for (size_t k = 0; k < Act[i].Sequence[j].Actors.Entity.size(); k++)
			{
				LOG("\tname = %s", Act[i].Sequence[j].Actors.Entity[k].name.c_str());
			}

			LOG("Story - Act - Sequence - Actors - ByCondition");
			LOG("\tname = %s", Act[i].Sequence[j].Actors.ByCondition.actor.c_str());

			for (size_t k = 0; k < Act[i].Sequence[j].Maneuver.size(); k++)
			{
				LOG("Story - Act - Sequence - Maneuver");
				Act[i].Sequence[j].Maneuver[k].printOSCManeuver();
			}

		}

		// Conditions
		for (size_t j = 0; j < Act[i].Conditions.start.ConditionGroup.size(); j++)
		{
			LOG("Story - Act - Conditions - Start - ");
			Act[i].Conditions.start.ConditionGroup[j].printOSCConditionGroup();
		}
		LOG("Story - Act - Conditions - End");
		LOG("Story - Act - Conditions - Cancel");


	}

	LOG("---------------------------------------");
}

