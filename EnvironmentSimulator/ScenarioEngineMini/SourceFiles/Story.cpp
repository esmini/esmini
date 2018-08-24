#include "Story.hpp"


Story::Story()
{
	std::cout << "Story: New Story created" << std::endl;
}

void Story::printStory()
{

	std::cout << "---------------------------------------" << std::endl;
	std::cout << "Print of Story: " << name << "\n" << std::endl;

	std::cout << "\t" << "Owner = " << owner << std::endl;
	std::cout << "\n" << std::endl;

	// Act
	for (size_t i = 0; i < Act.size(); i++)
	{
		std::cout << "Story - Act" << std::endl;
		std::cout << "\t" << "name = " << Act[i].name << std::endl;
		std::cout << "\n" << std::endl;

		// Sequence
		for (size_t j = 0; j < Act[i].Sequence.size(); j++)
		{
			std::cout << "Story - Act - Sequence" << std::endl;
			std::cout << "\t" << "name = " << Act[i].Sequence[j].name << std::endl;
			std::cout << "\t" << "numberOfExecutions = " << Act[i].Sequence[j].numberOfExecutions << std::endl;
			std::cout << "\n" << std::endl;
						
			std::cout << "Story - Act - Sequence - Actors - Entity" << std::endl;

			for (size_t k = 0; k < Act[i].Sequence[j].Actors.Entity.size(); k++)
			{
				std::cout << "\t" << "name = " << Act[i].Sequence[j].Actors.Entity[k].name << std::endl;
				std::cout << "\n" << std::endl;
			}

			std::cout << "Story - Act - Sequence - Actors - ByCondition" << std::endl;
			std::cout << "\t" << "name = " << Act[i].Sequence[j].Actors.ByCondition.actor << std::endl;
			std::cout << std::endl;

			for (size_t k = 0; k < Act[i].Sequence[j].Maneuver.size(); k++)
			{
				std::cout << "Story - Act - Sequence - Maneuver" << std::endl;
				Act[i].Sequence[j].Maneuver[k].printOSCManeuver();
			}

		}

		// Conditions
		for (size_t j = 0; j < Act[i].Conditions.start.ConditionGroup.size(); j++)
		{
			std::cout << "Story - Act - Conditions - Start - " << std::endl;
			Act[i].Conditions.start.ConditionGroup[j].printOSCConditionGroup();
		}
		std::cout << "Story - Act - Conditions - End" << std::endl;
		std::cout << "Story - Act - Conditions - Cancel" << std::endl;


	}

	std::cout << "---------------------------------------" << std::endl;
	std::cout << "\n" << std::endl;
}

