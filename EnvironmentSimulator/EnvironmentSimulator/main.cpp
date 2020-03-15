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

 /*
  * This application is an example of how to use the ScenarioEngine and Viewer modules to play and view scenarios.
  *
  * Instead of linking with the ScenarioEngineDLL it links all needed library functions statically into the 
  * all-inclusive executable.
  *
  * The Viewer is optionally (--threads) driven in a separate thread to enable camera movement even if simulation is paused.
  */

#include "playerbase.hpp"


int main(int argc, char *argv[])
{	
	ScenarioPlayer *player;

	try
	{
		player = new ScenarioPlayer(argc, argv);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}
	

	while (!player->IsQuitRequested())
	{
		player->Frame();
	}

	delete player;

	return 0;
}
