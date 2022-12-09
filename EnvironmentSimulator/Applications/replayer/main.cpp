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
  * This application uses the Replay class to read and replay binary recordings of previously executed scenarios
  */

#include <random>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <map>

#include "viewer.hpp"
#include "ScenarioGateway.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.hpp"
#include "CommonMini.hpp"
#include "Replay.hpp"
#include "helpText.hpp"
#include "collision.hpp"

using namespace scenarioengine;

#define TIME_SCALE_FACTOR 1.1
#define GHOST_CTRL_TYPE 100    // control type 100 indicates ghost
#define JUMP_DELTA_TIME_LARGE 1.0
#define JUMP_DELTA_TIME_SMALL 0.1

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.001;
static bool pause_player = false;  // continuous play
static double time_scale = 1.0;
static bool no_ghost = false;
static bool no_ghost_model = false;
static std::vector<int> removeObjects;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

static std::vector<ScenarioEntity> scenarioEntity;


void log_callback(const char* str)
{
	printf("%s\n", str);
}

void setEntityVisibility(int index, bool visible)
{
	if (index >= 0 && index < scenarioEntity.size())
	{
		if (visible != scenarioEntity[index].visible)
		{
			scenarioEntity[index].entityModel->txNode_->setNodeMask(visible ? 0xffffffff : 0x0);
			scenarioEntity[index].visible = visible;
			if (scenarioEntity[index].trajectory)
			{
				scenarioEntity[index].trajectory->SetNodeMaskLines(visible ? 0xffffffff : 0x0);
			}
		}
	}
}

ScenarioEntity *getScenarioEntityById(int id)
{
	for (size_t i=0; i<scenarioEntity.size(); i++)
	{
		if (scenarioEntity[i].id == id)
		{
			return &scenarioEntity[i];
		}
	}

	return 0;
}

int ShowGhosts(Replay* player, bool show)
{
	ObjectStateStructDat* state = 0;

	for (size_t j = 0; j < scenarioEntity.size(); j++)
	{
		ScenarioEntity* entity = &scenarioEntity[j];
		state = player->GetState(entity->id);

		if (entity->entityModel != nullptr && state->info.ctrl_type == GHOST_CTRL_TYPE)
		{
			entity->entityModel->txNode_->setNodeMask(show ? 0xffffffff : 0x0);
		}
	}

	return 0;
}

int ParseEntities(viewer::Viewer* viewer, Replay* player)
{
	double minTrajPointDist = 1;
	double z_offset = 0.2;
	double width = 1.75;

	struct OdoInfo
	{
		double x, y, odometer;
	};
	std::map<int, OdoInfo> odo_info;  // temporary keep track of entity odometers

	for (int i = 0; i < player->data_.size(); i++)
	{
		ReplayEntry* entry = &player->data_[i];
		ObjectStateStructDat* state = &entry->state;
		OdoInfo odo_entry;

		if (no_ghost && state->info.ctrl_type == GHOST_CTRL_TYPE)
		{
			continue;
		}

		if (std::find(removeObjects.begin(), removeObjects.end(), state->info.id) != removeObjects.end())
		{
			continue;
		}

		ScenarioEntity* sc = getScenarioEntityById(state->info.id);

		// If not available, create it
		if (sc == 0)
		{
			ScenarioEntity new_sc;

			new_sc.id = state->info.id;
			new_sc.trajPoints = 0;
			new_sc.pos = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f };
			new_sc.trajectory = nullptr;
			new_sc.wheel_angle = 0.0f;
			new_sc.wheel_rotation = 0.0f;
			new_sc.name = state->info.name;
			new_sc.visible = true;
			std::string filename;
			if (state->info.model_id >= 0)
			{
				filename = SE_Env::Inst().GetModelFilenameById(state->info.model_id);
			}

			if ((new_sc.entityModel = viewer->CreateEntityModel(filename.c_str(), osg::Vec4(0.5, 0.5, 0.5, 1.0),
				viewer::EntityModel::EntityType::VEHICLE, false, state->info.name, &state->info.boundingbox,
				static_cast<EntityScaleMode>(state->info.scaleMode))) == 0)
			{
				return -1;
			}
			else
			{
				if (viewer->AddEntityModel(new_sc.entityModel) != 0)
				{
					return -1;
				}
			}

			if (state->info.ctrl_type == GHOST_CTRL_TYPE && no_ghost_model)
			{
				new_sc.entityModel->txNode_->setNodeMask(0x0);
			}

			new_sc.bounding_box = state->info.boundingbox;

			// Add it to the list of scenario cars
			scenarioEntity.push_back(new_sc);

			sc = &scenarioEntity.back();

			odo_entry.x = state->pos.x;
			odo_entry.y = state->pos.y;
			odo_entry.odometer = 0.0;

			odo_info.insert(std::make_pair(new_sc.id, odo_entry));  // Set inital odometer value for the entity
		}

		if (sc->trajPoints == 0)
		{
			sc->trajPoints = new osg::Vec3Array;
		}

		if (sc->trajPoints->size() == 0)
		{
			sc->trajPoints->push_back(osg::Vec3d(state->pos.x, state->pos.y, state->pos.z + z_offset));
		}
		else
		{
			if (sc->trajPoints->size() > 2 && GetLengthOfLine2D(state->pos.x, state->pos.y,
				(*sc->trajPoints)[sc->trajPoints->size()-2][0], (*sc->trajPoints)[sc->trajPoints->size()-2][1]) < minTrajPointDist)
			{
				// Replace last point until distance is above threshold
				sc->trajPoints->back() = osg::Vec3d(state->pos.x, state->pos.y, state->pos.z + z_offset);
			}
			else
			{
				sc->trajPoints->push_back(osg::Vec3d(state->pos.x, state->pos.y, state->pos.z + z_offset));
			}
		}

		// calculate odometer
		odo_entry = odo_info[sc->id];
		double delta = GetLengthOfLine2D(odo_entry.x, odo_entry.y, state->pos.x, state->pos.y);
		odo_entry.x = state->pos.x;
		odo_entry.y = state->pos.y;
		odo_entry.odometer += delta;
		odo_info[sc->id] = odo_entry;  // save updated odo info for next calculation

		entry->odometer = odo_entry.odometer;  // update odometer
	}

	for (int i = 0; i < scenarioEntity.size(); i++)
	{
		osg::Vec4 color;
		if (scenarioEntity[i].id == 0)
		{
			color = osg::Vec4d(0.9, 0.8, 0.75, 1.0);
		}
		else
		{
			//color = osg::Vec4d(0.9, 0.3, 0.2, 1.0);
			color = osg::Vec4d(0.9, 0.7, 0.3, 1.0);
		}
		scenarioEntity[i].trajectory = viewer->AddPolyLine(viewer->trajectoryLines_, scenarioEntity[i].trajPoints, color, width);
	}

	return 0;
}

int GetGhostIdx()
{
	for (size_t i = 0; i < scenarioEntity.size(); i++)
	{
		if (scenarioEntity[i].name.find("_ghost") != std::string::npos)
		{
			return i;
		}
	}
	return -1; // No ghost
}

void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
{
	Replay* player = (Replay*)data;

	if (keyEvent->down_)
	{
		if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Right))
		{
			if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL) &&
				keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
			{
				player->GoToDeltaTime(JUMP_DELTA_TIME_LARGE);
			}
			else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
			{
				player->GoToDeltaTime(JUMP_DELTA_TIME_SMALL);
			}
			else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL))
			{
				player->GoToEnd();
			}
			else
			{
				player->GoToNextFrame();
			}

			pause_player = true;  // step by step
		}
		else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Left))
		{
			if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL) &&
				keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
			{
				player->GoToDeltaTime(-JUMP_DELTA_TIME_LARGE);
			}
			else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
			{
				player->GoToDeltaTime(-JUMP_DELTA_TIME_SMALL);
			}
			else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL))
			{
				// rewind to beginning
				player->GoToStart();
			}
			else
			{
				player->GoToPreviousFrame();
			}

			pause_player = true;  // step by step
		}
		else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
		{
			pause_player = !pause_player;
		}
		else if (keyEvent->key_ == 'H')
		{
			puts(helpText);
		}
		else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Up))
		{
			time_scale = MIN(100, time_scale * TIME_SCALE_FACTOR);
		}
		else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Down))
		{
			time_scale = MAX(0.01, time_scale / TIME_SCALE_FACTOR);
		}
		else if (keyEvent->key_ == static_cast<int>('g'))
		{
			no_ghost_model = !no_ghost_model;
			ShowGhosts(player, !no_ghost_model);
		}
	}
}


int main(int argc, char** argv)
{
	roadmanager::OpenDrive *odrManager;
	viewer::Viewer* viewer;
	std::unique_ptr<Replay> player;
	double simTime = 0;
	double view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL;
	bool overlap = false;
	static char info_str_buf[256];
	std::string arg_str;

	// Use logger callback for console output instead of logfile
	Logger::Inst().SetCallback(log_callback);
	Logger::Inst().LogVersion();

	SE_Env::Inst().AddPath(DirNameOf(argv[0]));  // Add location of exe file to search paths

	// use common options parser to manage the program arguments
	SE_Options opt;
	opt.AddOption("file", "Simulation recording data file (.dat)", "filename");
	opt.AddOption("camera_mode", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\", \"top\", \"driver\") (toggle during simulation by press 'k') ", "mode");
	opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
	opt.AddOption("collision", "Pauses the replay if the ego collides with another entity");
	opt.AddOption("custom_camera", "Additional custom fixed camera position <x,y,z,h,p> (multiple occurrences supported)", "position");
	opt.AddOption("custom_fixed_camera", "Additional custom camera position <x,y,z>[,h,p] (multiple occurrences supported)", "position and optional orientation");
	opt.AddOption("custom_fixed_top_camera", "Additional custom top camera <x,y,z,rot> (multiple occurrences supported)", "position and rotation");
	opt.AddOption("dir", "Directory containing replays to overlay, pair with \"file\" argument, where \"file\" is .dat filename match substring","path");
	opt.AddOption("disable_off_screen", "Disable esmini off-screen rendering, revert to OSG viewer default handling");
	opt.AddOption("hide_trajectories", "Hide trajectories from start (toggle with key 'n')");
	opt.AddOption("info_text", "Show on-screen info text (toggle key 'i') mode 0=None 1=current (default) 2=per_object 3=both", "mode");
	opt.AddOption("no_ghost", "Remove ghost entities");
	opt.AddOption("no_ghost_model", "Remove only ghost model, show trajectory (toggle with key 'g')");
	opt.AddOption("path", "Search path prefix for assets, e.g. model_ids.txt file (multiple occurrences supported)", "path");
	opt.AddOption("quit_at_end", "Quit application when reaching end of scenario");
	opt.AddOption("remove_object", "Remove object(s). Multiple ids separated by comma, e.g. 2,3,4.", "id");
	opt.AddOption("repeat", "loop scenario");
	opt.AddOption("res_path", "Path to resources root folder - relative or absolut", "path");
	opt.AddOption("road_features", "Show OpenDRIVE road features");
	opt.AddOption("save_merged", "Save merged data into one dat file, instead of viewing", "filename");
	opt.AddOption("start_time", "Start playing at timestamp", "ms");
	opt.AddOption("stop_time", "Stop playing at timestamp (set equal to time_start for single frame)", "ms");
	opt.AddOption("time_scale", "Playback speed scale factor (1.0 == normal)", "factor");
	opt.AddOption("view_mode", "Entity visualization: \"model\"(default)/\"boundingbox\"/\"both\"", "view_mode");

	if (argc < 2)
	{
		opt.PrintUsage();
#ifdef _USE_OSG
		viewer::Viewer::PrintUsage();
#endif
		return -1;
	}

	if (opt.ParseArgs(argc, argv) != 0)
	{
		return -1;
	}

	if (opt.GetOptionArg("file").empty())
	{
		printf("Missing file argument\n");
		opt.PrintUsage();
#ifdef _USE_OSG
		viewer::Viewer::PrintUsage();
#endif
		return -1;
	}

	if (opt.GetOptionArg("path") != "")
	{
		int counter = 0;
		while ((arg_str = opt.GetOptionArg("path", counter)) != "")
		{
			SE_Env::Inst().AddPath(arg_str);
			LOG("Added path %s", arg_str.c_str());
			counter++;
		}
	}

	arg_str = opt.GetOptionArg("res_path");
	if (!arg_str.empty())
	{
		SE_Env::Inst().AddPath(arg_str);
	}

	if (opt.GetOptionSet("disable_off_screen"))
	{
		SE_Env::Inst().SetOffScreenRendering(false);
	}

	// Create player
	arg_str = opt.GetOptionArg("dir");

	std::string save_merged = opt.GetOptionArg("save_merged"); // name of new dat file
	try
	{
		if (!arg_str.empty())
		{
			player = std::make_unique<Replay>(arg_str, opt.GetOptionArg("file"), save_merged);

			if (!save_merged.empty())
			{
				LOG("Merged data saved in %s", save_merged.c_str());
				return 0;
			}
		}
		else
		{
			if (!save_merged.empty())
			{
				LOG("\"--saved_merged\" works only in combination with \"--dir\" argument, combining multiple dat files");
				return -1;
			}
			player = std::make_unique<Replay>(opt.GetOptionArg("file"), true);
		}
	}
	catch (const std::exception& e)
	{
		LOG(std::string("Exception: ").append(e.what()).c_str());
		return -1;
	}

	try
	{
		if (strcmp(player->header_.odr_filename, ""))
		{
			// find and OpenDRIVE file. Test some combinations of paths and filename
			std::vector<std::string> file_name_candidates;

			// just filepath as stated in .dat file
			file_name_candidates.push_back(player->header_.odr_filename);

			// Check registered paths
			for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
			{
				// Including file path
				file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], player->header_.odr_filename));

				// Excluding file path
				file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(player->header_.odr_filename)));

				// Including file path and xodr sub folder
				file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i] + "/xodr/", FileNameOf(player->header_.odr_filename)));

				// Excluding file path but add xodr sub folder
				file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i] + "/xodr/", player->header_.odr_filename));
			}

			size_t i;
			for (i = 0; i < file_name_candidates.size(); i++)
			{
				if (FileExists(file_name_candidates[i].c_str()))
				{
					if (roadmanager::Position::LoadOpenDrive(file_name_candidates[i].c_str()))
					{
						break;
					}
				}
			}

			if (i == file_name_candidates.size())
			{
				printf("Failed to load OpenDRIVE file %s. Tried:\n", player->header_.odr_filename);
				for (int j = 0; j < file_name_candidates.size(); j++)
				{
					printf("   %s\n", file_name_candidates[j].c_str());
				}
				printf("continue without road description\n");
			}
		}

		odrManager = roadmanager::Position::GetOpenDrive();

		osg::ArgumentParser arguments(&argc, argv);

		viewer = new viewer::Viewer(
			odrManager,
			player->header_.model_filename,
			NULL,
			argv[0],
			arguments, &opt);

		if (viewer == nullptr)
		{
			printf("Failed to create viewer");
			return -1;
		}

		if ((arg_str = opt.GetOptionArg("camera_mode")) != "")
		{
			if (arg_str == "orbit")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_ORBIT);
			}
			else if (arg_str == "fixed")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_FIXED);
			}
			else if (arg_str == "flex")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND);
			}
			else if (arg_str == "flex-orbit")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND_ORBIT);
			}
			else if (arg_str == "top")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_TOP);
			}
			else if (arg_str == "driver")
			{
				viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_DRIVER);
			}
			else
			{
				LOG("Unsupported camera mode: %s - using default (orbit)", arg_str.c_str());
			}
		}

		if (opt.GetOptionSet("custom_camera") == true)
		{
			int counter = 0;

			while ((arg_str = opt.GetOptionArg("custom_camera", counter)) != "")
			{
				size_t pos = 0;
				double v[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
				for (int i = 0; i < 5; i++)
				{
					pos = arg_str.find(",");
					if (i < 4 && pos == std::string::npos)
					{
						LOG_AND_QUIT("Expected custom_camera <x,y,z,h,p>, got only %d values", i + 1);
					}
					v[i] = strtod(arg_str.substr(0, pos));
					arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);
				}
				if (!arg_str.empty())
				{
					LOG_AND_QUIT("Expected custom_camera <x,y,z,h,p>, got too many values. Make sure only 5 values is specified");
				}
				viewer->AddCustomCamera(v[0], v[1], v[2], v[3], v[4]);
				viewer->SetCameraMode(-1);  // activate last camera which is the one just added
				LOG("Created custom camera %d (%.2f, %.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3], v[4]);
				counter++;
			}
		}

		if (opt.GetOptionSet("custom_fixed_camera") == true)
		{
			int counter = 0;

			while ((arg_str = opt.GetOptionArg("custom_fixed_camera", counter)) != "")
			{
				size_t pos = 0;
				double v[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
				int i = 0;
				for (i = 0; i < 5; i++)
				{
					pos = arg_str.find(",");

					if (i < 2 && pos == std::string::npos)
					{
						LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got only %d values", i + 1);
					}
					else if (i == 3 && pos == std::string::npos)
					{
						LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got %d values", i + 1);
					}
					v[i] = strtod(arg_str.substr(0, pos));
					arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);

					if (i == 2 && pos == std::string::npos)
					{
						// Only position specified, stop now

						break;
					}
				}
				if (!arg_str.empty())
				{
					LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got too many values. Make sure only 3 or 5 values is specified");
				}

				if (i == 2)
				{
					viewer->AddCustomCamera(v[0], v[1], v[2], true);
					LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f)", counter, v[0], v[1], v[2]);
				}
				else
				{
					viewer->AddCustomCamera(v[0], v[1], v[2], v[3], v[4], true);
					LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3], v[4]);
				}
				viewer->SetCameraMode(-1);  // activate last camera which is the one just added
				counter++;
			}
		}

		if (opt.GetOptionSet("custom_fixed_top_camera") == true)
		{
			int counter = 0;

			while ((arg_str = opt.GetOptionArg("custom_fixed_top_camera", counter)) != "")
			{
				size_t pos = 0;
				double v[4] = { 0.0, 0.0, 0.0, 0.0 };
				for (int i = 0; i < 4; i++)
				{
					pos = arg_str.find(",");
					if (i < 3 && pos == std::string::npos)
					{
						LOG_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>, got only %d values", i + 1);
					}
					v[i] = strtod(arg_str.substr(0, pos));
					arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);
				}
				if (!arg_str.empty())
				{
					LOG_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>, got too many values. Make sure only 4 values is specified");
				}

				viewer->AddCustomFixedTopCamera(v[0], v[1], v[2], v[3]);
				viewer->SetCameraMode(-1);  // activate last camera which is the one just added
				LOG("Created custom fixed top camera %d (%.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3]);
				counter++;
			}
		}

		if ((arg_str = opt.GetOptionArg("info_text")) != "")
		{
			int mask = strtoi(arg_str);
			if (mask < 0 || mask > 3)
			{
				LOG_AND_QUIT("Invalid on-screen info mode %d. Valid range is 0-3", mask);
			}
			viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO |
				viewer::NodeMask::NODE_MASK_INFO_PER_OBJ, mask * viewer::NodeMask::NODE_MASK_INFO);
		}

		viewer->RegisterKeyEventCallback(ReportKeyEvent, player.get());

		if (opt.HasUnknownArgs())
		{
			opt.PrintUnknownArgs("Unrecognized arguments:");
			opt.PrintUsage();
#ifdef _USE_OSG
			viewer::Viewer::PrintUsage();
#endif
			return -1;
		}
		viewer->SetWindowTitle("esmini - " + FileNameWithoutExtOf(argv[0]) + " " + (FileNameOf(opt.GetOptionArg("file"))));

		__int64 now = 0;
		__int64 lastTimeStamp = 0;

		if (opt.GetOptionSet("time_scale"))
		{
			time_scale = atof(opt.GetOptionArg("time_scale").c_str());
			if (time_scale < SMALL_NUMBER)
			{
				time_scale = SMALL_NUMBER;
			}
		}

		if (opt.GetOptionSet("repeat"))
		{
			player->SetRepeat(true);
		}

		if (opt.GetOptionSet("capture_screen"))
		{
			LOG("Activate continuous screen capture");
			viewer->SaveImagesToFile(-1);
		}

		if (opt.GetOptionSet("road_features"))
		{
			viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
		}
		else
		{
			viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES, 0x0);
		}

		// Set visual representation of entities
		std::string view_mode_string = opt.GetOptionArg("view_mode");
		if (view_mode_string == "boundingbox")
		{
			view_mode = viewer::NodeMask::NODE_MASK_ENTITY_BB;
		}
		else if (view_mode_string == "both")
		{
			view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL | viewer::NodeMask::NODE_MASK_ENTITY_BB;
		}
		viewer->SetNodeMaskBits(
			viewer::NodeMask::NODE_MASK_ENTITY_MODEL |
			viewer::NodeMask::NODE_MASK_ENTITY_BB,
			view_mode);

		if (opt.GetOptionSet("no_ghost"))
		{
			no_ghost = true;
		}

		if (opt.GetOptionSet("no_ghost_model"))
		{
			no_ghost_model = true;
		}

		if (opt.GetOptionSet("remove_object"))
		{
			std::string ids = opt.GetOptionArg("remove_object");
			std::string delimiter = ",";
			size_t pivot = 0;
			size_t pos = 0;
			int id;
			do
			{
				pos = ids.find(delimiter, pivot);
				if (pos != std::string::npos)
				{
					id = strtoi(ids.substr(pivot, pos - pivot));
				}
				else
				{
					id = strtoi(ids.substr(pivot));
				}

				printf("Removing object[%d]\n", id);
				removeObjects.push_back(id);
				pivot = pos + delimiter.length();

			} while (pos != std::string::npos);
		}

		if (ParseEntities(viewer, player.get()) != 0)
		{
			delete viewer;
			return -1;
		}

		const int ghost_id = GetGhostIdx();

		/* TODO: Some functionality to distinguish "main replay" from variations
		for (size_t i = 0; i < viewer->entities_.size(); i++)
		{
			if (i % player->GetNumberOfScenarios() != 0)
			{
				viewer->entities_[i]->SetTransparency(0.5);
			}
		}
		*/

		if (opt.GetOptionSet("hide_trajectories"))
		{
			viewer->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAJECTORY_LINES);
		}

		std::string start_time_str = opt.GetOptionArg("start_time");
		if (!start_time_str.empty())
		{
			double startTime = 1E-3 * strtod(start_time_str);
			if (startTime < player->data_[0].state.info.timeStamp)
			{
				printf("Specified start time (%.2f) < first timestamp (%.2f), adapting.\n", startTime, player->data_[0].state.info.timeStamp);
				startTime = player->data_[0].state.info.timeStamp;
			}
			else if (startTime > player->data_.back().state.info.timeStamp)
			{
				printf("Specified start time (%.2f) > last timestamp (%.2f), adapting.\n", startTime, player->data_.back().state.info.timeStamp);
				startTime = player->data_.back().state.info.timeStamp;
			}
			player->SetStartTime(startTime);
			player->GoToTime(startTime);
		}

		std::string stop_time_str = opt.GetOptionArg("stop_time");
		if (!stop_time_str.empty())
		{
			double stopTime = 1E-3 * strtod(stop_time_str);
			if (stopTime > player->data_.back().state.info.timeStamp)
			{
				printf("Specified stop time (%.2f) > last timestamp (%.2f), adapting.\n", stopTime, player->data_.back().state.info.timeStamp);
				stopTime = player->data_.back().state.info.timeStamp;
			}
			else if (stopTime < player->data_[0].state.info.timeStamp)
			{
				printf("Specified stop time (%.2f) < first timestamp (%.2f), adapting.\n", simTime, player->data_[0].state.info.timeStamp);
				stopTime = player->data_[0].state.info.timeStamp;
			}
			player->SetStopTime(stopTime);
		}

		bool col_analysis = false;
		if (opt.GetOptionSet("collision"))
		{
			col_analysis = true;
		}


		while (!(viewer->osgViewer_->done() || (opt.GetOptionSet("quit_at_end") && simTime >= (player->GetStopTime() - SMALL_NUMBER))))
		{
			simTime = player->GetTime();  // potentially wrapped for repeat
			double targetSimTime = simTime;

			if (!pause_player)
			{
				if (viewer->GetSaveImagesToFile())
				{
					player->GoToNextFrame();
				}
				else
				{
					// Get milliseconds since Jan 1 1970
					now = SE_getSystemTime();
					deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
					lastTimeStamp = now;
					if (deltaSimTime > maxStepSize) // limit step size
					{
						deltaSimTime = maxStepSize;
					}
					else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
					{
						SE_sleep(minStepSize - deltaSimTime);
						deltaSimTime = minStepSize;
					}
					deltaSimTime *= time_scale;
					targetSimTime = simTime + deltaSimTime;
				}
			}

			do
			{
				if (!(pause_player || viewer->GetSaveImagesToFile()))
				{
					player->GoToDeltaTime(deltaSimTime, true);
					simTime = player->GetTime();  // potentially wrapped for repeat
				}

				// Fetch states of scenario objects
				ReplayEntry* entry = nullptr;
				ObjectStateStructDat* state = nullptr;
				for (int index = 0; index < scenarioEntity.size(); index++)
				{
					ScenarioEntity* sc = &scenarioEntity[index];

					entry = player->GetEntry(sc->id);
					if (entry)
					{
						state = &entry->state;
					}
					else
					{
						state = nullptr;
					}

					if (state == nullptr || (state->info.visibilityMask & 0x01) == 0)  // no state for given object (index) at this timeframe
					{
						setEntityVisibility(index, false);

						if (index == viewer->currentCarInFocus_)
						{
							// Update overlay info text
							snprintf(info_str_buf, sizeof(info_str_buf), "%.3fs entity[%d]: %s (%d) NO INFO",
								simTime, viewer->currentCarInFocus_, sc->name.c_str(), sc->id);
							viewer->SetInfoText(info_str_buf);
						}
						continue;
					}
					setEntityVisibility(index, true);

					// If not available, create it
					if (sc == 0)
					{
						throw std::runtime_error(std::string("Unexpected entity found: ").append(std::to_string(state->info.id)));
					}

					sc->pos = state->pos;
					sc->wheel_angle = state->info.wheel_angle;
					sc->wheel_rotation = state->info.wheel_rot;

					// on screen text following each entity
					snprintf(sc->entityModel->on_screen_info_.string_, sizeof(sc->entityModel->on_screen_info_.string_),
						" %s (%d) %.2fm\n %.2fkm/h road %d lane %d/%.2f s %.2f\n x %.2f y %.2f hdg %.2f\n osi x %.2f y %.2f \n|",
						state->info.name, state->info.id, entry->odometer,
						3.6 * state->info.speed, sc->pos.roadId,
						sc->pos.laneId, fabs(sc->pos.offset) < SMALL_NUMBER ? 0 : sc->pos.offset, sc->pos.s,
						sc->pos.x, sc->pos.y, sc->pos.h,
						sc->pos.x + sc->bounding_box.center_.x_ * cos(sc->pos.h),
						sc->pos.y + sc->bounding_box.center_.x_ * sin(sc->pos.h));
					sc->entityModel->on_screen_info_.osg_text_->setText(sc->entityModel->on_screen_info_.string_);

					if (index == viewer->currentCarInFocus_)
					{
						// Update overlay info text
						snprintf(info_str_buf, sizeof(info_str_buf), "%.3fs entity[%d]: %s (%d) %.2fs %.2fkm/h %.2fm (%d, %d, %.2f, %.2f)/(%.2f, %.2f %.2f) tScale: %.2f ",
							simTime, viewer->currentCarInFocus_, state->info.name, state->info.id, state->info.timeStamp, 3.6 * state->info.speed, entry->odometer, sc->pos.roadId,
							sc->pos.laneId, fabs(sc->pos.offset) < SMALL_NUMBER ? 0 : sc->pos.offset, sc->pos.s, sc->pos.x, sc->pos.y, sc->pos.h, time_scale);
						viewer->SetInfoText(info_str_buf);
					}
				}

				if (col_analysis && scenarioEntity.size() > 1)
				{
					state = player->GetState(scenarioEntity[0].id);
					if (state && state->info.visibilityMask != 0)  // skip if Ego invisible for graphics, traffic and sensors
					{
						for (size_t i = 0; i < scenarioEntity.size(); i++)
						{
							if (i != ghost_id) // Ignore ghost
							{
								updateCorners(scenarioEntity[i]);
							}
						}

						bool overlap_now = false;
						for (size_t i = 1; i < scenarioEntity.size(); i++)
						{
							state = player->GetState(scenarioEntity[i].id);

							if (i != ghost_id &&  // Ignore ghost and
								state && state->info.visibilityMask != 0) // and objects invisible for graphics, traffic and sensors
							{
								if (separating_axis_intersect(scenarioEntity[0], scenarioEntity[i]))
								{
									overlap_now = true;
									if (!overlap)
									{
										overlap = true;
										pause_player = true;
										double rel_speed = abs((player->GetState(scenarioEntity[0].id))->info.speed - (player->GetState(scenarioEntity[i].id)->info.speed)) * 3.6;
										double rel_angle = (scenarioEntity[0].pos.h - scenarioEntity[i].pos.h) * 180 / M_PI;
										LOG("Collision between %d and %d at time %.2f.\n- Relative speed %.2f km/h\n- Angle %.2f degrees (ego to target)",
										0, i, simTime, rel_speed, rel_angle);
									}
								}
							}
						}
						if (!overlap_now)
						{
							overlap = false;
						}
					}
				}

			} while (!pause_player &&
				simTime < player->GetStopTime() - SMALL_NUMBER &&  // As long as time is < end
				simTime > player->GetStartTime() + SMALL_NUMBER &&  // As long as time is > start time
				(deltaSimTime < 0 ? (player->GetTime() > targetSimTime) : (player->GetTime() < targetSimTime)));  // until reached target timestep


			// Visualize scenario cars
			for (size_t j=0; j<scenarioEntity.size(); j++)
			{
				ScenarioEntity *c = &scenarioEntity[j];
				if (c->entityModel != nullptr)
				{
					c->entityModel->SetPosition(c->pos.x, c->pos.y, c->pos.z);
					c->entityModel->SetRotation(c->pos.h, c->pos.p, c->pos.r);

					if (c->entityModel->GetType() == viewer::EntityModel::EntityType::VEHICLE)
					{
						((viewer::CarModel*)c->entityModel)->UpdateWheels(c->wheel_angle, c->wheel_rotation);
					}
				}
			}

			// Update graphics
			viewer->osgViewer_->frame();
		}
		delete viewer;
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return 2;
	}
	catch (std::runtime_error &e)
	{
		printf("%s\n", e.what());
		return 3;
	}


	return 0;
}
