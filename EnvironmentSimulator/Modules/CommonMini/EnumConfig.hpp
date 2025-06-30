#pragma once

#include <vector>
#include <unordered_map>

enum CONFIG_ENUM
{
    OSC = 0,
    AA_MODE,                         // 1
    ALIGN_ROUTEPOSITIONS,            // 2
    BOUNDING_BOXES,                  // 3
    CAPTURE_SCREEN,                  // 4
    CAMERA_MODE,                     // 5
    CSV_LOGGER,                      // 6
    COLLISION,                       // 7
    CONFIG_FILE_PATH,                // 8
    CUSTOM_CAMERA,                   // 9
    CUSTOM_FIXED_CAMERA,             // 10
    CUSTOM_FIXED_TOP_CAMERA,         // 11
    CUSTOM_LIGHT,                    // 12
    DISABLE_CONTROLLERS,             // 13
    DISABLE_LOG,                     // 14
    DISABLE_STDOUT,                  // 15
    ENFORCE_GENERATE_MODEL,          // 16
    FIXED_TIMESTEP,                  // 17
    FOLLOW_OBJECT,                   // 18
    GENERATE_NO_ROAD_OBJECTS,        // 19
    GENERATE_WITHOUT_TEXTURES,       // 20
    GROUND_PLANE,                    // 21
    HEADLESS,                        // 22
    HELP,                            // 23
    HIDE_ROUTE_WAYPOINTS,            // 24
    HIDE_TRAJECTORIES,               // 25
    IGNORE_HEADING_FOR_TRAJ_MOTION,  // 26
    IGNORE_ODR_OFFSET,               // 27
    IGNORE_Z,                        // 28
    IGNORE_P,                        // 29
    IGNORE_R,                        // 30
    INFO_TEXT,                       // 31
    LOG_APPEND,                      // 32
    LOGFILE_PATH,                    // 33
    LOG_META_DATA,                   // 34
    LOG_LEVEL,                       // 35
    LOG_ONLY_MODULES,                // 36
    LOG_SKIP_MODULES,                // 37
    OSC_STR,                         // 38
    OSG_SCREENSHOT_EVENT_HANDLER,    // 39
    OSCI_CROP_DYNAMIC,               // 40
    OSI_EXCLUDE_GHOST,               // 41
    OSI_FILE,                        // 42
    OSI_FREQ,                        // 43
    OSI_LINES,                       // 44
    OSI_POINTS,                      // 45
    OSI_RECEIVER_IP,                 // 46
    OSI_STATIC_REPORTING,            // 47
    PARAM_DIST,                      // 48
    PARAM_PERMUTATION,               // 49
    PAUSE,                           // 50
    PATH,                            // 51
    PLAYER_SERVER,                   // 52
    PLOT,                            // 53
    PLINE_INTERPOLATION,             // 54
    RECORD,                          // 55
    ROAD_FEATURES,                   // 56
    RETURN_NR_PERMUTATIONS,          // 57
    SAVE_GENERATED_MODEL,            // 58
    SAVE_XOSC,                       // 59
    SEED,                            // 60
    SENSORS,                         // 61
    SERVER,                          // 62
    TEXT_SCALE,                      // 63
    THREADS,                         // 64
    TRAIL_MODE,                      // 65
    TRAJ_FILTER,                     // 66
    TUNNEL_TRANSPARENCY,             // 67
    USE_SIGNS_IN_EXTERNAL_MODEL,     // 68
    VERSION,                         // 69
    CONFIGS_COUNT                    // this must be the last enum value
};

static const std::unordered_map<std::string, CONFIG_ENUM> configStrKeyEnumMap = {{"osc", OSC},
                                                                                 {"aa_mode", AA_MODE},
                                                                                 {"align_routepositions", ALIGN_ROUTEPOSITIONS},
                                                                                 {"bounding_boxes", BOUNDING_BOXES},
                                                                                 {"capture_screen", CAPTURE_SCREEN},
                                                                                 {"camera_mode", CAMERA_MODE},
                                                                                 {"csv_logger", CSV_LOGGER},
                                                                                 {"collision", COLLISION},
                                                                                 {"config_file_path", CONFIG_FILE_PATH},
                                                                                 {"custom_camera", CUSTOM_CAMERA},
                                                                                 {"custom_fixed_camera", CUSTOM_FIXED_CAMERA},
                                                                                 {"custom_fixed_top_camera", CUSTOM_FIXED_TOP_CAMERA},
                                                                                 {"custom_light", CUSTOM_LIGHT},
                                                                                 {"disable_controllers", DISABLE_CONTROLLERS},
                                                                                 {"disable_log", DISABLE_LOG},
                                                                                 {"disable_stdout", DISABLE_STDOUT},
                                                                                 {"enforce_generate_model", ENFORCE_GENERATE_MODEL},
                                                                                 {"fixed_timestep", FIXED_TIMESTEP},
                                                                                 {"follow_object", FOLLOW_OBJECT},
                                                                                 {"generate_no_road_objects", GENERATE_NO_ROAD_OBJECTS},
                                                                                 {"generate_without_textures", GENERATE_WITHOUT_TEXTURES},
                                                                                 {"ground_plane", GROUND_PLANE},
                                                                                 {"headless", HEADLESS},
                                                                                 {"help", HELP},
                                                                                 {"hide_route_waypoints", HIDE_ROUTE_WAYPOINTS},
                                                                                 {"hide_trajectories", HIDE_TRAJECTORIES},
                                                                                 {"ignore_heading_for_traj_motion", IGNORE_HEADING_FOR_TRAJ_MOTION},
                                                                                 {"ignore_odr_offset", IGNORE_ODR_OFFSET},
                                                                                 {"ignore_z", IGNORE_Z},
                                                                                 {"ignore_p", IGNORE_P},
                                                                                 {"ignore_r", IGNORE_R},
                                                                                 {"info_text", INFO_TEXT},
                                                                                 {"log_append", LOG_APPEND},
                                                                                 {"logfile_path", LOGFILE_PATH},
                                                                                 {"log_meta_data", LOG_META_DATA},
                                                                                 {"log_level", LOG_LEVEL},
                                                                                 {"log_only_modules", LOG_ONLY_MODULES},
                                                                                 {"log_skip_modules", LOG_SKIP_MODULES},
                                                                                 {"osc_str", OSC_STR},
                                                                                 {"osg_screenshot_event_handler", OSG_SCREENSHOT_EVENT_HANDLER},
                                                                                 {"osi_crop_dynamic", OSCI_CROP_DYNAMIC},
                                                                                 {"osi_exclude_ghost", OSI_EXCLUDE_GHOST},
                                                                                 {"osi_file", OSI_FILE},
                                                                                 {"osi_freq", OSI_FREQ},
                                                                                 {"osi_lines", OSI_LINES},
                                                                                 {"osi_points", OSI_POINTS},
                                                                                 {"osi_receiver_ip", OSI_RECEIVER_IP},
                                                                                 {"osi_static_reporting", OSI_STATIC_REPORTING},
                                                                                 {"param_dist", PARAM_DIST},
                                                                                 {"param_permutation", PARAM_PERMUTATION},
                                                                                 {"pause", PAUSE},
                                                                                 {"path", PATH},
                                                                                 {"player_server", PLAYER_SERVER},
                                                                                 {"plot", PLOT},
                                                                                 {"pline_interpolation", PLINE_INTERPOLATION},
                                                                                 {"record", RECORD},
                                                                                 {"road_features", ROAD_FEATURES},
                                                                                 {"return_nr_permutations", RETURN_NR_PERMUTATIONS},
                                                                                 {"save_generated_model", SAVE_GENERATED_MODEL},
                                                                                 {"save_xosc", SAVE_XOSC},
                                                                                 {"seed", SEED},
                                                                                 {"sensors", SENSORS},
                                                                                 {"server", SERVER},
                                                                                 {"text_scale", TEXT_SCALE},
                                                                                 {"threads", THREADS},
                                                                                 {"trail_mode", TRAIL_MODE},
                                                                                 {"traj_filter", TRAJ_FILTER},
                                                                                 {"tunnel_transparency", TUNNEL_TRANSPARENCY},
                                                                                 {"use_signs_in_external_model", USE_SIGNS_IN_EXTERNAL_MODEL},
                                                                                 {"version", VERSION}};

CONFIG_ENUM ConvertStrKeyToEnum(const std::string& key);
