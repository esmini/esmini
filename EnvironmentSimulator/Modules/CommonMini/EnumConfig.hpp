#pragma once

#include <vector>
#include <string>
#include <unordered_map>

namespace esmini_options
{
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
        OSI_CROP_DYNAMIC,                // 40
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
        ODR_STR,                         // 70
        DENSITY,                         // 71
        DISABLE_OFF_SCREEN,              // 72
        DURATION,                        // 73
        MODEL,                           // 74
        SPEED_FACTOR,                    // 75
        STOP_AT_END_OF_ROAD,             // 76
        TRAFFIC_RULE,                    // 77
        FILE,                            // 78
        DIR,                             // 79
        NO_GHOST,                        // 80
        NO_GHOST_MODEL,                  // 81
        QUIT_AT_END,                     // 82
        REMOVE_OBJECT,                   // 83
        REPEAT,                          // 84
        RES_PATH,                        // 85
        SAVE_MERGED,                     // 86
        START_TIME,                      // 87
        STOP_TIME,                       // 88
        TIME_SCALE,                      // 89
        VIEW_MODE,                       // 90
        HIDE_GHOST,                      // 91
        GHOST_TRAIL_DT,                  // 92
        VEHICLE_DYNAMICS,                // 93
        WIREFRAME,                       // 94
        VIEW_GHOST_RESTART,              // 95
        EXTENDED,                        // 96
        FILE_REFS,                       // 97
        PRINT_CSV,                       // 98
        CSV,                             // 99
        HIDE_OBJ_OUTLINE,                // 100
        SHOW_LIGHTS,                     // 101
        CONFIGS_COUNT                    // this must be the last enum value
    };

    static const std::unordered_map<std::string, CONFIG_ENUM> configStrKeyEnumMap = {
        {"osc", OSC},
        {"aa_mode", AA_MODE},
        {"align_routepositions", ALIGN_ROUTEPOSITIONS},
        {"bounding_boxes", BOUNDING_BOXES},
        {"capture_screen", CAPTURE_SCREEN},
        {"camera_mode", CAMERA_MODE},
        {"csv", CSV},
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
        {"extended", EXTENDED},
        {"file_refs", FILE_REFS},
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
        {"show_lights", SHOW_LIGHTS},
        {"log_append", LOG_APPEND},
        {"logfile_path", LOGFILE_PATH},
        {"log_meta_data", LOG_META_DATA},
        {"log_level", LOG_LEVEL},
        {"log_only_modules", LOG_ONLY_MODULES},
        {"log_skip_modules", LOG_SKIP_MODULES},
        {"osc_str", OSC_STR},
        {"osg_screenshot_event_handler", OSG_SCREENSHOT_EVENT_HANDLER},
        {"osi_crop_dynamic", OSI_CROP_DYNAMIC},
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
        {"vehicle_dynamics", VEHICLE_DYNAMICS},
        {"version", VERSION},
        {"odr", ODR_STR},
        {"print_csv", PRINT_CSV},
        {"density", DENSITY},
        {"disable_off_screen", DISABLE_OFF_SCREEN},
        {"duration", DURATION},
        {"model", MODEL},
        {"speed_factor", SPEED_FACTOR},
        {"stop_at_end_of_road", STOP_AT_END_OF_ROAD},
        {"traffic_rule", TRAFFIC_RULE},
        {"file", FILE},
        {"dir", DIR},
        {"no_ghost", NO_GHOST},
        {"no_ghost_model", NO_GHOST_MODEL},
        {"quit_at_end", QUIT_AT_END},
        {"remove_object", REMOVE_OBJECT},
        {"repeat", REPEAT},
        {"res_path", RES_PATH},
        {"save_merged", SAVE_MERGED},
        {"start_time", START_TIME},
        {"stop_time", STOP_TIME},
        {"time_scale", TIME_SCALE},
        {"view_mode", VIEW_MODE},
        {"hide_ghost", HIDE_GHOST},
        {"ghost_trail_dt", GHOST_TRAIL_DT},
        {"wireframe", WIREFRAME},
        {"view_ghost_restart", VIEW_GHOST_RESTART},
        {"hide_obj_outline", HIDE_OBJ_OUTLINE}};

    CONFIG_ENUM ConvertStrKeyToEnum(const std::string& key);
}  // namespace esmini_options
