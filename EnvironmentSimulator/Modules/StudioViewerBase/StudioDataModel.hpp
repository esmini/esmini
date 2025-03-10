#pragma once

#include "json.hpp"

enum class StudioMode
{
    UNKNOWN,
    VIEWER,
    INSPECTOR,
    ZOMBIE,
};

class StudioDataModel
{
public:
    StudioDataModel();
    ~StudioDataModel()
    {
    }

    void SetXodrPath(const std::string& path);

    StudioMode mode_      = StudioMode::VIEWER;
    StudioMode prev_mode_ = StudioMode::UNKNOWN;
    float      virtual_time_;
    float      virtual_time_max_value_;
    bool       virtual_time_manipulated_ = false;

    std::string json_path_;
    std::string xodr_path_;
    std::string tmp_xosc_path_;
    std::string tmp_rec_path_;

private:
    nlohmann::json decl_json_;
};
