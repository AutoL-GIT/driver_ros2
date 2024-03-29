#ifndef UTILS_HPP_
#define UTILS_HPP_
#include "define.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>

void ConvertPolorToOrthCood(float distance, float elevation, float azimuth_offset, float &pos_x, float &pos_y, float &pos_z, float z_axes_offset)
{
    distance *= 0.01;
    pos_x = distance * cos(elevation * PI / 180) * cos((azimuth_offset)*PI / 180);
    pos_y = distance * cos(elevation * PI / 180) * sin((azimuth_offset)*PI / 180);
    pos_z = (distance != 0.0) ? distance * sin(elevation * PI / 180) + z_axes_offset : 0.0;
}

struct SlamOffset
{
    float roll;
    float pitch;
    float yaw;
    float x_offset;
    float y_offset;
    float z_offset;
};

class Calibration
{
public:
    bool initialized;
    int node_type_;

    int num_lidars;
    std::vector<SlamOffset> lidar_slamoffset_corrections;

public:
    Calibration();
    void ReadSlamOffset(std::string file);

private:
};

namespace YAML
{
    // The >> operator disappeared in yaml-cpp 0.5, so this function is
    // added to provide support for code written under the yaml-cpp 0.3 API.
    template <typename T>
    void operator>>(const YAML::Node &node, T &i)
    {
        i = node.as<T>();
    }
} /* YAML */

const std::string NUM_LASERS = "num_lasers";
const std::string DISTANCE_RESOLUTION = "distance_resolution";
const std::string LASERS = "lasers";
const std::string LASER_ID = "laser_id";
const std::string ROT_CORRECTION = "rot_correction";
const std::string VERT_CORRECTION = "vert_correction";
const std::string DIST_CORRECTION = "dist_correction";
const std::string TWO_PT_CORRECTION_AVAILABLE = "two_pt_correction_available";
const std::string DIST_CORRECTION_X = "dist_correction_x";
const std::string DIST_CORRECTION_Y = "dist_correction_y";
const std::string VERT_OFFSET_CORRECTION = "vert_offset_correction";
const std::string HORIZ_OFFSET_CORRECTION = "horiz_offset_correction";
const std::string MAX_INTENSITY = "max_intensity";
const std::string MIN_INTENSITY = "min_intensity";
const std::string FOCAL_DISTANCE = "focal_distance";
const std::string FOCAL_SLOPE = "focal_slope";

Calibration::Calibration()
{
}

void operator>>(const YAML::Node &node, std::pair<int, SlamOffset> &correction)
{
    node["lidar_id"] >> correction.first;
    node["roll"] >> correction.second.roll;
    node["pitch"] >> correction.second.pitch;
    node["yaw"] >> correction.second.yaw;
    node["x_offset"] >> correction.second.x_offset;
    node["y_offset"] >> correction.second.y_offset;
    node["z_offset"] >> correction.second.z_offset;
}

/** Read entire calibration file. */
void operator>>(const YAML::Node &node, Calibration &calibration)
{
    if (calibration.node_type_ == 1)
    {
        int num_lidars;
        node["num_lidars"] >> num_lidars;
        const YAML::Node &offset = node["offset"];
        calibration.lidar_slamoffset_corrections.resize(num_lidars);

        for (int i = 0; i < num_lidars; i++)
        {
            std::pair<int, SlamOffset> slamoffset_correction;
            offset[i] >> slamoffset_correction;

            const int index = slamoffset_correction.first;

            if (index >= (int32_t)calibration.lidar_slamoffset_corrections.size())
            {
                calibration.lidar_slamoffset_corrections.resize(index + 1);
            }
            calibration.lidar_slamoffset_corrections[index] = (slamoffset_correction.second);
        }
    }
}

void Calibration::ReadSlamOffset(std::string file)
{
    std::ifstream fin(file.c_str());
    if (!fin.is_open())
    {
        initialized = false;
        return;
    }
    initialized = true;
    try
    {
        YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
        fin.close();
        doc = YAML::LoadFile(file);
#else
        doc = YAML::Load(fin);
#endif
        node_type_ = 1;
        doc >> *this;
    }
    catch (YAML::Exception &e)
    {
        std::cerr << "YAML Exception: " << e.what() << std::endl;
        initialized = false;
    }
    fin.close();
}

void ApplyRPY(float &pos_x, float &pos_y, float &pos_z, int lidar_id, std::vector<SlamOffset> &rpy)
{
    float rotated_x = 0, rotated_y = 0, rotated_z = 0;

    float angle = rpy[lidar_id].roll * PI / 180.;
    rotated_y = pos_y * cos(angle) - pos_z * sin(angle);
    rotated_z = pos_y * sin(angle) + pos_z * cos(angle);

    pos_y = rotated_y;
    pos_z = rotated_z;

    angle = rpy[lidar_id].pitch * PI / 180.;
    rotated_x = pos_z * sin(angle) + pos_x * cos(angle);
    rotated_z = pos_z * cos(angle) - pos_x * sin(angle);

    pos_x = rotated_x;
    pos_z = rotated_z;
    angle = rpy[lidar_id].yaw * PI / 180.;
    rotated_x = pos_x * cos(angle) - pos_y * sin(angle);
    rotated_y = pos_x * sin(angle) + pos_y * cos(angle);

    pos_x = rotated_x;
    pos_y = rotated_y;

    rotated_y = rotated_y + rpy[lidar_id].y_offset;
    rotated_x = rotated_x + rpy[lidar_id].x_offset;
    rotated_z = rotated_z + rpy[lidar_id].z_offset;

    pos_x = rotated_x;
    pos_y = rotated_y;
    pos_z = rotated_z;
}
#endif