#include "calibration.h"

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

namespace autol_pointcloud
{
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

                if (index >= calibration.lidar_slamoffset_corrections.size())
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
}
