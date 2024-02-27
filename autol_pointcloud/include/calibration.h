#ifndef AUTOL_ROS_CALIBRATION_H_
#define AUTOL_ROS_CALIBRATION_H_

#include "define.h"

namespace autol_pointcloud
{
    struct LaserCorrection
    {
        float rot_correction;
        float vert_correction;
        float dist_correction;
        bool two_pt_correction_available;
        float dist_correction_x;
        float dist_correction_y;
        float vert_offset_correction;
        float horiz_offset_correction;
        int max_intensity;
        int min_intensity;
        float focal_distance;
        float focal_slope;

        float cos_rot_correction;
        float sin_rot_correction;
        float cos_vert_correction;
        float sin_vert_correction;

        int laser_ring;
    };

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
}
#endif
