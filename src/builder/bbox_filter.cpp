#include "builder/bbox_filter.h"

namespace lidar_perception_ros
{
    BBoxFilter::BBoxFilter(BBoxParam param)
    {
        box_min_bottom_ = param.box_min_bottom_;
        box_min_top_ = param.box_min_top_;
        box_min_volume_ = param.box_min_volume_;
        box_max_volume_ = param.box_max_volume_;
        height_threshold_ = param.height_threshold_;
    }

    BBoxFilter::~BBoxFilter()
    {

    }

    float BBoxFilter::GetVolume(BBox box){
        float volume = box.dx * box.dy * box.dz;
        return volume;
    }

    void BBoxFilter::ConditionFilter(std::vector<BBox> &box_list)
    {
        for (std::vector<BBox>::iterator it = box_list.begin(); it != box_list.end();) {

            // erase too low and too high object
            float box_bottom = (*it).z - (*it).dz/2;
            float box_top = (*it).z + (*it).dz/2;
            float box_height_diff = (*it).dz;

            // erase too small and too large volume
            float volume = GetVolume(*it);

            if((box_bottom > box_min_bottom_) || box_top < box_min_top_){
                it = box_list.erase(it);
            }else if((volume > box_max_volume_) || (volume < box_min_volume_)){
                it = box_list.erase(it);
            }else if(box_height_diff < height_threshold_){
                it = box_list.erase(it);
            }
            else{
                ++it;
            }
        }
    }

}