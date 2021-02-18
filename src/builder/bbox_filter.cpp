#include "builder/bbox_filter.h"

namespace lidar_perception_ros
{
    const double eps = 1e-8;

    BBoxFilter::BBoxFilter(BBoxParam param)
    {
        box_min_bottom_ = param.box_min_bottom_;
        box_min_top_ = param.box_min_top_;
        box_min_area_ = param.box_min_area_;
        box_max_area_ = param.box_max_area_;
        box_min_volume_ = param.box_min_volume_;
        box_max_volume_ = param.box_max_volume_;
        height_threshold_ = param.height_threshold_;
    }

    BBoxFilter::~BBoxFilter()
    {

    }

    float BBoxFilter::GetArea(BBox box)
    {
        float area = box.dx * box.dy;
        return area;
    }

    float BBoxFilter::GetVolume(BBox box)
    {
        float volume = box.dx * box.dy * box.dz;
        return volume;
    }

    bool BBoxFilter::CompareVolume(BBox box_1, BBox box_2)
    {
        float volume_1 = GetVolume(box_1);
        float volume_2 = GetVolume(box_2);
        if((volume_1 < volume_2) && fabs(volume_1-volume_2) > 1e-6){
            return false;
        }else{
            return true;
        }
    }

    int BBoxFilter::DCompare(float x)
    {
        if(x > eps) return 1;
        return x < -eps ? -1 : 0;
    }

    float BBoxFilter::Cross(Point2D a, Point2D b, Point2D c)
    {
        return (a.x-c.x)*(b.y-c.y)-(b.x-c.x)*(a.y-c.y);
    }

    Point2D BBoxFilter::Intersection(Point2D a, Point2D b, Point2D c, Point2D d)
    {
        Point2D p = a;
        double t =((a.x-c.x)*(c.y-d.y)-(a.y-c.y)*(c.x-d.x))/((a.x-b.x)*(c.y-d.y)-(a.y-b.y)*(c.x-d.x));
        p.x +=(b.x-a.x)*t;
        p.y +=(b.y-a.y)*t;
        return p;
    }

    float BBoxFilter::CalcPolygonArea(Point2D p[], int n)
    {
        if(n < 3) return 0.0;
        double s = p[0].y * (p[n - 1].x - p[1].x);
        p[n] = p[0];
        for(int i = 1; i < n; ++ i)
            s += p[i].y * (p[i - 1].x - p[i + 1].x);
        return fabs(s * 0.5);
    }

    float BBoxFilter::CPIA(Point2D a[], Point2D b[], int na, int nb)
    {
        Point2D p[20], tmp[20];
        int tn, sflag, eflag;
        a[na] = a[0], b[nb] = b[0];
        memcpy(p,b,sizeof(Point2D)*(nb + 1));

        for(int i = 0; i < na && nb > 2; i++)
        {
            sflag = DCompare(Cross(a[i + 1], p[0],a[i]));
            for(int j = tn = 0; j < nb; j++, sflag = eflag)
            {
                if(sflag>=0) tmp[tn++] = p[j];
                eflag = DCompare(Cross(a[i + 1], p[j + 1],a[i]));
                if((sflag ^ eflag) == -2)
                    tmp[tn++] = Intersection(a[i], a[i + 1], p[j], p[j + 1]); ///求交点
            }
            memcpy(p, tmp, sizeof(Point2D) * tn);
            nb = tn, p[nb] = p[0];
        }

        if(nb < 3)
            return 0.0;
        return CalcPolygonArea(p, nb);
    }

    float BBoxFilter::SPIA(Point2D a[], Point2D b[], int na, int nb)
    {
        int i, j;
        Point2D t1[4], t2[4];
        double res = 0, num1, num2;
        a[na] = t1[0] = a[0], b[nb] = t2[0] = b[0];

        for(i = 2; i < na; i++)
        {
            t1[1] = a[i-1], t1[2] = a[i];
            num1 = DCompare(Cross(t1[1], t1[2],t1[0]));
            if(num1 < 0) std::swap(t1[1], t1[2]);

            for(j = 2; j < nb; j++)
            {

                t2[1] = b[j - 1], t2[2] = b[j];
                num2 = DCompare(Cross(t2[1], t2[2],t2[0]));
                if(num2 < 0) std::swap(t2[1], t2[2]);
                res += CPIA(t1, t2, 3, 3) * num1 * num2;
            }
        }

        return res;
    }

    float BBoxFilter::IntersectionArea(const BBox & r1, const BBox & r2)
    {
        Point2D p1[4],p2[4];

        p1[0].x=r1.vertex_pts[3].x;
        p1[0].y=r1.vertex_pts[3].y;
        p1[1].x=r1.vertex_pts[0].x;
        p1[1].y=r1.vertex_pts[0].y;
        p1[2].x=r1.vertex_pts[1].x;
        p1[2].y=r1.vertex_pts[1].y;
        p1[3].x=r1.vertex_pts[2].x;
        p1[3].y=r1.vertex_pts[2].y;

        p2[0].x=r2.vertex_pts[3].x;
        p2[0].y=r2.vertex_pts[3].y;
        p2[1].x=r2.vertex_pts[0].x;
        p2[1].y=r2.vertex_pts[0].y;
        p2[2].x=r2.vertex_pts[1].x;
        p2[2].y=r2.vertex_pts[1].y;
        p2[3].x=r2.vertex_pts[2].x;
        p2[3].y=r2.vertex_pts[2].y;
        double area = SPIA(p1, p2, 4, 4);

        return area;
    }

    float BBoxFilter::CalcArea(const BBox & r)
    {
        float d12=sqrt(pow(r.vertex_pts[1].x-r.vertex_pts[0].x,2)+pow(r.vertex_pts[1].y-r.vertex_pts[0].y,2));
        float d14=sqrt(pow(r.vertex_pts[3].x-r.vertex_pts[0].x,2)+pow(r.vertex_pts[3].y-r.vertex_pts[0].y,2));
        float d24=sqrt(pow(r.vertex_pts[1].x-r.vertex_pts[3].x,2)+pow(r.vertex_pts[1].y-r.vertex_pts[3].y,2));
        float d32=sqrt(pow(r.vertex_pts[1].x-r.vertex_pts[2].x,2)+pow(r.vertex_pts[1].y-r.vertex_pts[2].y,2));
        float d34=sqrt(pow(r.vertex_pts[2].x-r.vertex_pts[3].x,2)+pow(r.vertex_pts[2].y-r.vertex_pts[3].y,2));
        float p1=(d12+d14+d24)/2;
        float p2=(d24+d32+d34)/2;
        float s1=sqrt(p1*(p1-d12)*(p1-d14)*(p1-d24));
        float s2=sqrt(p2*(p2-d32)*(p2-d34)*(p2-d24));
        return s1+s2;
    }

    float BBoxFilter::CalcCoverage(const BBox & r1, const BBox & r2)
    {
        float inter_area = IntersectionArea(r1,r2);
        float area_r1 = CalcArea(r1);
        float area_r2 = CalcArea(r2);

        float overlap = (area_r1 > area_r2) ? (inter_area / area_r2) : (inter_area / area_r1);

        return (overlap >= 0) ? overlap : 0;
    }

    void BBoxFilter::FusionBBox(std::vector<BBox> &box_list, const double threshold) {
        if (box_list.empty()) {
            // std::cout << "List of all the bounding box is empty." << std::endl;
            return;
        }

        std::sort(box_list.begin(), box_list.end(), CompareVolume);

        std::vector<bool> del(box_list.size(), false);
        for (size_t i = 0; i < box_list.size(); i++)
        {
            if (!del[i]) {
                for (size_t j = i + 1; j < box_list.size(); j++) {
                    bool isContain = true;
                    for (int k = 0; k < 4; k++) {
                        Point2D point = box_list[j].vertex_pts[k];
                        if (!isPointInRect(box_list[i], point))
                            isContain = false;
                    }

                    if (!del[j] && isContain) {
                        del[j] = true;
                    }
                }
            }
        }
//        std::vector<bool> del(box_list.size(), false);
//        for(size_t i = 0; i < box_list.size(); i++){
//            if(!del[i]){
//                for(size_t j = i+1; j < box_list.size(); j++){
//                    if(!del[j] && CalcCoverage(box_list[i], box_list[j]) > threshold){
//                        del[j] = true;
//                    }
//                }
//            }
//        }
//
        std::vector<BBox> new_list;

        for (int i = 0; i < box_list.size(); i++) {
            if (!del[i])
                new_list.push_back(box_list[i]);
        }

        box_list.clear();
        std::vector<BBox>().swap(box_list);
        box_list = new_list;

    }

    bool BBoxFilter::isPointInRect(BBox box, Point2D point)
    {
        Point2D A = box.vertex_pts[0];
        Point2D B = box.vertex_pts[1];
        Point2D C = box.vertex_pts[2];
        Point2D D = box.vertex_pts[3];
        int x = point.x;
        int y = point.y;
        int a = (B.x - A.x)*(y - A.y) - (B.y - A.y)*(x - A.x);
        int b = (C.x - B.x)*(y - B.y) - (C.y - B.y)*(x - B.x);
        int c = (D.x - C.x)*(y - C.y) - (D.y - C.y)*(x - C.x);
        int d = (A.x - D.x)*(y - D.y) - (A.y - D.y)*(x - D.x);
        if((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0)) {
            return true;
        }
        return false;
    }

    void BBoxFilter::ConditionFilter(std::vector<BBox> &box_list)
    {
        for (std::vector<BBox>::iterator it = box_list.begin(); it != box_list.end();) {

            // erase too low and too high object
            float box_bottom = (*it).z - (*it).dz/2;
            float box_top = (*it).z + (*it).dz/2;
            float box_height_diff = (*it).dz;

            // erase too small and too large area or volume
            float area = GetArea(*it);
            float volume = GetVolume(*it);

            // erase the too small ratio of width and length, which might be wall etc.
            float ratio = std::min((*it).dx / (*it).dy, ((*it).dy / (*it).dx));

            if((box_bottom > box_min_bottom_) || box_top < box_min_top_){
                it = box_list.erase(it);
            }else if((area > box_max_area_) || (volume < box_min_area_)){
                it = box_list.erase(it);
            }else if((volume > box_max_volume_) || (volume < box_min_volume_)){
                it = box_list.erase(it);
            }else if(box_height_diff < height_threshold_){
                it = box_list.erase(it);
            }else if (ratio < 0.1){
                it = box_list.erase(it);
            }else{
                ++it;
            }
        }

        FusionBBox(box_list, 0.95);
    }

}