#include "tracker/rotated_iou.h"
namespace lidar_perception_ros{
    
    rotated_iou::rotated_iou(){};

    rotated_iou::~rotated_iou(){};


    int rotated_iou::isInf(float value){
        suf32 ieee754;
        ieee754.f = value;
        return (ieee754.u & 0x7fffffff) == 0x7f800000;
    }

    int rotated_iou::isNaN(float value){
        suf32 ieee754;
        ieee754.f = value;
        return (ieee754.u & 0x7fffffff) > 0x7f800000;
    }


    int rotated_iou::rotatedRectangleIntersection(const RotatedRect& rect1, const RotatedRect& rect2, std::vector<Point2D>& intersectingRegion){
        
        const float samePointEps = std::max(1e-16f, 1e-6f * (float)std::max(rect1.size.area(), rect2.size.area()));
        Point2D vec1[4], vec2[4];
        Point2D pts1[4], pts2[4];

        std::vector <Point2D> intersection;
        
        rect1.points(pts1);//矩形的点存入pst1中
        rect2.points(pts2);

        int ret = INTERSECT_FULL;

        // Specical case of rect1 == rect2 当两个框没有区别时
        {
            bool same = true;

            for( int i = 0; i < 4; i++ )
            {
                if( fabs(pts1[i].x - pts2[i].x) > samePointEps || (fabs(pts1[i].y - pts2[i].y) > samePointEps) )//判断四个点是否重合
                {
                    same = false;
                    break;
                }
            }

            if(same)
            {

                for( int i = 0; i < 4; i++ )
                {
                    intersectingRegion.push_back(pts1[i]);
                }

                return INTERSECT_FULL;
            }
        }

        // Line vector
        // A line from p1 to p2 is: p1 + (p2-p1)*t, t=[0,1]
        for( int i = 0; i < 4; i++ ) //算出两个矩形四条边的斜率
        {
            vec1[i].x = pts1[(i+1)%4].x - pts1[i].x;
            vec1[i].y = pts1[(i+1)%4].y - pts1[i].y;

            vec2[i].x = pts2[(i+1)%4].x - pts2[i].x;
            vec2[i].y = pts2[(i+1)%4].y - pts2[i].y;
        }

        // Line test - test all line combos for intersection
        for( int i = 0; i < 4; i++ ) //计算所有直线的交点
        {
            for( int j = 0; j < 4; j++ )
            {
                // Solve for 2x2 Ax=b
                float x21 = pts2[j].x - pts1[i].x;
                float y21 = pts2[j].y - pts1[i].y;

                float vx1 = vec1[i].x;
                float vy1 = vec1[i].y;

                float vx2 = vec2[j].x;
                float vy2 = vec2[j].y;

                float det = vx2*vy1 - vx1*vy2;

                float t1 = (vx2*y21 - vy2*x21) / det;
                float t2 = (vx1*y21 - vy1*x21) / det;

                // This takes care of parallel lines
                if( isInf(t1) || isInf(t2) || isNaN(t1) || isNaN(t2) )
                {
                    continue;
                }

                if( t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f )
                {
                    float xi = pts1[i].x + vec1[i].x*t1;
                    float yi = pts1[i].y + vec1[i].y*t1;

                    intersection.push_back(Point2D(xi,yi));//所有直线的交点放入intersection中
                }
            }
        }

        if( !intersection.empty() )
        {
            ret = INTERSECT_PARTIAL;
        }

         // Check for vertices from rect1 inside recct2
        for( int i = 0; i < 4; i++ )
        {
            // We do a sign test to see which side the point lies.
            // If the point all lie on the same sign for all 4 sides of the rect,
            // then there's an intersection
            int posSign = 0;
            int negSign = 0;

            float x = pts1[i].x;
            float y = pts1[i].y;

            for( int j = 0; j < 4; j++ )
            {
                // line equation: Ax + By + C = 0
                // see which side of the line this point is at
                float A = -vec2[j].y;
                float B = vec2[j].x;
                float C = -(A*pts2[j].x + B*pts2[j].y);

                float s = A*x+ B*y+ C;

                if( s >= 0 )
                {
                    posSign++;
                }
                else
                {
                    negSign++;
                }
            }

            if( posSign == 4 || negSign == 4 )
            {
                intersection.push_back(pts1[i]);
            }
        }

        // Reverse the check - check for vertices from rect2 inside recct1
        for( int i = 0; i < 4; i++ )
        {
            // We do a sign test to see which side the point lies.
            // If the point all lie on the same sign for all 4 sides of the rect,
            // then there's an intersection
            int posSign = 0;
            int negSign = 0;

            float x = pts2[i].x;
            float y = pts2[i].y;

            for( int j = 0; j < 4; j++ )
            {
                // line equation: Ax + By + C = 0
                // see which side of the line this point is at
                float A = -vec1[j].y;
                float B = vec1[j].x;
                float C = -(A*pts1[j].x + B*pts1[j].y);

                float s = A*x + B*y + C;

                if( s >= 0 )
                {
                    posSign++;
                }
                else
                {
                    negSign++;
                }
            }

            if( posSign == 4 || negSign == 4 )
            {
                intersection.push_back(pts2[i]);
            }
        }

        int N = (int)intersection.size();
        if (N == 0)
        {
            return INTERSECT_NONE;
        }

            // Get rid of duplicated points
        int Nstride = N;
        float distPt[N * N];
        int ptDistRemap[N];
        for (int i = 0; i < N; ++i)
        {
            const Point2D pt0 = intersection[i];
            ptDistRemap[i] = i;
            for (int j = i + 1; j < N; )
            {
                const Point2D pt1 = intersection[j];
                float d2 = (pt1.x - pt0.x)*(pt1.x - pt0.x) + (pt1.y - pt0.y)*(pt1.y - pt0.y);
                if(d2 <= samePointEps)
                {
                    if (j < N - 1)
                        intersection[j] =  intersection[N - 1];
                    N--;
                    continue;
                }
                distPt[i*Nstride + j] = d2;
                ++j;
            }
        }
        while (N > 8) // we still have duplicate points after samePointEps threshold (eliminate closest points) 交点最多不会超过8个
        {
            int minI = 0;
            int minJ = 1;
            float minD = distPt[1];
            for (int i = 0; i < N - 1; ++i)
            {
                float* pDist = distPt + Nstride * ptDistRemap[i];
                for (int j = i + 1; j < N; ++j)
                {
                    float d = pDist[ptDistRemap[j]];
                    if (d < minD)
                    {
                        minD = d;
                        minI = i;
                        minJ = j;
                    }
                }
            }
            assert(fabs((intersection[minI].x - intersection[minJ].x)*(intersection[minI].x - intersection[minJ].x)+
                        (intersection[minI].y - intersection[minJ].y)*(intersection[minI].y - intersection[minJ].y)
                           - minD) < 1e-6);  // ptDistRemap is not corrupted
            // drop minJ point
            if (minJ < N - 1)
            {
                intersection[minJ] =  intersection[N - 1];
                ptDistRemap[minJ] = ptDistRemap[N - 1];
            }
            N--;
        }

        // order points
        for (int i = 0; i < N - 1; ++i)
        {
            Point2D diffI(intersection[i + 1].x - intersection[i].x,intersection[i + 1].y - intersection[i].y);
            for (int j = i + 2; j < N; ++j)
            {
                Point2D diffJ(intersection[j].x - intersection[i].x,intersection[j].y - intersection[i].y);
                if (diffI.x*diffJ.y-diffJ.x*diffI.y < 0)
                {
                    std::swap(intersection[i + 1], intersection[j]);
                    diffI = diffJ;
                }
            }
        }

        intersection.resize(N);
        for(int i=0; i<intersection.size(); i++){
            intersectingRegion.push_back(intersection[i]);
        }

        return ret;

    }

    double CrossProduct(Point2D pt0, Point2D pt1, Point2D pt2)
    {
        return (pt1.x - pt0.x)*(pt2.y - pt0.y) - (pt2.x - pt0.x)*(pt1.y - pt0.y);
    }

    double Distance(Point2D pt1, Point2D pt2)
    {
        return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
    }
    
    Point2D origin_;

    double Cross(Point2D A,Point2D B)
    {
        return A.x*B.y-A.y*B.x;
    }


    void rotated_iou::convexHull(std::vector<Point2D> input_point,std::vector<Point2D>& output_point){
        int top = 2;
        int index = 0;
        for (int i = 1; i < input_point.size(); i++)
        {
            if(input_point[i].y < input_point[index].y || (input_point[i].y == input_point[index].y && input_point[i].x < input_point[index].x))
            {
                index = i;
            }
        }
        std::swap(input_point[0], input_point[index]);
        output_point.push_back(input_point[0]);

        origin_ = input_point[0];
        std::sort(input_point.begin()+1, input_point.end(), [](Point2D pt1, Point2D pt2){double tmp = CrossProduct(origin_, pt1, pt2); if(fabs(tmp) < 1e-6) return Distance(origin_, pt1) < Distance(origin_, pt2);else return tmp > 0;});
        
        
        output_point.push_back(input_point[1]);
        output_point.push_back(input_point[2]);


        for (int i = 3; i < input_point.size(); ++i)
        {
            while (top > 0 && CrossProduct(output_point[top - 1], input_point[i], output_point[top]) >= 0)
            {
                --top;
                //output_point.pop_back();
            }
            output_point.push_back(input_point[i]);
            ++top;
        }

        return;
    }

    double rotated_iou::counterArea(std::vector<Point2D> order_points){

        double area = 0;
        int length = order_points.size();
        Point2D prev(order_points[length-1].x,order_points[length-1].y);
        for(int i=0; i<length;i++){
            Point2D p(order_points[i].x,order_points[i].y);
            area+=(double)prev.x * p.y - (double)prev.y * p.x;
            prev = p;
        }

        area *= 0.5;

        return fabs(area);
    }
}