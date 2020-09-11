//
/******************************************************************************/
/*!
File name: Point3d.h

Description:
This file define class of the PointXYZI to define the point cloud structure

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/
#ifndef _LIDAR_PERCEPTION_ROS_POINT3D_H_
#define _LIDAR_PERCEPTION_ROS_POINT3D_H_

#include <vector>
#include <map>

using namespace std;

namespace lidar_perception_ros{

    template <typename T>
    class PointXYZI{
    public:
        /* Default constructor.*/
        PointXYZI() = default;

        /*Construct a PointXYZI with (x,y,z,i).*/
        PointXYZI(T x, T y, T z, T i)
        {
            this->data_[0] = x;
            this->data_[1] = y;
            this->data_[2] = z;
            this->data_[3] = i;
        }

        /*Construct a PointXYZI as the given data buffer.*/
        PointXYZI(const T* data)
        {
            for (int i = 0; i < 4; ++i) {
                this->data_[i] = data[i];
            }
        }

        /*Copy constructor.*/
        PointXYZI(const PointXYZI<T>& obj)
        {
            for (int i = 0; i < 4; ++i)
            {
                this->data_[i] = obj.data_[i];
            }
        }

        /*Add a point to get a new point.*/
        PointXYZI<T> operator + (const PointXYZI<T>& pt) const
        {
            PointXYZI<T> point;
            point.data_[0] = this->data_[0] + pt.data_[0];
            point.data_[1] = this->data_[1] + pt.data_[1];
            point.data_[2] = this->data_[2] + pt.data_[2];
            point.data_[3] = this->data_[3] + pt.data_[3];
            return point;
        }

        /*Subtract a point and update a new point.*/
        void operator - (const PointXYZI<T>& pt)
        {
            this->data_[0] -= pt.data_[0];
            this->data_[1] -= pt.data_[1];
            this->data_[2] -= pt.data_[2];
            this->data_[3] -= pt.data_[3];
        }

        void SetX(T x) { this->data_[0] = x;}
        void SetY(T y) { this->data_[1] = y;}
        void SetZ(T z) { this->data_[2] = z;}
        void SetI(T i) { this->data_[3] = i;}

        T GetX() const { return this->data_[0];};
        T GetY() const { return this->data_[1];};
        T GetZ() const { return this->data_[2];};
        T GetI() const { return this->data_[3];};

        float data_[4];
    };
}

#endif //_LIDAR_PERCEPTION_ROS_POINT3D_H_
