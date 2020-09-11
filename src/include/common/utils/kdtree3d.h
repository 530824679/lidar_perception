//
/******************************************************************************/
/*!
File name: kdtree3d.h

Description:
This file define struct of the KdTree to realize quick search algorithm

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_KDTREE3D_H_
#define _LIDAR_PERCEPTION_ROS_KDTREE3D_H_

#include <math.h>
#include <vector>
#include "point3d.h"

namespace lidar_perception_ros{
    struct Node {
        PointXYZI<float> point;
        int id;
        Node *left;
        Node *right;

        Node(PointXYZI<float> arr, int setId) : point(arr), id(setId), left(nullptr), right(nullptr) {}
    };

    struct KdTree{
        Node *root;

        KdTree() : root(nullptr) {}

        void InsertHelper(Node **node, int depth, PointXYZI<float> point, int id)
        {
            // Tree is empty
            if (*node == nullptr){
                *node = new Node{point, id};
            }else {
                // calculate current din
                int cd = depth % 3;
                if(cd == 0){
                    if(point.GetX() < (*node)->point.GetX()){
                        InsertHelper(&((*node)->left), depth + 1, point, id);
                    }else{
                        InsertHelper(&((*node)->right), depth + 1, point, id);
                    }
                }else if(cd == 1){
                    if(point.GetY() < (*node)->point.GetY()){
                        InsertHelper(&((*node)->left), depth + 1, point, id);
                    }else{
                        InsertHelper(&((*node)->right), depth + 1, point, id);
                    }
                }else{
                    if(point.GetZ() < (*node)->point.GetZ()){
                        InsertHelper(&((*node)->left), depth + 1, point, id);
                    }else{
                        InsertHelper(&((*node)->right), depth + 1, point, id);
                    }
                }
            }
        }

        void Insert(PointXYZI<float> point, int id)
        {
            // The function should create a new node and place correctly with in the root
            InsertHelper(&root, 0, point, id);
        }

        void SearchHelper(PointXYZI<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
        {
            if(node != nullptr){
                float delta_x = node->point.GetX() - target.GetX();
                float delta_y = node->point.GetY() - target.GetY();
                float delta_z = node->point.GetZ() - target.GetZ();

                if ((delta_x >= -distanceTol && delta_x <= distanceTol) &&
                    (delta_y >= -distanceTol && delta_y <= distanceTol) &&
                    (delta_z >= -distanceTol && delta_z <= distanceTol)) {
                    float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
                    if (distance <= distanceTol) {
                        ids.push_back(node->id);
                    }
                }

                // check across boundary
                if (depth % 3 == 0) {
                    if (delta_x > -distanceTol) {
                        SearchHelper(target, node->left, depth + 1, distanceTol, ids);
                    }
                    if (delta_x < distanceTol) {
                        SearchHelper(target, node->right, depth + 1, distanceTol, ids);
                    }
                } else if (depth % 3 == 1) {
                    if (delta_y > -distanceTol) {
                        SearchHelper(target, node->left, depth + 1, distanceTol, ids);
                    }
                    if (delta_y < distanceTol) {
                        SearchHelper(target, node->right, depth + 1, distanceTol, ids);
                    }
                } else {
                    if (delta_z > -distanceTol) {
                        SearchHelper(target, node->left, depth + 1, distanceTol, ids);
                    }
                    if (delta_z < distanceTol) {
                        SearchHelper(target, node->right, depth + 1, distanceTol, ids);
                    }
                }
            }
        }

        // return a list of point ids in the tree that are within distance of target
        std::vector<int> Search(PointXYZI<float> target, float distanceTol)
        {
            std::vector<int> ids;
            SearchHelper(target, root, 0, distanceTol, ids);
            return ids;
        }
    };
}

#endif //_LIDAR_PERCEPTION_ROS_KDTREE3D_H
