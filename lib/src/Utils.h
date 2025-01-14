#pragma once

#include "AssemblyCD.h"

struct ObjectIDHashFunction
{
    std::hash<string> hasher;
    size_t operator()(const std::pair<string, string> &obj_id_pair) const
    {
        size_t obj1_id_hash = this->hasher(obj_id_pair.first);
        size_t obj2_id_hash = this->hasher(obj_id_pair.second);
        //This is the hash function used by boost::hash_combine
        return obj1_id_hash ^ obj2_id_hash + 0x9e3779b9 + (obj1_id_hash << 6) + (obj1_id_hash >> 2);
    }
};

//Return the skew-symmetric matrix of a vector.
Eigen::Matrix3f skew(Vector3f vector);