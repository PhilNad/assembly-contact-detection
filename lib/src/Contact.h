#pragma once

#include <memory>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>
#include "AssemblyCD.h"


using namespace Eigen;
using namespace physx;
using namespace std;

class Contact
{
    private:
        Vector3f position;
        Vector3f normal;
        float separation;
        string object1_id;
        string object2_id;
    public:
        Contact(string object1_id, string object2_id, PxVec3 position, PxVec3 normal, PxReal separation);
        Contact(string object1_id, string object2_id, Vector3f position, Vector3f normal, float separation);
        ~Contact();
        Vector3f get_position();
        Vector3f get_normal();
        void set_position(Vector3f);
        void set_normal(Vector3f);
        float get_separation();
        pair<string, string> get_object_ids();
};