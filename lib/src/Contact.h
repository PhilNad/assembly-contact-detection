#pragma once

#include "PxPhysicsAPI.h"
#include <memory>
#include <Object.h>
#include <eigen3/Eigen/Eigen>


using namespace Eigen;
using namespace physx;
using namespace std;

class Contact
{
    private:
        Vector3f position;
        Vector3f normal;
        float separation;
        Object* object1;
        Object* object2;

    public:
        Contact(Object* object1_id, Object* object2_id, PxVec3 position, PxVec3 normal, PxReal separation);
        Contact(Object* object1_id, Object* object2_id, Vector3f position, Vector3f normal, float separation);
        ~Contact();
        Vector3f get_position();
        Vector3f get_normal();
        void set_position(Vector3f);
        void set_normal(Vector3f);
        float get_separation();
        pair<string, string> get_object_ids();
        pair<Object*, Object*> get_objects();
};