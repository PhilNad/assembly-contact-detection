#include "Contact.h"
#include <iostream>

Contact::Contact(string object1_id, string object2_id, PxVec3 position, PxVec3 normal, PxReal separation)
{
    this->object1_id = object1_id;
    this->object2_id = object2_id;
    this->position   = {position.x, position.y, position.z};
    this->normal     = {normal.x, normal.y, normal.z};
    this->separation = separation;

    //Print all information
    cout << "Contact between " << object1_id << " and " << object2_id << endl;
    cout << "Position: " << position.x << ", " << position.y << ", " << position.z << endl;
    cout << "Normal: " << normal.x << ", " << normal.y << ", " << normal.z << endl;
    cout << "Separation: " << separation << endl;
}

Contact::~Contact(){}

Vector3f Contact::get_position()
{
    return this->position;
}

Vector3f Contact::get_normal()
{
    return this->normal;
}

float Contact::get_separation()
{
    return this->separation;
}

pair<string, string> Contact::get_object_ids()
{
    return make_pair(this->object1_id, this->object2_id);
}

