#include "Contact.h"
#include <iostream>

Contact::Contact(Object* object1, Object* object2, PxVec3 position, PxVec3 normal, PxReal separation)
{
    this->object1 = object1;
    this->object2 = object2;
    this->position   = {position.x, position.y, position.z};
    this->normal     = {normal.x, normal.y, normal.z};
    this->separation = separation;

    //Print all information
    // cout << "Contact between " << object1_id << " and " << object2_id << endl;
    // cout << "Position: " << position.x << ", " << position.y << ", " << position.z << endl;
    // cout << "Normal: " << normal.x << ", " << normal.y << ", " << normal.z << endl;
    // cout << "Separation: " << separation << endl;
}


Contact::Contact(Object* object1, Object* object2, Vector3f position, Vector3f normal, float separation)
{
    this->object1 = object1;
    this->object2 = object2;
    this->position   = position;
    this->normal     = normal;
    this->separation = separation;

    //Print all information
    // cout << "Contact between " << object1->id << " and " << object2->id << endl;
    // cout << "Position: " << position[0] << ", " << position[1] << ", " << position[2] << endl;
    // cout << "Normal: " << normal[0] << ", " << normal[1] << ", " << normal[2] << endl;
    // cout << "Separation: " << separation << endl;
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
    string object1_id = this->object1->id;
    string object2_id = this->object2->id;
    return make_pair(object1_id, object2_id);
}


void Contact::set_position(Vector3f position)
{
    this->position = position;
}

void Contact::set_normal(Vector3f normal)
{
    this->normal = normal;
}


pair<Object*, Object*> Contact::get_objects()
{
    return make_pair(this->object1, this->object2);
}