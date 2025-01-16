#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <eigen3/Eigen/Eigen>
#include "Contact.h"


class ContactsManager
{
    private:
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

        //Positions of contact points at the interface of two objects (non-penetrating)
        unordered_map<pair<string, string>, std::vector<Contact>, ObjectIDHashFunction> surface_points;
        //Positions of contact points embedded in the volume of two objects (penetrating)
        unordered_map<pair<string, string>, std::vector<Contact>, ObjectIDHashFunction> penetration_points;
        //List of objects that are in surface contact with a given object (non-penetrating)
        unordered_map<string, unordered_set<string>> contacting_object_ids;
        //List of objects that are in volume contact with a given object (penetrating)
        unordered_map<string, unordered_set<string>> penetrating_object_ids;
        //List of objects that are waiting for contacts to be added 
        unordered_set<string> objects_waiting_for_contacts;
    public:
        ContactsManager();
        ~ContactsManager();
        void add_contact(Contact contact, bool is_penetrating = false);
        void add_contacts(std::vector<Contact> contacts, bool is_penetrating = false);
        std::vector<Contact> get_contact_points(string id1, string id2, bool penetrating = false);
        std::vector<Contact> get_contact_points(string id1, bool penetrating = false);
        std::unordered_set<string> get_contacted_object_ids(string target_object);
        std::unordered_set<string> get_penetrating_object_ids(string target_object);
        void invalidate_object_state(string object_id);
        void remove_object(string object_id);
        void remove_all_objects();
        bool needs_update(string object_id);
};