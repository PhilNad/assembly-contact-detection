#include "ContactsManager.h"

ContactsManager::ContactsManager()
{

}

ContactsManager::~ContactsManager()
{

}

/// @brief Needs to be called whenever an object is moved or added to the scene to let the ContactsManager know that the object needs to have its contacts updated.
/// @param object_id Unique ID of the object.
void ContactsManager::invalidate_object_state(string object_id)
{
    /*
    The idea is to enable lazy evaluation of the contacts between objects to minimize the number of times PhysX simulate() is called.

    The simulation should be called only if the user request contact information on an object that
    - (1) has no contacts, and
    - (2) that there was no contact update done since the object was last moved or added.

    Checking for condition (1) does not suffice because an object might simply have no contact with other objects, we should not
    run a simulation every time the user requests contact information on this isolated object.

    The user should not have to run a simulation after each move/add/remove of an object, but only when contact information is requested (lazy evaluation)
    such that multiple objects can be moved/added/removed before running a simulation, after which contact information is updated for all objects.

    Therefore, a simulation should be run only if ContactsManager::needs_update() returns true for an object whose contacts are requested.
    */
    this->remove_object(object_id);
    this->objects_waiting_for_contacts.insert(object_id);
    #ifndef NDEBUG
    cout << "ContactsManager::invalidate_object_state() called for object " << object_id << endl;
    #endif
}

/// @brief Check whether the object needs to have its contacts updated.
/// @param object_id Unique ID of the object.
/// @return Returns true if the object needs to have its contacts updated.
bool ContactsManager::needs_update(string object_id)
{
    return this->objects_waiting_for_contacts.count(object_id);
}

/// @brief Integrate the contact points from a list of Contact objects.
/// @param contacts List of Contact objects.
/// @param is_penetrating Whether the contact points are embedded in the volume of the objects (default: false).
void ContactsManager::add_contacts(std::vector<Contact> contacts, bool is_penetrating)
{
    //TODO: Record contact normal (currently not supported by add_contact())
    for (auto it = contacts.begin(); it != contacts.end(); ++it){
        this->add_contact(it->get_object_ids().first, it->get_object_ids().second, it->get_position(), is_penetrating);
    }

    //All objects have been updated
    this->objects_waiting_for_contacts.clear();

    #ifndef NDEBUG
    cout << "ContactsManager::add_contacts() called for " << contacts.size() << " contacts" << endl;
    #endif
}

/// @brief Record the position of a contact point between two objects.
/// @param id1 Unique ID of the first object.
/// @param id2 Unique ID of the second object.
/// @param position 3D position of the contact point.
/// @param is_penetrating Whether the contact point is embedded in the volume of the objects.
void ContactsManager::add_contact(string id1, string id2, Vector3f position, bool is_penetrating)
{
    //If the contact points is the first to be added to a map, create the map
    if (this->surface_contact_positions.find(make_pair(id1, id2)) == this->surface_contact_positions.end()){
        this->surface_contact_positions[make_pair(id1, id2)] = std::vector<Vector3f>();
    }
    if (this->surface_contact_positions.find(make_pair(id2, id1)) == this->surface_contact_positions.end()){
        this->surface_contact_positions[make_pair(id2, id1)] = std::vector<Vector3f>();
    }
    if (this->penetration_contact_positions.find(make_pair(id1, id2)) == this->penetration_contact_positions.end()){
        this->penetration_contact_positions[make_pair(id1, id2)] = std::vector<Vector3f>();
    }
    if (this->penetration_contact_positions.find(make_pair(id2, id1)) == this->penetration_contact_positions.end()){
        this->penetration_contact_positions[make_pair(id2, id1)] = std::vector<Vector3f>();
    }
    if (this->contacting_object_ids.find(id1) == this->contacting_object_ids.end()){
        this->contacting_object_ids[id1] = unordered_set<string>();
    }
    if (this->contacting_object_ids.find(id2) == this->contacting_object_ids.end()){
        this->contacting_object_ids[id2] = unordered_set<string>();
    }
    if (this->penetrating_object_ids.find(id1) == this->penetrating_object_ids.end()){
        this->penetrating_object_ids[id1] = unordered_set<string>();
    }
    if (this->penetrating_object_ids.find(id2) == this->penetrating_object_ids.end()){
        this->penetrating_object_ids[id2] = unordered_set<string>();
    }

    //Add the contact point to the map
    if (is_penetrating){
        this->penetration_contact_positions[make_pair(id1, id2)].push_back(position);
        this->penetration_contact_positions[make_pair(id2, id1)].push_back(position);
        this->penetrating_object_ids[id1].insert(id2);
        this->penetrating_object_ids[id2].insert(id1);
    }else{
        this->surface_contact_positions[make_pair(id1, id2)].push_back(position);
        this->surface_contact_positions[make_pair(id2, id1)].push_back(position);
        this->contacting_object_ids[id1].insert(id2);
        this->contacting_object_ids[id2].insert(id1);
    }
}

/// @brief Remove all contact points associated with an object.
/// @param object_id Unique ID of the object.
void ContactsManager::remove_object(string object_id)
{
    //Iterate over all objects in surface contact with the object
    unordered_set<string> surface_contact_ids = this->contacting_object_ids[object_id];
    for (auto it = surface_contact_ids.begin(); it != surface_contact_ids.end(); ++it){
        //Remove the contact points between the two objects
        this->surface_contact_positions.erase(make_pair(object_id, *it));
        this->surface_contact_positions.erase(make_pair(*it, object_id));
        //Remove the object from the contacting object ids map
        this->contacting_object_ids[*it].erase(object_id);
    }
    //Iterate over all objects in volume contact with the object
    unordered_set<string> volume_contact_ids = this->penetrating_object_ids[object_id];
    for (auto it = volume_contact_ids.begin(); it != volume_contact_ids.end(); ++it){
        //Remove the contact points between the two objects
        this->penetration_contact_positions.erase(make_pair(object_id, *it));
        this->penetration_contact_positions.erase(make_pair(*it, object_id));
        //Remove the object from the contacting object ids map
        this->penetrating_object_ids[*it].erase(object_id);
    }
    //Remove the object from the contacting object ids map
    if (this->contacting_object_ids.count(object_id)){
        this->contacting_object_ids.erase(object_id);
    }
    if (this->penetrating_object_ids.count(object_id)){
        this->penetrating_object_ids.erase(object_id);
    }

    //Remove the object from the list of objects waiting for contacts
    if (this->objects_waiting_for_contacts.count(object_id)){
        this->objects_waiting_for_contacts.erase(object_id);
    }
}

/// @brief Remove all contact points associated with all objects.
void ContactsManager::remove_all_objects()
{
    this->surface_contact_positions.clear();
    this->penetration_contact_positions.clear();
    this->contacting_object_ids.clear();
    this->penetrating_object_ids.clear();
    this->objects_waiting_for_contacts.clear();
}

/// @brief Return the list of contact points between two objects.
/// @param id1 Unique ID of the first object.
/// @param id2 Unique ID of the second object.
/// @param penetrating Whether to return the penetrating contact points. If false, return the surface contact points (default).
/// @return List of 3D positions of contact points.
std::vector<Vector3f> ContactsManager::get_contact_positions(string id1, string id2, bool penetrating)
{
    if (penetrating){
        return this->penetration_contact_positions[make_pair(id1, id2)];
    }else{
        return this->surface_contact_positions[make_pair(id1, id2)];
    }
}

/// @brief Return the list of contact points between an object and all other objects.
/// @param id1 Unique ID of the object.
/// @param penetrating Whether to return the penetrating contact points. If false, return the surface contact points (default).
/// @return List of 3D positions of contact points.
std::vector<Vector3f> ContactsManager::get_contact_positions(string id1, bool penetrating)
{
    //Concatenate the lists of contact points between the object and all other objects
    std::vector<Vector3f> contact_positions;
    if (penetrating){
        //Iterate over all objects in volume contact with the object
        for (auto it = this->penetrating_object_ids[id1].begin(); it != this->penetrating_object_ids[id1].end(); ++it){
            //Add the contact points between the two objects
            contact_positions.insert(contact_positions.end(), this->penetration_contact_positions[make_pair(id1, *it)].begin(), this->penetration_contact_positions[make_pair(id1, *it)].end());
        }
    }else{
        //Iterate over all objects in surface contact with the object
        for (auto it = this->contacting_object_ids[id1].begin(); it != this->contacting_object_ids[id1].end(); ++it){
            //Add the contact points between the two objects
            contact_positions.insert(contact_positions.end(), this->surface_contact_positions[make_pair(id1, *it)].begin(), this->surface_contact_positions[make_pair(id1, *it)].end());
        }
    }
    return contact_positions;
}

std::unordered_set<string> ContactsManager::get_contacted_object_ids(string target_object)
{
    return this->contacting_object_ids[target_object];
}

std::unordered_set<string> ContactsManager::get_penetrating_object_ids(string target_object)
{
    return this->penetrating_object_ids[target_object];
}