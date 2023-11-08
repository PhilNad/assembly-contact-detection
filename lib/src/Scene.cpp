#include "AssemblyCD.h"
#include <iostream>
#include <memory>
#include <unordered_set>
#include <iterator>
#include <set>
#include <thread>
#include <vector>
#include <chrono>
#include "PxPhysicsAPI.h"
#include "extensions/PxTetMakerExt.h"

using namespace physx;
using namespace std;
using namespace Eigen;

vector<Contact> gContacts;
vector<Contact> gPenetrationContacts;
vector<PxVec3> gContactPositions;
vector<pair<string, string>> gContactedObjects;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics;
static PxDefaultCpuDispatcher*	gDispatcher;
static PxScene*					gScene;
static PxMaterial*				gMaterial;

static PxFilterFlags contactReportFilterShader(	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
												PxFilterObjectAttributes attributes1, PxFilterData filterData1,
												PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

    //Do not use eSOLVE_CONTACT with kinematic bodies as no force is involved.
	pairFlags = PxPairFlag::eDETECT_DISCRETE_CONTACT
			    | PxPairFlag::eNOTIFY_TOUCH_FOUND 
			    | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
			    | PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}


/// @brief Callback class for contact reports. This will be called by fetchResults() at the end of each simulation step.
class ContactReportCallbackForVoxelgrid: public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count)					{ PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) 
	{
		PX_UNUSED((pairHeader));

        //If there is no contact but the callback was nonetheless trigerred, we return.
        if(nbPairs == 0){
            return;
        }else{
            cout << "Contact detected: " << nbPairs << " pairs" << endl;
        }

        //Maximum number of threads to use (can be zero)
        const int num_threads = std::thread::hardware_concurrency();

        if(num_threads > 1){
            //List of threads
            std::vector<std::thread> threads(num_threads);
            //Each thread will generate a list of surface contacts, all of which will be merged at the end.
            std::vector<std::vector<Contact>> thread_contacts(num_threads);
            //Each thread will generate a list of penetrating contacts, all of which will be merged at the end.
            std::vector<std::vector<Contact>> thread_pen_contacts(num_threads);
            //Each thread will generate a list of contacted objects, all of which will be merged at the end.
            std::vector<std::vector<pair<string, string>>> thread_contacted_objects(num_threads);

            std::cout << "Using " << num_threads << " threads" << std::endl;

            //Iterate over each thread
            for(int thread_i = 0; thread_i < num_threads; thread_i++){
                //Get the number of pairs to process in this thread
                int nb_pairs_per_thread = nbPairs / num_threads;
                //Get the index of the first pair to process in this thread
                int first_pair_index = thread_i*nb_pairs_per_thread;
                //Get the index of the last pair to process in this thread
                int last_pair_index = (thread_i == num_threads - 1) ? nbPairs : (thread_i+1)*nb_pairs_per_thread;
                //Launch the thread
                threads[thread_i] = std::thread(&ContactReportCallbackForVoxelgrid::process_contact_pairs, this, pairs, first_pair_index, last_pair_index, std::ref(thread_contacts[thread_i]), std::ref(thread_pen_contacts[thread_i]), std::ref(thread_contacted_objects[thread_i]));
            }
            //Wait for all threads to finish
            for(auto& thread : threads){
                thread.join();
            }
            //Merge the contact points from all threads
            int nb_added_points = 0;
            for(auto& thread_contact : thread_contacts){
                nb_added_points += thread_contact.size();
                gContacts.insert(gContacts.end(), thread_contact.begin(), thread_contact.end());
            }
            for(auto& thread_pen_contacts : thread_pen_contacts){
                gPenetrationContacts.insert(gPenetrationContacts.end(), thread_pen_contacts.begin(), thread_pen_contacts.end());
            }
            if(nb_added_points > 0){
                //Merge the contacted objects from all threads
                for(auto& thread_contacted_object : thread_contacted_objects){
                    gContactedObjects.insert(gContactedObjects.end(), thread_contacted_object.begin(), thread_contacted_object.end());
                }
            }
        }else{
            std::vector<Contact> contacts;
            std::vector<Contact> pen_contacts;
            std::vector<pair<string, string>> contact_object_ids;
            process_contact_pairs(pairs, 0, nbPairs, contacts, pen_contacts, contact_object_ids);
            //Add the pair of objects to the list of contacted objects.
            if(contacts.size() > 0){
                gContactedObjects.insert(gContactedObjects.end(), contact_object_ids.begin(), contact_object_ids.end());
            }
            //Add the surface contact points to the list of contact points
            gContacts.insert(gContacts.end(), contacts.begin(), contacts.end());
            //Add the penetrating contact points to the list of contact points
            gPenetrationContacts.insert(gPenetrationContacts.end(), pen_contacts.begin(), pen_contacts.end());
        }
	}

    /// @brief Process a subset of the contact pairs, possibly in a separate thread.
    /// @param pairs Array of contact pairs
    /// @param first_pair_index Index of the first pair to process
    /// @param last_pair_index Index of the last pair to process
    /// @param thread_contacts [Output] List of contacts to append to
    /// @param thread_pen_contacts [Output] List of penetrating contacts to append to
    /// @param thread_contacted_objects [Output] List of contacted objects to append to
    void process_contact_pairs(const PxContactPair* pairs, int first_pair_index, int last_pair_index, std::vector<Contact>& thread_contacts, std::vector<Contact>& thread_pen_contacts, std::vector<pair<string, string>>& thread_contacted_objects)
    {
        std::vector<PxContactPairPoint> contactPoints;
        //Iterate over each contact pair
        for(PxU32 i=first_pair_index;i<last_pair_index;i++){
            //cout << "Contact pair " << i << endl;
            PxContactPair pair = pairs[i];
            PxU32 contactCount = pair.contactCount;
            //cout << contactCount << " contacts between " << id_obj0 << " and " << id_obj1 << endl;
  
            if(contactCount)
            {
                //Get the shapes involved in the collision
                PxShape* shape0 = pair.shapes[0];
                PxShape* shape1 = pair.shapes[1];

                //Geometry type of each shape
                PxGeometryType::Enum shape0_type = shape0->getGeometry().getType();
                PxGeometryType::Enum shape1_type = shape1->getGeometry().getType();
                
                //Get the actors
                PxRigidActor* actor0 = shape0->getActor();
                PxRigidActor* actor1 = shape1->getActor();
                string id_obj0 = actor0->getName();
                string id_obj1 = actor1->getName();

                //Get the objects
                Object* obj0 = static_cast<Object*>(actor0->userData);
                Object* obj1 = static_cast<Object*>(actor1->userData);

                //Get useful scene-related parameters
                Scene* scene = obj0->scene;
                float max_distance_factor = scene->max_distance_factor;

                //Compute the position threshold based on the size of the voxels in contact
                // two contact points are merged if they are closer than this threshold
                Vector3f o1_sides = obj0->get_voxel_side_lengths();
                Vector3f o2_sides = obj1->get_voxel_side_lengths();
                float max_side = max(o1_sides.maxCoeff(), o2_sides.maxCoeff());
                float position_threshold = max(position_threshold, 0.1f*max_side);

                //If the collision is between a sphere and a tetrahedron, it means that there is an inter-penetration
                if((shape0_type == PxGeometryType::eSPHERE && shape1_type == PxGeometryType::eCONVEXMESH) ||
                    (shape1_type == PxGeometryType::eSPHERE && shape0_type == PxGeometryType::eCONVEXMESH) ){
                    PxVec3 p;
                    //Get the position of the sphere/point
                    if(shape0_type == PxGeometryType::eSPHERE){
                        p = shape0->getLocalPose().p;
                    }else{
                        p = shape1->getLocalPose().p;
                    }
                    Contact contact(obj0, obj1, Vector3f(p.x, p.y, p.z), Vector3f(0,0,1), 0.0f);
                    thread_pen_contacts.push_back(contact);
                }

                //The collision is between two gridcells
                if( shape0_type == PxGeometryType::eBOX && shape1_type == PxGeometryType::eBOX ){
                    //Get the grid cells involved in the collision
                    GridCell* gridCell0 = static_cast<GridCell*>(shape0->userData);
                    GridCell* gridCell1 = static_cast<GridCell*>(shape1->userData);

                    //Surface points can be obtained with
                    //  gridCell0->surface_points[0].get()->position
                    //  gridCell0->surface_points[1].get()->position
                    //  ...
                    //  gridCell0->surface_points[0].get()->normal
                    //  ...
                    PointSet3D intersections = all_triangles_overlap_over_AARectangle(*gridCell0, *gridCell1, max_distance_factor);
                    for(auto& p : intersections){
                        //Add a new contact point to the list
                        //TODO: Use the right normal
                        Contact contact(obj0, obj1, Vector3f(p[0], p[1], p[2]), Vector3f(0,0,1), 0.0f);
                        thread_contacts.push_back(contact);
                    }
                    thread_contacted_objects.push_back(make_pair(id_obj0, id_obj1));
                }
            }
        }
    }

    /// @brief Compute the extrema of the intersection between all combinations of triangles projected on the overlap between the two gridcells.
    /// @param g1 First gridcell in contact
    /// @param g2 Second gridcell in contact
    /// @return Point set of 3D points representing the extrema of the intersection between the two gridcells
    PointSet3D all_triangles_overlap_over_AARectangle(GridCell& g1, GridCell& g2, float max_distance_factor = 0.2)
    {
        //Triangles associated with the two gridcells
        vector<shared_ptr<Triangle<Vector3f>>> t1_list = g1.triangles;
        vector<shared_ptr<Triangle<Vector3f>>> t2_list = g2.triangles;

        //Axis aligned intersection rectangle between the two contacting gridcells
        AARectangle aarec_x = g1.gridcell_to_gridcell_intersection(g2, AARectangle::NORMAL_AXIS::X);
        AARectangle aarec_y = g1.gridcell_to_gridcell_intersection(g2, AARectangle::NORMAL_AXIS::Y);
        AARectangle aarec_z = g1.gridcell_to_gridcell_intersection(g2, AARectangle::NORMAL_AXIS::Z);
        
        //cout << "Gridcell IDs: #" << g1.id << " and #" << g2.id << endl;
        //cout << "AARectangle: Plane: (" << aarec_x.plane.n.x << ", " << aarec_x.plane.n.y << ", " << aarec_x.plane.n.z << ", " << aarec_x.plane.d << ")" << " Centre: (" << aarec_x.centre[0] << ", " << aarec_x.centre[1] << ", " << aarec_x.centre[2] << ") Half Extents: (" << aarec_x.half_extents[0] << ", " << aarec_x.half_extents[1] << ", " << aarec_x.half_extents[2] << ")" << endl;

        //Maximum distance from an intersection point to the triangle planes
        float x_max_dist = max(min(g1.half_extents[1], g2.half_extents[1]), min(g1.half_extents[2], g2.half_extents[2]))*max_distance_factor;
        float y_max_dist = max(min(g1.half_extents[0], g2.half_extents[0]), min(g1.half_extents[2], g2.half_extents[2]))*max_distance_factor;
        float z_max_dist = max(min(g1.half_extents[0], g2.half_extents[0]), min(g1.half_extents[1], g2.half_extents[1]))*max_distance_factor;

        //Iterate over each AARectangle and accumulate intersections with the triangles
        vector<AARectangle> aarec_list = {aarec_x, aarec_y, aarec_z};
        vector<float> max_dist_list = {x_max_dist, y_max_dist, z_max_dist};
        PointSet3D all_intersections;
        for(int aarec_i = 0; aarec_i < aarec_list.size(); aarec_i++){
            AARectangle aarec = aarec_list[aarec_i];
            //If the intersection area is too small, we don't consider the intersection
            float minimum_area = max_dist_list[aarec_i]*max_dist_list[aarec_i];
            if(aarec.area > minimum_area){
                //Iterates over all triangle combinations
                for(int t1_i = 0; t1_i < t1_list.size(); t1_i++){
                    shared_ptr<Triangle<Vector3f>> t1 = t1_list[t1_i];
                    for(int t2_i = 0; t2_i < t2_list.size(); t2_i++){
                        shared_ptr<Triangle<Vector3f>> t2 = t2_list[t2_i];
                        //Compute the intersection points between the two triangles
                        PointSet3D intersections = triangle_triangle_AARectangle_intersection(aarec, t1, t2, max_dist_list[aarec_i]);
                        //Append the intersections to the list
                        all_intersections.insert(intersections);
                    }
                }
            }
        }

        return all_intersections;
    }

    /// @brief Computes the line at the intersection of the planes supporting the two oriented points, and finds the
    ///         point on this line that is closest to both oriented points.
    /// @param op0 Oriented point 0
    /// @param op1 Oriented point 1
    /// @return Position of the intersection point in the world frame
    Vector3f compute_projected_intersection_point(OrientedPoint op0, OrientedPoint op1)
    {
        //If the contact is surface-to-edge, we find the line that is the intersection of the two planes
        // for which we have the normal and a point (the OrientedPoint). We then project the surface points
        // onto this line and compute the average of the projections.
        
        //The line is orthogonal to both normals
        Vector3f line = op0.normal.cross(op1.normal);
        line.normalize();

        //Distance from the origin that the surface point is along the surface normal
        float d0 = op0.position.dot(op0.normal);
        float d1 = op1.position.dot(op1.normal);
        
        //Find a point on the line, which will be its origin.
        // See: https://math.stackexchange.com/a/4113687
        Vector3f line_origin = Vector3f::Zero();
        if(abs(line[0]) > abs(line[1]) && abs(line[0]) > abs(line[2])){
            line_origin[1] = (d0*op1.normal[2] - d1*op0.normal[2]) / (op0.normal[1]*op1.normal[2] - op1.normal[1]*op0.normal[2]);
            line_origin[2] = (d1*op0.normal[1] - d0*op1.normal[1]) / (op0.normal[1]*op1.normal[2] - op1.normal[1]*op0.normal[2]);
        }else if(abs(line[1]) > abs(line[0]) && abs(line[1]) > abs(line[2])){
            line_origin[0] = (d0*op1.normal[2] - d1*op0.normal[2]) / (op0.normal[0]*op1.normal[2] - op1.normal[0]*op0.normal[2]);
            line_origin[2] = (d1*op0.normal[0] - d0*op1.normal[0]) / (op0.normal[0]*op1.normal[2] - op1.normal[0]*op0.normal[2]);
        }else{
            line_origin[0] = (d0*op1.normal[1] - d1*op0.normal[1]) / (op0.normal[0]*op1.normal[1] - op1.normal[0]*op0.normal[1]);
            line_origin[1] = (d1*op0.normal[0] - d0*op1.normal[0]) / (op0.normal[0]*op1.normal[1] - op1.normal[0]*op0.normal[1]);
        }

        //Express the surface points relative to the line's origin
        Vector3f op0_in_line = op0.position - line_origin;
        Vector3f op1_in_line = op1.position - line_origin;

        //Project the surface points onto the line
        float op0_proj = op0_in_line.dot(line);
        float op1_proj = op1_in_line.dot(line);

        //Compute the average of the projections
        float avg_proj = (op0_proj + op1_proj) / 2;

        //Express the average of the projections in the original frame.
        // This is the position of the contact point on the line intersection both planes.
        return line_origin + avg_proj*line;
    }

};

/// @brief Initialize the physics engine.
void Scene::startupPhysics()
{
    //Only one foundation object can be spawned per process. So we should keep track of it
    // for the lifetime of the process (even if we destroy the scene).
    if(gFoundation == NULL){
        gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    }

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());

    //To be used if the tetrahedral method is used.
    //gContactReportCallback = new ContactReportCallbackForTetrahedra();
    //To be used if the voxel grid method is used.
    gContactReportCallback = new ContactReportCallbackForVoxelgrid();

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, 0.0f, -9.81f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher	= gDispatcher;
    // It seems that kine-kine collisions between TriangleMeshes are not reported even
    // with the following settings. See: https://github.com/NVIDIA-Omniverse/PhysX/discussions/180
    // However, kine-kine collisions between ConvexMeshes are reported.
    sceneDesc.kineKineFilteringMode   = PxPairFilteringMode::eKEEP;
    sceneDesc.staticKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc.filterShader	          = contactReportFilterShader;
    sceneDesc.simulationEventCallback = gContactReportCallback;
    //When getting more contact points matters, its better to disable PCM.
    // See: https://nvidia-omniverse.github.io/PhysX/physx/5.2.1/docs/AdvancedCollisionDetection.html#persistent-contact-manifold-pcm
    // sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;

    gScene = gPhysics->createScene(sceneDesc);
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.1f);

    gContacts.clear();
    gContactedObjects.clear();
    gContactPositions.clear();
}

/// @brief Clean up the physics engine by releasing memory.
void Scene::cleanupPhysics()
{
    //Clear the list of contacted objects and contact points
    gContacts.clear();
    gContactedObjects.clear(); 
    gContactPositions.clear();

    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    //As noted in startupPhysics(), we should not release the foundation object.
    //PX_RELEASE(gFoundation);
}

Scene::Scene(){
    startupPhysics();
}
Scene::~Scene(){
    cleanupPhysics();
}

/// @brief Clear all recorded contacts such that the next call to step_simulation() will record new contacts.
void Scene::clear_contacts()
{
    //Clear the list of contacted objects and contact points
    gContacts.clear();
    gPenetrationContacts.clear();
    gContactedObjects.clear(); 
    gContactPositions.clear();

    cout << "Cleared contacts" << endl;
}

/// @brief Sets a factor that multiplies that maximal distance an intersection point can be from the objects considered in contact.
// The smaller the factor, the more contact points are filtered out.
/// @param max_distance_factor Distance factor (no unit).
/// @note Setting this to zero will disable filtering, and is not recommended.
/// @note See: all_triangles_overlap_over_AARectangle() in Scene.cpp
void Scene::set_max_distance_factor(float max_distance_factor){
    assert(max_distance_factor >= 0);
    this->max_distance_factor = max_distance_factor;
}

/// @brief Step the simulation by a given time step.
/// @param dt time step
void Scene::step_simulation(float dt)
{
    assert(dt > 0);

    //Clear the list of contacted objects and contact points
    clear_contacts();

    gScene->simulate(dt);
    gScene->fetchResults(true);
}

/// @brief Get the list of objects in contact with a given object.
/// @param target_object id of the object
/// @return list of objects in contact with the target object
set<string> Scene::get_contacted_objects(string target_object)
{
    //If there are no contacts, step the simulation by a small amount
    // to make sure that the collision detection is performed.
    if(gContacts.size() == 0){
        this->step_simulation(1/1000.0f);
    }

    set<string> contacted_objects;
    for (int i = 0; i < gContactedObjects.size(); i++)
    {
        if (gContactedObjects[i].first == target_object)
        {
            contacted_objects.insert(gContactedObjects[i].second);
        }
        else if (gContactedObjects[i].second == target_object)
        {
            contacted_objects.insert(gContactedObjects[i].first);
        }
    }
    return contacted_objects;
}

/// @brief Get the contact points between two objects.
/// @param id1 id of the first object
/// @param id2 id of the second object
/// @return Nx3 matrix representing the contact points between the two objects
MatrixX3f Scene::get_contact_points(string id1, string id2)
{
    //If there are no contacts, step the simulation by a small amount
    // to make sure that the collision detection is performed.
    if(gContacts.size() == 0){
        this->step_simulation(1/1000.0f);
    }

    //Iterate over gContacts and find the contact points between the two objects
    vector<Vector3f> contact_points;
    for (int i = 0; i < gContacts.size(); i++)
    {
        pair<string, string> object_ids = gContacts[i].get_object_ids();
        if (object_ids.first == id1 && object_ids.second == id2)
            contact_points.push_back(gContacts[i].get_position());
        if (object_ids.first == id2 && object_ids.second == id1)
            contact_points.push_back(gContacts[i].get_position());
    }

    //Convert the vector of contact points to a matrix
    MatrixX3f contact_points_matrix(contact_points.size(), 3);
    for (int i = 0; i < contact_points.size(); i++)
    {
        //The contact point is a column vector, but we want to store it as a row vector.
        contact_points_matrix.row(i) = contact_points[i].transpose();
    }
    return contact_points_matrix;
}

/// @brief Get the contact points between two objects.
/// @param id1 id of the first object
/// @param id2 id of the second object
/// @return Nx3 matrix representing the contact points between the two objects
MatrixX3f Scene::get_penetrating_contact_points(string id1, string id2)
{
    //If there are no contacts, step the simulation by a small amount
    // to make sure that the collision detection is performed.
    if(gPenetrationContacts.size() == 0){
        this->step_simulation(1/1000.0f);
    }

    //Iterate over gPenetrationContacts and find the contact points between the two objects
    vector<Vector3f> contact_points;
    for (int i = 0; i < gPenetrationContacts.size(); i++)
    {
        pair<string, string> object_ids = gPenetrationContacts[i].get_object_ids();
        if (object_ids.first == id1 && object_ids.second == id2)
            contact_points.push_back(gPenetrationContacts[i].get_position());
        if (object_ids.first == id2 && object_ids.second == id1)
            contact_points.push_back(gPenetrationContacts[i].get_position());
    }

    //Convert the vector of contact points to a matrix
    MatrixX3f contact_points_matrix(contact_points.size(), 3);
    for (int i = 0; i < contact_points.size(); i++)
    {
        //The contact point is a column vector, but we want to store it as a row vector.
        contact_points_matrix.row(i) = contact_points[i].transpose();
    }
    return contact_points_matrix;
}

/// @brief Compare each pair of contact points and merge them if they are close enough.
/// @param position_threshold Maximal distance between two contact points to be merged
/// @param normal_threshold Maximum angle cosine between the normals of two contact points to be merged
/// @note This is slow, use it only when necessary.
void Scene::merge_similar_contact_points(float position_threshold = 0, float normal_threshold = 0.1)
{
    //If the position threshold is not specified, use an adaptive threshold based on the
    // size of the voxels in contact.
    bool compute_adaptive_threshold = (position_threshold == 0);

    //Merge points that are less than some distance apart if their normal is mostly aligned
    for (int i = 0; i < gContacts.size(); i++)
    {
        Contact contact1 = gContacts[i];

        if(compute_adaptive_threshold){
            pair<Object*, Object*> objects = contact1.get_objects();
            Object* object1 = objects.first;
            Object* object2 = objects.second;
            Vector3f o1_sides = object1->get_voxel_side_lengths();
            Vector3f o2_sides = object2->get_voxel_side_lengths();
            float max_side = max(o1_sides.maxCoeff(), o2_sides.maxCoeff());
            float position_threshold = 0.1*max_side;
        }

        for (int j = i + 1; j < gContacts.size(); j++)
        {
            Contact contact2 = gContacts[j];

            if(compute_adaptive_threshold){
                pair<Object*, Object*> objects = contact1.get_objects();
                Object* object1 = objects.first;
                Object* object2 = objects.second;
                Vector3f o1_sides = object1->get_voxel_side_lengths();
                Vector3f o2_sides = object2->get_voxel_side_lengths();
                float max_side = max(o1_sides.maxCoeff(), o2_sides.maxCoeff());
                position_threshold = max(position_threshold, 0.1f*max_side);
            }

            Vector3f pos1 = contact1.get_position();
            Vector3f pos2 = contact2.get_position();
            Vector3f normal1 = contact1.get_normal();
            Vector3f normal2 = contact2.get_normal();
            float pos_dist = (pos1 - pos2).norm();
            float normal_dist = 1 - normal1.dot(normal2);
            //If the two contact points are close enough, merge them
            if (pos_dist < position_threshold && normal_dist < normal_threshold)
            {
                //Merge the two contact points
                Vector3f pos = (pos1 + pos2) / 2;
                Vector3f normal = (normal1 + normal2) / 2;
                contact1.set_position(pos);
                contact1.set_normal(normal);
                //Remove the second contact point
                gContacts.erase(gContacts.begin() + j);
                j--;
            }
        }
    }
}

/// @brief Create a shape representing a tetrahedron as described in convexMeshDesc
/// @param params Parameters for cooking the mesh.
/// @param convexMeshDesc Description of the tetrahedral convex mesh.
/// @return Managed Pointer to the output PxShape.
PxShape* createTetrahedronShape(PxCookingParams params, PxConvexMeshDesc convexMeshDesc){
    //WARNING: I had no luck in making 
    //      bool res = PxValidateConvexMesh(params, convexMeshDesc);
    // work, as it always complained about
    //      physx/source/geomutils/src/cooking/GuCookingConvexHullBuilder.cpp#L479

    // cout << "Vertices: " << endl;
    // for(int i=0; i < convexMeshDesc.points.count; i++){
    //     PxVec3* v;
    //     v = (PxVec3*)(convexMeshDesc.points.data + i*sizeof(PxVec3));
    //     cout << "    " << v->x << ", " << v->y << ", " << v->z << endl;
    // }

    //Cooking: Build a convex mesh and output the result in a buffer
    PxConvexMeshCookingResult::Enum result;
    PxDefaultMemoryOutputStream buf;
    if(!PxCookConvexMesh(params, convexMeshDesc, buf, &result))
        throw runtime_error("Error cooking mesh");

    //Read the result from the buffer and create a convex mesh
    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);
    if(!convexMesh)
        throw runtime_error("Error creating mesh");

    //Create a geometry from the convex mesh
    PxConvexMeshGeometry convexGeometry = PxConvexMeshGeometry(convexMesh);
    bool isValid = convexGeometry.isValid();
    if(!isValid)
        throw runtime_error("Invalid convex mesh geometry");

    //Create a shape from the geometry
    // The shape has to be exclusive (hence the "true") such that it is guaranteed to be
    // associated with only one actor. That way, the actor can be retrieved from the shape
    // in the contact report callback.
    PxShape* shape = gPhysics->createShape(convexGeometry, *gMaterial, true);
    if(!shape)
        throw runtime_error("Error creating shape");

    return shape;
}

/// @brief Create a PxShape representing a cube for the given occupancy cell.
/// @return Shape representing the cube.
PxShape* createVoxelShape(GridCell* cell)
{
    PxBoxGeometry boxGeometry = PxBoxGeometry(cell->half_extents[0], cell->half_extents[1], cell->half_extents[2]);
    PxShape* shape = gPhysics->createShape(boxGeometry, *gMaterial, true);
    if(!shape)
        throw runtime_error("Error creating shape");

    shape->userData = cell;
    //This is the pose relative to the actor's frame. If the actor is moved, the shape will move with it.
    shape->setLocalPose(PxTransform(PxVec3(cell->centre[0], cell->centre[1], cell->centre[2])));
    //Since the voxel will be at a half-extent away from the surface of the object,
    // we set the contact offset to zero, the minimal contact distance is actually the half-extent.
    //               |-----------------|
    //        |------|-----|           |
    //        |<--D--X     |           |
    //        |------|-----|           |
    //               |                 |
    //               |-----------------|
    shape->setRestOffset(0);
    shape->setContactOffset(1e-6);

    return shape;
}


/// @brief Produce a sphere for at each voxel corner that is contained in a tetrahedron defining the volume of the object.
///         These spheres can be used to detect interpenetration as they are embedded in the volume of the object.
/// @param obj Object for which to generate the spheres (must be volumetric).
/// @param tetConvexShapes Array of tetrahedra from make_tetmesh().
/// @return An array of shapes, each representing a sphere.
PxArray<PxShape*> Scene::make_canary_spheres(Object* obj, PxArray<PxShape*> tetConvexShapes)
{
    //Surface voxels information
    MatrixX3f voxel_centres = obj->get_voxel_centres();
    Vector3f voxel_side_lengths = obj->get_voxel_side_lengths();

    auto t1 = chrono::high_resolution_clock::now();

    //Get the half-extents of the voxel
    Vector3f voxel_half_extents = voxel_side_lengths/2;

    //A 3xN matrix of the vertices of the voxels where N is the number of voxels
    Matrix3Xf voxel_vertices_matrix(3, voxel_centres.rows()*2);

    //Generate a point at the two extremities of each voxel
    Matrix3Xf batch = (voxel_centres.rowwise() + RowVector3f(voxel_half_extents[0], voxel_half_extents[1], voxel_half_extents[2])).transpose();
    voxel_vertices_matrix.block(0, 0, 3, batch.cols()) = batch;
    batch = (voxel_centres.rowwise() - RowVector3f(voxel_half_extents[0], voxel_half_extents[1], voxel_half_extents[2])).transpose();
    voxel_vertices_matrix.block(0, batch.cols(), 3, batch.cols()) = batch;

    auto t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Generated voxel vertices in " << duration << " ms" << endl;

    t1 = chrono::high_resolution_clock::now();

    //Create a list of tetrahedron
    vector<vector<Vector3f>> tetra_vertices_list;
    for(int i = 0; i < tetConvexShapes.size(); i++){
        PxShape* tetConvexShape = tetConvexShapes[i];
        const PxConvexMeshGeometry& convexMeshGeometry = static_cast<const PxConvexMeshGeometry&>(tetConvexShape->getGeometry());
        PxConvexMesh* convexMesh = convexMeshGeometry.convexMesh;
        //Get the vertices
        const PxVec3* vertices = convexMesh->getVertices();
        Vector3f v1(vertices[0].x, vertices[0].y, vertices[0].z);
        Vector3f v2(vertices[1].x, vertices[1].y, vertices[1].z);
        Vector3f v3(vertices[2].x, vertices[2].y, vertices[2].z);
        Vector3f v4(vertices[3].x, vertices[3].y, vertices[3].z);
        //Vector of vertices
        vector<Vector3f> tetra_vertices = {v1, v2, v3, v4};
        tetra_vertices_list.push_back(tetra_vertices);
    }

    //Iterate over tetrahedra and flag the points that are inside the volume
    VectorXd inside_volume = VectorXd::Zero(voxel_vertices_matrix.cols());
    //Number of threads to use (at most the number of tetrahedra)
    const int num_threads = min(tetConvexShapes.size(), std::thread::hardware_concurrency());
    if(num_threads > 1){
        //List of threads
        std::vector<std::thread> threads(num_threads);
        //List of output vectors
        std::vector<VectorXd> thread_output(num_threads);
        for(int thread_i = 0; thread_i < num_threads; thread_i++){
            //Get the number of tetrahedron to process in this thread
            int nb_tet_per_thread = tetConvexShapes.size() / num_threads;
            //Get the index of the first tet to process in this thread
            int first_tet_index = thread_i*nb_tet_per_thread;
            //Get the index of the last tet to process in this thread
            int last_tet_index = (thread_i == num_threads - 1) ? tetConvexShapes.size() : (thread_i+1)*nb_tet_per_thread;
            //Initialize the thread output
            thread_output[thread_i] = VectorXd::Zero(voxel_vertices_matrix.cols());
            //Launch the thread
            threads[thread_i] = std::thread(&Scene::points_in_tetrahedron, this, 
                voxel_vertices_matrix, tetra_vertices_list, first_tet_index, last_tet_index, 
                std::ref(thread_output[thread_i]));
        }
        //Join the threads
        for(auto& thread : threads){
            thread.join();
        }
        //Merge the outputs
        for(auto& output : thread_output){
            inside_volume += output;
        }
    }else{
        points_in_tetrahedron(voxel_vertices_matrix, tetra_vertices_list, 0, tetra_vertices_list.size(), inside_volume);
    }
    t2 = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Iterated over " << tetConvexShapes.size() << " tetrahedra in " << duration << " ms" << endl;

    t1 = chrono::high_resolution_clock::now();
    //For each vertex flagged as inside the volume, create a sphere
    PxArray<PxShape*> spheres;
    for(int i = 0; i < inside_volume.size(); i++){
        if(inside_volume[i] > 0){
            //Create a sphere at the voxel vertex
            PxSphereGeometry sphereGeometry = PxSphereGeometry(voxel_side_lengths.minCoeff()/10);
            PxShape* sphere = gPhysics->createShape(sphereGeometry, *gMaterial, true);
            if(!sphere)
                throw runtime_error("Error creating shape");
            //Set the position of the sphere
            Vector3f pos = voxel_vertices_matrix.col(i);
            sphere->setLocalPose(PxTransform(PxVec3(pos[0], pos[1], pos[2])));
            sphere->setRestOffset(0);
            sphere->setContactOffset(1e-6);
            //Add the sphere to the list
            spheres.pushBack(sphere);
        }
    }
    t2 = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Created " << spheres.size() << " spheres for object " << obj->id << " in " << duration << " ms" << endl;
    return spheres;
}

/// @brief Determine if the points in voxel_vertices_matrix are inside the tetrahedron defined by tetra_vertices.
/// @param voxel_vertices_matrix Matrix where each column is a 3D point to test.
/// @param tetra_vertices_list List of vertices of the tetrahedra.
/// @param index_first_tet Index of the first tetrahedron to process.
/// @param index_last_tet Index of the last tetrahedron to process.
/// @param inside_volume [output] Vector of values, either 0 or 1, indicating if each point is inside at least one tetrahedron.
void Scene::points_in_tetrahedron(Matrix3Xf voxel_vertices_matrix, vector<vector<Vector3f>> tetra_vertices_list, int index_first_tet, int index_last_tet, VectorXd& inside_volume)
{
    for(int tet_i = index_first_tet; tet_i < index_last_tet; tet_i++){
        vector<Vector3f> tetra_vertices = tetra_vertices_list[tet_i];
        //Compute the tetrahedron coordinate system
        Matrix3f T_world_to_local = tetra_local_frame(tetra_vertices);
        //Test if the points are inside the tetrahedron
        inside_volume += check_points_in_tetra(voxel_vertices_matrix, T_world_to_local, tetra_vertices[0]);
    }
}

/// @brief Define an affine transformation that maps a world point to barycentric coordinates in the tetrahedron.
/// @param tetra_vertices Vertices of the tetrahedron.
/// @note See https://stackoverflow.com/a/60745339
/// and https://math.stackexchange.com/a/2207879
Matrix3f Scene::tetra_local_frame(const vector<Vector3f>& tetra_vertices) 
{
    //The frame is defined relative to the first vertex
    Vector3f origin = tetra_vertices[0];
    //Each edge defines a basis vector of a matrix that maps barycentric coordinates
    // to world coordinates. 
    Matrix3f mat;
    for (int i = 0; i < 3; i++) {
        mat.col(i) = tetra_vertices[i + 1] - origin;
    }
    //The inverse of this matrix maps world coordinates to barycentric coordinates.
    Matrix3f T_world_to_local = mat.inverse();
    return T_world_to_local;
}

/// @brief Determine if points are inside a tetrahedron.
/// @param points Matrix 3xN representing the points to test.
/// @param T_world_to_local Transformation matrix from the world frame to the local frame of the tetrahedron.
/// @param origin Position of the origin of the tetrahedron frame in the world frame.
/// @return Vector of booleans, one for each point, indicating whether the point is inside the tetrahedron.
VectorXd Scene::check_points_in_tetra(const Matrix3Xf& points, const Matrix3f& T_world_to_local, const Vector3f& origin) 
{
    //A greater tolerance is more strict on inclusion
    float tol = 0.1;

    //Get the barycentric coordinates of the points WRT to this tetrahedron
    Matrix3Xf mapped_points = T_world_to_local * (points.colwise() - origin);

    VectorXf minCoeffs = mapped_points.colwise().minCoeff();
    VectorXf maxCoeffs = mapped_points.colwise().maxCoeff();
    VectorXf sums = mapped_points.colwise().sum();

    // Create a vector of booleans indicating whether each point is inside the tetrahedron
    Array<bool, Dynamic, 1> inside(points.cols());
    inside = (minCoeffs.array() >= tol) && (maxCoeffs.array() <= 1-tol) && (sums.array() <= 1);

    return inside.matrix().cast<double>();
}

/// @brief Generate a tetrahedral mesh from the triangular surface mesh of the given object.
/// @param obj Object for which to generate the tetrahedral mesh.
/// @return Array of shapes, each representing a tetrahedron.
PxArray<PxShape*> Scene::make_tetmesh(Object* obj)
{
    PxArray<PxVec3> tetMeshVertices;
    PxArray<PxU32> tetMeshIndices;

    shared_ptr<PxSimpleTriangleMesh> triSurfaceMesh_ptr = obj->tri_mesh;
    PxSimpleTriangleMesh triSurfaceMesh = *triSurfaceMesh_ptr;

    //Create a tetrahedral mesh from the triangle mesh
    bool result = obj->create_tetra_mesh(triSurfaceMesh, tetMeshVertices, tetMeshIndices);

    if(!result)
        throw runtime_error("Error creating tetrahedral mesh");
    
    //For each tetrahedron, create a convex mesh.
    PxArray<PxConvexMeshDesc> convexMeshDescs;
    result = obj->create_tetra_convex_set(tetMeshVertices, tetMeshIndices, convexMeshDescs);

    if(!result)
        throw runtime_error("Error creating convex mesh");

    //Record the tetrahedral mesh in the object
    obj->set_tetra_mesh(tetMeshVertices, tetMeshIndices);

    PxArray<PxShape*> convexShapes;
    //For each convex mesh, create a shape
    for(int i = 0; i < convexMeshDescs.size(); i++){
        PxConvexMeshDesc convexMeshDesc = convexMeshDescs[i];
        //Cooking parameters
        PxTolerancesScale scale;
        PxCookingParams params(scale);
        PxShape* convexShape = createTetrahedronShape(params, convexMeshDesc);

        //Contacts are genereated when an object is at the contact offset from this shape
        // which must be positive and slightly larger than the rest offset that defines
        // the depth of the equilibrium position of the object.
        convexShape->setRestOffset(0);
        convexShape->setContactOffset(1e-6);

        convexShapes.pushBack(convexShape);
    }
    return convexShapes;
}

/// @brief Add an object to the scene
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object WRT the object frame
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param resolution resolution of the occupancy grid (default: 15)
/// @param is_volumetric boolean representing whether the object is volumetric (default: true)
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object WRT the object frame (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
void Scene::add_object(string id, Matrix4f pose, MatrixX3f vertices, MatrixX3i triangles, int resolution, bool is_volumetric, bool is_fixed, float mass, Vector3f com, string material_name)
{
    assert(vertices.rows() > 0);
    assert(triangles.rows() > 0);
    assert(resolution > 0);
    assert(mass > 0);

    //The vertices are given relative to the object frame, so we transform them to the world frame
    Matrix3f pose_R = pose.block<3,3>(0,0);
    Vector3f pose_t = pose.block<3,1>(0,3);
    vertices = (pose_R*vertices.transpose()).transpose().rowwise() + pose_t.transpose();

    //Create an object instance and add it to the scene
    // When adding an element to the vector, the vector may reallocate memory and move the elements which will change their addresses.
    // However, we need to supply the callback with a persistent pointer to the object.
    // Therefore, we store pointers to preallocated objects in the vector instead of the objects themselves.
    shared_ptr<Object> obj = make_shared<Object>(this, id, pose, vertices, triangles, is_volumetric, is_fixed, mass, com, material_name);
    object_ptrs.push_back(obj);

    //Record the triangle mesh in the object
    obj->set_tri_mesh(vertices, triangles);
    //obj->remesh_surface_trimesh();

    //Create a occupancy grid
    auto t1 = chrono::high_resolution_clock::now();
    shared_ptr<OccupancyGrid> grid = obj->create_occupancy_grid(resolution, OccupancyGrid::sampling_method::random);
    auto t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Occupancy grid created in " << duration << " ms" << endl;

    PxTolerancesScale scale;
    PxCookingParams params(scale);
    PxArray<PxShape*> convexShapes;

    t1 = chrono::high_resolution_clock::now();
    //Create a shape from each voxel in the occupancy grid.
    unordered_map<uint32_t, GridCell>* cells = grid->get_grid_cells();
    //Create a shape for each occupancy grid cell
    for(auto& item : *cells){
        uint32_t index = item.first;
        GridCell* cell  = &item.second;
        PxShape* voxelShape = createVoxelShape(cell);
        convexShapes.pushBack(voxelShape);
    }
    t2 = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Created voxel shapes in " << duration << " ms" << endl;

    //If the object is volumetric, create a tetrahedral mesh and spheres embedded in the volume
    // to enable interpenetration detection.
    if(is_volumetric){
        t1 = chrono::high_resolution_clock::now();
        //Create the tetrahedral mesh built from the triangle mesh.
        PxArray<PxShape*> tetConvexShapes = make_tetmesh(obj.get());
        //Augment convexShapes with the shapes
        for(auto& shape : tetConvexShapes){
            convexShapes.pushBack(shape);
        }
        t2 = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Created tetrahedral shapes in " << duration << " ms" << endl;

        t1 = chrono::high_resolution_clock::now();
        //Create spheres embedded in the volume of the object that can be used to detect interpenetration.
        PxArray<PxShape*> sphereShapes = make_canary_spheres(obj.get(), tetConvexShapes);
        //Augment convexShapes with the shapes
        for(auto& shape : sphereShapes){
            convexShapes.pushBack(shape);
        }
        t2 = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Created canary spheres in " << duration << " ms" << endl;
    }

    //The pose of the object/actor is always set to identity
    // but the triangles are transformed to the correct pose.
    PxTransform pxPose = PxTransform(PxIdentity);
    
    //If the object is fixed, there is no dynamics involved
    if(is_fixed){
        PxRigidStatic* actor = gPhysics->createRigidStatic(pxPose);
        //Attach all shapes in the object to the actor
        for(int i = 0; i < convexShapes.size(); i++){
            actor->attachShape(*convexShapes[i]);
            //We release the shape after it has been attached such that when the actor
            // is released, the shape is also released.
            convexShapes[i]->release();
        }
        gScene->addActor(*actor);
        actor->setName(obj->id.c_str());
        actor->userData = obj.get();
    }
    else{
        PxRigidDynamic* actor = gPhysics->createRigidDynamic(pxPose);
        //A dynamic actor made from a triangle mesh must have a defined SDF, which requires tuning parameters.
        // If the forces exerted on the objects are not needed, the actor can be made kinematic, which does not require a SDF.
        // A kinematic actor will push dynamic objects with infinite force and can go through static objects.
        // By default collisions between kinematic rigid bodies and kinematic and static rigid bodies will not get reported. 
        // To enable these reports use the PxSceneDesc::kineKineFilteringMode and PxSceneDesc::staticKineFilteringMode parameters 
        // when creating a scene.
        actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        //Attach all shapes in the object to the actor
        for(int i = 0; i < convexShapes.size(); i++){
            actor->attachShape(*convexShapes[i]);
        }
        PxVec3 pxCom = PxVec3(com(0), com(1), com(2));
        //PxRigidBodyExt::setMassAndUpdateInertia(*actor, mass, &pxCom, true);
        actor->setMass(mass);
        actor->setCMassLocalPose(PxTransform(pxCom));
        gScene->addActor(*actor);
        actor->setKinematicTarget(pxPose);
        actor->setName(obj->id.c_str());
        actor->userData = obj.get();
    }

    //The contacts are no longer valid, so clear them
    clear_contacts();
}

/// @brief Get the actor with a specified name from the scene
/// @param name name of the actor / object
/// @return pointer to the actor or NULL if not found
PxRigidActor* Scene::get_actor(string name)
{
    //Get the number of actors in the scene that are either dynamic or static
    PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

    //Get all rigid actors in the scene
    PxActor** actors = new PxActor*[nbActors];
    PxU32 nbActorsReturned = gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, actors, nbActors);

    //Iterate over actors and find the one with the specified name
    for (int i = 0; i < nbActorsReturned; i++)
    {
        PxRigidActor* actor = static_cast<PxRigidActor*>(actors[i]);
        if (actor->getName() == name)
        {
            return actor;
        }
    }
    return NULL;
}

/// @brief Get the shapes attached to an actor
/// @param actor pointer to the actor
/// @return vector of pointers to the shapes attached to the actor
vector<PxShape*> Scene::get_actor_shapes(PxRigidActor* actor)
{
    vector<PxShape*> shapes;
    //Get the number of shapes attached to the actor
    PxU32 nbShapes = actor->getNbShapes();
    //Get all shapes attached to the actor
    PxShape** shapesArray = new PxShape*[nbShapes];
    PxU32 nbShapesReturned = actor->getShapes(shapesArray, nbShapes);
    //Iterate over shapes and add them to the vector
    for (int i = 0; i < nbShapesReturned; i++)
    {
        shapes.push_back(shapesArray[i]);
    }
    return shapes;
}

/// @brief Remove an object from the scene along with its attached shapes
/// @param id unique identifier for the object
void Scene::remove_object(string id)
{
    //Get the actor with the specified name
    PxRigidActor* actor = get_actor(id);
    if (actor == NULL)
    {
        cout << "Object with id " << id << " not found" << endl;
        return;
    }

    //Get the shapes attached to the actor
    vector<PxShape*> shapes = get_actor_shapes(actor);
    //Remove the shapes from the actor
    for (int i = 0; i < shapes.size(); i++)
    {
        actor->detachShape(*shapes[i]);
    }

    //Remove the actor from the scene
    gScene->removeActor(*actor);
    actor->release();

    //Remove the object from the list of objects
    for (int i = 0; i < this->object_ptrs.size(); i++)
    {
        if (this->object_ptrs[i]->id == id)
        {
            this->object_ptrs.erase(this->object_ptrs.begin() + i);
            break;
        }
    }

    //The contacts are no longer valid, so clear them
    clear_contacts();
}

/// @brief Set the pose of an object by recreating a new object.
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @note This is a relatively expensive operation.
void Scene::set_object_pose(string id, Matrix4f pose)
{
    //Although it might seems to be inefficient, we need to recreate the object
    // as the OccupancyGrid has to be axis-aligned, making simple transformation
    // of the grid cells impossible. Furthermore, the shapes need to refer to the
    // new grid cells for the contact report callback to work properly.

    //Get object by ID
    Object* obj = get_object_by_id(id);

    if(obj != nullptr){
        //Get object mesh
        Matrix4f previous_pose = obj->pose;
        MatrixX3f vertices  = obj->tri_vertices;
        MatrixX3i triangles = obj->tri_triangles;
        int resolution      = obj->get_grid_resolution();
        bool is_fixed       = obj->is_fixed;
        bool is_volumetric  = obj->is_volumetric;
        float mass          = obj->mass;
        Vector3f com        = obj->com;
        string material_name = obj->material_name;

        //Remove the object from the scene
        remove_object(id);

        //The vertices are in the world frame, but the add_object() function expects them to be in the object frame.
        // Hence, we need to transform them back to the object frame by using the inverse of the previous pose.
        Matrix4f previous_pose_inv = previous_pose.inverse();
        MatrixX3f ppi_R = previous_pose_inv.block<3,3>(0,0);
        Vector3f  ppi_t = previous_pose_inv.block<3,1>(0,3);
        vertices = (ppi_R*vertices.transpose()).transpose().rowwise() + ppi_t.transpose();

        //Create a new object with the same mesh and the new pose
        add_object(id, pose, vertices, triangles, resolution, is_volumetric, is_fixed, mass, com, material_name);
    }
}

/// @brief Return the object that has the given ID
/// @param id unique identifier for the object
/// @return pointer to the object
Object* Scene::get_object_by_id(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj.get();
    cout << "Object with id " << id << " not found" << endl;
    return nullptr;
}

/// @brief Get the pose of an object
/// @param id unique identifier for the object
/// @return 4x4 matrix representing the pose of the object
Matrix4f Scene::get_object_pose(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
        return obj->pose;
    return Matrix4f::Identity();
}

/// @brief Get the vertices of the triangle mesh of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Scene::get_tri_vertices(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
        return obj->tri_vertices;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the triangles of the triangle mesh of an object
/// @param id unique identifier for the object
/// @return Mx3 matrix representing the triangles of the object
MatrixX3i Scene::get_tri_triangles(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
            return obj->tri_triangles;
    return MatrixX3i::Zero(0, 3);
}

/// @brief Get the vertices of the tetrahedral mesh of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Scene::get_tetra_vertices(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
            return obj->tetra_vertices;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the tetrahedra of the tetrahedral mesh of an object
/// @param id unique identifier for the object
/// @return Mx4 matrix representing the tetrahedra of the object
MatrixX4i Scene::get_tetra_indices(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
            return obj->tetra_indices;
    return MatrixX4i::Zero(0, 4);
}

/// @brief Get the voxel centres of the occupied voxels of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the voxel centres of the object
MatrixX3f Scene::get_voxel_centres(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
            return obj->get_voxel_centres();
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the voxel side lengths of the voxels of an object (assuming at least one voxel is occupied)
/// @param id unique identifier for the object
/// @return 3x1 vector representing the voxel side lengths of the object
Vector3f Scene::get_voxel_side_lengths(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
            return obj->get_voxel_side_lengths();
    return Vector3f::Zero();
}