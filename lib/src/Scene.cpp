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
#ifndef NDEBUG
    #include "omnipvd/PxOmniPvd.h"
    #include "../pvdruntime/include/OmniPvdWriter.h"
    #include "../pvdruntime/include/OmniPvdFileWriteStream.h"
#endif

using namespace physx;
using namespace std;
using namespace Eigen;

/* TODO: If two objects are exactly overlapping, the contact points and the penetration
    points are not detected.
*/

ContactsManager contacts_manager;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxOmniPvd*               gOmniPvd = NULL;
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

    //Do not use eSOLVE_CONTACT with kinematic bodies as no force is involved (eCONTACT_DEFAULT = eSOLVE_CONTACT | eDETECT_DISCRETE_CONTACT)
    //When using eDETECT_DISCRETE_CONTACT "Contacts are only responded to if eSOLVE_CONTACT is enabled."
    // However, not using it result in: "warning : Filtering: Pair did not request either eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT. It is recommended to suppress/kill such pairs for performance reasons."
	//Is PxPairFlag::eNOTIFY_TOUCH_FOUND needed if we use eNOTIFY_TOUCH_PERSISTS
    // WARNING: No report will get sent if the objects in contact are sleeping.
    // Note: We are not using eNOTIFY_TOUCH_PERSISTS since the introduction of the ContactsManager.
	pairFlags = PxPairFlag::eDETECT_DISCRETE_CONTACT
                //| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
                | PxPairFlag::eNOTIFY_TOUCH_FOUND
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
            #ifndef NDEBUG
            cout << "No contact detected" << endl;
            #endif
            return;
        }else{
            #ifndef NDEBUG
            cout << "Contact detected: " << nbPairs << " pairs" << endl;
            #endif
        }

        //In debug, we only use one thread to make debugging easier
        #ifndef NDEBUG
        const int num_threads = 1;
        #else
        //Maximum number of threads to use (can be zero)
        const int num_threads = std::thread::hardware_concurrency();
        #endif

        //NOTE: PhysX provides a way to handle multi-threading for contact resolution
        // without the user having to implement the mechanics.
        // See: https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/Simulation.html#split-fetchresults

        if(num_threads > 1){
            //List of threads
            std::vector<std::thread> threads(num_threads);
            //Each thread will generate a list of surface contacts, all of which will be merged at the end.
            std::vector<std::vector<Contact>> thread_contacts(num_threads);
            //Each thread will generate a list of penetrating contacts, all of which will be merged at the end.
            std::vector<std::vector<Contact>> thread_pen_contacts(num_threads);
            //Each thread will generate a list of contacted objects, all of which will be merged at the end.
            std::vector<std::vector<pair<string, string>>> thread_contacted_objects(num_threads);

            #ifndef NDEBUG
            std::cout << "Using " << num_threads << " threads" << std::endl;
            #endif

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
                contacts_manager.add_contacts(thread_contact, false);
            }
            for(auto& thread_pen_contacts : thread_pen_contacts){
                contacts_manager.add_contacts(thread_pen_contacts, true);
            }
        }else{
            std::vector<Contact> contacts;
            std::vector<Contact> pen_contacts;
            std::vector<pair<string, string>> contact_object_ids;
            process_contact_pairs(pairs, 0, nbPairs, contacts, pen_contacts, contact_object_ids);
            //Add the surface contact points to the list of contact points
            contacts_manager.add_contacts(contacts, false);
            //Add the penetrating contact points to the list of contact points
            contacts_manager.add_contacts(pen_contacts, true);

            #ifndef NDEBUG
            cout << "Added " << contacts.size() << " surface contact points" << endl;
            cout << "Added " << pen_contacts.size() << " penetrating contact points" << endl;
            #endif
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

            //Ignore pairs when shapes have been deleted
            if(pair.flags & (PxContactPairFlag::eREMOVED_SHAPE_0 | PxContactPairFlag::eREMOVED_SHAPE_1)){
                #ifndef NDEBUG
                cout << "Scene::process_contact_pairs(): One of the shapes has been deleted. Ignoring pair." << endl;
                #endif
                continue;
            }
  
            if(contactCount)
            {
                //Get the shapes involved in the collision
                PxShape* shape0 = pair.shapes[0];
                PxShape* shape1 = pair.shapes[1];

                //If the pointers are null, we skip this pair
                if(shape0 == nullptr || shape1 == nullptr){
                    #ifndef NDEBUG
                    cout << "Scene::process_contact_pairs(): One of the shapes is null. Ignoring pair." << endl;
                    #endif
                    continue;
                }

                //Geometry type of each shape
                PxGeometryType::Enum shape0_type = shape0->getGeometry().getType();
                PxGeometryType::Enum shape1_type = shape1->getGeometry().getType();
                
                //Get the actors
                PxRigidActor* actor0 = shape0->getActor();
                PxRigidActor* actor1 = shape1->getActor();

                //If the pointers are null, we skip this pair
                if(actor0 == nullptr || actor1 == nullptr){
                    #ifndef NDEBUG
                    cout << "Scene::process_contact_pairs(): One of the actors is null. Ignoring pair." << endl;
                    #endif
                    continue;
                }

                string id_obj0 = actor0->getName();
                string id_obj1 = actor1->getName();

                //If the collision is between an actor and itself, we skip this pair
                if(id_obj0 == id_obj1){
                    continue;
                }

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
                if( (shape0_type == PxGeometryType::eSPHERE && shape1_type == PxGeometryType::eCONVEXMESH) ||
                    (shape1_type == PxGeometryType::eSPHERE && shape0_type == PxGeometryType::eCONVEXMESH)){
                    PxVec3 p;
                    //Get the position of the sphere/point
                    if(shape0_type == PxGeometryType::eSPHERE){
                        p = shape0->getLocalPose().p;
                        //Get the position of the sphere/point in the world frame
                        PxTransform actor_pose = actor0->getGlobalPose();
                        p = actor_pose.transform(p);
                    }else{
                        p = shape1->getLocalPose().p;
                        //Get the position of the sphere/point in the world frame
                        PxTransform actor_pose = actor1->getGlobalPose();
                        p = actor_pose.transform(p);
                    }
                    Contact contact(obj0, obj1, Vector3f(p.x, p.y, p.z), Vector3f(0,0,1), 0.0f);
                    thread_pen_contacts.push_back(contact);
                    //Mark the two objects as being in contact
                    thread_contacted_objects.push_back(make_pair(id_obj0, id_obj1));
                }

                //The collision is between two gridcells
                if( shape0_type == PxGeometryType::eBOX && shape1_type == PxGeometryType::eBOX ){
                    if(shape0->userData != nullptr && shape1->userData != nullptr){
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
                            Contact contact(obj0, obj1, Vector3f(p[0], p[1], p[2]), p.normal, 0.0f);
                            thread_contacts.push_back(contact);
                        }
                        //Mark the two objects as being in contact
                        thread_contacted_objects.push_back(make_pair(id_obj0, id_obj1));
                    }else{
                        #ifndef NDEBUG
                        cout << "Scene::process_contact_pairs(): One of the gridcells has no userData. Ignoring pair." << endl;
                        #endif
                    }
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
    //Releases contacts, scene, dispatcher, physics
    cleanupPhysics();

    //Only one foundation object can be spawned per process. So we should keep track of it
    // for the lifetime of the process (even if we destroy the scene).
    if(gFoundation == NULL){
        gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    }

    //The OmniPVD data stream is only generated in debug and checked builds, not in release builds.
    #ifndef NDEBUG
    
        // Create the OmniPVD instance
        gOmniPvd = PxCreateOmniPvd(*gFoundation);
        bool success = false;
        if (gOmniPvd)
        {
            OmniPvdWriter* omniWriter = gOmniPvd->getWriter();
            OmniPvdFileWriteStream* omniFileWriteStream = gOmniPvd->getFileWriteStream();
            if (omniWriter && omniFileWriteStream)
            {
                omniFileWriteStream->setFileName("/home/phil/physx_debug_record.ovd");
                omniWriter->setWriteStream(static_cast<OmniPvdWriteStream&>(*omniFileWriteStream));
                gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
                if (gPhysics && gPhysics->getOmniPvd()){
                    success = gPhysics->getOmniPvd()->startSampling();
                    if(success){
                        cout << "OmniPvd sampling started." << endl;
                    }else{
                        cout << "Error: Failed to start OmniPvd sampling." << endl;
                    }
                }else{
                    cout << "Error: Failed to get OmniPvd instance from physics." << endl;
                }
            }else{
                cout << "Error: Failed to create OmniPvd writer or file stream." << endl;
            }
        }else{
            cout << "Error: Failed to create OmniPvd instance. Make sure that libPVDRuntime_64.so can be dynamically loaded." << endl;
        }

        if(!success){
            gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());
        }
    
    #else
        gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());
    #endif // NDEBUG

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
    sceneDesc.maxNbContactDataBlocks  = PX_MAX_U32;
    //When getting more contact points matters, its better to disable PCM.
    // See: https://nvidia-omniverse.github.io/PhysX/physx/5.2.1/docs/AdvancedCollisionDetection.html#persistent-contact-manifold-pcm
    sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;

    gScene = gPhysics->createScene(sceneDesc);
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.1f);
}

/// @brief Clean up the physics engine by releasing memory.
void Scene::cleanupPhysics()
{
    #ifndef NDEBUG
    cout << "Cleaning up physics" << endl;
    #endif

    //Clear the list of contacted objects and contact points
    contacts_manager.remove_all_objects();

    //Remove all actors from the scene
    // This should also release all shapes.
    remove_all_actors();

    //Free all the objects
    for(auto& obj_ptr : object_ptrs){
        obj_ptr = nullptr;
    }
    object_ptrs.clear();

    //"When a scene is released, any actors contained in the scene are automatically removed from the scene, but they are not released."
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    #ifndef NDEBUG
        PX_RELEASE(gOmniPvd);
    #endif
    //As noted in startupPhysics(), we should NOT release the foundation object.
    //PX_RELEASE(gFoundation);

    //Free the Contact Report Callback which was allocated in startupPhysics()
    if(gContactReportCallback != nullptr)
        delete gContactReportCallback;
}

Scene::Scene(){
    startupPhysics();
}
Scene::~Scene(){
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

/// @brief Wake up all actors in the scene by seetting their target pose.
void Scene::wake_all_actors()
{
    //Iterate over all objects and wake up their actors
    for(auto& obj_ptr : object_ptrs){
        PxRigidActor* actor = get_actor(obj_ptr->id);
        if(actor != nullptr){
            PxActorType::Enum actor_type = actor->getType();
            if(actor_type == PxActorType::Enum::eRIGID_DYNAMIC){
                PxTransform pxPose = obj_ptr->get_pose();
                //Cast to dynamic actor and set the target pose
                PxRigidDynamic* rigidDynamicActor = static_cast<PxRigidDynamic*>(actor);
                rigidDynamicActor->setKinematicTarget(pxPose);

                #ifndef NDEBUG
                cout << "Woke up actor " << obj_ptr->id << endl;
                #endif

            }else{
                #ifndef NDEBUG
                cout << "Object " << obj_ptr->id << " is not a dynamic actor." << endl;
                #endif
            }
        }else{
            #ifndef NDEBUG
            cout << "Could not wake up (nullptr) actor " << obj_ptr->id << endl;
            #endif
        }
    }
}

/// @brief Step the simulation by a given time step.
/// @param dt time step
void Scene::step_simulation(float dt)
{
    if(gScene == nullptr){
        #ifndef NDEBUG
        cout << "Scene is null" << endl;
        #endif
        return;
    }

    assert(dt > 0);

    /*
    READ THIS: SLEEPING
        For kinematic actors, special sleep rules apply. 
        A kinematic actor is asleep unless a target pose has been set (in which case it will stay awake until 
        two consecutive simulation steps without a target pose being set have passed). 
        As a consequence, it is not allowed to use setWakeCounter() for kinematic actors. 
        The wake counter of a kinematic actor is solely defined based on whether a target pose has been set.

        "Kinematic actors are expected to not attain the kinematic target immediately, but only at the end of the next simulation step.
         We infer the velocity of the kinematic from the current pose and the kinematic target, and use this velocity for interactions with dynamics."
         - From an NVIDIA employee: https://github.com/NVIDIA-Omniverse/PhysX/issues/32

        This involves that two subsequent simulation steps are required to get all contacts.
        See: https://github.com/NVIDIA-Omniverse/PhysX/blob/b7182d5e563b763a340053864658202a3ee966ab/physx/source/simulationcontroller/src/ScKinematics.cpp#L44
    
        Note: Since the introduction of the ContactsManager, we do not need to wake up actors anymore at each simulation step.
    */

    #ifndef NDEBUG
    auto t1 = std::chrono::high_resolution_clock::now();
    cout << "Stepping simulation by " << dt << " seconds" << endl;
    #endif

    //Perform two subsequent simulation steps to get contacts
    // from kinematic actors.
    gScene->simulate(dt/2);
    gScene->fetchResults(true);
    #ifndef NDEBUG
    PxU32 nbUsedDataBlocks = gScene->getNbContactDataBlocksUsed();
    PxU32 maxNbDataBlocks = gScene->getMaxNbContactDataBlocksUsed();
    cout << "Number of used contact data blocks: " << nbUsedDataBlocks << "/" << maxNbDataBlocks << endl;
    #endif
    gScene->simulate(dt/2);

    #ifndef NDEBUG
    cout << "Simulation step done. Fetching results." << endl;
    #endif

    gScene->fetchResults(true);

    #ifndef NDEBUG
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    cout << "Simulation step took " << duration << " ms" << endl;
    #endif
}

/// @brief Get the list of objects in contact with a given object.
/// @param target_object id of the object
/// @return list of objects in contact with the target object
unordered_set<string> Scene::get_contacted_objects(string target_object)
{

    if(contacts_manager.needs_update(target_object)){
        this->step_simulation(1/1000.0f);
    }

    unordered_set<string> contacted_objects = contacts_manager.get_contacted_object_ids(target_object);

    #ifndef NDEBUG
    cout << "Found " << contacted_objects.size() << " contacted objects with " << target_object << endl;
    #endif

    return contacted_objects;
}

/// @brief Get the contact points between two objects that are not penetrating.
/// @param id1 id of the first object
/// @param id2 id of the second object
/// @return Nx3 matrix representing the contact points between the two objects
/// @note If id2 is set to an empty string, returns all contact points involving id1.
MatrixX3f Scene::get_contact_points_positions(string id1, string id2)
{
    //If id2 is set to an empty string, we return all contact points involving id1
    bool return_all_contact_points = (id2 == "");
    vector<Vector3f> contact_points;
    if(return_all_contact_points){
        if(contacts_manager.needs_update(id1)){
            this->step_simulation(1/1000.0f);
        }
        contact_points = contacts_manager.get_contact_positions(id1);
    }else{
        if(contacts_manager.needs_update(id1) || contacts_manager.needs_update(id2)){
            this->step_simulation(1/1000.0f);
        }
        contact_points = contacts_manager.get_contact_positions(id1, id2);
    }

    //Convert the vector of contact points to a matrix
    MatrixX3f contact_points_matrix(contact_points.size(), 3);
    for (int i = 0; i < contact_points.size(); i++){
        //The contact point is a column vector, but we want to store it as a row vector.
        contact_points_matrix.row(i) = contact_points[i].transpose();
    }
    return contact_points_matrix;
}

/// @brief Get the contact points between two objects that are penetrating.
/// @param id1 id of the first object
/// @param id2 id of the second object
/// @return Nx3 matrix representing the contact points between the two objects
/// @note If id2 is set to an empty string, returns all contact points involving id1.
MatrixX3f Scene::get_penetrating_contact_point_positions(string id1, string id2)
{
   //If id2 is set to an empty string, we return all contact points involving id1
    bool return_all_contact_points = (id2 == "");
    vector<Vector3f> contact_points;
    if(return_all_contact_points){
        if(contacts_manager.needs_update(id1)){
            this->step_simulation(1/1000.0f);
        }
        contact_points = contacts_manager.get_contact_positions(id1, true);
    }else{
        if(contacts_manager.needs_update(id1) || contacts_manager.needs_update(id2)){
            this->step_simulation(1/1000.0f);
        }
        contact_points = contacts_manager.get_contact_positions(id1, id2, true);
    }

    //Convert the vector of contact points to a matrix
    MatrixX3f contact_points_matrix(contact_points.size(), 3);
    for (int i = 0; i < contact_points.size(); i++){
        //The contact point is a column vector, but we want to store it as a row vector.
        contact_points_matrix.row(i) = contact_points[i].transpose();
    }
    return contact_points_matrix;
}

/// @brief Get all contact points involving the object with the given id.
/// @param id Id of the object for which to get the contact points.
/// @return Nx3 matrix representing the contact points with the scene.
MatrixX3f Scene::get_all_contact_points(string id)
{
    return this->get_contact_points_positions(id, "");
}

/// @brief Get all penetrating contact points involving the object with the given id.
/// @param id Id of the object for which to get the contact points.
/// @return Nx3 matrix representing the contact points with the scene.
MatrixX3f Scene::get_all_penetrating_contact_points(string id)
{
    return this->get_penetrating_contact_point_positions(id, "");
}

/// @brief Return the vertices of the convex hull over the contact points between two objects, or between an object and the scene.
/// @param id1 ID of the (first) object for which to get the contact points.
/// @param id2 ID of the second object on which the contact points lie. If empty, use all contact points lying on the first object.
/// @param vertex_limit Maximum number of vertices of the convex hull (default: 255)
/// @return Nx3 matrix representing the subset of contact points lying on the convex hull.
/// @note A small perturbation is added to each contact point to make sure that the convex hull
///         is full dimensional. Equivalent to QHull's "QJ" option.
MatrixX3f Scene::get_contact_convex_hull(string id1, string id2, int vertex_limit)
{
    //With ePLANE_SHIFTING, the minimum vertex_limit is 4
    if(vertex_limit < 4){
        vertex_limit = 4;
    }

    MatrixX3f obj_contact_points = this->get_contact_points_positions(id1, id2);

    int num_contact_points = obj_contact_points.rows();

    //If there are no contact points, return an empty matrix
    if(num_contact_points == 0){
        return MatrixX3f(0,3);
    }

    //Computing a convex hull requires at least 4 points
    // A plane can always be defined by 3 points, a line with 2.
    if(num_contact_points <= 4){
        return obj_contact_points;
    }

    PxTolerancesScale scale;
    PxCookingParams params(scale);
    PxConvexMeshDesc convexDesc;

    params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX | 
                        PxConvexFlag::ePLANE_SHIFTING | 
                        PxConvexFlag::eDISABLE_MESH_VALIDATION | 
                        PxConvexFlag::eFAST_INERTIA_COMPUTATION;

    //Amplitude of the perturbation added to each point
    float perturbation_amplitude = 0.001;

    //Create a convex mesh from the object's contact points
    PxVec3* vertices = new PxVec3[num_contact_points];
    for(int i=0; i < num_contact_points; i++){
        //Add a random perturbation to the point such that the point cloud is full dimensional
        Vector3f perturbation = perturbation_amplitude*Vector3f::Random();
        vertices[i] = PxVec3(obj_contact_points(i,0) + perturbation[0], obj_contact_points(i,1) + perturbation[1], obj_contact_points(i,2) + perturbation[2]);
    }

    convexDesc.points.count = num_contact_points;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = vertices;
    convexDesc.vertexLimit = vertex_limit;

    PxConvexMesh* convexMesh = PxCreateConvexMesh(params, convexDesc, gPhysics->getPhysicsInsertionCallback());

    //Get the vertices of the convex hull
    PxU32 numVerts = convexMesh->getNbVertices();
    PxU32 numPoly = convexMesh->getNbPolygons();
    const PxVec3* convexVerts = convexMesh->getVertices();
    const PxU8* idxBuffer = convexMesh->getIndexBuffer();
        
    //Convert the vertices to a matrix
    MatrixX3f convex_hull(numVerts, 3);
    for(int i=0; i < numVerts; i++){
        convex_hull(i,0) = convexVerts[i].x;
        convex_hull(i,1) = convexVerts[i].y;
        convex_hull(i,2) = convexVerts[i].z;
    }

    //Free the vertices
    delete[] vertices;

    return convex_hull;
}

/// @brief Get the contact forces between objects in the scene.
/// @return List of contact forces.
std::vector<ContactForce> Scene::get_contact_forces()
{
    ForceSolver force_solver = ForceSolver(this);
    return force_solver.get_contact_forces();
}

/// @brief Find the contact point between two objects that is the closest to the given point.
/// @param id1 Id of the first object in contact.
/// @param id2 Id of the second object in contact.
/// @param point 3D query point expressed in the world frame.
/// @return 3D coordinates of the closest contact point expressed in the world frame.
/// @note This iterates over contact points and could be improved by using a spatial data structure.
Vector3f Scene::get_closest_contact_point(string id1, string id2, const Vector3f point)
{

    //Get the contact points between both objects
    MatrixX3f contact_points = this->get_contact_points_positions(id1, id2);

    //If there are no contact points, return NAN
    if(contact_points.rows() == 0){
        return Vector3f(NAN, NAN, NAN);
    }

    //Find the closest contact point
    float min_dist = numeric_limits<float>::infinity();
    Vector3f closest_contact_point;
    for(int i=0; i < contact_points.rows(); i++){
        Vector3f contact_point = contact_points.row(i);
        float dist = (contact_point - point).squaredNorm();
        if(dist < min_dist){
            min_dist = dist;
            closest_contact_point = contact_point;
            if(min_dist == 0){
                break;
            }
        }
    }

    return closest_contact_point;
}


/// @brief Find the ID of the object that is in contact at the given point with the object whose ID is given.
/// @param point 3D query point expressed in the world frame.
/// @param max_distance Distance between the query point and the contact point below which the contact point is considered to be the closest.
/// @param max_num_iterations Maximum number of iterations to perform, increasing the max_distance by a factor of 10 at each iteration.
/// @return Object ID of the object in contact at the given point or empty string if no object could be found.
/// @note The function is mainly useful after performing a convex hull computation that looses object ID information.
string Scene::get_contact_id_at_point(string id, Vector3f point, float max_distance, int max_num_iterations)
{
    //Get the objects in contact with the given object
    unordered_set<string> contacted_objects = this->get_contacted_objects(id);

    //If there is no object in contact, return an empty string
    if(contacted_objects.size() == 0){

        #ifndef NDEBUG
        cout << "No object in contact with " << id << endl;
        #endif

        return "";
    }

    //If there is a single object in contact, return its ID
    if(contacted_objects.size() == 1){
        return *contacted_objects.begin();
    }

    //Iterate over object's OccupancyGrid and check if the grid has a voxel which contains the point
    vector<string> candidate_objects;
    for(auto& obj_id : contacted_objects){
        Object* obj = this->get_object_by_id(obj_id);
        if(obj == nullptr){
            #ifdef NDEBUG
            cout << "Object #" << obj_id << " not found (nullptr)." << endl;
            #endif
            continue;
        }

        if(obj->occupancy_grid == nullptr){
            #ifdef NDEBUG
            cout << "Object #" << obj_id << " has no occupancy grid." << endl;
            #endif
            continue;
        }

        //TODO: Verify that this works well.
        if(obj->occupancy_grid->is_cell_occupied(point)){
            candidate_objects.push_back(obj_id);
        }
    }

    #ifndef NDEBUG
    cout << "Found " << candidate_objects.size() << " objects in contact with " << id << " that contain the query point" << endl;
    #endif

    //If there is a single object in contact that contains the query point, return its ID
    if(candidate_objects.size() == 1){
        return candidate_objects[0];
    }

    //If no occupied voxel contains the query point
    // Compute the distance between the query point and the voxel centers
    // and return the ID of the object with the closest voxel center.
    if(candidate_objects.size() == 0){
        float min_dist = numeric_limits<float>::infinity();
        string closest_object_id = "";
        //Iterate over all objects in contact
        for(auto& obj_id : contacted_objects){
            Object* obj = this->get_object_by_id(obj_id);
            if(obj == nullptr){
                #ifdef NDEBUG
                cout << "Object #" << obj_id << " not found (nullptr)." << endl;
                #endif
                continue;
            }

            if(obj->occupancy_grid == nullptr){
                #ifdef NDEBUG
                cout << "Object #" << obj_id << " has no occupancy grid." << endl;
                #endif
                continue;
            }

            //Get the distance between the query point and the voxel centers
            MatrixX3f voxel_centres = obj->get_voxel_centres();

            //Compute the squared norm between the query point and each voxel center
            MatrixX3f query_point_matrix = point.transpose().replicate(voxel_centres.rows(), 1);
            MatrixX3f diff = voxel_centres - query_point_matrix;
            VectorXf dist = diff.rowwise().squaredNorm();

            //Find the voxel with the smallest distance to the query point
            float min_dist = dist.minCoeff();

            //If the distance is smaller than the current minimum, update the minimum
            if(min_dist < min_dist){
                min_dist = min_dist;
                closest_object_id = obj_id;
            }
        }
        return closest_object_id;
    }else{
        //Iterate over the candidate objects and find the one that has the query point
        for(int iter=0; iter < max_num_iterations; iter++){
            for(auto& obj_id : candidate_objects){
                //Get the contact points between both objects
                Eigen::MatrixX3f contacts = this->get_contact_points_positions(id, obj_id);

                #ifndef NDEBUG
                cout << "Found " << contacts.size() << " contact points between " << id << " and " << obj_id << endl;
                #endif

                //Iterate over the contact points and find the one that is equal to the query point
                for(int i=0; i < contacts.rows(); i++){
                    Vector3f contact_point = contacts.row(i);
                    //Compute the squared distance between the two points
                    float dist = (contact_point - point).squaredNorm();
                    //If distantce is small enough, return the object ID
                    if(dist < max_distance){
                        //Return the object ID
                        return obj_id;
                    }
                }
            }

            //None of the objects in contact has a contact point equal to the query point
            //Restart with a larger distance
            max_distance *= 10;
            #ifndef NDEBUG
                cout << "No contact point found. Increasing max distance to " << max_distance << endl;
            #endif
        }
    }
    return "";
}

/// @brief Get the normal at the surface point on the object with the given ID, that is closest to the query point.
/// @param id Unique ID of the object.
/// @param query_point 3D query point expressed in the world frame.
/// @return Normal unit-length vector expressed in the world frame, and pointing towards the outside of the object.
Eigen::Vector3f Scene::get_normal_at_point(string id, Eigen::Vector3f query_point)
{
    Object* obj = this->get_object_by_id(id);
    uint32_t grid_idx = obj->occupancy_grid->idx_cell_at(query_point);
    if( obj->occupancy_grid->is_cell_occupied(grid_idx) != 0){
        //Get grid cells
        GridCell gc = obj->occupancy_grid->get_grid_cell(grid_idx);
        //Get the normal at the point
        OrientedPoint op = gc.weighted_average(query_point);
        return op.normal;
    }else{
        return Vector3f(NAN, NAN, NAN);
    }
}

/// @brief Find the three contact points that create the best stable support for an object.
/// @param id Id of the object for which to get the contact points.
/// @param hull_max_size Maximum number of points on the convex hull (default: 255)
/// @return Vector of pairs of object IDs and contact points positions.
/// @note This is computationally very expensive with worst case O(hull_max_size^3)
vector<pair<string, Vector3f>> Scene::get_three_most_stable_contact_points(string id, int hull_max_size, bool random_third_point)
{
    vector<pair<string, Vector3f>> contact_points_with_ids;

    #ifndef NDEBUG
    auto t1 = std::chrono::high_resolution_clock::now();
    #endif

    //The maximum number of points on the convex hull must be between [4-255]
    if(hull_max_size < 4){
        hull_max_size = 4;
    }else if(hull_max_size > 255){
        hull_max_size = 255;
    }

    MatrixX3f obj_contact_points;

    if(random_third_point){
        #ifndef NDEBUG
        cout << "Using two points on the cvx hull and a random third point." << endl;
        #endif

        obj_contact_points = this->get_all_contact_points(id);
    }

    //The contact points are expressed in the world frame.
    MatrixX3f hull_contact_points = get_contact_convex_hull(id, "", hull_max_size);

    //The number of contact points on the convex hull must be at least 3
    if(hull_contact_points.rows() < 3){
        contact_points_with_ids.push_back(make_pair("", Vector3f(NAN, NAN, NAN)));
        contact_points_with_ids.push_back(make_pair("", Vector3f(NAN, NAN, NAN)));
        contact_points_with_ids.push_back(make_pair("", Vector3f(NAN, NAN, NAN)));

        cout << "Object #" << id << " has less than 3 contact points. Returning NAN." << endl;

        return contact_points_with_ids;
    }

    //Create a Triangle for all combinations
    vector<Triangle<Vector3f>> triangles;
    for(int i=0; i < hull_contact_points.rows(); i++){
        Vector3f p1 = hull_contact_points.row(i);
        if(p1.hasNaN()){
            continue;
        }

        //Consider only points for which a normal force at the contact point,
        // when directed towards the inside of the object, would oppose gravity.
        Vector3f p1_normal = this->get_normal_at_point(id, p1);
        if(p1_normal.hasNaN() || p1_normal.dot(Vector3f(0,0,-1)) < 0){
            continue;
        }

        for(int j=i+1; j < hull_contact_points.rows(); j++){
            Vector3f p2 = hull_contact_points.row(j);
            if(p2.hasNaN()){
                continue;
            }

            //Consider only points for which a normal force at the contact point,
            // when directed towards the inside of the object, would oppose gravity.
            Vector3f p2_normal = this->get_normal_at_point(id, p2);
            if(p2_normal.hasNaN() || p2_normal.dot(Vector3f(0,0,-1)) < 0){
                continue;
            }

            if(random_third_point){
                //Get a random point on the object
                int random_index = rand() % obj_contact_points.rows();
                Vector3f p3 = obj_contact_points.row(random_index);
                if(p3.hasNaN()){
                    continue;
                }
                //If the triangle is degenerate, skip it
                if((p2-p1).cross(p3-p1).norm() < 1e-6){
                    continue;
                }
                Triangle<Vector3f> triangle(p1, p2, p3);
                triangles.push_back(triangle);
            }else{
                //Produce all possible triangles
                for(int k=j+1; k < hull_contact_points.rows(); k++){
                    Vector3f p3 = hull_contact_points.row(k);
                    if(p3.hasNaN()){
                        continue;
                    }
                    //If the triangle is degenerate, skip it
                    if((p2-p1).cross(p3-p1).norm() < 1e-6){
                        continue;
                    }

                    //Consider only points for which a normal force at the contact point,
                    // when directed towards the inside of the object, would oppose gravity.
                    Vector3f p3_normal = this->get_normal_at_point(id, p3);
                    if(p3_normal.hasNaN() || p3_normal.dot(Vector3f(0,0,-1)) < 0){
                        continue;
                    }

                    /*
                    cout << "Normals: " << endl;
                    cout << p1_normal.transpose() << endl;
                    cout << p2_normal.transpose() << endl;
                    cout << p3_normal.transpose() << endl;
                    */

                    //A valid triangle is determined
                    Triangle<Vector3f> triangle(p1, p2, p3);
                    triangles.push_back(triangle);
                }
            }
        }
    }

    #ifndef NDEBUG
    cout << "Determined " << triangles.size() << " triangles" << endl;
    cout << "but only considering the 100 largest ones." << endl;
    #endif

    Matrix3f contact_points = get_best_contact_triangle(id, triangles, true, 100);

    #ifndef NDEBUG
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    cout << "Got three most stable contact points in " << duration << " ms" << endl;
    cout << "Contact points: " << endl << contact_points << endl;
    #endif

    //Find the object ID of the other object in contact at each contact point
    // and make a vector of pairs of object IDs and contact points.

    //If any element of the position is NAN, set the id to an empty string
    Vector3f contact_point0 = contact_points.row(0);
    Vector3f contact_point1 = contact_points.row(1);
    Vector3f contact_point2 = contact_points.row(2);

    string id0 = "";
    string id1 = "";
    string id2 = "";

    if(!contact_point0.hasNaN()){
        id0 = get_contact_id_at_point(id, contact_points.row(0));
    }

    if(!contact_point1.hasNaN()){
        id1 = get_contact_id_at_point(id, contact_points.row(1));
    }

    if(!contact_point2.hasNaN()){
        id2 = get_contact_id_at_point(id, contact_points.row(2));
    }

    contact_points_with_ids.push_back(make_pair(id0, contact_points.row(0)));
    contact_points_with_ids.push_back(make_pair(id1, contact_points.row(1)));
    contact_points_with_ids.push_back(make_pair(id2, contact_points.row(2)));

    #ifndef NDEBUG
    cout << "Contact points with IDs: " << endl;
    for(auto& contact_point_with_id : contact_points_with_ids){
        cout << contact_point_with_id.first << ": " << contact_point_with_id.second.transpose() << endl;
    }
    #endif

    return contact_points_with_ids;
}

/// @brief Amongst the given triangles, find the one that creates the best stable support for an object.
/// @param id Id of the object for which to get the contact points.
/// @param triangles List of triangles to consider, with each triangle represented by a tuple of three 3D points.
/// @param stable If true, the triangle must be stable under gravity. Otherwise, the largest triangle is returned.
/// @param max_tri_to_consider Consider only the N largest triangles. Set to -1 to consider all triangles (default).
/// @return 3x3 matrix representing the three contact points that create the best stable support,
///         with each row representing a contact point.
Matrix3f Scene::get_best_contact_triangle(string id, vector<Triangle<Vector3f>> triangles, bool stable, int max_tri_to_consider)
{
    //Tolerance for rejecting triangles if their normal is (almost) orthogonal to the gravity vector
    float tol = 1e-2;

    //Sort the triangles by area
    sort(triangles.begin(), triangles.end(), [](const Triangle<Vector3f>& t1, const Triangle<Vector3f>& t2){
        return t1.signed_area > t2.signed_area;
    });

    //Contact points for the best stable support
    Matrix3f contact_points;
    contact_points.row(0) = Vector3f(NAN, NAN, NAN);
    contact_points.row(1) = Vector3f(NAN, NAN, NAN);
    contact_points.row(2) = Vector3f(NAN, NAN, NAN);

    if(stable){
        Object* obj = this->get_object_by_id(id);
        //The centre of mass is expressed in the object frame
        Vector3f com_o = obj->com;
        //Express the CoM in the world frame
        Matrix4f pose_w = obj->pose;
        Vector3f com_w = pose_w.block<3,3>(0,0)*com_o + pose_w.block<3,1>(0,3);
        //The gravity direction vector is expressed in the world frame
        Vector3f g_w = Vector3f(0,0,-1);

        //Distance between the projected CoM and the closest triangle side
        // for the currently best stable support. The best stable support
        // has the largest min_dist.
        float min_dist = 0;

        //Iterate over the triangles
        for(int i=0; i < triangles.size(); i++){

            //If we have already considered enough triangles, stop
            if(max_tri_to_consider > 0 && i >= max_tri_to_consider){
                break;
            }

            Triangle<Vector3f> tri = triangles[i];
            Vector3f p1 = tri.vertex_0;
            Vector3f p2 = tri.vertex_1;
            Vector3f p3 = tri.vertex_2;

            #ifndef NDEBUG
            cout << "Considering triangle " << i << " with vertices: " << endl;
            cout << p1.transpose() << endl;
            cout << p2.transpose() << endl;
            cout << p3.transpose() << endl;
            #endif

            //Compute the normal of the triangle
            Vector3f tri_normal = (p2 - p1).cross(p3 - p1);
            tri_normal.normalize();

            //Distance between the origin and the plane of the triangle
            float tri_plane_distance = p1.dot(tri_normal);

            //If the gravity vector is orthogonal to the triangle normal
            // there is no possible stable support with the selected triangle.
            if(abs(tri_normal.dot(g_w)) < tol){
                #ifndef NDEBUG
                cout << "Triangle normal is orthogonal to gravity vector. Skipping triangle." << endl;
                #endif
                continue;
            }

            Vector3f centroid = (p1 + p2 + p3) / 3;
            //Compute the distance between the centroid and each triangle side
            Vector3f rej_13 = (centroid - p1) - (centroid - p1).dot(p3 - p1) * ((p3 - p1) / (p3 - p1).norm());
            float d1 = rej_13.norm();
            Vector3f rej_12 = (centroid - p1) - (centroid - p1).dot(p2 - p1) * ((p2 - p1) / (p2 - p1).norm());
            float d2 = rej_12.norm();
            Vector3f rej_23 = (centroid - p2) - (centroid - p2).dot(p3 - p2) * ((p3 - p2) / (p3 - p2).norm());
            float d3 = rej_23.norm();

            //Smallest distance
            float min_d = min(min(d1, d2), d3);
            
            //Since the centroid is the point furthest away from the triangle sides,
            // if the distance between the centroid and the closest triangle side
            // is smaller than the current min_dist, this triangle cannot be the best.
            if(min_d < min_dist){
                #ifndef NDEBUG
                cout << "Centroid is too close to sides. Skipping triangle." << endl;
                #endif
                continue;
            }

            //Define the direction along which the CoM must be projected such that it
            // hits the triangle (the sign of g_w).
            Vector3f centroid_wrt_com = centroid - com_w;
            float sign = 1;
            if(centroid_wrt_com.dot(g_w) < 0){
                sign = -1;
            }

            //Compute the intersection point between the plane of the triangle and 
            // the gravity vector starting at the CoM
            Vector3f p12 = p2 - p1;
            Vector3f p13 = p3 - p1;
            Vector3f p12_cp_p13 = p12.cross(p13);
            float num = p12_cp_p13.dot(com_w - p1);
            float den = -g_w.dot(p12_cp_p13);
            float t = num / den;
            Vector3f intersection_point = com_w + t*sign*g_w;

            //Check if the triangle contains the intersection point
            bool tri_contains_point = tri.contains(intersection_point, false);

            //If the CoM projects into the triangle, this triangle is stable.
            // But is it the best stable support?
            if(tri_contains_point){

                //Compute the distance between the projected CoM and each triangle side
                Vector3f p = intersection_point;

                Vector3f u_13 = (p3 - p1) / (p3 - p1).norm();
                Vector3f rej_13 = (p - p1) - (p - p1).dot(u_13) * u_13;
                float d1 = rej_13.norm();

                Vector3f u_12 = (p2 - p1) / (p2 - p1).norm();
                Vector3f rej_12 = (p - p1) - (p - p1).dot(u_12) * u_12;
                float d2 = rej_12.norm();

                Vector3f u_23 = (p3 - p2) / (p3 - p2).norm();
                Vector3f rej_23 = (p - p2) - (p - p2).dot(u_23) * u_23;
                float d3 = rej_23.norm();

                //Distance with the closest triangle side
                float min_d = min(min(d1, d2), d3);

                //Torque exerted by gravity about the triangle sides
                float tau_1 = (com_w - p1).cross(g_w).dot(((p3 - p1) / (p3 - p1).norm()));
                float tau_2 = (com_w - p1).cross(g_w).dot(((p2 - p1) / (p2 - p1).norm()));
                float tau_3 = (com_w - p2).cross(g_w).dot(((p3 - p2) / (p3 - p2).norm()));

                //Compute the smallest absolute torque
                float min_tau = min(min(abs(tau_1), abs(tau_2)), abs(tau_3));

                if(min_tau > min_dist){
                    min_dist = min_tau;
                    contact_points.row(0) = p1;
                    contact_points.row(1) = p2;
                    contact_points.row(2) = p3;
                }else{
                    #ifndef NDEBUG
                    cout << "Torque is too small. Skipping triangle." << endl;
                    #endif
                }
                continue;

                //If the distance between the projected CoM and the closest
                // triangle side is larger than the current min_dist, this
                // triangle is the best stable support.
                if(min_d > min_dist){
                    min_dist = min_d;
                    contact_points.row(0) = p1;
                    contact_points.row(1) = p2;
                    contact_points.row(2) = p3;
                }
            }else{
                #ifndef NDEBUG
                cout << "Triangle does not contain intersection point. Skipping triangle." << endl;
                cout << "Intersection point: " << intersection_point.transpose() << endl;
                #endif
            }
        }

        if(min_dist == 0){
            #ifndef NDEBUG
            cout << "No stable support found for object #" << id << endl;
            #endif
        }

    }else{
        //If stability is not required, return the largest triangle
        contact_points.row(0) = triangles[0].vertex_0;
        contact_points.row(1) = triangles[0].vertex_1;
        contact_points.row(2) = triangles[0].vertex_2;
    }
    return contact_points;
}

/// @brief Compare each pair of contact points and merge them if they are close enough.
/// @param position_threshold Maximal distance between two contact points to be merged
/// @param normal_threshold Maximum angle cosine between the normals of two contact points to be merged
/// @note This is slow, use it only when necessary.
void Scene::merge_similar_contact_points(vector<Contact> contact_points, float position_threshold = 0, float normal_threshold = 0.1)
{
    //If the position threshold is not specified, use an adaptive threshold based on the
    // size of the voxels in contact.
    bool compute_adaptive_threshold = (position_threshold == 0);

    //Merge points that are less than some distance apart if their normal is mostly aligned
    for (int i = 0; i < contact_points.size(); i++)
    {
        Contact contact1 = contact_points[i];

        if(compute_adaptive_threshold){
            pair<Object*, Object*> objects = contact1.get_objects();
            Object* object1 = objects.first;
            Object* object2 = objects.second;
            Vector3f o1_sides = object1->get_voxel_side_lengths();
            Vector3f o2_sides = object2->get_voxel_side_lengths();
            float max_side = max(o1_sides.maxCoeff(), o2_sides.maxCoeff());
            float position_threshold = 0.1*max_side;
        }

        for (int j = i + 1; j < contact_points.size(); j++)
        {
            Contact contact2 = contact_points[j];

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
                contact_points.erase(contact_points.begin() + j);
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

    if(!convexMeshDesc.isValid()){
        cout << "Invalid convex mesh description" << endl;
        return nullptr;
    }

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
/// @param cell Occupancy cell for which to create the shape.
/// @param obj_world_pose Pose of the object in the world frame.
/// @return Shape representing the cube.
PxShape* Scene::create_voxel_shape(GridCell* cell, Matrix4f obj_world_pose)
{
    PxBoxGeometry boxGeometry = PxBoxGeometry(cell->half_extents[0], cell->half_extents[1], cell->half_extents[2]);
    PxShape* shape = gPhysics->createShape(boxGeometry, *gMaterial, true);
    if(!shape)
        throw runtime_error("Error creating shape");

    shape->userData = cell;
    //Gridcells, and therefore voxels, are defined with respect to the world frame.
    // When actor->setGlobalPose() is called, the shapes are transformed accordingly.
    // However, this transformation misalignes the voxels, which must stay Axis-Aligned.
    // Therefore, we set the box pose in the object frame, and expect PhysX to transform
    // it to the world frame later.
    PxTransform objPose = PxTransform(PxMat44(
        PxVec3(obj_world_pose(0,0), obj_world_pose(1,0), obj_world_pose(2,0)),
        PxVec3(obj_world_pose(0,1), obj_world_pose(1,1), obj_world_pose(2,1)),
        PxVec3(obj_world_pose(0,2), obj_world_pose(1,2), obj_world_pose(2,2)),
        PxVec3(obj_world_pose(0,3), obj_world_pose(1,3), obj_world_pose(2,3))));
    PxTransform boxPose_w = PxTransform(PxVec3(cell->centre[0], cell->centre[1], cell->centre[2]));
    PxTransform boxPose_o = objPose.transformInv(boxPose_w);
    shape->setLocalPose(boxPose_o);
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


/// @brief Produces a set of spheres positioned slightly below the surface of the object.
///         These spheres can be used to detect interpenetration as they are embedded in the volume of the object.
/// @param obj Object for which to generate the spheres (must be volumetric).
/// @param tetConvexShapes Array of tetrahedra from make_tetmesh().
/// @return An array of shapes, each representing a sphere.
PxArray<PxShape*> Scene::make_canary_spheres(Object* obj, PxArray<PxShape*> tetConvexShapes)
{
    //Surface voxels information
    MatrixX3f voxel_centres = obj->get_voxel_centres();
    Vector3f voxel_side_lengths = obj->get_voxel_side_lengths();

    //Array of sphere, each with the same radius
    PxArray<PxShape*> spheres;
    vector<Vector3f> sphere_positions;
    PxReal sphere_radius = voxel_side_lengths.minCoeff() / 100;

    if(!obj->has_canary_spheres()){

        #ifndef NDEBUG
        auto t1 = chrono::high_resolution_clock::now();
        #endif

        //Generate a set of candidate canary positions
        // To do so, the OrientedPoint at the (distance weighted) average position of the surface points
        // associated with each voxel is used. The canary position is set to be slightly below the surface
        // by assuming that the normal of the OrientedPoint is pointing outwards.
        std::unordered_map<uint32_t, GridCell>* grid = obj->occupancy_grid->get_grid_cells();
        Matrix3Xf canary_positions(3, grid->size());
        int i = 0;
        for (auto& it : *grid) {
            GridCell cell = it.second;
            OrientedPoint op = cell.weighted_average(cell.centre);
            Vector3f canary_position = op.position - 100 * sphere_radius * op.normal;
            canary_positions.col(i) = canary_position;
            i++;
        }

        //The voxels are defined in the world frame but the tetrahedra are defined in the object frame
        // and the position of the spheres will be set in the object frame. Therefore, we need to express
        // the canary positions in the object frame.
        Matrix4f pose_inv = obj->pose.inverse();
        Matrix3f T_R = pose_inv.block(0,0,3,3);
        Vector3f T_t = pose_inv.block(0,3,3,1);
        canary_positions = (T_R*canary_positions).colwise() + T_t;

        #ifndef NDEBUG
        auto t2 = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Generated voxel vertices in " << duration << " ms" << endl;
        #endif

        #ifndef NDEBUG
        t1 = chrono::high_resolution_clock::now();
        #endif

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
        VectorXd inside_volume = VectorXd::Zero(canary_positions.cols());

        //In debug, we only use one thread to make debugging easier
        #ifndef NDEBUG
        const int num_threads = 1;
        cout << "Using only one thread for debugging" << endl;
        #else
        //Maximum number of threads to use (can be zero), at most the number of tetrahedra
        const int num_threads = min(tetConvexShapes.size(), std::thread::hardware_concurrency());
        #endif

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
                thread_output[thread_i] = VectorXd::Zero(canary_positions.cols());
                //Launch the thread
                threads[thread_i] = std::thread(&Scene::points_in_tetrahedron, this, 
                    canary_positions, tetra_vertices_list, first_tet_index, last_tet_index, 
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
            points_in_tetrahedron(canary_positions, tetra_vertices_list, 0, tetra_vertices_list.size(), inside_volume);
        }

        #ifndef NDEBUG
        t2 = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Iterated over " << tetConvexShapes.size() << " tetrahedra in " << duration << " ms" << endl;
        #endif

        //For each vertex flagged as inside the volume, create a sphere
        for(int i = 0; i < inside_volume.size(); i++){
            if(inside_volume[i] > 0){
                //Create a sphere at the voxel vertex
                PxSphereGeometry sphereGeometry = PxSphereGeometry(sphere_radius);
                PxShape* sphere = gPhysics->createShape(sphereGeometry, *gMaterial, true);
                if(!sphere)
                    throw runtime_error("Error creating shape");
                //Set the position of the sphere. The voxel vertices are in the object frame here.
                Vector3f pos = canary_positions.col(i);
                sphere_positions.push_back(pos);
                sphere->setLocalPose(PxTransform(PxVec3(pos[0], pos[1], pos[2])));
                sphere->setRestOffset(0);
                sphere->setContactOffset(1e-6);
                //Add the sphere to the list
                spheres.pushBack(sphere);
            }
        }
        //Record the sphere positions in the object
        MatrixX3f canary_sphere_positions(sphere_positions.size(), 3);
        for(int i=0; i < sphere_positions.size(); i++){
            canary_sphere_positions.row(i) = sphere_positions[i];
        }
        obj->canary_sphere_positions = canary_sphere_positions;
    }else{
        //The object already has canary sphere positions, so we just need to create the spheres
        for(int i = 0; i < obj->canary_sphere_positions.rows(); i++){
            //Create a sphere with the predefined radius
            PxSphereGeometry sphereGeometry = PxSphereGeometry(sphere_radius);
            PxShape* sphere = gPhysics->createShape(sphereGeometry, *gMaterial, true);
            if(!sphere)
                throw runtime_error("Error creating shape");
            //Set the position of the sphere.
            Vector3f pos = obj->canary_sphere_positions.row(i);
            sphere_positions.push_back(pos);
            sphere->setLocalPose(PxTransform(PxVec3(pos[0], pos[1], pos[2])));
            sphere->setRestOffset(0);
            sphere->setContactOffset(1e-6);
            //Add the sphere to the list
            spheres.pushBack(sphere);
        }
    }

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
    float tol = 0;

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

    if(!obj->has_tetra_mesh()){
        shared_ptr<PxSimpleTriangleMesh> triSurfaceMesh_ptr = obj->tri_mesh;
        PxSimpleTriangleMesh triSurfaceMesh = *triSurfaceMesh_ptr;

        //Create a tetrahedral mesh from the triangle mesh
        bool result = obj->create_tetra_mesh(triSurfaceMesh, tetMeshVertices, tetMeshIndices);

        if(!result)
            throw runtime_error("Error creating tetrahedral mesh");

        //Record the tetrahedral mesh in the object
        obj->set_tetra_mesh(tetMeshVertices, tetMeshIndices);
    }else{
        //Get the tetrahedral mesh from the object
        // and make PxArrays from the Eigen matrices
        PxArray<PxVec3> tetMeshVertices(obj->tetra_vertices.rows());
        PxArray<PxU32> tetMeshIndices(4*obj->tetra_indices.rows());
        for(int i=0; i < obj->tetra_vertices.rows(); i++){
            tetMeshVertices[i] = PxVec3(obj->tetra_vertices(i,0), obj->tetra_vertices(i,1), obj->tetra_vertices(i,2));
        }
        //Set the indices. We have 4 indices per tetrahedron.
        //  obj->tetra_indices is a MatrixX4i but we need a PxArray<PxU32>
        for(int i=0; i < obj->tetra_indices.rows(); i++){
            tetMeshIndices[4*i]   = obj->tetra_indices(i,0);
            tetMeshIndices[4*i+1] = obj->tetra_indices(i,1);
            tetMeshIndices[4*i+2] = obj->tetra_indices(i,2);
            tetMeshIndices[4*i+3] = obj->tetra_indices(i,3);
        }
    }

    //For each tetrahedron, create a convex mesh.
    PxArray<PxConvexMeshDesc> convexMeshDescs;
    bool result = obj->create_tetra_convex_set(tetMeshVertices, tetMeshIndices, convexMeshDescs);

    if(!result)
        throw runtime_error("Error creating convex mesh");

    PxArray<PxShape*> convexShapes;
    //For each convex mesh, create a shape
    for(int i = 0; i < convexMeshDescs.size(); i++){
        PxConvexMeshDesc convexMeshDesc = convexMeshDescs[i];
        //Cooking parameters
        PxTolerancesScale scale;
        PxCookingParams params(scale);
        PxShape* convexShape = createTetrahedronShape(params, convexMeshDesc);

        if(convexShape != nullptr){    
            //Contacts are genereated when an object is at the contact offset from this shape
            // which must be positive and slightly larger than the rest offset that defines
            // the depth of the equilibrium position of the object.
            convexShape->setRestOffset(0);
            convexShape->setContactOffset(1e-6);

            convexShapes.pushBack(convexShape);
        }else{
            cout << "Null convex shape in Scene.cpp:make_tetmesh()" << endl;
        }

        //Release the convex mesh description
        PxVec3* points = (PxVec3*)convexMeshDesc.points.data;
        PxHullPolygon* polygons = (PxHullPolygon*)convexMeshDesc.polygons.data;
        PxU32* indices = (PxU32*)convexMeshDesc.indices.data;

        //Free the memory
        delete[] points;
        delete[] polygons;
        delete[] indices;
    }
    return convexShapes;
}

void Scene::create_object_shapes(Object* obj, Matrix4f pose, int resolution, bool is_volumetric, bool is_fixed, float mass, Vector3f com)
{
    //Create a occupancy grid
    #ifndef NDEBUG
    auto t1 = chrono::high_resolution_clock::now();
    #endif
    shared_ptr<OccupancyGrid> grid = obj->create_occupancy_grid(resolution, OccupancyGrid::sampling_method::random);
    #ifndef NDEBUG
    auto t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Occupancy grid created in " << duration << " ms" << endl;
    #endif

    PxArray<PxShape*> convexShapes;

    #ifndef NDEBUG
    t1 = chrono::high_resolution_clock::now();
    #endif

    //Create a shape from each voxel in the occupancy grid.
    unordered_map<uint32_t, GridCell>* cells = grid->get_grid_cells();
    //Create a shape for each occupancy grid cell
    for(auto& item : *cells){
        uint32_t index = item.first;
        GridCell* cell  = &item.second;
        if(cell != nullptr){
            PxShape* voxelShape = create_voxel_shape(cell, pose);
            convexShapes.pushBack(voxelShape);
        }else{
            cout << "Null cell in Scene.cpp:create_object_shapes()" << endl;
        }
    }

    #ifndef NDEBUG
    t2 = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
    cout << "Created " << convexShapes.size() << " voxel shapes in " << duration << " ms" << endl;
    #endif

    //If the object is volumetric, create a tetrahedral mesh and spheres embedded in the volume
    // to enable interpenetration detection.
    if(is_volumetric){
        #ifndef NDEBUG
        t1 = chrono::high_resolution_clock::now();
        #endif
        
        //Create the tetrahedral mesh built from the triangle mesh.
        PxArray<PxShape*> tetConvexShapes = make_tetmesh(obj);
        //Augment convexShapes with the shapes
        for(auto& shape : tetConvexShapes){
            convexShapes.pushBack(shape);
        }

        #ifndef NDEBUG
        t2 = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Created tetrahedral shapes in " << duration << " ms" << endl;
        #endif

        #ifndef NDEBUG
        t1 = chrono::high_resolution_clock::now();
        #endif

        //Create spheres embedded in the volume of the object that can be used to detect interpenetration.
        PxArray<PxShape*> sphereShapes = make_canary_spheres(obj, tetConvexShapes);
        //Augment convexShapes with the shapes
        for(auto& shape : sphereShapes){
            convexShapes.pushBack(shape);
        }

        #ifndef NDEBUG
        t2 = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Created canary spheres in " << duration << " ms" << endl;
        #endif
    }

    //Pose of the actor/object in the world frame column by column [col0 | col1 | col2 | col3]
    PxTransform pxPose = PxTransform(PxMat44(
        PxVec3(pose(0,0), pose(1,0), pose(2,0)),
        PxVec3(pose(0,1), pose(1,1), pose(2,1)),
        PxVec3(pose(0,2), pose(1,2), pose(2,2)),
        PxVec3(pose(0,3), pose(1,3), pose(2,3))));

    //If the object is fixed, there is no dynamics involved
    //TODO: Currently disabled the creation of static objects as they
    // do not seem to generate contacts / can be woken up.
    if(false && is_fixed){
        PxRigidStatic* actor = gPhysics->createRigidStatic(pxPose);
        //Attach all shapes in the object to the actor
        for(int i = 0; i < convexShapes.size(); i++){
            if(convexShapes[i] != nullptr){
                actor->attachShape(*convexShapes[i]);
                //We release the shape after it has been attached such that when the actor
                // is released, the shape is also released.
                convexShapes[i]->release();
            }
        }
        gScene->addActor(*actor);
        actor->setGlobalPose(pxPose);
        actor->setName(obj->id.c_str());
        actor->userData = obj;
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
            if(convexShapes[i] != nullptr){
                actor->attachShape(*convexShapes[i]);
                //We release the shape after it has been attached such that when the actor
                // is released, the shape is also released.
                convexShapes[i]->release();
            }
        }
        PxVec3 pxCom = PxVec3(com(0), com(1), com(2));
        //PxRigidBodyExt::setMassAndUpdateInertia(*actor, mass, &pxCom, true);
        actor->setMass(mass);
        actor->setCMassLocalPose(PxTransform(pxCom));
        gScene->addActor(*actor);
        actor->setGlobalPose(pxPose);
        actor->setKinematicTarget(pxPose);
        //Since our actors are kinematic, this is not enough to disable sleeping.
        // We need to do setKinematicTarget at every simulation step.
        // See: https://github.com/NVIDIA-Omniverse/PhysX/blob/b7182d5e563b763a340053864658202a3ee966ab/physx/source/simulationcontroller/src/ScBodySim.cpp#L608C12-L608C12
        actor->setSleepThreshold(0.0f);
        actor->setName(obj->id.c_str());
        actor->userData = obj;
    }
}

/// @brief Add a volumetric object to the scene defined by its triangular surface mesh, its tetrahedral volume mesh, and its canary spheres.
///         If the tetrahedral mesh and the canary spheres matrices are empty, they will be generated.
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object expressed in the local/object frame
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param tetra_vertices (Optional) Nx3 matrix representing the vertices of the tetrahedra of the object expressed in the local/object frame
/// @param tetra_indices (Optional) Mx4 matrix representing the indices of the vertices of the tetrahedra of the object
/// @param canary_sphere_positions (Optional) Nx3 matrix representing the positions of the canary spheres of the object expressed in the local/object frame
/// @param resolution resolution of the occupancy grid (default: 15)
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object WRT the object frame (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
void Scene::add_volumetric_object(string id, Matrix4f pose, 
            MatrixX3f tri_vertices, MatrixX3i tri_indices,
            MatrixX3f tetra_vertices,MatrixX4i tetra_indices,
            MatrixX3f canary_sphere_positions,
            int resolution, bool is_fixed, float mass, Vector3f com, string material_name)
{
    assert(tri_vertices.rows() > 0);
    assert(tri_indices.rows() > 0);
    assert(resolution > 0);
    assert(mass > 0);

    //Verify that the object does not exists already. If it does, remove it first.
    Object* existing_obj = get_object_by_id(id);
    if(existing_obj != nullptr){
        cout << "Object with id " << id << " already exists. Removing it first." << endl;
        remove_object(id);
    }

    shared_ptr<Object> obj = make_shared<Object>(this, id, pose, tri_vertices, tri_indices, true, is_fixed, mass, com, material_name);
    
    if (obj->is_valid_pose_matrix(pose)){
        object_ptrs.push_back(obj);

        //Record the triangle mesh in the object
        obj->set_tri_mesh(tri_vertices, tri_indices);
        //Record the tetrahedral mesh in the object
        obj->set_tetra_mesh(tetra_vertices, tetra_indices);
        //Record the canary sphere positions in the object
        obj->canary_sphere_positions = canary_sphere_positions;

        //Create the shapes (occupancy grid, tetrahedral mesh, canary spheres) and attach them to an actor for the object
        create_object_shapes(obj.get(), pose, resolution, true, is_fixed, mass, com);

        //The contacts are no longer valid, so clear them
        contacts_manager.invalidate_object_state(id);

        #ifndef NDEBUG
        std::cout << "Added volumetric object with id " << id << endl;
        std::cout << "at pose " << endl;
        std::cout << pose << endl;
        std::cout << "with " << tri_vertices.rows() << " vertices and " << tri_indices.rows() << " triangles" << endl;
        std::cout << "with " << tetra_vertices.rows() << " tetrahedra and " << tetra_indices.rows() << " indices" << endl;
        std::cout << "with " << canary_sphere_positions.rows() << " canary spheres" << endl;
        #endif
    }else{
        std::cout << "Invalid pose matrix when adding object with id " << id << endl;
    }
}

/// @brief Add an object to the scene
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param tri_vertices Nx3 matrix representing the vertices of the object expressed in the local/object frame
/// @param tri_indices Mx3 matrix representing the triangles of the object
/// @param resolution resolution of the occupancy grid (default: 15)
/// @param compute_volume boolean Expressing if the volume of the object should be computed, which requires a watertight surface mesh (default: false)
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object WRT the object frame (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
void Scene::add_object(string id, Matrix4f pose, MatrixX3f tri_vertices, MatrixX3i tri_indices, int resolution, bool compute_volume, bool is_fixed, float mass, Vector3f com, string material_name)
{
    assert(tri_vertices.rows() > 0);
    assert(tri_indices.rows() > 0);
    assert(resolution > 0);
    assert(mass > 0);

    //Verify that the object does not exists already. If it does, remove it first.
    Object* existing_obj = get_object_by_id(id);
    if(existing_obj != nullptr){
        std::cout << "Object with id " << id << " already exists. Removing it first." << endl;
        remove_object(id);
    }

    //Create an object instance and add it to the scene
    // When adding an element to the vector, the vector may reallocate memory and move the elements which will change their addresses.
    // However, we need to supply the callback with a persistent pointer to the object.
    // Therefore, we store pointers to preallocated objects in the vector instead of the objects themselves.
    shared_ptr<Object> obj = make_shared<Object>(this, id, pose, tri_vertices, tri_indices, compute_volume, is_fixed, mass, com, material_name);
    
    if (obj->is_valid_pose_matrix(pose)){
        object_ptrs.push_back(obj);

        //Record the triangle mesh in the object
        obj->set_tri_mesh(tri_vertices, tri_indices);

        //Create the shapes (ocupancy grid, possibly tetrahedral mesh and canary spheres) and attach them to an actor for the object
        create_object_shapes(obj.get(), pose, resolution, compute_volume, is_fixed, mass, com);

        //The contacts are no longer valid, so clear them
        contacts_manager.invalidate_object_state(id);
    }else{
        std::cout << "Invalid pose matrix when adding object with id " << id << endl;
    }

}

/// @brief Get the actor with a specified name from the scene
/// @param name name of the actor / object
/// @return pointer to the actor or NULL if not found
PxRigidActor* Scene::get_actor(string name)
{
    //Get the number of actors in the scene that are either dynamic or static
    PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

    //Get all rigid actors in the scene
    PxActor* actors[nbActors];
    PxU32 nbActorsReturned = gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, actors, nbActors);

    PxRigidActor* actor = nullptr;
    //Iterate over actors and find the one with the specified name
    for (int i = 0; i < nbActorsReturned; i++)
    {
        PxRigidActor* candidate_actor = static_cast<PxRigidActor*>(actors[i]);
        if (candidate_actor->getName() == name)
        {
            actor = candidate_actor;
        }
    }

    return actor;
}

/// @brief Get the shapes attached to an actor
/// @param actor pointer to the actor
/// @return vector of pointers to the shapes attached to the actor
vector<PxShape*> Scene::get_actor_shapes(PxRigidActor* actor)
{
    vector<PxShape*> shapes;
    if (actor == NULL)
    {
        #ifndef NDEBUG
        cout << "Scene::get_actor_shapes : Actor is NULL" << endl;
        #endif
        return shapes;
    }
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

    //Release the shapes array
    delete[] shapesArray;

    return shapes;
}


/// @brief Remove all actors from the scene, this should also remove all shapes attached to the actors.
/// @note See: https://github.com/NVIDIAGameWorks/PhysX/issues/541#issuecomment-1102264977 for information about actor remove/release.
void Scene::remove_all_actors()
{
    if (gScene != NULL){
        //Get the number of actors in the scene that are either dynamic or static
        PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

        //Get all rigid actors in the scene
        PxActor* actors[nbActors];
        PxU32 nbActorsReturned = gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, actors, nbActors);

        //Iterate over actors and find the one with the specified name
        for (int i = 0; i < nbActorsReturned; i++)
        {
            PxRigidActor* actor = static_cast<PxRigidActor*>(actors[i]);
            //Remove the actor from the scene
            gScene->removeActor(*actor);
            //Release the memory allocated for the actor
            PX_RELEASE(actor);
        }

        //The contacts are no longer valid, so clear them
        contacts_manager.remove_all_objects();
    }
}

/// @brief Remove an object from the scene along with its attached shapes
/// @param id unique identifier for the object
void Scene::remove_object(string id)
{
    //Get the actor with the specified name
    PxRigidActor* actor = get_actor(id);
    if (actor == nullptr)
    {
        cout << "Actor with id " << id << " not found" << endl;
        return;
    }else{
        #ifndef NDEBUG
        cout << "Found actor " << id << endl;
        #endif
    

        //When the actor is released, the shapes should also be released
        // if the release() method was called right after the shape was created.
        // See: https://github.com/NVIDIAGameWorks/PhysX/issues/541#issuecomment-1102264977

        //Remove the actor from the scene
        // This seems to also release the attached shapes.
        gScene->removeActor(*actor);
        
        //Release the memory allocated for the actor
        // WARNING: This seems to cause memory related issues inside simulate()
        //  when PhysX tries to access the actor after it has been released.
        //  Possibly related to https://github.com/NVIDIA-Omniverse/PhysX/issues/211
        // Hmm, it seems it might not have been the cause, we are still getting same segfault.
        //PX_RELEASE(actor);

        #ifndef NDEBUG
        cout << "Removed actor from scene." << endl;
        #endif
    }

    //Remove the object from the list of objects
    for (int i = 0; i < this->object_ptrs.size(); i++){
        if (this->object_ptrs[i]->id == id){
            this->object_ptrs.erase(this->object_ptrs.begin() + i);
            #ifndef NDEBUG
            cout << "Removed object " << id << " from list of objects." << endl;
            #endif
            break;
        }
    }

    //The contacts are no longer valid, so clear them
    contacts_manager.remove_object(id);
}

/// @brief Set the pose of an object by recreating a new object.
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @note This is a relatively expensive operation.
void Scene::set_object_pose(string id, Matrix4f pose)
{

    //Every time the object is moved, the OccupancyGrid has to be recreated
    // such that the voxels are axis-aligned (our geometric intersection
    // algorithm relies on that to be fast). However, other shapes attached
    // to the object (tetrahedra, spheres) do not have to be recreated when
    // relying on shape->setLocalPose() and actor->setGlobalPose().

    //The OccupancyGrid is created in the world frame. Therefore, the voxels
    // are also in the world frame. We set the local pose of the voxel shapes
    // such that, when setGlobalPose() is called, the voxels are correctly positioned. 
    //   See: create_voxel_shape()

    #ifndef NDEBUG
    auto t1 = chrono::high_resolution_clock::now();
    #endif

    //Get object by ID, actor and shapes
    Object* obj = get_object_by_id(id);

    //New Method
    if(obj != nullptr){
        PxRigidActor* actor = get_actor(id);
        PxActorType::Enum actor_type = actor->getType();
        vector<PxShape*> attached_shapes = get_actor_shapes(actor);

        //Detach all box shapes from the actor
        for (int i = 0; i < attached_shapes.size(); i++)
        {
            PxShape* shape = attached_shapes[i];
            if(shape != nullptr){
                PxGeometryType::Enum shape_type = shape->getGeometry().getType();

                if(shape_type == PxGeometryType::eBOX){
                    actor->detachShape(*shape);
                }
            }
        }

        //Change the pose of the actor such that the tetrahedra and spheres
        // are in the correct position.
        PxTransform pxPose = PxTransform(PxMat44(
            PxVec3(pose(0,0), pose(1,0), pose(2,0)),
            PxVec3(pose(0,1), pose(1,1), pose(2,1)),
            PxVec3(pose(0,2), pose(1,2), pose(2,2)),
            PxVec3(pose(0,3), pose(1,3), pose(2,3))
        ));

        actor->setGlobalPose(pxPose);
        //Static actors do not have a kinematic target
        if(actor_type == PxActorType::Enum::eRIGID_DYNAMIC){
            //Cast to dynamic actor
            static_cast<PxRigidDynamic*>(actor)->setKinematicTarget(pxPose);
        }

        //Create a new occupancy grid
        obj->reset_pose(pose);
        shared_ptr<OccupancyGrid> new_grid = obj->occupancy_grid;

        //Create a shape from each voxel in the occupancy grid.
        unordered_map<uint32_t, GridCell>* new_cells = new_grid->get_grid_cells();
        for(auto& item : *new_cells){
            GridCell* cell  = &item.second;
            PxShape* voxelShape = create_voxel_shape(cell, pose);
            actor->attachShape(*voxelShape);
            voxelShape->release();
        }

        #ifndef NDEBUG
        auto t2 = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        cout << "Moved object " << id << " in " << duration << " ms to pose " << endl << pose << endl;
        #endif

    }else{
        #ifndef NDEBUG
        cout << "Try creating the object first." << endl;
        #endif
    }

    //The contacts are no longer valid, so clear them
    contacts_manager.invalidate_object_state(id);

}

/// @brief Return a list of object IDs in the scene.
/// @return List of object IDs.
vector<string> Scene::get_all_object_ids()
{
    vector<string> ids;
    for(const auto& obj : this->object_ptrs)
        ids.push_back(obj->id);
    return ids;
}

/// @brief Return the object that has the given ID
/// @param id unique identifier for the object
/// @return pointer to the object or nullptr if not found
Object* Scene::get_object_by_id(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj.get();
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

/// @brief Get the vertices of the triangle mesh of an object expressed in the local/object frame
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Scene::get_tri_vertices(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
        return obj->tri_vertices;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the positions of the canary spheres embedded in the volume of an object for interpenetration detection, expressed in the object frame.
/// @param id Unique identifier for the object.
/// @return Nx3 matrix with each row representing the 3D position of a sphere.
MatrixX3f Scene::get_canary_sphere_positions(string id)
{
    Object* obj = get_object_by_id(id);
    if(obj != nullptr)
        return obj->canary_sphere_positions;
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

/// @brief Get the vertices of the tetrahedral mesh of an object expressed in the local/object frame
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

/// @brief Get the voxel centres of the occupied voxels of an object expressed in the world frame
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

void Scene::set_friction_coefficient(string mat_name1, string mat_name2, float friction_coefficient)
{
    this->friction_coefficients[make_pair(mat_name1, mat_name2)] = friction_coefficient;
    this->friction_coefficients[make_pair(mat_name2, mat_name1)] = friction_coefficient;
}

float Scene::get_friction_coefficient(string mat_name1, string mat_name2)
{
    if(this->friction_coefficients.find(make_pair(mat_name1, mat_name2)) == this->friction_coefficients.end()){
        cout << "Friction coefficient not found for " << mat_name1 << " and " << mat_name2 << endl;
        return 0.5;
    }

    return this->friction_coefficients[make_pair(mat_name1, mat_name2)];
}