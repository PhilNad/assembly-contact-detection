#include "Scene.h"
#include <iostream>
#include <memory>
#include <set>
#include <chrono>
#include "PxPhysicsAPI.h"
#include "extensions/PxTetMakerExt.h"

using namespace physx;
using namespace std;
using namespace Eigen;

vector<Contact> gContacts;
vector<PxVec3> gContactPositions;
vector<pair<string, string>> gContactedObjects;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation;
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
class ContactReportCallbackForTetrahedra: public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count)					{ PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) 
	{
        //These are the global variables that will be used to store the contact data.
        // They are defined in lib/src/Scene.cpp.
        //extern vector<Contact> gContacts;
        //extern vector<PxVec3> gContactPositions;
        //extern vector<pair<string, string>> gContactedObjects;

        cout << "Contact detected" << endl;

		PX_UNUSED((pairHeader));
		std::vector<PxContactPairPoint> contactPoints;
		
		for(PxU32 i=0;i<nbPairs;i++)
		{
            cout << "Contact pair " << i << endl;
            PxContactPair pair = pairs[i];
            //Get the shapes involved in the collision
            PxShape* shape0 = pair.shapes[0];
            PxShape* shape1 = pair.shapes[1];
            
            //Get the actors
            PxRigidActor* actor0 = shape0->getActor();
            PxRigidActor* actor1 = shape1->getActor();
            string id_obj0 = actor0->getName();
            string id_obj1 = actor1->getName();
            //Get the objects
            Object* obj0 = static_cast<Object*>(actor0->userData);
            Object* obj1 = static_cast<Object*>(actor1->userData);

            //Get the contact points
			PxU32 contactCount = pair.contactCount;
            //cout << contactCount << " contacts between " << id_obj0 << " and " << id_obj1 << endl;
			if(contactCount)
			{

                //Add the pair of objects to the list of contacted objects.
                gContactedObjects.push_back(make_pair(id_obj0, id_obj1));

				contactPoints.resize(contactCount);
				pair.extractContacts(&contactPoints[0], contactCount);

				for(PxU32 j=0;j<contactCount;j++)
				{
                    cout << "Contact point " << j << endl;
                    //Contact point data
                    PxVec3 pos = contactPoints[j].position;
                    PxReal sep = contactPoints[j].separation;
                    PxVec3 normal = contactPoints[j].normal;
                    //Create a contact instance and add it to the list of contact points
                    if(abs(sep) < min(obj0->max_separation, obj1->max_separation)){
                        Contact contact(obj0, obj1, pos, normal, sep);
                        gContacts.push_back(contact);
                        //Add the contact position to the list of contact positions
                        gContactPositions.push_back(contactPoints[j].position);
                    }
				}
			}
		}
	}
};

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
		std::vector<PxContactPairPoint> contactPoints;
		
		for(PxU32 i=0;i<nbPairs;i++)
		{
            //cout << "Contact pair " << i << endl;
            PxContactPair pair = pairs[i];
            //Get the shapes involved in the collision
            PxShape* shape0 = pair.shapes[0];
            PxShape* shape1 = pair.shapes[1];
            
            //Get the actors
            PxRigidActor* actor0 = shape0->getActor();
            PxRigidActor* actor1 = shape1->getActor();
            string id_obj0 = actor0->getName();
            string id_obj1 = actor1->getName();

            //Get the objects
            Object* obj0 = static_cast<Object*>(actor0->userData);
            Object* obj1 = static_cast<Object*>(actor1->userData);

            //Get the grid cells involved in the collision
            GridCell* gridCell0 = static_cast<GridCell*>(shape0->userData);
            GridCell* gridCell1 = static_cast<GridCell*>(shape1->userData);

            //Surface points can be obtained with
            //  gridCell0->surface_points[0].get()->position
            //  gridCell0->surface_points[1].get()->position
            //  ...
            //  gridCell0->surface_points[0].get()->normal
            //  ...

            //Compute the position threshold based on the size of the voxels in contact
            // two contact points are merged if they are closer than this threshold
            Vector3f o1_sides = obj0->get_voxel_side_lengths();
            Vector3f o2_sides = obj1->get_voxel_side_lengths();
            float max_side = max(o1_sides.maxCoeff(), o2_sides.maxCoeff());
            float position_threshold = max(position_threshold, 0.1f*max_side);

            //Get the contact points
			PxU32 contactCount = pair.contactCount;
            //cout << contactCount << " contacts between " << id_obj0 << " and " << id_obj1 << endl;
			if(contactCount)
			{

                //Add the pair of objects to the list of contacted objects.
                gContactedObjects.push_back(make_pair(id_obj0, id_obj1));

				contactPoints.resize(contactCount);
				pair.extractContacts(&contactPoints[0], contactCount);

				for(PxU32 j=0;j<contactCount;j++)
				{
                    //If at least one contact point has been added, we set contact_added to true
                    bool contact_added = false;
                    //cout << "Contact point " << j << endl;
                    //Contact point data
                    PxVec3 cp_pos = contactPoints[j].position;
                    PxReal cp_sep = contactPoints[j].separation;

                    //Create a contact instance and add it to the list of contact points
                    if(abs(cp_sep) < min(obj0->max_separation, obj1->max_separation)){
                        Vector3f query_point = Vector3f(contactPoints[j].position.x, contactPoints[j].position.y, contactPoints[j].position.z);
                        //Get the weighted average of the surface points of the two grid cells
                        OrientedPoint op0 = gridCell0->weighted_average(query_point);
                        OrientedPoint op1 = gridCell1->weighted_average(query_point);
                        
                        //Compute the average of the two oriented points by first aligning the normal in the same half-space
                        Vector3f pos = (op0.position + op1.position) / 2;
                        if(op0.normal.dot(op1.normal) < 0){
                            op1.normal = -op1.normal;
                        }
                        Vector3f normal = (op0.normal + op1.normal) / 2;
                        normal.normalize();

                        //Distance between the two oriented points along the normal
                        float pos_dist = (op0.position - op1.position).norm();
                        float normal_dist = 1-op0.normal.dot(op1.normal);

                        bool normals_aligned = (normal_dist < 0.25);
                        bool close_enough = true; (pos_dist < gridCell0->half_extents.maxCoeff() + gridCell1->half_extents.maxCoeff());
                        //If the normals are aligned, then the contact is probably surface-to-surface
                        //If the normals are not aligned, then the contact is probably surface-to-edge 
                        if(close_enough && normals_aligned){
                            // cout << "Object 1 Point Position: (" << op0.position[0] << ", " << op0.position[1] << ", " << op0.position[2] << ")" << endl;
                            // cout << "Object 2 Point Position: (" << op1.position[0] << ", " << op1.position[1] << ", " << op1.position[2] << ")" << endl;
                            // cout << "Object 1 Point Normal: (" << op0.normal[0] << ", " << op0.normal[1] << ", " << op0.normal[2] << ")" << endl;
                            // cout << "Object 2 Point Normal: (" << op1.normal[0] << ", " << op1.normal[1] << ", " << op1.normal[2] << ")" << endl;
                            // cout << "pos_dist: " << pos_dist << " and normal_dist: " << normal_dist << endl;
                            // cout << "-----------------------------------" << endl;
                            //Compute the distance between this contact and the previous one from this pair of objects
                            if(j > 0 && contact_added){
                                Contact previous_contact = gContacts.back();
                                Vector3f prev_pos = previous_contact.get_position();
                                float prev_pos_dist = (prev_pos - pos).norm();
                                //If the distance is too small, we consider that its the same point and we don't add it
                                if(prev_pos_dist < position_threshold){
                                    continue;
                                }
                            }
                            //Add a new contact point to the list
                            Contact contact(obj0, obj1, pos, normal, pos_dist);
                            gContacts.push_back(contact);
                            contact_added = true;
                            //cout << "Contact added" << endl;
                        }else{
                            //cout << "Contact not added, pos_dist: " << pos_dist << " and normal_dist: " << normal_dist << endl;
                        }
                    }else{
                        //cout << "Contact not added, separation too large." << endl;
                    }
				}
			}
		}
	}

    struct LineSegmentIntersection {
        Vector2f intersection_point_1 = Vector2f(NAN, NAN);
        Vector2f intersection_point_2 = Vector2f(NAN, NAN);
        bool intersects = false;
    };

    /// @brief Computes the intersection of two line segments.
    /// @param p1 Start point (2D) of line segment 1.
    /// @param q1 End point (2D) of line segment 1.
    /// @param p2 Start point (2D) of line segment 2.
    /// @param q2 End point (2D) of line segment 2.
    /// @return LineSegmentIntersection struct containing the intersection points and a boolean indicating if the line segments intersect.
    LineSegmentIntersection line_segment_intersection(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2)
    {
        //Each line can be expressed as a parametric equation
        // r = p + t*(q-p)
        // where (p, q) are two points on the line and t is a scalar.
        // The intersection point is the point for which the two parametric equations are equal:
        //  p1 + s*(q1-p1) = p2 + t*(q2-p2)
        // which can be rewritten as
        //  t*(q2-p2) - s*(q1-p1) = p1 - p2
        // and simplified with
        //  c = p1 - p2
        //  d1 = q1 - p1
        //  d2 = q2 - p2
        // producing 
        //  t*d2 - s*d1 = c
        // whose the solution obtained via Cramer's rule
        //  s = (d1[0] * c[1] - d1[1] * c[0]) / det
        //  t = (d2[0] * c[1] - d2[1] * c[0]) / det
        // with det=(a1*b2 - a2*b1) being the determinant of the system
        // that will be zero if the lines are parallel.
        // Relevant: https://stackoverflow.com/a/565282
        Vector2f d1 = q1 - p1;
        Vector2f d2 = q2 - p2;
        Vector2f c  = p1 - p2;
        //Numerators
        float num_s = (d1[0] * c[1] - d1[1] * c[0]);
        float num_t = (d2[0] * c[1] - d2[1] * c[0]);
        //Denominator / determinant
        float det = (-d2[0] * d1[1] + d1[0] * d2[1]);

        struct LineSegmentIntersection result;

        //If both the numerator and denominator are zero,
        // then the lines are collinear.
        if( abs(det)   < 1e-6 && 
            abs(num_s) < 1e-6 && 
            abs(num_t) < 1e-6){
            //We compute the overlap between the two line segments (possibly zero)
            float s1 = d1.dot(p2 - p1) / d1.dot(d1);
            float s2 = s1 + d1.dot(d2) / d1.dot(d1);
            float s_min = min(s1, s2);
            float s_max = max(s1, s2);
            //Line segments do not overlap
            if(s_max < 0 || s_min > 1){
                //Lines are collinear but disjoint
                return result;
            }
            //Line segments overlap from s_min to s_max
            struct LineSegmentIntersection result;
            result.intersects = true;
            result.intersection_point_1 = p1 + s_min*d1;
            result.intersection_point_2 = p1 + s_max*d1;
            return result;
        }

        //If the denominator is zero but the numerators are not,
        // then the lines are parallel and non-intersecting.
        if(abs(det)   < 1e-6 && 
          (abs(num_s) > 1e-6 || abs(num_t) > 1e-6)){
            //Lines are parallel
            return result;
        }

        //Otherwise, there is a unique solution given by
        float s = num_s / det;
        float t = num_t / det;

        //If the intersection happens when a parameter is between 0 and 1,
        // then it means that the intersection happens within the line segment.
        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            //Return the intersection point
            result.intersects = true;
            result.intersection_point_1 = p1 + (s * d1);
        }

        //The intersection happens outside of the line segment
        return result;
    }

    void triangle_overlap_over_AARectangle(GridCell& g1, GridCell& g2){
        //Triagnles associated with the two gridcells
        // Note: This should be a list
        Triangle t1 = *g1.triangle;
        Triangle t2 = *g2.triangle;
        //Axis aligned intersection rectangle between the two contacting gridcells
        AARectangle r1 = g1.gridcell_to_gridcell_intersection(g2);
        //Project the triangles onto the rectangle.
        // Since the triangles are projected onto an Axis-Aligned rectangle,
        // at least one of the world coordinates of the triangle vertices will be zero.
        // such that we can now work in 2D.
        Vector2f t1_v0_proj = r1.project_point(t1.vertex_0);
        Vector2f t1_v1_proj = r1.project_point(t1.vertex_1);
        Vector2f t1_v2_proj = r1.project_point(t1.vertex_2);
        Vector2f t2_v0_proj = r1.project_point(t2.vertex_0);
        Vector2f t2_v1_proj = r1.project_point(t2.vertex_1);
        Vector2f t2_v2_proj = r1.project_point(t2.vertex_2);
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
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

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
}

/// @brief Clean up the physics engine by releasing memory.
void Scene::cleanupPhysics()
{
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    PX_RELEASE(gFoundation);
}

Scene::Scene(){
    startupPhysics();
}
Scene::~Scene(){
    cleanupPhysics();
}

/// @brief Step the simulation by a given time step.
/// @param dt time step
void Scene::step_simulation(float dt)
{
    assert(dt > 0);

    //Clear the list of contacted objects and contact points
    gContacts.clear();
    gContactedObjects.clear(); 
    gContactPositions.clear();

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
    // as it always complained about
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

/// @brief Add an object to the scene
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param resolution resolution of the occupancy grid (default: 15)
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
void Scene::add_object(string id, Matrix4f pose, MatrixX3f vertices, MatrixX3i triangles, int resolution, bool is_fixed, float mass, Vector3f com, string material_name)
{
    //Create an object instance and add it to the scene
    // When adding an element to the vector, the vector may reallocate memory and move the elements which will change their addresses.
    // However, we need to supply the callback with a persistent pointer to the object.
    // Therefore, we store pointers to preallocated objects in the vector instead of the objects themselves.
    shared_ptr<Object> obj = make_shared<Object>(id, pose, vertices, triangles, is_fixed, mass, com, material_name);
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

    //Create a shape from each voxel in the occupancy grid.
    unordered_map<uint32_t, GridCell>* cells = grid->get_grid_cells();
    //Create a shape for each occupancy grid cell
    for(auto& item : *cells){
        uint32_t index = item.first;
        GridCell* cell  = &item.second;
        PxShape* voxelShape = createVoxelShape(cell);
        convexShapes.pushBack(voxelShape);
    }

    //Pose of the object/actor with respect to the world frame
    PxTransform pxPose = PxTransform(PxMat44(
        PxVec4(pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3)),
        PxVec4(pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3)),
        PxVec4(pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3)),
        PxVec4(pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3))
    ));
    
    //If the object is fixed, there is no dynamics involved
    if(is_fixed){
        PxRigidStatic* actor = gPhysics->createRigidStatic(pxPose);
        //Attach all shapes in the object to the actor
        for(int i = 0; i < convexShapes.size(); i++){
            actor->attachShape(*convexShapes[i]);
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
}

/// @brief Get the pose of an object
/// @param id unique identifier for the object
/// @return 4x4 matrix representing the pose of the object
Matrix4f Scene::get_object_pose(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->pose;
    cout << "Object with id " << id << " not found" << endl;
    return Matrix4f::Identity();
}

/// @brief Get the vertices of the triangle mesh of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Scene::get_tri_vertices(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->tri_vertices;
    cout << "Object with id " << id << " not found" << endl;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the triangles of the triangle mesh of an object
/// @param id unique identifier for the object
/// @return Mx3 matrix representing the triangles of the object
MatrixX3i Scene::get_tri_triangles(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->tri_triangles;
    cout << "Object with id " << id << " not found" << endl;
    return MatrixX3i::Zero(0, 3);
}

/// @brief Get the vertices of the tetrahedral mesh of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Scene::get_tetra_vertices(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->tetra_vertices;
    cout << "Object with id " << id << " not found" << endl;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the tetrahedra of the tetrahedral mesh of an object
/// @param id unique identifier for the object
/// @return Mx4 matrix representing the tetrahedra of the object
MatrixX4i Scene::get_tetra_indices(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->tetra_indices;
    cout << "Object with id " << id << " not found" << endl;
    return MatrixX4i::Zero(0, 4);
}

/// @brief Get the voxel centres of the occupied voxels of an object
/// @param id unique identifier for the object
/// @return Nx3 matrix representing the voxel centres of the object
MatrixX3f Scene::get_voxel_centres(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->get_voxel_centres();
    cout << "Object with id " << id << " not found" << endl;
    return MatrixX3f::Zero(0, 3);
}

/// @brief Get the voxel side lengths of the voxels of an object (assuming at least one voxel is occupied)
/// @param id unique identifier for the object
/// @return 3x1 vector representing the voxel side lengths of the object
Vector3f Scene::get_voxel_side_lengths(string id)
{
    for(const auto& obj : this->object_ptrs)
        if(obj->id == id)
            return obj->get_voxel_side_lengths();
    cout << "Object with id " << id << " not found" << endl;
    return Vector3f::Zero();
}