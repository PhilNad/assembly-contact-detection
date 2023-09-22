#include "Scene.h"
#include <iostream>
#include <memory>
#include "PxPhysicsAPI.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxSoftBodyExt.h"

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
class ContactReportCallback: public PxSimulationEventCallback
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
            PxContactPair pair = pairs[i];
            //Get the shapes involved in the collision
            PxShape* shape0 = pair.shapes[0];
            PxShape* shape1 = pair.shapes[1];
            
            //Get the objects
            Object* obj0 = static_cast<Object*>(shape0->userData);
            Object* obj1 = static_cast<Object*>(shape1->userData);
            
            //Get the actors
            PxRigidActor* actor0 = shape0->getActor();
            PxRigidActor* actor1 = shape1->getActor();
            string id_obj0 = actor0->getName();
            string id_obj1 = actor1->getName();

            //Get the contact points
			PxU32 contactCount = pair.contactCount;
            cout << contactCount << " contacts between " << id_obj0 << " and " << id_obj1 << endl;
			if(contactCount)
			{

                //Add the pair of objects to the list of contacted objects.
                gContactedObjects.push_back(make_pair(id_obj0, id_obj1));

				contactPoints.resize(contactCount);
				pair.extractContacts(&contactPoints[0], contactCount);

				for(PxU32 j=0;j<contactCount;j++)
				{
                    //Contact point data
                    PxVec3 pos = contactPoints[j].position;
                    PxReal sep = contactPoints[j].separation;
                    PxVec3 normal = contactPoints[j].normal;
                    //Create a contact instance and add it to the list of contact points
                    Contact contact(id_obj0, id_obj1, pos, normal, sep);
                    gContacts.push_back(contact);
                    //Add the contact position to the list of contact positions
					gContactPositions.push_back(contactPoints[j].position);
				}
			}
		}
	}
};

/// @brief Initialize the physics engine.
void Scene::startupPhysics()
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());

    gContactReportCallback = new ContactReportCallback();

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
    //sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;

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
vector<string> Scene::get_contacted_objects(string target_object)
{
    vector<string> contacted_objects;
    for (int i = 0; i < gContactedObjects.size(); i++)
    {
        if (gContactedObjects[i].first == target_object)
        {
            contacted_objects.push_back(gContactedObjects[i].second);
        }
        else if (gContactedObjects[i].second == target_object)
        {
            contacted_objects.push_back(gContactedObjects[i].first);
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

/// @brief  Create a set of convex PxShapes from a tetrahedron mesh.
/// @param tetMeshVertices Points of the tetrahedron mesh expressed in the reference frame of the actor/object.
/// @param tetMeshIndices For each tetrahedron, the indices of the four vertices.
/// @return Array of PxShapes that can be attached to an actor.
void Scene::shapesFromTetMesh(PxArray<PxVec3>* tetMeshVertices, PxArray<PxU32>* tetMeshIndices, PxArray<PxShape>* outShapes)
{
    // PxTolerancesScale scale;
    // PxCookingParams params(scale);
    
    // for(PxArray<PxU32>::Iterator it = tetMeshIndices.begin(); it != tetMeshIndices.end(); ++it)
    // {
    //     PxU32 indices;
    // }

    // PxDefaultMemoryOutputStream writeBuffer;
    // PxTriangleMeshCookingResult::Enum result;

    // bool status = PxCookConvexMesh(params, meshDesc, writeBuffer);
    // if(!status)
    //     throw runtime_error("Error cooking mesh");

    // PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
    // PxConvexMesh* trimesh = gPhysics->createConvexMesh(readBuffer);

    // //The shape has to be exclusive (hence the "true") such that it is guaranteed to be
    // // associated with only one actor. That way, the actor can be retrieved from the shape
    // // in the contact report callback.
    // PxShape* shape = gPhysics->createShape(PxConvexMeshGeometry(trimesh), *gMaterial, true);
    // shapes.pushBack(*shape);
}

void createCube(PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices, const PxVec3& pos, PxReal scaling)
{
	triVerts.clear();
	triIndices.clear();
	triVerts.pushBack(scaling * PxVec3(0.5f, -0.5f, -0.5f) + pos);  //0
	triVerts.pushBack(scaling * PxVec3(0.5f, -0.5f, 0.5f) + pos);   //1
	triVerts.pushBack(scaling * PxVec3(-0.5f, -0.5f, 0.5f) + pos);  //2
	triVerts.pushBack(scaling * PxVec3(-0.5f, -0.5f, -0.5f) + pos); //3
	triVerts.pushBack(scaling * PxVec3(0.5f, 0.5f, -0.5f) + pos);   //4
	triVerts.pushBack(scaling * PxVec3(0.5f, 0.5f, 0.5f) + pos);    //5
	triVerts.pushBack(scaling * PxVec3(-0.5f, 0.5f, 0.5f) + pos);   //6
	triVerts.pushBack(scaling * PxVec3(-0.5f, 0.5f, -0.5f) + pos);  //7

	triIndices.pushBack(1); triIndices.pushBack(2); triIndices.pushBack(3);
	triIndices.pushBack(7); triIndices.pushBack(6); triIndices.pushBack(5);
	triIndices.pushBack(4); triIndices.pushBack(5); triIndices.pushBack(1);
	triIndices.pushBack(5); triIndices.pushBack(6); triIndices.pushBack(2);

	triIndices.pushBack(2); triIndices.pushBack(6); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(3); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(1); triIndices.pushBack(3);
	triIndices.pushBack(4); triIndices.pushBack(7); triIndices.pushBack(5);

	triIndices.pushBack(0); triIndices.pushBack(4); triIndices.pushBack(1);
	triIndices.pushBack(1); triIndices.pushBack(5); triIndices.pushBack(2);
	triIndices.pushBack(3); triIndices.pushBack(2); triIndices.pushBack(7);
	triIndices.pushBack(4); triIndices.pushBack(0); triIndices.pushBack(7);
}

/// @brief Check the triangle mesh for any common issues that the tetrahedron meshing algorithm cannot handle.
/// @param surfaceMesh Description of the triangle mesh.
/// @return True if the mesh is valid (even if small problems are detected), false otherwise.
bool validateMesh(PxSimpleTriangleMesh& surfaceMesh){
     /*
    NOTE: It seems that a proper triangle mesh requires that the cross product between
    the two edges defined by the triangle vertices always points in the same direction.
    If the triangle is [a,b,c] then (a-b)x(c-b) should be towards the inside of the mesh.
    */
    PxTriangleMeshAnalysisResults results = PxTetMaker::validateTriangleMesh(surfaceMesh);
    if(results){
        /*
        eVALID = 0,
        eZERO_VOLUME = (1 << 0),							//!< invalid:		Flat mesh without meaningful amount of volume - cannot be meshed since a tetmesh is volumetric	
        eOPEN_BOUNDARIES = (1 << 1),						//!< problematic:	Open boundary means that the mesh is not watertight and that there are holes. The mesher can fill holes but the surface might have an unexpected shape where the hole was.
        eSELF_INTERSECTIONS = (1 << 2),						//!< problematic:	The surface of the resulting mesh won't match exactly at locations of self-intersections. The tetmesh might be connected at self-intersections even if the input triangle mesh is not
        eINCONSISTENT_TRIANGLE_ORIENTATION = (1 << 3),		//!< invalid:		It is not possible to distinguish what is inside and outside of the mesh. If there are no self-intersections and not edges shared by more than two triangles, a call to makeTriOrientationConsistent can fix this. Without fixing it, the output from the tetmesher will be incorrect
        eCONTAINS_ACUTE_ANGLED_TRIANGLES = (1 << 4),		//!< problematic:	An ideal mesh for a softbody has triangles with similar angles and evenly distributed vertices. Acute angles can be handled but might lead to a poor quality tetmesh.
        eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES = (1 << 5),	//!< problematic:	Border case of a self-intersecting mesh. The tetmesh might not match the surace exactly near such edges.
        eCONTAINS_DUPLICATE_POINTS = (1 << 6),				//!< ok:			Duplicate points can be handled by the mesher without problems. The resulting tetmesh will only make use of first unique point that is found, duplicate points will get mapped to that unique point in the tetmesh. Therefore the tetmesh can contain points that are not accessed by a tet.
        eCONTAINS_INVALID_POINTS = (1 << 7),				//!< invalid:		Points contain NAN, infinity or similar values that will lead to an invalid mesh
        eREQUIRES_32BIT_INDEX_BUFFER = (1 << 8),			//!< invalid:		Mesh contains more indices than a 16bit index buffer can address

        eMESH_IS_PROBLEMATIC = (1 << 9),					//!< flag is set if the mesh is categorized as problematic
        eMESH_IS_INVALID = (1 << 10)						//!< flag is set if the mesh is categorized as invalid
        */
        if(results.isSet(PxTriangleMeshAnalysisResult::eZERO_VOLUME))
            cout << "INVALID Mesh: Zero volume" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eOPEN_BOUNDARIES))
            cout << "Open boundaries" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eSELF_INTERSECTIONS))
            cout << "Self intersections" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eINCONSISTENT_TRIANGLE_ORIENTATION))
            cout << "INVALID Mesh: Inconsistent triangle orientation. Use makeTriOrientationConsistent()." << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eCONTAINS_ACUTE_ANGLED_TRIANGLES))
            cout << "Acute angled triangles" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES))
            cout << "Edge shared by more than two triangles" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eCONTAINS_DUPLICATE_POINTS))
            cout << "Contains duplicate points" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eCONTAINS_INVALID_POINTS))
            cout << "INVALID Mesh: Contains invalid points" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eREQUIRES_32BIT_INDEX_BUFFER))
            cout << "INVALID Mesh: Requires 32 bit index buffer" << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC))
            cout << "Mesh is problematic, the result might not be optimal even with remeshing." << endl;
        if(results.isSet(PxTriangleMeshAnalysisResult::eMESH_IS_INVALID))
            throw runtime_error("Mesh is invalid, cannot process this mesh.");
        return false;
    }
    return true;
}

/// @brief From a tetrahedral mesh, create a set of convex meshes, each describing a tetrahedron.
/// @param tetVertices Vertices of the tetrahedral mesh.
/// @param tetIndices For each tetrahedron, the indices of the four vertices.
/// @param convexMeshDescs Output array of convex mesh descriptions.
void createThetrahedronSet(PxArray<PxVec3> tetVertices, PxArray<PxU32> tetIndices, PxArray<PxConvexMeshDesc>& convexMeshDescs){
    //Since a tetrahedron is convex, the average of its vertices will be located inside the tetrahedron.
    // This provides a way to know the inside/outside of each face of the tetrahedron by computing
    // the vector starting at the center/average and ending at the face center. This vector must
    // be pointing towards the outside of the tetrahedron such that:
    //      (v1 - v0).cross(v2 - v0) points towards the outside
    // and can be used to retrieve the normal of the face directly from the vertices.

    for(int j=0; j < tetIndices.size(); j+=4){
        //Vertices
        PxVec3 v0 = tetVertices[tetIndices[j+0]];
        PxVec3 v1 = tetVertices[tetIndices[j+1]];
        PxVec3 v2 = tetVertices[tetIndices[j+2]];
        PxVec3 v3 = tetVertices[tetIndices[j+3]];
        //This should make 48 consecutive bytes.
        PxVec3* vertices = new PxVec3[4]; 
        vertices[0] = v0; vertices[1] = v1; vertices[2] = v2; vertices[3] = v3;
        //Centre of the tetrahedron
        PxVec3 tetCentre = (v0 + v1 + v2 + v3)/4;
        //Indices of the vertices associated with each face (3 per face)
        PxU32* indexBuffer = new PxU32[12];
        PxU32* f1_idx = indexBuffer;
        PxU32* f2_idx = indexBuffer + 3;
        PxU32* f3_idx = indexBuffer + 6;
        PxU32* f4_idx = indexBuffer + 9;
        //with the following order, the normals should be pointing towards the outside
        f1_idx[0] = 0; f1_idx[1] = 2; f1_idx[2] = 1;
        f2_idx[0] = 3; f2_idx[1] = 0; f2_idx[2] = 1;
        f3_idx[0] = 3; f3_idx[1] = 1; f3_idx[2] = 2;
        f4_idx[0] = 3; f4_idx[1] = 2; f4_idx[2] = 0;

        //Each face is composed of three vertices
        PxVec3* face1 = new PxVec3[3]; face1[0] = v0; face1[1] = v1; face1[2] = v2;
        PxVec3* face2 = new PxVec3[3]; face2[0] = v0; face2[1] = v2; face2[2] = v3;
        PxVec3* face3 = new PxVec3[3]; face3[0] = v0; face3[1] = v3; face3[2] = v1;
        PxVec3* face4 = new PxVec3[3]; face4[0] = v1; face4[1] = v2; face4[2] = v3;
        //Compute the centre of each face
        PxVec3 face1_centre = (vertices[f1_idx[0]] + vertices[f1_idx[1]] + vertices[f1_idx[2]])/3;
        PxVec3 face2_centre = (vertices[f2_idx[0]] + vertices[f2_idx[1]] + vertices[f2_idx[2]])/3;
        PxVec3 face3_centre = (vertices[f3_idx[0]] + vertices[f3_idx[1]] + vertices[f3_idx[2]])/3;
        PxVec3 face4_centre = (vertices[f4_idx[0]] + vertices[f4_idx[1]] + vertices[f4_idx[2]])/3;
        //Compute the direction of each face, pointing towards the outside
        PxVec3 face1_direction = (face1_centre - tetCentre).getNormalized();
        PxVec3 face2_direction = (face2_centre - tetCentre).getNormalized();
        PxVec3 face3_direction = (face3_centre - tetCentre).getNormalized();
        PxVec3 face4_direction = (face4_centre - tetCentre).getNormalized();
        //Compute the normal of each face
        PxVec3 face1_normal = (vertices[f1_idx[1]] - vertices[f1_idx[0]]).cross(vertices[f1_idx[2]] - vertices[f1_idx[0]]).getNormalized();
        PxVec3 face2_normal = (vertices[f2_idx[1]] - vertices[f2_idx[0]]).cross(vertices[f2_idx[2]] - vertices[f2_idx[0]]).getNormalized();
        PxVec3 face3_normal = (vertices[f3_idx[1]] - vertices[f3_idx[0]]).cross(vertices[f3_idx[2]] - vertices[f3_idx[0]]).getNormalized();
        PxVec3 face4_normal = (vertices[f4_idx[1]] - vertices[f4_idx[0]]).cross(vertices[f4_idx[2]] - vertices[f4_idx[0]]).getNormalized();
        //Make sure that the normals are pointing towards the outside
        // and that 
        //      (v1 - v0).cross(v2 - v0) 
        // points towards the outside / gives the normal as the PhysX libraries 
        // assume that the face normal of a triangle with vertices [a,b,c] can be computed as:
		//      edge1 = b-a
		//      edge2 = c-a
		//      face_normal = edge1 x edge2.
        if(face1_normal.dot(face1_direction) < 0){
            face1_normal = -face1_normal;
            PxSwap(f1_idx[1], f1_idx[2]);
        }
        if(face2_normal.dot(face2_direction) < 0){
            face2_normal = -face2_normal;
            PxSwap(f2_idx[1], f2_idx[2]);
        }
        if(face3_normal.dot(face3_direction) < 0){
            face3_normal = -face3_normal;
            PxSwap(f3_idx[1], f3_idx[2]);
        }
        if(face4_normal.dot(face4_direction) < 0){
            face4_normal = -face4_normal;
            PxSwap(f4_idx[1], f4_idx[2]);
        }
        // Computing the distance from the centre of the tetrahedron to the centre of each face
        // along the normal of the face.
        PxReal face1_distance = face1_normal.dot(face1_centre - tetCentre);
        PxReal face2_distance = face2_normal.dot(face2_centre - tetCentre);
        PxReal face3_distance = face3_normal.dot(face3_centre - tetCentre);
        PxReal face4_distance = face4_normal.dot(face4_centre - tetCentre);
        //Plane coefficients for each face
        // The coefficients should be such that: ax + by + cz + d = 0
        PxReal face1_plane[4] = {face1_normal.x, face1_normal.y, face1_normal.z, face1_distance};
        PxReal face2_plane[4] = {face2_normal.x, face2_normal.y, face2_normal.z, face2_distance};
        PxReal face3_plane[4] = {face3_normal.x, face3_normal.y, face3_normal.z, face3_distance};
        PxReal face4_plane[4] = {face4_normal.x, face4_normal.y, face4_normal.z, face4_distance};

        //Compute the volume of the tetrahedron
        PxReal volume = (1.0/6.0) * (v3 - v0).dot((v1 - v0).cross(v2 - v0));

        //Since the PxHullPolygon::mPlane essentially describe a unit vector at a distance from the origin,
        // the information about the vertices/shape of the face must also be provided. This is done through
        // a indices buffer starting at PxHullPolygon::mIndexBase and containing a sequence of
        // PxHullPolygon::mNbVerts PxU32 indices such that indices[mIndexBase + i] is the index of the ith vertex.
        // There is a maximum of 255 vertices per face so the index must be below 256.
        PxHullPolygon* polyFaces = new PxHullPolygon[4];
        polyFaces[0].mNbVerts = 3; polyFaces[1].mNbVerts = 3; polyFaces[2].mNbVerts = 3; polyFaces[3].mNbVerts = 3;
        polyFaces[0].mIndexBase = 0; polyFaces[1].mIndexBase = 3; polyFaces[2].mIndexBase = 6; polyFaces[3].mIndexBase = 9;
        //Each PxHullPolygon contains a plane equation where the plane format is
        //      (mPlane[0],mPlane[1],mPlane[2]).dot(x) + mPlane[3] = 0 
        // with the normal outward-facing from the hull.
        polyFaces[0].mPlane[0] = face1_plane[0]; polyFaces[0].mPlane[1] = face1_plane[1]; polyFaces[0].mPlane[2] = face1_plane[2]; polyFaces[0].mPlane[3] = face1_plane[3];
        polyFaces[1].mPlane[0] = face2_plane[0]; polyFaces[1].mPlane[1] = face2_plane[1]; polyFaces[1].mPlane[2] = face2_plane[2]; polyFaces[1].mPlane[3] = face2_plane[3];
        polyFaces[2].mPlane[0] = face3_plane[0]; polyFaces[2].mPlane[1] = face3_plane[1]; polyFaces[2].mPlane[2] = face3_plane[2]; polyFaces[2].mPlane[3] = face3_plane[3];
        polyFaces[3].mPlane[0] = face4_plane[0]; polyFaces[3].mPlane[1] = face4_plane[1]; polyFaces[3].mPlane[2] = face4_plane[2]; polyFaces[3].mPlane[3] = face4_plane[3];

        //Build a description of the convex tetrahedral mesh
        PxConvexMeshDesc convexDesc;
        convexDesc.points.count             = 4;
        convexDesc.points.stride            = sizeof(PxVec3);
        convexDesc.points.data              = vertices;
        
        convexDesc.polygons.count           = 4;
        convexDesc.polygons.stride          = sizeof(PxHullPolygon);
        convexDesc.polygons.data            = polyFaces;

        convexDesc.indices.count            = 12;
        convexDesc.indices.stride           = sizeof(PxU32);
        convexDesc.indices.data             = indexBuffer;

        //Setting the following flag can be useful to verify that the convexDesc contains
        // the right information. In the debug console, it is then possible to check
        // (PxHullPolygon*)(convexMeshDesc.polygons.data + i*sizeof(PxHullPolygon))
        // where i is set to the index of the polygon (0-3).
        // convexDesc.flags                    |= PxConvexFlag::eCOMPUTE_CONVEX;
        //For some reason, the mesh does not pass the validation test as apparently
        // "Some hull vertices seems to be too far from hull planes"
        // although we produce the exact same mesh as when using eCOMPUTE_CONVEX.
        convexDesc.flags                    |= PxConvexFlag::eDISABLE_MESH_VALIDATION;

        cout << "Vertices: " << endl;
        for(int i=0; i < convexDesc.points.count; i++){
            cout << "    " << vertices[i].x << ", " << vertices[i].y << ", " << vertices[i].z << endl;
        }

        cout << "Planes: " << endl;
        for(int i=0; i < convexDesc.polygons.count; i++){
            cout << "    " << polyFaces[i].mPlane[0] << ", " << polyFaces[i].mPlane[1] << ", " << polyFaces[i].mPlane[2] << ", " << polyFaces[i].mPlane[3] << endl;
        }

        cout << "Indices: " << endl;
        for(int i=0; i < convexDesc.indices.count/3; i++){
            cout << "    " << indexBuffer[i*3] << ", " << indexBuffer[i*3+1] << ", " << indexBuffer[i*3+2] << endl;
        }

        if(!convexDesc.isValid())
            throw runtime_error("Invalid convex mesh description");

        //Add the convex mesh description to the list of convex mesh descriptions
        convexMeshDescs.pushBack(convexDesc);
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

/// @brief Add an object to the scene
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
void Scene::add_object(
    string id, 
    Matrix4f pose, 
    MatrixX3f vertices, 
    MatrixX3i triangles,
    bool is_fixed,
    float mass,
    Vector3f com,
    string material_name
)
{
    //Create an object instance and add it to the scene
    // When adding an element to the vector, the vector may reallocate memory and move the elements which will change their addresses.
    // However, we need to supply the callback with a persistent pointer to the object.
    // Therefore, we store pointers to preallocated objects in the vector instead of the objects themselves.
    shared_ptr<Object> obj = make_shared<Object>(id, pose, vertices, triangles, is_fixed, mass, com, material_name);
    object_ptrs.push_back(obj);

    //Perform remeshing to make sure the triangle mesh is adequate for further processing.
    PxArray<PxVec3> triVerts, remeshVerts, simplifiedVerts;
    PxArray<PxU32> triIndices, remeshIndices, simplifiedIndices;
    for(int i = 0; i < vertices.rows(); i++){
        triVerts.pushBack(PxVec3(vertices(i, 0), vertices(i, 1), vertices(i, 2)));
    }
    for(int i = 0; i < triangles.rows(); i++){
        triIndices.pushBack(triangles(i, 0));
        triIndices.pushBack(triangles(i, 1));
        triIndices.pushBack(triangles(i, 2));
    }
    PxTetMaker::remeshTriangleMesh(triVerts, triIndices, PxU32(10), remeshVerts, remeshIndices);
    //Remeshing creates a lot of vertices and triangles. We can simplify the mesh to reduce the number of vertices and triangles.
    // It can also alleviate the problem of eCONTAINS_ACUTE_ANGLED_TRIANGLES.
    PxTetMaker::simplifyTriangleMesh(remeshVerts, remeshIndices, 100, 0, simplifiedVerts, simplifiedIndices);

    /*

    PxTriangleMeshDesc meshDesc;
    //PxConvexMeshDesc meshDesc;
    PxU32 numVertices = static_cast<uint32_t>(vertices.rows());
    meshDesc.points.count           = numVertices;
    meshDesc.points.stride          = sizeof(PxVec3);
    PxVec3* pxVertices = new PxVec3[numVertices];
    for (PxU32 i = 0; i < numVertices; i++)
    {
        pxVertices[i] = PxVec3(vertices(i, 0), vertices(i, 1), vertices(i, 2));
    }
    meshDesc.points.data            = pxVertices;

    PxU32 numTriangles = static_cast<uint32_t>(triangles.rows());
    meshDesc.triangles.count        = numTriangles;
    meshDesc.triangles.stride       = 3*sizeof(PxU32);
    PxU32* pxTriangles = new PxU32[numTriangles*3];
    for (PxU32 i = 0; i < numTriangles; i++)
    {
        pxTriangles[i*3]    = triangles(i, 0);
        pxTriangles[i*3+1]  = triangles(i, 1);
        pxTriangles[i*3+2]  = triangles(i, 2);
    }
    meshDesc.triangles.data         = pxTriangles;
    //meshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxTolerancesScale scale;
    PxCookingParams params(scale);
    PxDefaultMemoryOutputStream writeBuffer;
    PxDefaultMemoryInputData readBuffer = PxDefaultMemoryInputData(writeBuffer.getData(), writeBuffer.getSize());
    PxTriangleMeshCookingResult::Enum result;

    //Create a tetrahedron mesh from the surface mesh
    //PxSimpleTriangleMesh surfaceMesh;
    //surfaceMesh.points      = meshDesc.points;
    //surfaceMesh.triangles   = meshDesc.triangles;

    //Cooking the triangle mesh will make sure it is adequate for further processing.
    bool status = PxCookTriangleMesh(params, meshDesc, writeBuffer);
    if(!status)
        throw runtime_error("Error cooking mesh");

    PxDefaultMemoryInputData buffer(writeBuffer.getData(), writeBuffer.getSize());
    PxTriangleMesh* triMesh = gPhysics->createTriangleMesh(buffer);

    triVerts.clear();
	triIndices.clear();

    const PxVec3* triMeshVertices = triMesh->getVertices();
    for(int i = 0; i < triMesh->getNbVertices(); i++){
        triVerts.pushBack(triMeshVertices[i]);
    }
    const void* idxBuffer = triMesh->getTriangles();
    PxTriangleMeshFlags flags = triMesh->getTriangleMeshFlags();
    const PxU32* remapTable = triMesh->getTrianglesRemap();
    
    if(flags.isSet(PxTriangleMeshFlag::e16_BIT_INDICES)){
        for(int i = 0; i < triMesh->getNbTriangles(); i++){
            const physx::PxU16* triMeshIndices = reinterpret_cast<const physx::PxU16*>(idxBuffer);
            triIndices.pushBack(static_cast<const PxU16*>(triMeshIndices)[i*3]);
            triIndices.pushBack(static_cast<const PxU16*>(triMeshIndices)[i*3+1]);
            triIndices.pushBack(static_cast<const PxU16*>(triMeshIndices)[i*3+2]);
        }
    }else{
        for(int i = 0; i < triMesh->getNbTriangles(); i++){
            const physx::PxU32* triMeshIndices = reinterpret_cast<const physx::PxU32*>(idxBuffer);
            triIndices.pushBack(static_cast<const PxU32*>(triMeshIndices)[i*3]);
            triIndices.pushBack(static_cast<const PxU32*>(triMeshIndices)[i*3+1]);
            triIndices.pushBack(static_cast<const PxU32*>(triMeshIndices)[i*3+2]);
        }
    }

    // for(int i = 0; i < vertices.rows(); i++){
    //     triVerts.pushBack(PxVec3(vertices(i, 0), vertices(i, 1), vertices(i, 2)));
    // }
    // for(int i = 0; i < triangles.rows(); i++){
    //     triIndices.pushBack(triangles(i, 0));
    //     triIndices.pushBack(triangles(i, 1));
    //     triIndices.pushBack(triangles(i, 2));
    // }
    PxSimpleTriangleMesh surfaceMesh;
	surfaceMesh.points.count = triVerts.size();
	surfaceMesh.points.data = triVerts.begin();
	surfaceMesh.triangles.count = triIndices.size() / 3;
	surfaceMesh.triangles.data = triIndices.begin();

    */

    PxSimpleTriangleMesh newSurfaceMesh;
	newSurfaceMesh.points.count = simplifiedVerts.size();
	newSurfaceMesh.points.data = simplifiedVerts.begin();
	newSurfaceMesh.triangles.count = simplifiedIndices.size() / 3;
	newSurfaceMesh.triangles.data = simplifiedIndices.begin();

    PxArray<PxVec3> tetMeshVertices;
    PxArray<PxU32> tetMeshIndices;
   
    bool isValid = validateMesh(newSurfaceMesh);

    //Create a tetrahedron mesh from the surface mesh
    bool success = PxTetMaker::createConformingTetrahedronMesh(newSurfaceMesh, tetMeshVertices, tetMeshIndices);
    //For each tetrahedron, create a convex mesh.
    PxArray<PxConvexMeshDesc> convexMeshDescs;
    createThetrahedronSet(tetMeshVertices, tetMeshIndices, convexMeshDescs);
    
    PxTolerancesScale scale;
    PxCookingParams params(scale);
    PxArray<PxShape*> convexShapes;
    //Create a shape from each mesh description.
    for(int i = 0; i < convexMeshDescs.size(); i++){
        cout << "Creating shape " << i << endl;
        PxConvexMeshDesc convexMeshDesc = convexMeshDescs[i];
        PxShape* convexShape = createTetrahedronShape(params, convexMeshDesc);
        //Assign a pointer to the object such that it can be retrieved in the contact report callback.
        convexShape->userData = obj.get();
        //And also set the name of the shape as the object id.
        convexShape->setName(obj->id.c_str());
        convexShapes.pushBack(convexShape);
        /*
        //Cook the convex mesh
        PxConvexMeshCookingResult::Enum result;
        PxDefaultMemoryOutputStream buf;
        if(!PxCookConvexMesh(params, convexMeshDesc, buf, &result))
            throw runtime_error("Error cooking mesh");

        //writeBuffer = PxDefaultMemoryOutputStream();
        //readBuffer  = PxDefaultMemoryInputData(writeBuffer.getData(), writeBuffer.getSize());
        //PxConvexMesh* convexMesh = gPhysics->createConvexMesh(readBuffer);

        PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
        PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);

        //WARNING: I had no luck in making below function work as it always complained about
        // physx/source/geomutils/src/cooking/GuCookingConvexHullBuilder.cpp#L479
        //bool res = PxValidateConvexMesh(params, convexMeshDesc);

        //Create convex mesh without cooking
        //PxConvexMesh* convexMesh = PxCreateConvexMesh(params, convexMeshDesc, gPhysics->getPhysicsInsertionCallback());
        if(!convexMesh)
            throw runtime_error("Error creating mesh");

        //Create a shape from the convex mesh
        // The shape has to be exclusive (hence the "true") such that it is guaranteed to be
        // associated with only one actor. That way, the actor can be retrieved from the shape
        // in the contact report callback.
        PxConvexMeshGeometry convexGeometry = PxConvexMeshGeometry(convexMesh);
        bool isValid = convexGeometry.isValid();
        if(!isValid)
            throw runtime_error("Invalid convex mesh geometry");
        PxShape* convexShape = gPhysics->createShape(convexGeometry, *gMaterial, true);
        convexShapes.pushBack(convexShape);
        */
    }

    /*
    PxTetrahedronMeshDesc tetMeshDesc = PxTetrahedronMeshDesc(tetMeshVertices, tetMeshIndices);
    status = PxCookTetrahedronMesh(params, tetMeshDesc, writeBuffer);
    if(!status)
        throw runtime_error("Error cooking mesh");
    readBuffer = PxDefaultMemoryInputData(writeBuffer.getData(), writeBuffer.getSize());
    PxTetrahedronMesh* tetMesh = gPhysics->createTetrahedronMesh(readBuffer);
    PxShape* tetShape = gPhysics->createShape(PxTetrahedronMeshGeometry(tetMesh), *gMaterial, true);

    //PxSoftBodyMesh* softBodyMesh = PxSoftBodyExt::createSoftBodyMeshNoVoxels(params, surfaceMesh, gPhysics->getPhysicsInsertionCallback());
    //PxTetrahedronMeshGeometry tetGeometry(softBodyMesh->getCollisionMesh());

    //bool status = PxCookConvexMesh(params, meshDesc, writeBuffer);
    status = PxCookTriangleMesh(params, meshDesc, writeBuffer);
    if(!status)
        throw runtime_error("Error cooking mesh");

    readBuffer = PxDefaultMemoryInputData(writeBuffer.getData(), writeBuffer.getSize());
    PxTriangleMesh* trimesh = gPhysics->createTriangleMesh(readBuffer);
    //PxConvexMesh* trimesh = gPhysics->createConvexMesh(readBuffer);

    //The shape has to be exclusive (hence the "true") such that it is guaranteed to be
    // associated with only one actor. That way, the actor can be retrieved from the shape
    // in the contact report callback.
    PxShape* shape = gPhysics->createShape(PxTriangleMeshGeometry(trimesh), *gMaterial, true);
    //PxShape* shape = gPhysics->createShape(PxConvexMeshGeometry(trimesh), *gMaterial, true);
    
    shape->userData = obj.get();
    shape->setName(obj->id.c_str());
    */
    PxTransform pxPose = PxTransform(PxMat44(
        PxVec4(pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3)),
        PxVec4(pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3)),
        PxVec4(pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3)),
        PxVec4(pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3))
    ));
    
    if(is_fixed){
        PxRigidStatic* actor = gPhysics->createRigidStatic(pxPose);
        //Attach all shapes in the object to the actor
        for(int i = 0; i < convexShapes.size(); i++){
            actor->attachShape(*convexShapes[i]);
        }
        //actor->attachShape(*shape);
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
        //actor->attachShape(*shape);
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
        if(obj->get_id() == id)
            return obj->get_pose();
    cout << "Object with id " << id << " not found" << endl;
    return Matrix4f::Identity();
}