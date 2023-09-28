#include "Object.h"

/// @brief Define an object in the scene with the following properties
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
Object::Object(string id, Matrix4f pose, MatrixX3f vertices, MatrixX3i triangles, 
        bool is_fixed = false, 
        float mass = 1.0f, 
        Vector3f com = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(), 
        string material_name = "wood")
:
    id{id},
    pose{pose},
    tri_vertices{vertices},
    tri_triangles{triangles},
    is_fixed{is_fixed},
    mass{mass},
    com{com},
    material_name{material_name},
    max_separation{0.04f}
{}
Object::~Object(){}

/// @brief Record the description of the tetrahedral mesh that represents the volume of the object
/// @param vertices Vertices of the mesh
/// @param indices Indices of the mesh, referring to the vertices, 4 per tetrahedron
void Object::set_tetra_mesh(PxArray<PxVec3> vertices, PxArray<PxU32> indices)
{
    //Convert PxArray to Eigen Matrix
    this->tetra_vertices.resize(vertices.size(), 3);
    for (int i = 0; i < vertices.size(); i++) {
        this->tetra_vertices(i, 0) = vertices[i].x;
        this->tetra_vertices(i, 1) = vertices[i].y;
        this->tetra_vertices(i, 2) = vertices[i].z;
    }

    this->tetra_indices.resize(indices.size()/4, 4);
    for (int i = 0; i < indices.size()/4; i++) {
        this->tetra_indices(i, 0) = indices[i*4+0];
        this->tetra_indices(i, 1) = indices[i*4+1];
        this->tetra_indices(i, 2) = indices[i*4+2];
        this->tetra_indices(i, 3) = indices[i*4+3];
    }
}

/// @brief Record the description of the triangle mesh that represents the surface of the object
/// @param simpleTriMesh Triangle mesh
void Object::set_tri_mesh(PxSimpleTriangleMesh& simpleTriMesh)
{
    //Convert PxSimpleTriangleMesh to Eigen Matrix
    this->tri_vertices.resize(simpleTriMesh.points.count, 3);
    for (int i = 0; i < simpleTriMesh.points.count; i++) {
        PxVec3* data;
        data = (PxVec3*)(simpleTriMesh.points.data + i*sizeof(PxVec3));
        this->tri_vertices(i, 0) = data->x;
        this->tri_vertices(i, 1) = data->y;
        this->tri_vertices(i, 2) = data->z;
    }

    this->tri_triangles.resize(simpleTriMesh.triangles.count, 3);
    PxU32* data;
    data = (PxU32*)(simpleTriMesh.triangles.data);
    for (int i = 0; i < simpleTriMesh.triangles.count; i++) {
        this->tri_triangles(i, 0) = data[i*3+0];
        this->tri_triangles(i, 1) = data[i*3+1];
        this->tri_triangles(i, 2) = data[i*3+2];
    }
}

/// @brief Record the description of the triangle mesh that represents the surface of the object
/// @param vertices Nx3 matrix with each row representing the coordinates of a vertex
/// @param triangles Mx3 matrix with each row representing the indices of a triangle
void Object::set_tri_mesh(MatrixX3f& vertices, MatrixX3i& triangles)
{
    this->tri_vertices  = vertices;
    this->tri_triangles = triangles;
}

/// @brief Create a grid of cells that represent the volume of the object.
/// @param resolution Number of cells per unit of length.
/// @return Pointer to the occupancy grid
shared_ptr<OccupancyGrid> Object::create_occupancy_grid(int resolution, int sampling_method)
{
    MatrixX3f vertices = this->tri_vertices;
    MatrixX3i triangles = this->tri_triangles;
    //Build the grid
    shared_ptr<OccupancyGrid> grid = make_shared<OccupancyGrid>(vertices, triangles, resolution, sampling_method);
    this->occupancy_grid = grid;
    return this->occupancy_grid;
}


/// @brief Iterates over the vertices and triangles of a PxTriangleMesh and stores them in supplied arrays
/// @param triMesh The triangle mesh to extract the vertices and triangles from
/// @param triVerts Array to store the vertices in
/// @param triIndices Array to store the indices in
void get_triangles_from_trimesh(PxTriangleMesh* triMesh, PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices)
{
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
}

/// @brief Check the triangle mesh for any common issues that the tetrahedron meshing algorithm cannot handle.
/// @param surfaceMesh Description of the triangle mesh.
/// @return True if the mesh is valid (even if small problems are detected), false otherwise.
bool Object::validate_mesh(PxSimpleTriangleMesh& surfaceMesh)
{
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
/// @return True if the convex mesh descriptions were successfully created, false otherwise.
bool Object::create_tetra_convex_set(PxArray<PxVec3> tetVertices, PxArray<PxU32> tetIndices, PxArray<PxConvexMeshDesc>& convexMeshDescs)
{
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

        // cout << "Vertices: " << endl;
        // for(int i=0; i < convexDesc.points.count; i++){
        //     cout << "    " << vertices[i].x << ", " << vertices[i].y << ", " << vertices[i].z << endl;
        // }

        // cout << "Planes: " << endl;
        // for(int i=0; i < convexDesc.polygons.count; i++){
        //     cout << "    " << polyFaces[i].mPlane[0] << ", " << polyFaces[i].mPlane[1] << ", " << polyFaces[i].mPlane[2] << ", " << polyFaces[i].mPlane[3] << endl;
        // }

        // cout << "Indices: " << endl;
        // for(int i=0; i < convexDesc.indices.count/3; i++){
        //     cout << "    " << indexBuffer[i*3] << ", " << indexBuffer[i*3+1] << ", " << indexBuffer[i*3+2] << endl;
        // }

        if(convexDesc.isValid() == false)
            return false;

        //Add the convex mesh description to the list of convex mesh descriptions
        convexMeshDescs.pushBack(convexDesc);
    }
    return true;
}

/// @brief Redo the meshing of the surface mesh to make sure it is adequate for further processing.
/// @param in_vertices Nx3 matrix representing the vertices of the surface mesh.
/// @param in_triangles Mx3 matrix representing the triangles of the surface mesh.
/// @return Simplified surface mesh.
PxSimpleTriangleMesh Object::remesh_surface_trimesh(MatrixX3f in_vertices, MatrixX3i in_triangles)
{
    //Perform remeshing to make sure the triangle mesh is adequate for further processing.
    PxArray<PxVec3> triVerts, remeshVerts, simplifiedVerts;
    PxArray<PxU32> triIndices, remeshIndices, simplifiedIndices;

    for(int i = 0; i < in_vertices.rows(); i++){
        triVerts.pushBack(PxVec3(in_vertices(i, 0), in_vertices(i, 1), in_vertices(i, 2)));
    }

    for(int i = 0; i < in_triangles.rows(); i++){
        triIndices.pushBack(in_triangles(i, 0));
        triIndices.pushBack(in_triangles(i, 1));
        triIndices.pushBack(in_triangles(i, 2));
    }

    PxTetMaker::remeshTriangleMesh(triVerts, triIndices, PxU32(10), remeshVerts, remeshIndices);
    
    //Remeshing creates a lot of vertices and triangles. We can simplify the mesh to reduce the number of vertices and triangles.
    // It can also alleviate the problem of eCONTAINS_ACUTE_ANGLED_TRIANGLES.
    PxTetMaker::simplifyTriangleMesh(remeshVerts, remeshIndices, 100, 0, simplifiedVerts, simplifiedIndices);

    PxSimpleTriangleMesh newSurfaceMesh;
	newSurfaceMesh.points.count     = simplifiedVerts.size();
	newSurfaceMesh.points.data      = simplifiedVerts.begin();
	newSurfaceMesh.triangles.count  = simplifiedIndices.size() / 3;
	newSurfaceMesh.triangles.data   = simplifiedIndices.begin();

    return newSurfaceMesh;
}

/// @brief Perform the tetrahedralization of the surface mesh.
/// @param triSurfaceMesh Input triangular surface mesh.
/// @param tetMeshVertices Output array of 3D vertices of the tetrahedral mesh.
/// @param tetMeshIndices  Output array of indices of the tetrahedral mesh, with each tetrahedron being described by four indices.
/// @return True if the tetrahedralization was successful, false otherwise.
bool Object::create_tetra_mesh(PxSimpleTriangleMesh& triSurfaceMesh, PxArray<PxVec3>& tetMeshVertices, PxArray<PxU32>& tetMeshIndices)
{
    //Verify that the mesh is valid for tetrahedralization
    bool is_valid = this->validate_mesh(triSurfaceMesh);

    if(is_valid){
        //Create a tetrahedron mesh from the surface mesh
        bool success = PxTetMaker::createConformingTetrahedronMesh(triSurfaceMesh, tetMeshVertices, tetMeshIndices);
        return success;
    }

    return false;
}

/// @brief Get the position of the centre of each occupied cell in the occupancy grid
/// @return Nx3 matrix representing the centre positions
MatrixX3f Object::get_voxel_centres()
{
    std::unordered_map<uint32_t, GridCell>* grid = this->occupancy_grid->get_grid_cells();
    MatrixX3f cube_centres(grid->size(), 3);
    int i = 0;
    for (auto& it : *grid) {
        cube_centres(i, 0) = it.second.centre(0);
        cube_centres(i, 1) = it.second.centre(1);
        cube_centres(i, 2) = it.second.centre(2);
        i++;
    }
    return cube_centres;
}

/// @brief Get the side lengths of each occupied cell in the occupancy grid
/// @return 3x1 vector representing the side lengths
Vector3f Object::get_voxel_side_lengths()
{
    std::unordered_map<uint32_t, GridCell>* grid = this->occupancy_grid->get_grid_cells();
    //All cells have the same side lengths
    Vector3f cube_side_lengths = grid->begin()->second.half_extents * 2;
    return cube_side_lengths;
}

/// @brief Set the maximal distance from the object to a valid contact point
/// @param max_separation Distance in the object's units
void Object::set_max_separation(float max_separation)
{
    this->max_separation = abs(max_separation);
}

/// @brief Set the pose of the object
/// @param pose 4x4 matrix representing the pose of the object
void Object::set_pose(Matrix4f pose)
{
    if(this->is_valid_pose_matrix(pose))
        this->pose = pose;
    else
        throw runtime_error("Invalid pose matrix.");
}

/// @brief Get the vertices of the triangle mesh in world coordinates
/// @return Nx3 matrix representing the vertices of the object in world coordinates
MatrixX3f Object::get_world_vertices()
{
    Matrix3f R = this->pose.block<3, 3>(0, 0);
    Vector3f t = this->pose.block<3, 1>(0, 3);
    MatrixX3f vert_wrt_world = (R * this->tri_vertices.transpose()).transpose().rowwise() + t.transpose();
    return vert_wrt_world;
}

/// @brief  Test if the supplied matrix is a 4x4 homogeneous transformation matrix.
/// @param  4x4 matrix to test
/// @return True is the matrix can describe a pose or a transformation in 3D space, false otherwise.
bool Object::is_valid_pose_matrix(Matrix4f matrix)
{
    //Extract the rotation and translation parts
    Matrix3f R = matrix.block<3, 3>(0, 0);
    Vector3f t = matrix.block<3, 1>(0, 3);

    //Check that the rotation matrix is orthogonal
    Matrix3f RtR = R.transpose() * R;
    Matrix3f I = Matrix3f::Identity();
    if((RtR - I).norm() > 1e-6)
        return false;

    //Check that the determinant of the rotation matrix is 1
    if(abs(R.determinant() - 1) > 1e-6)
        return false;

    //Check that the translation vector is not NaN
    if(t.hasNaN())
        return false;

    //Check that the last row of the matrix is [0, 0, 0, 1]
    if(matrix(3, 0) != 0 || matrix(3, 1) != 0 || matrix(3, 2) != 0 || matrix(3, 3) != 1)
        return false;

    return true;
}