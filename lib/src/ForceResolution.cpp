#include "ForceResolution.h"

//NOTE: At the time of implementing this, OSQP's documentation is severely outdated.
// Look at the source code for the most up-to-date information.

ContactForce::ContactForce(Vector3f position, Vector3f normal_dir, Vector3f tangent_dir_u, Vector3f tangent_dir_v, string object1_by_id, string object2_to_id, float friction_coefficient):
    position(position),
    normal_dir(normal_dir),
    tangent_dir_u(tangent_dir_u),
    tangent_dir_v(tangent_dir_v),
    object1_by_id(object1_by_id),
    object2_to_id(object2_to_id),
    friction_coefficient(friction_coefficient)
{
}

ContactForce::~ContactForce()
{
}

Eigen::Vector<float, 6> ContactForce::get_global_wrench()
{
    //The wrench is the force and torque exerted on object1_by by object2_to
    // expressed in the global frame of reference.
    // The force is directed from object1_by to object2_to.
    // The torque is computed as the cross product of the position vector and the force.
    Matrix3f R_l_w;
    R_l_w.col(0) = this->tangent_dir_u;
    R_l_w.col(1) = this->tangent_dir_v;
    R_l_w.col(2) = this->normal_dir;

    Eigen::Vector<float, 6> wrench;
    //The local force is expressed in the local frame of reference.
    // but the wrench is expressed in the global frame of reference.
    wrench.block<3, 1>(0, 0) = R_l_w * this->local_force;
    wrench.block<3, 1>(3, 0) = this->position.cross(wrench.block<3, 1>(0, 0));
    return wrench;
}

ForceSolver::ForceSolver(Scene* scene, int nb_contacts_per_object_pair, int nb_coulomb_polygon_sides): 
    scene(scene), 
    nb_coulomb_polygon_sides(nb_coulomb_polygon_sides)
{
    //Validate the number of contact points per object pair
    this->set_nb_contacts_per_object_pair(nb_contacts_per_object_pair);

    //Get all object IDs
    vector<string> object_ids = this->scene->get_all_object_ids();

    //Map each object pair to the contact points between them
    unordered_map<pair<string, string>, vector<ContactForce>, ObjectIDHashFunction> contact_forces;

    //Get all combinations of objects in contact, and select the N farthest contact points
    // between each pair of objects to locate the contact forces.
    int nb_nonfixed_objects = 0;
    for (size_t i = 0; i < object_ids.size(); i++){
        
        string id = object_ids[i];
        Object* obj = this->scene->get_object_by_id(id);
        if(!obj->is_fixed){
            nb_nonfixed_objects++;
        }

        for (size_t j = i+1; j < object_ids.size(); j++){
            string id1 = object_ids[i];
            string id2 = object_ids[j];
            //Get the points on the convex hull over the contact points
            MatrixX3f points = this->scene->get_contact_convex_hull(id1, id2, this->nb_contacts_per_object_pair);
            
            //For each point, create a ContactForce object
            vector<ContactForce> obj_pair_contact_forces;
            for (size_t k = 0; k < points.rows(); k++){
                Vector3f position = points.row(k);
                //The normal is outward from object1 and inward to object2
                Vector3f normal = this->scene->get_normal_at_point(id1, position);
                //Define vectors tangent to the surface at the contact point
                Vector3f tangent_u = normal.cross(Vector3f(1.0f, 0.0f, 0.0f));
                if (tangent_u.norm() < 1e-3){
                    tangent_u = normal.cross(Vector3f(0.0f, 1.0f, 0.0f));
                }
                tangent_u.normalize();
                Vector3f tangent_v = normal.cross(tangent_u);
                tangent_v.normalize();

                //Friction coefficient
                Object* obj1 = this->scene->get_object_by_id(id1);
                Object* obj2 = this->scene->get_object_by_id(id2);
                float mu = this->scene->get_friction_coefficient(obj1->material_name, obj2->material_name);

                //Create the ContactForce object and add it to the map
                ContactForce contact_force = ContactForce(position, 
                                                            normal, 
                                                            tangent_u, 
                                                            tangent_v,
                                                            id1,
                                                            id2,
                                                            mu);

                obj_pair_contact_forces.push_back(contact_force);
                all_contact_forces.push_back(contact_force);
            }
            contact_forces[make_pair(id1, id2)] = obj_pair_contact_forces;
        }
    }

    this->nb_contact_forces = all_contact_forces.size();
    //This is OSQP's n
    this->nb_optim_variables = 3*this->nb_contact_forces;
    //This is OSQP's m 
    this->nb_constraints = 6*nb_nonfixed_objects 
            + this->nb_coulomb_polygon_sides*this->nb_contact_forces 
            + 3*this->nb_contact_forces; 

    //Shorthands
    int n = this->nb_optim_variables;
    int m = this->nb_constraints;
    int I = this->nb_contact_forces;
    int J = nb_nonfixed_objects;
    int N = this->nb_coulomb_polygon_sides;

    //OSQP Matrix Sizes
    //  A: m x n
    //  P: Identity matrix of size n
    //  q: Zero vector of size n
    //  l: m x 1
    //  u: m x 1
    A(m, n); A.setZero();
    P(n, n); P.setIdentity();
    q(n); q.setZero();
    u(m); u.setZero();
    l(m); l.setZero();
    //The last (m-6J) elements of l are -inf
    Eigen::VectorXf l_inf(m-6*J);
    l_inf.setConstant(-std::numeric_limits<float>::infinity());
    l.segment(6*J, m-6*J) = l_inf;

    //Equilibrium of each object
    for (size_t j = 0; j < object_ids.size(); j++){
        string id = object_ids[j];
        Object* obj = this->scene->get_object_by_id(id);

        //Only consider objects that are not fixed
        // Fixed objects are magically held in place.
        if(obj->is_fixed){
            continue;
        }

        Matrix3f R_o_w = obj->pose.block<3, 3>(0, 0);
        Vector3f p_o_w = obj->pose.block<3, 1>(0, 3);
        Vector3f p_c_o = obj->com;
        float mass = obj->mass;

        //Compute the centre of mass relative to the global frame
        Vector3f p_c_w = R_o_w*p_c_o + p_o_w;

        //Consider the wrench due to gravity acting at the centre of mass
        Eigen::Matrix<float, 6, 3> B_gj;
        B_gj.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
        B_gj.block<3, 3>(3, 0) = skew(p_c_w);
        Eigen::Vector<float, 6> w_gj;
        w_gj = mass * B_gj * this->acceleration;
        //Set the lower and upper bounds equal to the (negated) wrench
        // to produce an equality/equilibrium constraint.
        l.segment(6*j, 6) = -w_gj;
        u.segment(6*j, 6) = -w_gj;

        //Iterate over all contact forces acting on the object to encode the equilibrium constraints
        // in matrix A.
        Eigen::Matrix<float, 6, -1> Bj(6, 3*I);
        for(int i = 0; i < I; i++){
            ContactForce contact_force = all_contact_forces[i];
            if(contact_force.object1_by_id == id){
                //The contact force is exerted on o2 by o1 so we need to flip the sign of the wrench
                // if o1 and o2 are inverted because we want the force exerted on o1 by o2.
                Bj.block<6, 3>(i, 3*i) = -1*this->build_B_matrix(contact_force);
            }
            else if(contact_force.object2_to_id == id){
                Bj.block<6, 3>(i, 3*i) = this->build_B_matrix(contact_force);
            }
            else{
                //The object is not involved in the contact force
                Bj.block<6, 3>(i, 3*i).setZero();
            }
        }

        //Add the Bj matrix to the A matrix
        A.block(6*j, 0, 6, 3*I) = Bj;
    }

    //Iterate over all contact forces to encode the Coulomb friction constraints in matrix A.
    for(int i = 0; i < I; i++){
        ContactForce contact_force = all_contact_forces[i];
        A.block(6*J + N*i, 3*i, N, 3) = this->build_C_matrix(contact_force, N);
    }

    //The D matrix occupies the last I rows of A
    A.block(6*J + N*I, 0, I, 3*I) = this->build_D_matrix();

    //Encode the matrices in CSC format for OSQP
    this->encode_matrix_to_csc(A, &osqp_csc_A);
    this->encode_matrix_to_csc(P, &osqp_csc_P);
    
    //Allocate memory for the OSQP vectors
    // and set their values.
    osqp_q = (OSQPFloat*)malloc(n*sizeof(OSQPFloat));
    osqp_l = (OSQPFloat*)malloc(m*sizeof(OSQPFloat));
    osqp_u = (OSQPFloat*)malloc(m*sizeof(OSQPFloat));
    for(int i = 0; i < n; i++){
        osqp_q[i] = q(i);
    }
    for(int i = 0; i < m; i++){
        osqp_l[i] = l(i);
        osqp_u[i] = u(i);
    }
}

ForceSolver::~ForceSolver()
{
    //If the OSQP solver was initialized from solve_forces()
    // then osqp_cleanup() should free all data structures.
    osqp_cleanup(osqp_solver);
    //However, if the constructor was called but solve_forces() was not,
    // then we need to free the memory ourselves.
    if(osqp_csc_A.p) free(osqp_csc_A.p);
    if(osqp_csc_A.i) free(osqp_csc_A.i);
    if(osqp_csc_A.x) free(osqp_csc_A.x);
    if(osqp_csc_P.p) free(osqp_csc_P.p);
    if(osqp_csc_P.i) free(osqp_csc_P.i);
    if(osqp_csc_P.x) free(osqp_csc_P.x);
    //Free the OSQP vectors
    if(osqp_q) free(osqp_q);
    if(osqp_l) free(osqp_l);
    if(osqp_u) free(osqp_u);
}

std::vector<ContactForce> ForceSolver::solve_forces()
{
    //See implementation of osqp_set_default_settings in the OSQP source code
    // for the default settings.
    this->osqp_settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(this->osqp_settings);

    OSQPInt setup_failure = osqp_setup(&this->osqp_solver, 
                &this->osqp_csc_P, 
                this->osqp_q, 
                &this->osqp_csc_A, 
                this->osqp_l, 
                this->osqp_u, 
                this->nb_constraints, 
                this->nb_optim_variables, 
                this->osqp_settings);
    
    if(setup_failure){
        cout << "OSQP setup failed with error code: " << setup_failure << endl;
        return std::vector<ContactForce>();
    }

    OSQPInt solver_failure = osqp_solve(this->osqp_solver);
    if(solver_failure){
        cout << "OSQP solver failed with error code: " << solver_failure << endl;
        return std::vector<ContactForce>();
    }

    //The solution corresponds to the scalar forces x = [f1_u, f1_v, f1_n, f2_u, f2_v, f2_n, ...]
    for(int i = 0; i < this->nb_contact_forces; i++){
        ContactForce contact_force = this->all_contact_forces[i];
        contact_force.local_force = Eigen::Vector3f(osqp_solver->solution->x[3*i], 
                                                    osqp_solver->solution->x[3*i+1], 
                                                    osqp_solver->solution->x[3*i+2]);
    }

    return this->all_contact_forces;
}

/// @brief Encode a Eigen::Matrix into a OSQP CSC matrix as per the format of https://people.sc.fsu.edu/~jburkardt/data/cc/cc.html
/// @param matrix The Eigen::Matrix to encode.
/// @param csc_matrix A pointer to the OSQP CSC matrix to encode the Eigen::Matrix into. Memory will be allocated.
void ForceSolver::encode_matrix_to_csc(Eigen::Matrix<float, -1, -1> matrix, OSQPCscMatrix* csc_matrix)
{

    /*
    typedef struct {
    OSQPInt    m;     ///< number of rows
    OSQPInt    n;     ///< number of columns
    OSQPInt   *p;     ///< column pointers (size n+1); col indices (size nzmax) starting from 0 for triplet format
    OSQPInt   *i;     ///< row indices, size nzmax starting from 0
    OSQPFloat *x;     ///< numerical values, size nzmax
    OSQPInt    nzmax; ///< maximum number of entries
    OSQPInt    nz;    ///< number of entries in triplet matrix, -1 for csc
    } OSQPCscMatrix;
    */

    //Get the number of rows and columns
    int m = matrix.rows();
    int n = matrix.cols();

    //Count the number of non-zero elements
    int nnz = (matrix.array() > 0).count() + (matrix.array() < 0).count();

    //Allocate memory for the CSC matrix
    csc_matrix->n = n;
    csc_matrix->m = m;
    csc_matrix->nz = -1;
    csc_matrix->nzmax = nnz;
    csc_matrix->p = (OSQPInt*)malloc((n+1)*sizeof(OSQPInt));
    csc_matrix->i = (OSQPInt*)malloc(nnz*sizeof(OSQPInt));
    csc_matrix->x = (OSQPFloat*)malloc(nnz*sizeof(OSQPFloat));

    //Populate the CSC matrix
    int nnz_index = 0;
    csc_matrix->p[0] = 0;
    for(int j = 0; j < n; j++){
        for(int i = 0; i < m; i++){
            if(matrix(i, j) != 0){
                csc_matrix->x[nnz_index] = matrix(i, j);
                csc_matrix->i[nnz_index] = i;
                nnz_index++;
            }
        }
        csc_matrix->p[j+1] = nnz_index;
    }
}

Eigen::Matrix<float, 6, 3> ForceSolver::build_B_matrix(ContactForce contact_force)
{
    /*
    B_i = [ I(3); Skew(contact_force.position) ] * [ contact_force.tangent_dir_u, contact_force.tangent_dir_v, contact_force.normal_dir ]
    */

    Eigen::Matrix<float, 6, 3> B1;
    Eigen::Matrix<float, 3, 3> B2;
    Eigen::Matrix<float, 6, 3> Bi;
    
    //B1 is  [ I(3);  Skew(contact_force.position) ]
    B1 << Eigen::Matrix3f::Identity(), skew(contact_force.position);

    //B2 is [ contact_force.tangent_dir_u, contact_force.tangent_dir_v, contact_force.normal_dir ]
    B2.col(0) = contact_force.tangent_dir_u;
    B2.col(1) = contact_force.tangent_dir_v;
    B2.col(2) = contact_force.normal_dir;

    //Bi is B1 * B2
    Bi = B1 * B2;

    return Bi;   
}

Eigen::Matrix<float, -1, 3> ForceSolver::build_C_matrix(ContactForce contact_force, int nb_coulomb_polygon_sides)
{
    float mu = contact_force.friction_coefficient;
    int N = nb_coulomb_polygon_sides;
    //C_i has size (N, 3)
    Eigen::Matrix<float, -1, 3> Ci(N, 3);

    //The Coulomb polygon is a regular polygon with N sides that is
    // inscribed in a circle of radius mu. 
    for(int k = 0; k < N; k++){
        Ci << cos(2*M_PI*k/N), sin(2*M_PI*k/N), -mu*cos(M_PI*N);
    }

    return Ci;
}

Eigen::Matrix<float, -1, -1> ForceSolver::build_D_matrix()
{
    //Di is a matrix of size (1, 3) that encodes the normal force unilateral constraint.
    //D is a matrix whose (block) diagonal elements are Di. Its size is (I, 3I).
    int I = this->nb_contact_forces;
    Eigen::Matrix<float, -1, -1> D(I, 3*I);
    D.setZero();

    //All entries of D are zeros except for (i, 3i-1), which are -1.
    for(int i = 0; i < I; i++){
        D(i, 3*i-1) = -1;
    }

    return D;
}

/// @brief Define the number of contact points to consider for pair of contacting objects.
/// @param nb_contacts Number of contact points to consider (minimum 4, default 5).
void ForceSolver::set_nb_contacts_per_object_pair(int nb_contacts)
{
    //The minimum number of contact points is 4 (due to requirements from convex hull computation)
    if(nb_contacts < 4){
        nb_contacts = 4;
    }else{
        this->nb_contacts_per_object_pair = nb_contacts;
    }
}

/// @brief Return the contact forces between objects in the scene (solve them if not already solved).
/// @return A vector of ContactForce objects.
std::vector<ContactForce> ForceSolver::get_contact_forces()
{
    //If the solver has not yet been called, call it.
    // Otherwise, return the computed contact forces.
    if(!this->osqp_solver){
        return this->solve_forces();
    }else{
        return this->all_contact_forces;
    }
}