#pragma once

#include <memory>
#include <set>
#include <unordered_map>
#include <random>
#include <algorithm>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>
#include "osqp.h"
#include "AssemblyCD.h"


using namespace Eigen;
using namespace physx;
using namespace std;

class ContactForce
{
    public:
        //Position of the contact point in the global frame of reference.
        Vector3f position;
        /*
        A local frame of reference at the contact point is defined by the normal vector and two tangent vectors.
          The normal is directed from object1_by to object2_to, so it is outward from object1_by and inward to object2_to.
        */
        Vector3f normal_dir;
        Vector3f tangent_dir_u;
        Vector3f tangent_dir_v;
        string object1_by_id;
        string object2_to_id;
        //The force is expressed in the local frame of reference.
        // The force is directed from object1_by to object2_to.
        Vector3f local_force = Vector3f(0.0f, 0.0f, 0.0f);
        //The friction coefficient at the contact point depends on the materials of the two objects.
        float friction_coefficient;
        //Wrench at the contact point expressed in the global frame of reference.
        Eigen::Vector<float, 6> get_global_wrench();

        ContactForce(Vector3f position, 
            Vector3f normal_dir, 
            Vector3f tangent_dir_u, 
            Vector3f tangent_dir_v, 
            string object1_by_id,
            string object2_to_id, 
            float friction_coefficient);
        ~ContactForce();
};

class ForceSolver
{
    private:
        Scene* scene;
        int nb_contacts_per_object_pair = 10;
        int nb_coulomb_polygon_sides = 8;
        int nb_optim_variables;
        int nb_constraints;
        int nb_contact_forces;
        std::vector<ContactForce> all_contact_forces;
        void set_nb_contacts_per_object_pair(int nb_contacts_per_object_pair);
        std::vector<Contact> select_contact_points(string id1, string id2);
        //The acceleration is the instantaneous acceleration of the center of mass of the objects.
        Vector3f acceleration = Vector3f(0.0f, 0.0f, -9.81f);
        
        std::vector<ContactForce> solve_forces();
        PointSet3D get_contact_points(string object_id1, string object_id2);
        
        //B_i is the matrix that maps a force from the local frame at i to the global frame.
        Eigen::Matrix<float, 6, 3> build_B_matrix(ContactForce contact_force);
        //C_i is the matrix of Coulomb polygon (linearized circle) at i.
        Eigen::Matrix<float, -1, 3> build_C_matrix(ContactForce contact_force, int nb_coulomb_polygon_sides);
        //D is the matrix encoding the normal force unilateral constraint.
        Eigen::Matrix<float, -1, -1> build_D_matrix();
        //Encode a given matrix into CSC format for OSQP.
        void encode_matrix_to_csc(Eigen::Matrix<float, -1, -1> matrix, OSQPCscMatrix* csc_matrix);
        
        //Eigen Matrices
        Eigen::Matrix<float, -1, -1> A;
        Eigen::Matrix<float, -1, -1> P;
        Eigen::Matrix<float, -1, 1> q;
        Eigen::Matrix<float, -1, 1> l;
        Eigen::Matrix<float, -1, 1> u;

        //OSQP Matrices
        OSQPCscMatrix osqp_csc_A;
        OSQPCscMatrix osqp_csc_P;
        OSQPFloat *osqp_q;
        OSQPFloat *osqp_l;
        OSQPFloat *osqp_u;
        
        //OSQP Data structures
        OSQPSolver* osqp_solver = NULL;
        OSQPSettings* osqp_settings = NULL;
        OSQPInt osqp_status;
    public:
        ForceSolver(Scene* scene, int nb_contacts_per_object_pair = 10, int nb_coulomb_polygon_sides = 8);
        ~ForceSolver();
        //Update the OSQP bound vectors and re-solve the force distribution.
        void set_acceleration(Vector3f acceleration);
        std::vector<ContactForce> get_contact_forces();
        enum result_status {UNSOLVED, STABLE, UNSTABLE, FAILURE};
        result_status get_status();
};