/**
 * @file lscm.cpp
 * @brief LSCM (Least Squares Conformal Maps) parameterization
 *
 * SKELETON - YOU IMPLEMENT THIS
 *
 * This is the MOST COMPLEX part of the assignment.
 *
 * IMPORTANT:
 * - See reference/lscm_matrix_example.cpp for matrix assembly example
 * - See reference/algorithms.md for mathematical background
 * - Use Eigen library for sparse linear algebra
 *
 * Algorithm:
 * 1. Build local vertex mapping (global → local indices)
 * 2. Assemble LSCM sparse matrix
 * 3. Set boundary conditions (pin 2 vertices)
 * 4. Solve sparse linear system
 * 5. Normalize UVs to [0,1]²
 */

#include "lscm.h"
#include "math_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <map>
#include <vector>
#include <set>

// Eigen library for sparse matrices
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
// Alternative: #include <Eigen/IterativeLinearSolvers>

int find_boundary_vertices(const Mesh* mesh,
                          const int* face_indices,
                          int num_faces,
                          int** boundary_out) {
    // Edge structure for counting
    struct Edge {
        int v0, v1;
        Edge(int a, int b) {
            if (a < b) { v0 = a; v1 = b; }
            else { v0 = b; v1 = a; }
        }
        bool operator<(const Edge& other) const {
            if (v0 != other.v0) return v0 < other.v0;
            return v1 < other.v1;
        }
    };

    // Count how many times each edge appears
    std::map<Edge, int> edge_count;

    for (int i = 0; i < num_faces; i++) {
        int face_idx = face_indices[i];
        int v0 = mesh->triangles[face_idx * 3 + 0];
        int v1 = mesh->triangles[face_idx * 3 + 1];
        int v2 = mesh->triangles[face_idx * 3 + 2];

        edge_count[Edge(v0, v1)]++;
        edge_count[Edge(v1, v2)]++;
        edge_count[Edge(v2, v0)]++;
    }

    // Boundary edges appear only once
    std::set<int> boundary_verts;
    for (const auto& pair : edge_count) {
        if (pair.second == 1) {
            boundary_verts.insert(pair.first.v0);
            boundary_verts.insert(pair.first.v1);
        }
    }

    // Convert to array
    int num_boundary = boundary_verts.size();
    *boundary_out = (int*)malloc(num_boundary * sizeof(int));

    int idx = 0;
    for (int v : boundary_verts) {
        (*boundary_out)[idx++] = v;
    }

    return num_boundary;
}

void normalize_uvs_to_unit_square(float* uvs, int num_verts) {
    if (!uvs || num_verts == 0) return;

    // Find bounding box
    float min_u = FLT_MAX, max_u = -FLT_MAX;
    float min_v = FLT_MAX, max_v = -FLT_MAX;

    for (int i = 0; i < num_verts; i++) {
        float u = uvs[i * 2];
        float v = uvs[i * 2 + 1];

        min_u = min_float(min_u, u);
        max_u = max_float(max_u, u);
        min_v = min_float(min_v, v);
        max_v = max_float(max_v, v);
    }

    float u_range = max_u - min_u;
    float v_range = max_v - min_v;

    if (u_range < 1e-6f) u_range = 1.0f;
    if (v_range < 1e-6f) v_range = 1.0f;

    // Normalize to [0, 1]
    for (int i = 0; i < num_verts; i++) {
        uvs[i * 2] = (uvs[i * 2] - min_u) / u_range;
        uvs[i * 2 + 1] = (uvs[i * 2 + 1] - min_v) / v_range;
    }
}

float* lscm_parameterize(const Mesh* mesh,
                         const int* face_indices,
                         int num_faces) {
    if (!mesh || !face_indices || num_faces == 0) return NULL;

    printf("LSCM parameterizing %d faces...\n", num_faces);

    // STEP 1: Build local vertex mapping
    std::map<int, int> global_to_local;
    std::vector<int> local_to_global;

    for (int i = 0; i < num_faces; i++) {
        int face_idx = face_indices[i];
        for (int j = 0; j < 3; j++) {
            int global_idx = mesh->triangles[face_idx * 3 + j];
            if (global_to_local.find(global_idx) == global_to_local.end()) {
                int local_idx = local_to_global.size();
                global_to_local[global_idx] = local_idx;
                local_to_global.push_back(global_idx);
            }
        }
    }

    int n = local_to_global.size();
    printf("  Island has %d vertices\n", n);

    if (n < 3) {
        fprintf(stderr, "LSCM: Island too small (%d vertices)\n", n);
        return NULL;
    }

    // STEP 2: Build LSCM sparse matrix
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets;

    for (int i = 0; i < num_faces; i++) {
        int face_idx = face_indices[i];
        
        // Get global vertex indices
        int gv0 = mesh->triangles[face_idx * 3 + 0];
        int gv1 = mesh->triangles[face_idx * 3 + 1];
        int gv2 = mesh->triangles[face_idx * 3 + 2];

        // Convert to local indices
        int v0 = global_to_local[gv0];
        int v1 = global_to_local[gv1];
        int v2 = global_to_local[gv2];

        // Get 3D positions
        Vec3 p0 = get_vertex_position(mesh, gv0);
        Vec3 p1 = get_vertex_position(mesh, gv1);
        Vec3 p2 = get_vertex_position(mesh, gv2);

        // Project triangle to its plane
        Vec3 e1 = vec3_sub(p1, p0);
        Vec3 e2 = vec3_sub(p2, p0);

        Vec3 normal = vec3_normalize(vec3_cross(e1, e2));
        Vec3 u_axis = vec3_normalize(e1);
        Vec3 v_axis = vec3_cross(normal, u_axis);

        // Local 2D coordinates
        double q0_x = 0.0, q0_y = 0.0;
        double q1_x = vec3_dot(e1, u_axis);
        double q1_y = vec3_dot(e1, v_axis);
        double q2_x = vec3_dot(e2, u_axis);
        double q2_y = vec3_dot(e2, v_axis);

        // Triangle area
        double area = 0.5 * fabs(q1_x * q2_y - q1_y * q2_x);
        
        if (area < 1e-10) continue; // Skip degenerate triangles

        // Add LSCM energy terms for each edge (from reference example)
        
        // Edge v0 → v1
        double dx = q1_x - q0_x;
        double dy = q1_y - q0_y;
        triplets.push_back(T(2*v0,   2*v1,    area * dx));
        triplets.push_back(T(2*v0,   2*v1+1,  area * dy));
        triplets.push_back(T(2*v0+1, 2*v1,    area * dy));
        triplets.push_back(T(2*v0+1, 2*v1+1,  area * (-dx)));
        triplets.push_back(T(2*v0,   2*v0,    -area * dx));
        triplets.push_back(T(2*v0,   2*v0+1,  -area * dy));
        triplets.push_back(T(2*v0+1, 2*v0,    -area * dy));
        triplets.push_back(T(2*v0+1, 2*v0+1,  -area * (-dx)));

        // Edge v1 → v2
        dx = q2_x - q1_x;
        dy = q2_y - q1_y;
        triplets.push_back(T(2*v1,   2*v2,    area * dx));
        triplets.push_back(T(2*v1,   2*v2+1,  area * dy));
        triplets.push_back(T(2*v1+1, 2*v2,    area * dy));
        triplets.push_back(T(2*v1+1, 2*v2+1,  area * (-dx)));
        triplets.push_back(T(2*v1,   2*v1,    -area * dx));
        triplets.push_back(T(2*v1,   2*v1+1,  -area * dy));
        triplets.push_back(T(2*v1+1, 2*v1,    -area * dy));
        triplets.push_back(T(2*v1+1, 2*v1+1,  -area * (-dx)));

        // Edge v2 → v0
        dx = q0_x - q2_x;
        dy = q0_y - q2_y;
        triplets.push_back(T(2*v2,   2*v0,    area * dx));
        triplets.push_back(T(2*v2,   2*v0+1,  area * dy));
        triplets.push_back(T(2*v2+1, 2*v0,    area * dy));
        triplets.push_back(T(2*v2+1, 2*v0+1,  area * (-dx)));
        triplets.push_back(T(2*v2,   2*v2,    -area * dx));
        triplets.push_back(T(2*v2,   2*v2+1,  -area * dy));
        triplets.push_back(T(2*v2+1, 2*v2,    -area * dy));
        triplets.push_back(T(2*v2+1, 2*v2+1,  -area * (-dx)));
    }

    // STEP 3: Set boundary conditions - find 2 vertices to pin
    int* boundary = NULL;
    int num_boundary = find_boundary_vertices(mesh, face_indices, num_faces, &boundary);

    int pin1 = -1, pin2 = -1;

    if (num_boundary >= 2) {
        // Find two boundary vertices far apart
        pin1 = global_to_local[boundary[0]];
        pin2 = global_to_local[boundary[num_boundary / 2]];
    } else {
        // Closed surface or single vertex island - pick any two vertices
        pin1 = 0;
        pin2 = n > 1 ? n / 2 : 0;
    }

    if (boundary) free(boundary);

    // Pin vertices by clearing rows and setting diagonal
    for (auto& triplet : triplets) {
        // Clear rows for pinned vertices
        if (triplet.row() == 2*pin1 || triplet.row() == 2*pin1+1 ||
            triplet.row() == 2*pin2 || triplet.row() == 2*pin2+1) {
            triplet = T(triplet.row(), triplet.col(), 0.0);
        }
    }

    // Add diagonal entries for pinned vertices
    triplets.push_back(T(2*pin1,   2*pin1,   1.0));
    triplets.push_back(T(2*pin1+1, 2*pin1+1, 1.0));
    triplets.push_back(T(2*pin2,   2*pin2,   1.0));
    triplets.push_back(T(2*pin2+1, 2*pin2+1, 1.0));

    // STEP 4: Solve sparse linear system
    Eigen::SparseMatrix<double> A(2*n, 2*n);
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd b = Eigen::VectorXd::Zero(2*n);
    // Set pinned positions
    b(2*pin1) = 0.0;
    b(2*pin1+1) = 0.0;
    b(2*pin2) = 1.0;
    b(2*pin2+1) = 0.0;

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    
    if (solver.info() != Eigen::Success) {
        fprintf(stderr, "LSCM: Matrix factorization failed\n");
        return NULL;
    }

    Eigen::VectorXd x = solver.solve(b);

    if (solver.info() != Eigen::Success) {
        fprintf(stderr, "LSCM: Solving failed\n");
        return NULL;
    }

    // STEP 5: Extract UVs
    float* uvs = (float*)malloc(n * 2 * sizeof(float));

    for (int i = 0; i < n; i++) {
        uvs[i * 2 + 0] = (float)x(2*i);
        uvs[i * 2 + 1] = (float)x(2*i + 1);
    }

    normalize_uvs_to_unit_square(uvs, n);

    printf("  LSCM completed\n");
    return uvs;
}
