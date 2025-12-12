/**
 * @file seam_detection.cpp
 * @brief Seam detection using spanning tree + angular defect
 *
 * SKELETON - YOU IMPLEMENT THIS
 *
 * Algorithm:
 * 1. Build dual graph (faces as nodes, shared edges as edges)
 * 2. Compute spanning tree via BFS
 * 3. Mark non-tree edges as seam candidates
 * 4. Refine using angular defect
 *
 * See reference/algorithms.md for detailed description
 */

#include "unwrap.h"
#include "math_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <set>
#include <queue>
#include <map>

/**
 * @brief Compute angular defect at a vertex
 *
 * Angular defect = 2π - sum of angles at vertex
 *
 * - Flat surface: defect ≈ 0
 * - Corner (like cube): defect > 0
 * - Saddle: defect < 0
 *
 * @param mesh Input mesh
 * @param vertex_idx Vertex index
 * @return Angular defect in radians
 */
static float compute_angular_defect(const Mesh* mesh, int vertex_idx) {
    float angle_sum = 0.0f;

    // Find all triangles containing this vertex
    for (int tri_idx = 0; tri_idx < mesh->num_triangles; tri_idx++) {
        int v0 = mesh->triangles[tri_idx * 3 + 0];
        int v1 = mesh->triangles[tri_idx * 3 + 1];
        int v2 = mesh->triangles[tri_idx * 3 + 2];

        // Check if this triangle contains the vertex
        if (v0 == vertex_idx || v1 == vertex_idx || v2 == vertex_idx) {
            // Compute angle at this vertex in this triangle
            float angle = compute_vertex_angle_in_triangle(mesh, tri_idx, vertex_idx);
            angle_sum += angle;
        }
    }

    // Angular defect = 2π - sum of angles
    return 2.0f * M_PI - angle_sum;
}

/**
 * @brief Get all edges incident to a vertex
 */
static std::vector<int> get_vertex_edges(const TopologyInfo* topo, int vertex_idx) {
    std::vector<int> edges;

    // Iterate through all edges, add those touching vertex_idx
    for (int edge_idx = 0; edge_idx < topo->num_edges; edge_idx++) {
        int v0 = topo->edges[edge_idx * 2 + 0];
        int v1 = topo->edges[edge_idx * 2 + 1];

        if (v0 == vertex_idx || v1 == vertex_idx) {
            edges.push_back(edge_idx);
        }
    }

    return edges;
}

int* detect_seams(const Mesh* mesh,
                  const TopologyInfo* topo,
                  float angle_threshold,
                  int* num_seams_out) {
    if (!mesh || !topo || !num_seams_out) return NULL;

    // STEP 1: Build dual graph adjacency list
    // Map each edge to its index for quick lookup
    std::map<std::pair<int, int>, int> edge_index_map;
    for (int edge_idx = 0; edge_idx < topo->num_edges; edge_idx++) {
        int v0 = topo->edges[edge_idx * 2 + 0];
        int v1 = topo->edges[edge_idx * 2 + 1];
        edge_index_map[std::make_pair(v0, v1)] = edge_idx;
    }

    // Build dual graph: faces as nodes, shared edges as connections
    std::vector<std::vector<std::pair<int, int>>> dual_graph(mesh->num_triangles);
    
    for (int edge_idx = 0; edge_idx < topo->num_edges; edge_idx++) {
        int face0 = topo->edge_faces[edge_idx * 2 + 0];
        int face1 = topo->edge_faces[edge_idx * 2 + 1];

        // Only interior edges (with 2 adjacent faces)
        if (face0 != -1 && face1 != -1) {
            dual_graph[face0].push_back(std::make_pair(face1, edge_idx));
            dual_graph[face1].push_back(std::make_pair(face0, edge_idx));
        }
    }

    // STEP 2: Spanning tree via BFS
    std::set<int> tree_edges;
    std::vector<bool> visited(mesh->num_triangles, false);
    std::queue<int> bfs_queue;

    // Start from face 0
    if (mesh->num_triangles > 0) {
        bfs_queue.push(0);
        visited[0] = true;

        while (!bfs_queue.empty()) {
            int current_face = bfs_queue.front();
            bfs_queue.pop();

            // Visit all neighbors
            for (const auto& neighbor_pair : dual_graph[current_face]) {
                int neighbor_face = neighbor_pair.first;
                int edge_idx = neighbor_pair.second;

                if (!visited[neighbor_face]) {
                    visited[neighbor_face] = true;
                    tree_edges.insert(edge_idx);
                    bfs_queue.push(neighbor_face);
                }
            }
        }
    }

    // STEP 3: Initial seam candidates = non-tree edges (interior edges only)
    std::set<int> seam_candidates;
    
    for (int edge_idx = 0; edge_idx < topo->num_edges; edge_idx++) {
        int face0 = topo->edge_faces[edge_idx * 2 + 0];
        int face1 = topo->edge_faces[edge_idx * 2 + 1];

        // Interior edges not in spanning tree are seam candidates
        if (face0 != -1 && face1 != -1 && tree_edges.find(edge_idx) == tree_edges.end()) {
            seam_candidates.insert(edge_idx);
        }
    }

    // STEP 4: Angular defect refinement (optional - be conservative)
    // Only add edges for vertices with VERY high curvature
    // Use a higher threshold to avoid adding too many seams
    
    for (int v = 0; v < mesh->num_vertices; v++) {
        float defect = compute_angular_defect(mesh, v);

        // Very high curvature vertex (corners, sharp features)
        // Use higher threshold (1.0 radians ≈ 57 degrees) to be more selective
        if (fabs(defect) > 1.0f) {
            std::vector<int> incident_edges = get_vertex_edges(topo, v);
            
            for (int edge_idx : incident_edges) {
                int face0 = topo->edge_faces[edge_idx * 2 + 0];
                int face1 = topo->edge_faces[edge_idx * 2 + 1];

                // Only consider interior edges not in spanning tree
                if (face0 != -1 && face1 != -1 && tree_edges.find(edge_idx) == tree_edges.end()) {
                    seam_candidates.insert(edge_idx);
                }
            }
        }
    }

    // STEP 5: Convert seam candidates to array
    *num_seams_out = seam_candidates.size();
    int* seams = (int*)malloc(*num_seams_out * sizeof(int));

    int idx = 0;
    for (int edge_idx : seam_candidates) {
        seams[idx++] = edge_idx;
    }

    printf("Detected %d seams\n", *num_seams_out);

    return seams;
}
