/**
 * @file unwrap.cpp
 * @brief Main UV unwrapping orchestrator
 *
 * SKELETON - YOU IMPLEMENT THIS
 *
 * This file ties together all the components:
 * - Topology building
 * - Seam detection
 * - Island extraction
 * - LSCM parameterization
 * - Island packing
 */

#include "unwrap.h"
#include "lscm.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <set>
#include <map>
#include <queue>

/**
 * @brief Extract UV islands after seam cuts
 *
 * Uses connected components algorithm on face graph after removing seam edges.
 *
 * @param mesh Input mesh
 * @param topo Topology info
 * @param seam_edges Array of seam edge indices
 * @param num_seams Number of seams
 * @param num_islands_out Output: number of islands
 * @return Array of island IDs per face
 */
static int* extract_islands(const Mesh* mesh,
                           const TopologyInfo* topo,
                           const int* seam_edges,
                           int num_seams,
                           int* num_islands_out) {
    // Create set of seam edge indices for fast lookup
    std::set<int> seam_set;
    for (int i = 0; i < num_seams; i++) {
        seam_set.insert(seam_edges[i]);
    }

    // Build face adjacency graph (only through non-seam edges)
    std::vector<std::vector<int>> face_adjacency(mesh->num_triangles);
    
    for (int edge_idx = 0; edge_idx < topo->num_edges; edge_idx++) {
        // Skip seam edges
        if (seam_set.find(edge_idx) != seam_set.end()) {
            continue;
        }

        int face0 = topo->edge_faces[edge_idx * 2 + 0];
        int face1 = topo->edge_faces[edge_idx * 2 + 1];

        // Only interior edges
        if (face0 != -1 && face1 != -1) {
            face_adjacency[face0].push_back(face1);
            face_adjacency[face1].push_back(face0);
        }
    }

    // Run BFS to find connected components
    int* face_island_ids = (int*)malloc(mesh->num_triangles * sizeof(int));
    for (int i = 0; i < mesh->num_triangles; i++) {
        face_island_ids[i] = -1;
    }

    int current_island = 0;
    std::queue<int> bfs_queue;

    for (int start_face = 0; start_face < mesh->num_triangles; start_face++) {
        if (face_island_ids[start_face] != -1) continue;

        // Start new island
        bfs_queue.push(start_face);
        face_island_ids[start_face] = current_island;

        while (!bfs_queue.empty()) {
            int face = bfs_queue.front();
            bfs_queue.pop();

            // Visit all adjacent faces
            for (int neighbor : face_adjacency[face]) {
                if (face_island_ids[neighbor] == -1) {
                    face_island_ids[neighbor] = current_island;
                    bfs_queue.push(neighbor);
                }
            }
        }

        current_island++;
    }

    *num_islands_out = current_island;

    printf("Extracted %d UV islands\n", *num_islands_out);

    return face_island_ids;
}

/**
 * @brief Copy UVs from island parameterization to result mesh
 */
static void copy_island_uvs(Mesh* result,
                           const float* island_uvs,
                           const int* face_indices,
                           int num_faces,
                           const std::map<int, int>& global_to_local) {
    // Copy UVs from island parameterization to result mesh
    for (int i = 0; i < num_faces; i++) {
        int face_idx = face_indices[i];
        
        for (int j = 0; j < 3; j++) {
            int global_idx = result->triangles[face_idx * 3 + j];
            
            auto it = global_to_local.find(global_idx);
            if (it != global_to_local.end()) {
                int local_idx = it->second;
                result->uvs[global_idx * 2 + 0] = island_uvs[local_idx * 2 + 0];
                result->uvs[global_idx * 2 + 1] = island_uvs[local_idx * 2 + 1];
            }
        }
    }
}

Mesh* unwrap_mesh(const Mesh* mesh,
                  const UnwrapParams* params,
                  UnwrapResult** result_out) {
    if (!mesh || !params || !result_out) {
        fprintf(stderr, "unwrap_mesh: Invalid arguments\n");
        return NULL;
    }

    printf("\n=== UV Unwrapping ===\n");
    printf("Input: %d vertices, %d triangles\n",
           mesh->num_vertices, mesh->num_triangles);
    printf("Parameters:\n");
    printf("  Angle threshold: %.1f°\n", params->angle_threshold);
    printf("  Min island faces: %d\n", params->min_island_faces);
    printf("  Pack islands: %s\n", params->pack_islands ? "yes" : "no");
    printf("  Island margin: %.3f\n", params->island_margin);
    printf("\n");

    // TODO: Implement main unwrapping pipeline
    //
    // STEP 1: Build topology
    TopologyInfo* topo = build_topology(mesh);
    if (!topo) {
        fprintf(stderr, "Failed to build topology\n");
        return NULL;
    }
    validate_topology(mesh, topo);

    // STEP 2: Detect seams
    int num_seams;
    int* seam_edges = detect_seams(mesh, topo, params->angle_threshold, &num_seams);

    // STEP 3: Extract islands
    int num_islands;
    int* face_island_ids = extract_islands(mesh, topo, seam_edges, num_seams, &num_islands);

    // STEP 4: Parameterize each island using LSCM
    Mesh* result = allocate_mesh_copy(mesh);
    result->uvs = (float*)calloc(mesh->num_vertices * 2, sizeof(float));

    for (int island_id = 0; island_id < num_islands; island_id++) {
        printf("\nProcessing island %d/%d...\n", island_id + 1, num_islands);

        // Get faces in this island
        std::vector<int> island_faces;
        for (int f = 0; f < mesh->num_triangles; f++) {
            if (face_island_ids[f] == island_id) {
                island_faces.push_back(f);
            }
        }

        printf("  %d faces in island\n", (int)island_faces.size());

        if (island_faces.size() < params->min_island_faces) {
            printf("  Skipping (too small)\n");
            continue;
        }

        // Parameterize this island using LSCM
        float* island_uvs = lscm_parameterize(mesh, island_faces.data(), island_faces.size());
        
        if (!island_uvs) {
            printf("  LSCM failed for island\n");
            continue;
        }

        // Build global_to_local mapping for this island
        std::map<int, int> global_to_local;
        for (size_t i = 0; i < island_faces.size(); i++) {
            int face_idx = island_faces[i];
            for (int j = 0; j < 3; j++) {
                int global_idx = mesh->triangles[face_idx * 3 + j];
                if (global_to_local.find(global_idx) == global_to_local.end()) {
                    global_to_local[global_idx] = global_to_local.size();
                }
            }
        }

        // Copy UVs to result mesh
        copy_island_uvs(result, island_uvs, island_faces.data(), island_faces.size(), global_to_local);
        
        free(island_uvs);
    }

    // STEP 5: Pack islands if requested
    if (params->pack_islands) {
        UnwrapResult temp_result;
        temp_result.num_islands = num_islands;
        temp_result.face_island_ids = face_island_ids;

        pack_uv_islands(result, &temp_result, params->island_margin);
    }

    // STEP 6: Compute quality metrics
    UnwrapResult* result_data = (UnwrapResult*)malloc(sizeof(UnwrapResult));
    result_data->num_islands = num_islands;
    result_data->face_island_ids = face_island_ids;
    compute_quality_metrics(result, result_data);

    *result_out = result_data;

    // Cleanup
    free_topology(topo);
    free(seam_edges);

    printf("\n=== Unwrapping Complete ===\n");

    return result;
}

void free_unwrap_result(UnwrapResult* result) {
    if (!result) return;

    if (result->face_island_ids) {
        free(result->face_island_ids);
    }
    free(result);
}
