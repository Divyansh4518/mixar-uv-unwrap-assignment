/**
 * @file topology.cpp
 * @brief Topology builder implementation
 *
 * SKELETON - YOU IMPLEMENT THIS
 *
 * Algorithm:
 * 1. Extract all edges from triangles
 * 2. Ensure uniqueness (always store as v0 < v1)
 * 3. For each edge, find adjacent faces
 * 4. Validate using Euler characteristic
 */

#include "topology.h"
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <vector>

/**
 * @brief Edge structure for uniqueness
 */
struct Edge {
    int v0, v1;

    Edge(int a, int b) {
        // Always store smaller vertex first
        if (a < b) {
            v0 = a;
            v1 = b;
        } else {
            v0 = b;
            v1 = a;
        }
    }

    bool operator<(const Edge& other) const {
        if (v0 != other.v0) return v0 < other.v0;
        return v1 < other.v1;
    }
};

/**
 * @brief Edge information
 */
struct EdgeInfo {
    int face0;
    int face1;

    EdgeInfo() : face0(-1), face1(-1) {}
};

TopologyInfo* build_topology(const Mesh* mesh) {
    if (!mesh) return NULL;

    // STEP 1: Create map to collect edges
    std::map<Edge, EdgeInfo> edge_map;

    // STEP 2: Iterate through all triangles and extract edges
    for (int face_idx = 0; face_idx < mesh->num_triangles; face_idx++) {
        // Get the three vertices of this triangle
        int v0 = mesh->triangles[face_idx * 3 + 0];
        int v1 = mesh->triangles[face_idx * 3 + 1];
        int v2 = mesh->triangles[face_idx * 3 + 2];

        // Extract three edges from the triangle
        Edge edges[3] = {
            Edge(v0, v1),
            Edge(v1, v2),
            Edge(v2, v0)
        };

        // Add each edge to the map and track adjacent faces
        for (int i = 0; i < 3; i++) {
            Edge& edge = edges[i];
            
            // Get or create edge info
            EdgeInfo& info = edge_map[edge];
            
            // Add this face as adjacent to the edge
            if (info.face0 == -1) {
                info.face0 = face_idx;
            } else if (info.face1 == -1) {
                info.face1 = face_idx;
            } else {
                // This edge has more than 2 adjacent faces (non-manifold mesh)
                fprintf(stderr, "Warning: Non-manifold edge detected (v%d-v%d has >2 faces)\n",
                        edge.v0, edge.v1);
            }
        }
    }

    // STEP 3: Convert map to arrays
    TopologyInfo* topo = (TopologyInfo*)malloc(sizeof(TopologyInfo));
    topo->num_edges = edge_map.size();
    
    // Allocate arrays
    topo->edges = (int*)malloc(topo->num_edges * 2 * sizeof(int));
    topo->edge_faces = (int*)malloc(topo->num_edges * 2 * sizeof(int));

    // Fill arrays from map
    int idx = 0;
    for (const auto& pair : edge_map) {
        const Edge& edge = pair.first;
        const EdgeInfo& info = pair.second;

        // Store edge vertices
        topo->edges[idx * 2 + 0] = edge.v0;
        topo->edges[idx * 2 + 1] = edge.v1;

        // Store adjacent faces
        topo->edge_faces[idx * 2 + 0] = info.face0;
        topo->edge_faces[idx * 2 + 1] = info.face1;

        idx++;
    }

    printf("Built topology: %d edges\n", topo->num_edges);

    return topo;
}

void free_topology(TopologyInfo* topo) {
    if (!topo) return;

    if (topo->edges) free(topo->edges);
    if (topo->edge_faces) free(topo->edge_faces);
    free(topo);
}

int validate_topology(const Mesh* mesh, const TopologyInfo* topo) {
    if (!mesh || !topo) return 0;

    int V = mesh->num_vertices;
    int E = topo->num_edges;
    int F = mesh->num_triangles;

    int euler = V - E + F;

    printf("Topology validation:\n");
    printf("  V=%d, E=%d, F=%d\n", V, E, F);
    printf("  Euler characteristic: %d (expected 2 for closed mesh)\n", euler);

    // Closed meshes should have Euler = 2
    // Open meshes or meshes with holes may differ
    if (euler != 2) {
        printf("  Warning: Non-standard Euler characteristic\n");
        printf("  (This may be OK for open meshes or meshes with boundaries)\n");
    }

    return 1;
}
