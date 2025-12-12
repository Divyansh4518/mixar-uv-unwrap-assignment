/**
 * @file packing.cpp
 * @brief UV island packing into [0,1]² texture space
 *
 * SKELETON - YOU IMPLEMENT THIS
 *
 * Algorithm: Shelf packing
 * 1. Compute bounding box for each island
 * 2. Sort islands by height (descending)
 * 3. Pack using shelf algorithm
 * 4. Scale to fit [0,1]²
 */

#include "unwrap.h"
#include "math_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <algorithm>

/**
 * @brief Island bounding box info
 */
struct Island {
    int id;
    float min_u, max_u, min_v, max_v;
    float width, height;
    float target_x, target_y;  // Packed position
    std::vector<int> vertex_indices;
};

void pack_uv_islands(Mesh* mesh,
                     const UnwrapResult* result,
                     float margin) {
    if (!mesh || !result || !mesh->uvs) return;

    if (result->num_islands <= 1) {
        // Single island, already normalized to [0,1]
        return;
    }

    printf("Packing %d islands...\n", result->num_islands);

    std::vector<Island> islands(result->num_islands);

    // STEP 1: Compute bounding box for each island
    for (int i = 0; i < result->num_islands; i++) {
        islands[i].id = i;
        islands[i].min_u = FLT_MAX;
        islands[i].max_u = -FLT_MAX;
        islands[i].min_v = FLT_MAX;
        islands[i].max_v = -FLT_MAX;
    }

    // Find all vertices in each island
    for (int face_idx = 0; face_idx < mesh->num_triangles; face_idx++) {
        int island_id = result->face_island_ids[face_idx];
        if (island_id < 0 || island_id >= result->num_islands) continue;

        for (int j = 0; j < 3; j++) {
            int v_idx = mesh->triangles[face_idx * 3 + j];
            float u = mesh->uvs[v_idx * 2 + 0];
            float v = mesh->uvs[v_idx * 2 + 1];

            islands[island_id].min_u = min_float(islands[island_id].min_u, u);
            islands[island_id].max_u = max_float(islands[island_id].max_u, u);
            islands[island_id].min_v = min_float(islands[island_id].min_v, v);
            islands[island_id].max_v = max_float(islands[island_id].max_v, v);
            
            // Track vertex for later offset application
            if (std::find(islands[island_id].vertex_indices.begin(), 
                         islands[island_id].vertex_indices.end(), 
                         v_idx) == islands[island_id].vertex_indices.end()) {
                islands[island_id].vertex_indices.push_back(v_idx);
            }
        }
    }

    // Compute widths and heights
    for (int i = 0; i < result->num_islands; i++) {
        islands[i].width = islands[i].max_u - islands[i].min_u;
        islands[i].height = islands[i].max_v - islands[i].min_v;
    }

    // STEP 2: Sort by height (descending)
    std::sort(islands.begin(), islands.end(), 
              [](const Island& a, const Island& b) { return a.height > b.height; });

    // STEP 3: Shelf packing
    struct Shelf {
        float x;
        float y;
        float height;
        float width_used;
    };

    std::vector<Shelf> shelves;
    shelves.push_back({0.0f, 0.0f, islands[0].height, 0.0f});

    for (size_t i = 0; i < islands.size(); i++) {
        Island& island = islands[i];
        bool placed = false;

        // Try to fit in existing shelves
        for (auto& shelf : shelves) {
            if (shelf.width_used + island.width + margin <= 1.0f && 
                island.height <= shelf.height) {
                // Place in this shelf
                island.target_x = shelf.width_used;
                island.target_y = shelf.y;
                shelf.width_used += island.width + margin;
                placed = true;
                break;
            }
        }

        if (!placed) {
            // Create new shelf
            float new_y = shelves.back().y + shelves.back().height + margin;
            island.target_x = 0.0f;
            island.target_y = new_y;
            shelves.push_back({0.0f, new_y, island.height, island.width + margin});
        }
    }

    // STEP 4: Move islands to packed positions
    for (const Island& island : islands) {
        float offset_x = island.target_x - island.min_u;
        float offset_y = island.target_y - island.min_v;

        for (int v_idx : island.vertex_indices) {
            mesh->uvs[v_idx * 2 + 0] += offset_x;
            mesh->uvs[v_idx * 2 + 1] += offset_y;
        }
    }

    // STEP 5: Scale everything to fit [0,1]²
    float max_u = 0.0f, max_v = 0.0f;
    
    for (int v = 0; v < mesh->num_vertices; v++) {
        max_u = max_float(max_u, mesh->uvs[v * 2 + 0]);
        max_v = max_float(max_v, mesh->uvs[v * 2 + 1]);
    }

    float scale = 1.0f / max_float(max_u, max_v);
    if (scale < 1.0f) {
        for (int v = 0; v < mesh->num_vertices; v++) {
            mesh->uvs[v * 2 + 0] *= scale;
            mesh->uvs[v * 2 + 1] *= scale;
        }
    }

    printf("  Packing completed\n");
}

void compute_quality_metrics(const Mesh* mesh, UnwrapResult* result) {
    if (!mesh || !result || !mesh->uvs) return;

    // TODO: Implement quality metrics computation
    //
    // NOTE: This function is OPTIONAL for Part 1.
    // You will implement full metrics in Part 2 (Python).
    // For Part 1, you can either:
    //   (A) Leave these as defaults (tests will still pass)
    //   (B) Implement basic estimation for testing
    //
    // ALGORITHM (see reference/algorithms.md and part2_python/reference/metrics_spec.md):
    //
    // STRETCH METRIC (SVD-based):
    //   For each triangle:
    //     1. Build Jacobian matrix J (3x2): maps UV space to 3D space
    //        J = [dp/du, dp/dv] where p is 3D position
    //     2. Compute J^T * J (2x2 Gramian matrix)
    //     3. Find eigenvalues λ1, λ2 of J^T * J
    //     4. Singular values: σ1 = sqrt(λ1), σ2 = sqrt(λ2)
    //     5. Stretch = σ1 / σ2 (ratio of max/min stretching)
    //   Average and max stretch across all triangles
    //
    // COVERAGE METRIC (Rasterization-based):
    //   1. Create 1024x1024 bitmap of [0,1]² UV space
    //   2. Rasterize all UV triangles
    //   3. Coverage = (pixels_filled / total_pixels)
    //   Alternative: Use bounding box as approximation
    //
    // EXPECTED RESULTS:
    //   - Good unwrap: avg_stretch < 1.5, max_stretch < 2.0
    //   - Shelf packing: coverage > 0.60 (60%)
    //   - MaxRects packing: coverage > 0.75 (75%)

    // Default values (replace with your implementation)
    result->avg_stretch = 1.0f;
    result->max_stretch = 1.0f;
    result->coverage = 0.7f;

    printf("Quality metrics: (using defaults - implement for accurate values)\n");
    printf("  Avg stretch: %.2f\n", result->avg_stretch);
    printf("  Max stretch: %.2f\n", result->max_stretch);
    printf("  Coverage: %.1f%%\n", result->coverage * 100);
}
