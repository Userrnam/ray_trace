#include "data_structures.hcl"
#include "random.hcl"

#define WORLD_COMPONENT(type, name) __global type* name, name##_count

#define VEC3(arr) (float3)(arr[0], arr[1], arr[2])

Ray get_ray(Camera* camera, int x, int y) {
    float v = ((float) y / (float) camera->height) * 2.0f - 1.0f;  // [-1; 1]
    float u = ((float) x / (float) camera->width) * 2.0f - 1.0f;   // [-1; 1]
    u += 0.5/camera->width  * rand();
    v += 0.5/camera->height * rand();

    Ray ray;
    ray.origin = VEC3(camera->pos);
    ray.dir = normalize(VEC3(camera->center) + u * VEC3(camera->horizontal) + v * VEC3(camera->vertical) - VEC3(camera->pos));
    ray.invdir = 1.0f / ray.dir;
    return ray;
}

// fill 1 row.
__kernel void main(__global float4* out, Camera camera, 
    // world
    WORLD_COMPONENT(Material, materials),
    WORLD_COMPONENT(int, mesh_indices),
    WORLD_COMPONENT(BVH_Node, bvh_nodes),
    WORLD_COMPONENT(int, triangle_indices),

    WORLD_COMPONENT(int, vertex_indices),
    WORLD_COMPONENT(int, normal_indices),

    WORLD_COMPONENT(float4, vertices),
    WORLD_COMPONENT(float4, normals),

    WORLD_COMPONENT(Mesh, meshes)
) {
    int gid = get_global_id(0);
    int start = gid * camera.width;

    seed(53256 * gid + 660253 * sin(gid * 52366.0f));

    for (int i = 0; i < camera.width; ++i) {
        int y = gid;
        int x = i;

        float u = -1.0f + (float)x / (float)camera.width * 2.0f;
        float v = -1.0f + (float)y / (float)get_global_size(0) * 2.0f;

        int k = 0;
        if (u*u + v*v < 0.5) {
            if (probability_value(u * v)) {
                k = 1;
            }
        } else {
            k = 2;
        }
        out[start + i] = (float4)(materials[k].color[0], materials[k].color[1], materials[k].color[2], 1.0f);
    }
}

#include "random.cl"
