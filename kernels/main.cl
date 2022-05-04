#include "data_structures.hcl"
#include "random.hcl"
#include "ray_trace.hcl"

#define WORLD_COMPONENT(type, name) __global type* name, name##_count
#define SET_WORLD_COMPONENT(name) world.name.data = name; world.name.count = name##_count

Ray get_ray(Camera* camera, int x, int y) {
    float v = ((float) y / (float) camera->height) * 2.0f - 1.0f;  // [-1; 1]
    float u = ((float) x / (float) camera->width)  * 2.0f - 1.0f;  // [-1; 1]
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

    WORLD_COMPONENT(Mesh, meshes),

    int bounce_count, int iteration, uint _seed, __global float4 *sum
) {
    int gid = get_global_id(0);
    int start = gid * camera.width;

    float dummy;
    seed((gid + 5900257) * fract(sin(_seed * 523062147.0f), &dummy));

    World world;
    SET_WORLD_COMPONENT(materials);
    SET_WORLD_COMPONENT(mesh_indices);
    SET_WORLD_COMPONENT(bvh_nodes);
    SET_WORLD_COMPONENT(triangle_indices);

    SET_WORLD_COMPONENT(vertex_indices);
    SET_WORLD_COMPONENT(normal_indices);

    SET_WORLD_COMPONENT(vertices);
    SET_WORLD_COMPONENT(normals);

    SET_WORLD_COMPONENT(meshes);

    for (int i = 0; i < camera.width; ++i) {
        int y = gid;
        int x = i;

        Ray ray = get_ray(&camera, x, y);

        out[start + i] = ray_color(&world, ray, 1, bounce_count, iteration, &sum[start + i]);
    }
}

#include "random.cl"
#include "ray_trace.cl"
