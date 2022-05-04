#ifndef DATA_STRUCTURES_HCL
#define DATA_STRUCTURES_HCL

#define ARRAY_DECLARE(data_type) \
typedef struct { \
    __global data_type *data; \
    int count; \
} Array_##data_type;

#define array(data_type) Array_##data_type

#define VEC3(v) (float3)(v[0], v[1], v[2])

typedef struct {
	float color[4];
	float emissive[4];
	float specular;
	float refractiveness;
	float n;
} Material;

ARRAY_DECLARE(Material);
ARRAY_DECLARE(int);
ARRAY_DECLARE(float4);

// I don't use float3 due to padding.
typedef struct {
    float x;
    float y;
    float z;
    float w;
} Point;

typedef struct {
    Point points[2];
} Bounding_Box;

typedef struct {
    Bounding_Box bounding_box;
    int left;
    int right;
    int first_triangle;
    int triangle_count;
} BVH_Node;

ARRAY_DECLARE(BVH_Node);

typedef struct {
    int bvh_first;
    int vi_first; int vi_count; // vertex indices
    int ni_first; int ni_count; // normal indices
    int material_index;
} Mesh;

ARRAY_DECLARE(Mesh);

typedef struct {
    array(Material) materials;
    array(int) mesh_indices; // only these meshes are rendered.
    array(BVH_Node) bvh_nodes;
    array(int) triangle_indices;

    array(int) vertex_indices;
    array(int) normal_indices;

    array(float4) vertices;
    array(float4) normals;

    array(Mesh) meshes;
} World;

typedef struct {
	float pos[4];
	float dir[4];
	float center[4];
	float vertical[4];
	float horizontal[4];
	int width, height;
	float lense_distance;
} Camera;

typedef struct {
    float3 origin;
    float3 dir;
    float3 invdir;
} Ray;

#endif
