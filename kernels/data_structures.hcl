#ifndef DATA_STRUCTURES_HCL
#define DATA_STRUCTURES_HCL

#define ARRAY_DECLARE(data_type) \
typedef struct { \
    __global data_type *data; \
    int count; \
} Array_##data_type;

#define array(data_type) Array_##data_type

typedef struct {
	float color[3];
	float emissive[3];
	float specular;
	float refractiveness;
	float n;
} Material;

ARRAY_DECLARE(Material);
ARRAY_DECLARE(int);
ARRAY_DECLARE(float4);

typedef struct {
    float points[6];
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
	float pos[3];
	float dir[3];
	float center[3];
	float vertical[3];
	float horizontal[3];
	int width, height;
	float lense_distance;
} Camera;

typedef struct {
    float3 origin;
    float3 dir;
    float3 invdir;
} Ray;

#endif
