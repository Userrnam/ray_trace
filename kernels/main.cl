#define ARRAY_DECLARE(data_type) \
typedef struct { \
    data_type *data; \
    int count; \
} Array_##data_type;

#define array(data_type) Array_##data_type

typedef struct {
	float4 color;
	float4 emissive;
	float specular;
	float refractiveness;
	float n;
} Material;

ARRAY_DECLARE(Material);
ARRAY_DECLARE(int);
ARRAY_DECLARE(float4);

typedef struct {
    float3 points[2];
} Bounding_Box;

typedef struct {
    Bounding_Box bounding_box;
    int left;
    int right;
    array(int) triangle_indices;
} BVH_Node;

ARRAY_DECLARE(BVH_Node);

typedef struct {
    array(BVH_Node) nodes;
} BVH;

typedef struct {
    array(int) vertex_indices;
    array(int) normal_indices;
    int material_index;
} Mesh;

ARRAY_DECLARE(Mesh);

typedef struct {
    array(Material) materials;
    array(int) mesh_indices; // only these meshes are rendered.
    array(Mesh) meshes;
    array(float4) vertices;
    array(float4) normals;
} World;

typedef struct {
	float3 pos;
	float3 dir;
	float3 center;
	float3 vertical;
	float3 horizontal;
	int width, height;
	float lense_distance;
} Camera;

// fill 1 row.
__kernel void main(__global float4* out, int width, Camera camera) {
    int gid = get_global_id(0);
    int start = gid * width;

    for (int i = 0; i < width; ++i) {
        int y = gid;
        int x = i;

        float u = -1.0f + (float)x / (float)width * 2.0f;
        float v = -1.0f + (float)y / (float)get_global_size(0) * 2.0f;

        if (u*u + v*v < 0.5) {
            out[start + i] = (float4)(1.0f, 0.0f, 0.0f, 1.0f);
        } else {
            out[start + i] = (float4)(0.0f, 0.0f, 1.0f, 1.0f);
        }
    }
}
