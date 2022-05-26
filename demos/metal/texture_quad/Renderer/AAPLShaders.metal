#include <metal_stdlib>

using namespace metal;


// Vertex shader outputs and fragment shader inputs
struct RasterizerData
{
    // The [[position]] attribute of this member indicates that this value
    // is the clip space position of the vertex when this structure is
    // returned from the vertex function.
    float4 position [[position]];

    // Since this member does not have a special attribute, the rasterizer
    // interpolates its value with the values of the other triangle vertices
    // and then passes the interpolated value to the fragment shader for each
    // fragment in the triangle.
    float4 color;
};

struct VertexInput
{
    float4 pos_uv [[attribute(0)]];
};

vertex RasterizerData vertexShader(VertexInput in [[stage_in]])
{
    RasterizerData out;
    //out.position = in.
    return out;
}

fragment float4 fragmentShader(RasterizerData in [[stage_in]])
{
    // Return the interpolated color.
    return in.color;
}

