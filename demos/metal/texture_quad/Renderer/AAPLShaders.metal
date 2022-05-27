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
    float2 uv;
};

struct VertexInput
{
    float4 pos_uv [[attribute(0)]];
};

vertex RasterizerData vertexShader(VertexInput in [[stage_in]])
{
    RasterizerData out;
    out.position = float4(in.pos_uv.x, in.pos_uv.y, 0.0, 1.0);
    out.uv = in.pos_uv.zw;
    return out;
}

fragment float4 fragmentShader(RasterizerData in [[stage_in]],
                               texture2d<float> input_texture [[texture(1)]],
                               sampler input_textureSmplr [[sampler(1)]])
{
    // Return the interpolated color.
    return input_texture.sample(input_textureSmplr, in.uv);
}

