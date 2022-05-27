#include "fullscreen_pass.h"

VISITABLE_STRUCT(zsw::BasicFullScreenQuadMesh, pos_uv);
VISITABLE_STRUCT(zsw::BasicFullScreenPSParameters, ts);
 
namespace zsw{
    std::shared_ptr<BasicFullScreenQuadMesh> createBasicFullScreenQuadMesh(id<MTLDevice> device)
    {
        auto mesh = std::make_shared<BasicFullScreenQuadMesh>();
        mesh->pos_uv.attributes = {{ 0, 0, 16, MTLVertexFormatFloat4 }};

        // data
        float vertices[] = {
            -1, -1, 0, 1,
            1, -1, 1, 1,
            1, 1, 1, 0,
            
            -1, -1, 0, 1,
            1, 1, 1, 0,
            -1, 1, 0, 0
        };
        mesh->pos_uv.buffer = [device newBufferWithBytes:vertices length:sizeof(vertices) options:MTLResourceStorageModeShared];
        return mesh;
    }

    id<MTLRenderPipelineState> createBasicFullScreenQuadPipeline(id<MTLDevice> device, MTLVertexDescriptor * vertex_desc)
    {
        id<MTLLibrary> defaultLibrary = [device newDefaultLibrary];
        id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
        id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];
        
        MTLRenderPipelineDescriptor *pipelineStateDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
        pipelineStateDescriptor.label = @"Texturing Pipeline";
        pipelineStateDescriptor.vertexFunction = vertexFunction;
        pipelineStateDescriptor.fragmentFunction = fragmentFunction;
        pipelineStateDescriptor.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        pipelineStateDescriptor.vertexDescriptor = vertex_desc;

        NSError *error = NULL;
        auto pipeline = [device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor
                                                                 error:&error];
        assert(pipeline != nil);
        return pipeline;
    }

    bool FullScreenPass::bindResource(id<MTLRenderCommandEncoder> encoder) {
        uint8_t begin_index = bindMeshData(encoder, *mesh_, 0);
        return bindShaderParameter(encoder, *ps_parameters_, begin_index);
    }

    bool FullScreenPass::doRender(id<MTLRenderCommandEncoder> encoder, MTLRenderPassDescriptor * rpd)
    {
        [encoder setRenderPipelineState: pipeline_];

        bindResource(encoder);

        // Draw the triangles.
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle
                          vertexStart:0
                          vertexCount:6];
        return true;
    }


    FullScreenPass::FullScreenPass(id<MTLDevice> device)
    {
        mesh_ = createBasicFullScreenQuadMesh(device);
        vertex_desc = createVertexDesc(*mesh_, 0);
        pipeline_ = createBasicFullScreenQuadPipeline(device, vertex_desc);
    }
}
