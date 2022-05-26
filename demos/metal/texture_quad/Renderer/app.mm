#include "app.h"

#include <string>
#include <memory>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

static constexpr size_t MAX_FRAMES_IN_FLIGHT = 3;

App *App::createApp(MTKView *mtk_view) {
    auto app = new App;
    app->device_ = mtk_view.device;
    app->cmd_queue_ = [app->device_ newCommandQueue];
    return app;
}

void App::setupRenderPipeline()
{
    /// 
    /// vertex data
    static float vertices_data[] =
    {
        -1, -1, 0, 1,
        1, -1, 1, 1,
        1, 1, 1, 0,
        
        -1, -1, 0, 1,
        1, 1, 1, 0,
        -1, 1, 0, 0
    };
    
    vb_ = [device_ newBufferWithBytes:vertices_data length:sizeof(vertices_data) options: MTLResourceStorageModeShared];
    
    id<MTLLibrary> defaultLibrary = [device_ newDefaultLibrary];
    id<MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
    id<MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];
    
    MTLRenderPipelineDescriptor *pipelineStateDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineStateDescriptor.label = @"Texturing Pipeline";
    pipelineStateDescriptor.vertexFunction = vertexFunction;
    pipelineStateDescriptor.fragmentFunction = fragmentFunction;
    pipelineStateDescriptor.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA8Unorm;

    NSError *error = NULL;
    pipelineState_ = [device_ newRenderPipelineStateWithDescriptor:pipelineStateDescriptor
                                                             error:&error];
    assert(pipelineState_ != nil);
}


void App::renderOneFrame()
{
    auto cmd_buffer = [cmd_queue_ commandBuffer];
    auto rpd = view.currentRenderPassDescriptor;
    if(rpd != nil)
    {
        auto renderEncoder = [cmd_buffer renderCommandEncoderWithDescriptor:rpd];
        [renderEncoder setViewport:(MTLViewport){0.0, 0.0, static_cast<double>(width_), static_cast<double>(height_), -1.0, 1.0 }];

        [renderEncoder setRenderPipelineState:pipelineState_];

        [renderEncoder setVertexBuffer:vb_
                                offset:0
                              atIndex:0];
        

        // Set the texture object.  The AAPLTextureIndexBaseColor enum value corresponds
        ///  to the 'colorMap' argument in the 'samplingShader' function because its
        //   texture attribute qualifier also uses AAPLTextureIndexBaseColor for its index.
        [renderEncoder setFragmentTexture:texture_ atIndex:1];

        // Draw the triangles.
        [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle
                          vertexStart:0
                          vertexCount:6];

        //[renderEncoder waitForFence:_fence beforeStages:MTLRenderStageVertex];
        [renderEncoder endEncoding];
        [cmd_buffer presentDrawable:view.currentDrawable];
        //[cmd_buffer encodeSignalEvent:_event value:ind];
        // Schedule a present once the framebuffer is complete using the current drawable
        [cmd_buffer commit];
    }
}
