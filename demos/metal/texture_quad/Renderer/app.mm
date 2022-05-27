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
    app->view_ = mtk_view;
    return app;
}

void App::setupRenderPipeline()
{
    MTLTextureDescriptor * tex_desc = [MTLTextureDescriptor new];
    tex_desc.width = 600;
    tex_desc.height = 600;
    tex_desc.pixelFormat = MTLPixelFormatRGBA8Unorm;
    texture_ = [device_ newTextureWithDescriptor:tex_desc];
    rpd_ = [MTLRenderPassDescriptor new];
    rpd_.colorAttachments[0].texture = texture_;
    rpd_.colorAttachments[0].loadAction = MTLLoadAction::MTLLoadActionDontCare;
    rpd_.colorAttachments[0].storeAction = MTLStoreAction::MTLStoreActionStore;
    fullscreen_pass_.reset(new zsw::FullScreenPass(device_));
    
    fullscreen_pass_->ps_parameters_.reset(new zsw::BasicFullScreenPSParameters);
    fullscreen_pass_->ps_parameters_->ts.binding = 1;
    
    
    fullscreen_pass_->ps_parameters_->ts.texture = ;
    
    MTLSamplerDescriptor *mtl_sampler_desc = [MTLSamplerDescriptor new];
    mtl_sampler_desc.minFilter = MTLSamplerMinMagFilterLinear;
    mtl_sampler_desc.magFilter = MTLSamplerMinMagFilterLinear;
    mtl_sampler_desc.sAddressMode = MTLSamplerAddressModeRepeat;
    mtl_sampler_desc.tAddressMode = MTLSamplerAddressModeRepeat;
    fullscreen_pass_->ps_parameters_->ts.sampler = [device_ newSamplerStateWithDescriptor:mtl_sampler_desc];
}


void App::renderOneFrame()
{
    auto cmd_buffer = [cmd_queue_ commandBuffer];
    auto rpd = view_.currentRenderPassDescriptor;
    if(rpd != nil)
    {
        auto renderEncoder = [cmd_buffer renderCommandEncoderWithDescriptor:rpd];
        [renderEncoder setViewport:(MTLViewport){0.0, 0.0, static_cast<double>(width_), static_cast<double>(height_), -1.0, 1.0 }];
        fullscreen_pass_->doRender(renderEncoder, rpd);
        [renderEncoder endEncoding];
        [cmd_buffer presentDrawable:view_.currentDrawable];
        [cmd_buffer commit];
    }
}
