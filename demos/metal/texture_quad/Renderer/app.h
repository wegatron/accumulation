#ifndef KIWI_APP_H
#define KIWI_APP_H

#include <memory>
#include <MetalKit/MetalKit.h>
#include "fullscreen_pass.h"

class App final {
public:

    // disallow copy and assignment
    App(App const &) = delete;
    App(App &&) = delete;
    App &operator=(App const &) = delete;
    App &operator=(App &&) = delete;

    static App * createApp(MTKView *mtk_view);

    void setViewportSize(const size_t width, const size_t height)
    {
        width_ = width;
        height_ = height;
    }
        
    void setupRenderPipeline();
    
    void renderOneFrame();

private:
    
    App() = default;
    
    std::unique_ptr<zsw::FullScreenPass> fullscreen_pass_;
    
    size_t width_ = 0;
    size_t height_ = 0;
    
    MTKView * view_ = nil;
    id<MTLDevice> device_ = nil;
    id<MTLCommandQueue> cmd_queue_ = nil;
    id<MTLTexture> texture_ = nil;
    MTLRenderPassDescriptor * rpd_ = nil;

};

#endif // KIWI_APP_H
