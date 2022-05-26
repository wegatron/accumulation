#ifndef KIWI_APP_H
#define KIWI_APP_H

#include <MetalKit/MetalKit.h>

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
    
    size_t width_ = 0;
    size_t height_ = 0;
    
    MTKView * view = nil;
    id<MTLDevice> device_ = nil;
    id<MTLCommandQueue> cmd_queue_ = nil;
    id<MTLBuffer> vb_ = nil;
    id<MTLTexture> texture_ = nil;
    id<MTLRenderPipelineState> pipelineState_;
};

#endif // KIWI_APP_H
