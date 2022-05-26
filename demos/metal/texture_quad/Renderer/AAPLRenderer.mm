/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Implementation of a platform independent renderer class, which performs Metal setup and per frame rendering
*/

#import <MetalKit/MetalKit.h>
#include <Metal/Metal.h>
#import <simd/simd.h>
#include <memory>
#include "app.h"


#import "AAPLRenderer.h"

// Header shared between C code here, which executes Metal API commands, and .metal files, which
// uses these types as inputs to the shaders.
#import "AAPLShaderTypes.h"

// Main class performing the rendering
@implementation AAPLRenderer
{
    std::unique_ptr<App> app_;
}

- (nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)mtk_view
{
    self = [super init];
    
    app_.reset(App::createApp(mtk_view));
    // init
    app_->setupRenderPipeline();
    return self;
}

/// Called whenever view changes orientation or is resized
- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size
{
    app_->setViewportSize(size.width, size.height);
}

/// Called whenever the view needs to render a frame.
- (void)drawInMTKView:(nonnull MTKView *)view
{
    app_->renderOneFrame();
}

@end
