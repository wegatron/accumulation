#ifndef FULL_SCREEN_PASS_H
#define FULL_SCREEN_PASS_H

#include <memory>
#include <vector>
#include <iostream>
#include <Metal/Metal.h>
#include "visit_struct.hpp"
namespace zsw
{
    struct VertexAttribute
    {
        uint8_t index = 0; //!< attribute index
        uint16_t offset = 0; //!< attribute offset, relative to begining of component data
        uint16_t stride = 0; //!< stride of data
        MTLVertexFormat format;
    };

    struct VertexComponent
    {
        id<MTLBuffer> buffer = nil; //!< buffer in vertex component should be unique
        uint32_t buffer_offset = 0; //!< offset to the start of buffer.
        std::vector<VertexAttribute> attributes;
    };
    
    struct ShaderParameter
    {
        uint8_t binding = 0;
    };

    struct TextureSampler : public ShaderParameter
    {
        id<MTLTexture> texture = nil;
        id<MTLSamplerState> sampler = nil;
    };

    // struct MVP : public ShaderParameter
    // {
    //     struct Data
    //     {
    //         float mvp_mat[16];
    //     };
    //     void updateHwResource();
    //     id<MTLBuffer> buffer;
    //     std::shared_ptr<Data> data_ptr;
    // };

    template<typename T>
    uint8_t bindMeshData(id<MTLRenderCommandEncoder> encoder, T & mesh_data, uint8_t beg_binding_point)
    {
        uint8_t binding_point = beg_binding_point;
        visit_struct::for_each(mesh_data,
            [&binding_point, encoder](const char * name, const auto & value) {
                // binding vertex component
                [encoder setVertexBuffer: value.buffer offset: value.buffer_offset atIndex: binding_point];
                ++binding_point;
        });
        return binding_point;
    }

    template<typename T>
    MTLVertexDescriptor * createVertexDesc(T & mesh_data, uint8_t beg_binding_point)
    {
        MTLVertexDescriptor *vertex_desc = [MTLVertexDescriptor new];
        uint8_t binding_point = beg_binding_point;
        visit_struct::for_each(mesh_data,
            [&vertex_desc, binding_point](const char * name, const auto & value) {
                
                
                for(auto & attr : value.attributes)
                {
                    vertex_desc.attributes[attr.index].format = attr.format;
                    vertex_desc.attributes[attr.index].offset = attr.offset;
                    vertex_desc.attributes[attr.index].bufferIndex = binding_point;
                }
                
                // for stride
                vertex_desc.layouts[binding_point].stride = value.attributes[0].stride;
        });

        return vertex_desc;
    }

    template<typename T>
    bool bindShaderParameter(id<MTLRenderCommandEncoder> encoder, T & parameter, uint8_t beg_binding_point)
    {
        uint8_t binding_point = beg_binding_point;
        visit_struct::for_each(parameter,
            [&binding_point, encoder](const char * name, auto & value) {
                if(name[0] == 't' && name[1] == 's')
                {
                    auto ts = static_cast<TextureSampler*>(&value);
                    if(ts != nullptr && ts->texture != nil) {
                        [encoder setFragmentTexture: ts->texture atIndex: ts->binding];
                        [encoder setFragmentSamplerState: ts->sampler atIndex: ts->binding];
                    } else {
                        std::cerr << "[WARNING] texture is null in fragment shader!" << std::endl;
                    }
                }
                ++binding_point;
        });
        return true;
    }
    

    struct BasicFullScreenQuadMesh
    {
        VertexComponent pos_uv;
    };


    struct BasicFullScreenPSParameters
    {
        TextureSampler ts; //!< begin with ts
    };

    std::shared_ptr<BasicFullScreenQuadMesh> createBasicFullScreenQuadMesh(id<MTLDevice> device);
    
    id<MTLRenderPipelineState> createBasicFullScreenQuadPipeline(id<MTLDevice> device, MTLVertexDescriptor * vertex_desc);


    class FullScreenPass
    {
    public:
        
        FullScreenPass(id<MTLDevice> device);
        
        bool doRender(id<MTLRenderCommandEncoder> encoder, MTLRenderPassDescriptor * rpd);

        std::shared_ptr<BasicFullScreenPSParameters> ps_parameters_ = nullptr;
        
        id<MTLRenderPipelineState> pipeline_ = nil;
    
    private:
        
        bool bindResource(id<MTLRenderCommandEncoder> encoder);
        
        std::shared_ptr<BasicFullScreenQuadMesh> mesh_ = nullptr;
        MTLVertexDescriptor * vertex_desc = nil;
    };
}



#endif //FULL_SCREEN_PASS_H
