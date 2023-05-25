
import imageio
import numpy as np
import torch
import nvdiffrast.torch as dr
import math
from pytorch3d.io import load_obj


def tensor(*args, **kwargs):
    return torch.tensor(*args, device='cuda', **kwargs)


def load_image(img_path):
    tex = np.asarray(imageio.v2.imread(img_path))
    tex = tex.astype(np.float32) / 255.0
    return tex


def transform_v(mtx, pos):
    t_mtx = torch.from_numpy(mtx).cuda() if isinstance(mtx, np.ndarray) else mtx
    posw = torch.cat([pos, torch.ones([pos.shape[0], 1]).cuda()], axis=1)
    return torch.matmul(posw, t_mtx.t())[None, ...]


class ObjRender:
    def __init__(self, ctx=None):
        if ctx is None:
            self.glctx = dr.RasterizeCudaContext()
        else:
            self.glctx = ctx


    def rasterize_torch(self, mvp, vertex, tris, resolution):
        """
        mvp: 4x4 torch.float32 tensor
        vertex: Nx4 torch.float32 tensor
        tris: Mx3 int32 tensor
        """
        pos = torch.matmul(vertex, mvp.t())[None, ...]
        rast, _ = dr.rasterize(self.glctx, pos, tris, resolution)
        return rast
    
    def dorender_torch(self, mvp, vertex, tris, uv, tris_uv, diffuse_img, resolution):
        """
        mvp: 4x4 torch.float32 tensor
        vertex: Nx4 torch.float32 tensor
        tris: Mx3 int32 tensor
        uv: Nx2 torch.float32 tensor
        diffuse_img: 1xHxWx3 torch.float32 tensor
        resolution: tuple of (H, W), int32 should by divisible by 8, max 2048        
        """
        pos = torch.matmul(vertex, mvp.t())[None, ...]
        rast, _ = dr.rasterize(self.glctx, pos, tris, resolution)
        texc, _ = dr.interpolate(uv, rast, tris_uv)
        color_fg = dr.texture(diffuse_img[None, ...], texc, filter_mode='linear')
        fg_mask = torch.clamp(rast[..., -1:], 0, 1)
        color_fg = color_fg * fg_mask
        return color_fg[0]

    def dorender(self, mvp, vertex, tris, uv, tris_uv, diffuse_img, resolution):
        """
        mvp: 4x4 np.float32
        vertex: Nx3 np.float32
        tris: Mx3 int32
        uv: Nx2 np.float32
        tris_uv: Mx3 int32
        diffuse_img: HxWx3 np.float32
        resolution: tuple of (H, W), int32 should by divisible by 8, max 2048
        """
        # render front
        v = tensor(vertex, dtype=torch.float32)
        pos = transform_v(mvp.astype(np.float32), v)
        tris_t = tensor(tris, dtype=torch.int32)
        rast, _ = dr.rasterize(self.glctx, pos, tris_t, resolution)

        tris_uv_t = tensor(tris_uv, dtype=torch.int32)
        texc, _ = dr.interpolate(tensor(uv, dtype=torch.float32), rast, tris_uv_t)
        color_fg = dr.texture(tensor(diffuse_img, dtype=torch.float32)[None, ...], texc, filter_mode='linear')
        fg_mask = torch.clamp(rast[..., -1:], 0, 1)
        color_fg = color_fg * fg_mask
        return color_fg[0].cpu().numpy()

# if __name__ == "__main__":
#     diffuse_img = load_image('/home/wegatron/win-data/workspace/DECA/TestSamples/jqw_head.jpg')
#     obj_render = ObjRender()
#     mvp = np.identity(4, dtype=np.float32)
#     vertex = np.array([[-1,-1, 0], [1, 1, 0], [1,-1,0.5]])
#     uv = np.array([[0, 0], [1,0], [1,1]])
#     tris = np.array([[0, 1, 2]], dtype=np.int32)
#     output_img = obj_render.dorender(mvp, vertex, tris, uv, tris, diffuse_img, (512, 512))
#     print("Saving to 'tri.png'.")
#     img_data = (output_img * 255).astype(np.uint8)
#     imageio.imsave('tri.png', img_data)

def perspective(fovy, aspect, near, far):
    f = 1.0 / math.tan(fovy / 2.0)
    return torch.tensor([[f / aspect, 0, 0, 0],
                     [0, f, 0, 0],
                     [0, 0, (far + near) / (near - far), (2 * far * near) / (near - far)],
                     [0, 0, -1, 0]], dtype=torch.float32, device='cuda')

if __name__ == "__main__":
    verts, faces, props = load_obj('/home/wegatron/win-data/workspace/head_fusion/MeInGame/data/mesh/mine/target.obj', 
                                   load_textures=True,
                                   device='cuda')
    diffuse_img = list(props.texture_images.values())[0]
    obj_render = ObjRender()
    #normalize verts to [-1, 1]
    verts = verts - verts.mean(dim=0, keepdim=True)
    verts = verts / verts.abs().max()

    # camera in (0,0,2) look at (0,0,0)
    view_mat = torch.tensor([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., -2.],
                            [0., 0., 0., 1.]],
                            dtype=torch.float32, device='cuda')
    
    proj_mat = perspective(0.8, 1, 1, 3) # 45 degree
    view_projection_mat = torch.matmul(proj_mat, view_mat)

    verts_b = torch.cat([verts, torch.ones([verts.shape[0], 1]).cuda()], axis=1)
    props.verts_uvs[:, 1] = 1 - props.verts_uvs[:, 1]
    out_img = obj_render.dorender_torch(view_projection_mat, verts_b, 
                                        faces.verts_idx.to(torch.int32),
                                        props.verts_uvs,
                                        faces.textures_idx.to(torch.int32),
                                        diffuse_img.cuda(), (512, 512))
    out_img = torch.flip(out_img, dims=[0])
    imageio.imwrite('obj.png', (out_img*255).to(torch.uint8).cpu().numpy())