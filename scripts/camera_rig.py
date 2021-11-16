import bpy
import os

path_dir = os.path.dirname(bpy.context.blend_data.filepath) + "/images" #save for restore

for cam in [obj for obj in bpy.data.objects if obj.type == 'CAMERA']:
    bpy.context.scene.camera = cam
    bpy.context.scene.render.filepath = os.path.join(path_dir, cam.name)
    bpy.ops.render.render(write_still=True)
    bpy.context.scene.render.filepath = path_dir