"""XPS 导入 operator — 包装 XNALaraMesh 的 bpy.ops.xps_tools.import_model。

未启用 XNALaraMesh 时给出提示；导入后把生成的 armature 设为 active，
可选自动缩放过大的模型到合理尺寸。
"""
import bpy
from bpy.props import StringProperty, BoolProperty
from bpy_extras.io_utils import ImportHelper

from ..bone_utils import check_and_scale_skeleton


_XNALARA_INSTALL_URL = "https://github.com/johnzero7/XNALaraMesh"


def _xnalara_available():
    return hasattr(bpy.ops.xps_tools, "import_model")


def _find_new_armature(before_names):
    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE' and obj.name not in before_names:
            return obj
    return None


class OBJECT_OT_import_xps(bpy.types.Operator, ImportHelper):
    """导入 XPS / XNALara 模型 (.xps / .mesh / .ascii)，需先启用 XNALaraMesh addon"""
    bl_idname = "object.import_xps"
    bl_label = "Import XPS"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".xps"
    filter_glob: StringProperty(
        default="*.xps;*.mesh;*.xps_bin;*.mesh.ascii;*.ascii",
        options={'HIDDEN'},
    )  # type: ignore
    auto_scale: BoolProperty(
        name="自动缩放过大模型",
        description="检测骨架高度 > 10m 时自动按 10 倍缩放回合理尺寸",
        default=True,
    )  # type: ignore

    def execute(self, context):
        if not _xnalara_available():
            self.report({'ERROR'}, f"未检测到 XNALaraMesh / xps_tools addon。请先安装并启用: {_XNALARA_INSTALL_URL}")
            return {'CANCELLED'}

        before_armatures = {o.name for o in bpy.data.objects if o.type == 'ARMATURE'}
        try:
            bpy.ops.xps_tools.import_model(filepath=self.filepath)
        except Exception as exc:
            self.report({'ERROR'}, f"XPS 导入失败: {exc}")
            return {'CANCELLED'}

        armature = _find_new_armature(before_armatures)
        if armature is None:
            self.report({'WARNING'}, "XPS 导入完成但未找到新 armature")
            return {'FINISHED'}

        bpy.ops.object.select_all(action='DESELECT')
        armature.select_set(True)
        context.view_layer.objects.active = armature

        if self.auto_scale:
            try:
                scaled, factor, orig_h = check_and_scale_skeleton(armature)
                if scaled:
                    self.report({'INFO'}, f"骨架高度 {orig_h:.2f}m，已按 {factor} 倍缩放")
            except Exception as exc:
                self.report({'WARNING'}, f"自动缩放失败 (可忽略): {exc}")

        bpy.ops.object.mode_set(mode='OBJECT')
        self.report({'INFO'}, f"已导入 XPS：{armature.name}")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_import_xps)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_import_xps)


if __name__ == "__main__":
    register()
