"""设置 MMD 标准付与(additional transform)关系，使次标准骨在导出 PMX 时带付与。

Convert_to_MMD 的捩骨/D骨/肩C operator 用 _dummy_/_shadow_ + Blender 约束实现
功能，但不写 mmd_bone.additional_transform，导致导出 PMX 缺少付与标志。
本 operator 在骨骼齐备后批量设置标准付与关系：

- {左右}肩C    ← {左右}肩P     (rot, -1.0)
- {左右}腕捩1/2/3 ← {左右}腕捩  (rot, 0.25/0.50/0.75)
- {左右}手捩1/2/3 ← {左右}手捩  (rot, 0.25/0.50/0.75)
- {左右}足D    ← {左右}足      (rot, 1.0)
- {左右}ひざD  ← {左右}ひざ    (rot, 1.0)
- {左右}足首D  ← {左右}足首    (rot, 1.0)

(腰キャンセル 由 complete_missing_bones 设置)
对导出 PMX 而言只有 mmd_bone 生效；_dummy_/_shadow_ 会被导出器排除。
"""
import bpy


def standard_grant_specs():
    """返回 (bone_name, source_name, influence) 列表。"""
    specs = []
    for s in ("左", "右"):
        specs.append((f"{s}肩C", f"{s}肩P", -1.0))
        for i, infl in ((1, 0.25), (2, 0.50), (3, 0.75)):
            specs.append((f"{s}腕捩{i}", f"{s}腕捩", infl))
            specs.append((f"{s}手捩{i}", f"{s}手捩", infl))
        specs.append((f"{s}足D", f"{s}足", 1.0))
        specs.append((f"{s}ひざD", f"{s}ひざ", 1.0))
        specs.append((f"{s}足首D", f"{s}足首", 1.0))
    return specs


def apply_standard_grants(arm):
    """在 arm 上设置标准付与关系，返回设置成功的数量。"""
    n = 0
    for name, src, infl in standard_grant_specs():
        pb = arm.pose.bones.get(name)
        if not pb or not arm.pose.bones.get(src):
            continue
        mb = getattr(pb, "mmd_bone", None)
        if mb is None:
            continue
        try:
            mb.has_additional_rotation = True
            mb.has_additional_location = False
            mb.additional_transform_bone = src
            mb.additional_transform_influence = infl
            n += 1
        except Exception as e:
            print(f"[mmd_grant] {name} ← {src} 设置失败: {e}")
    return n


class OBJECT_OT_setup_mmd_grants(bpy.types.Operator):
    """设置 MMD 标准付与(肩C/腕捩/手捩/D骨)，让导出 PMX 带付与标志"""
    bl_idname = "object.setup_mmd_grants"
    bl_label = "设置标准付与(付与)"
    bl_description = "为捩骨/D骨/肩C 设置 mmd_bone 付与关系，导出 PMX 时生效"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        # 允许从 mmd 模型的任意子物体定位骨架
        if obj and obj.type != 'ARMATURE':
            arm = next((o for o in bpy.data.objects
                        if o.type == 'ARMATURE' and 'backup' not in o.name.lower()), None)
        else:
            arm = obj
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "未找到骨架")
            return {'CANCELLED'}
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        n = apply_standard_grants(arm)
        self.report({'INFO'}, f"已设置 {n} 个标准付与关系")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_setup_mmd_grants)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_setup_mmd_grants)
