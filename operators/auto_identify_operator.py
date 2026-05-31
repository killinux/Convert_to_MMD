"""Operator: auto-identify skeleton bone roles by topology + geometry.

Fills the panel's bone-mapping slots automatically — no preset needed.
Ported from xps_to_mmd-main; adapted to Convert_to_MMD's prefix-less scene props.
"""

import bpy
from ..skeleton_identifier import identify_skeleton, clear_cache


class OBJECT_OT_auto_identify_skeleton(bpy.types.Operator):
    """自动识别骨架角色（纯拓扑+几何，不依赖骨名/预设）"""
    bl_idname = "object.auto_identify_skeleton"
    bl_label = "Auto Identify Skeleton"
    bl_description = "分析骨架拓扑与几何，自动填充各 MMD 骨骼槽位（无需预设）"

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "未选择骨架对象")
            return {'CANCELLED'}

        clear_cache()
        mapping = identify_skeleton(obj.data)

        from ..helper_classifier import classify_helpers
        cls = classify_helpers(obj.data, mapping)

        scene = context.scene
        filled = 0
        for prop_name, bone_name in mapping.items():
            if bone_name and hasattr(scene, prop_name):
                setattr(scene, prop_name, bone_name)
                filled += 1

        # 自动检测上半身链 (上半身2..5)，与重命名前的逻辑一致
        try:
            from .preset_operator import auto_detect_upper_body_chain
            auto_detect_upper_body_chain(scene, obj)
        except Exception as e:
            print(f"[Auto Identify] auto_detect_upper_body_chain: {e}")

        # --- 日志 ---
        from ..bone_map_and_group import mmd_bone_map
        print("\n========== [Auto Identify] 骨架自动识别结果 ==========")
        matched, unmatched = [], []
        for prop_name, mmd_name in mmd_bone_map.items():
            xps_name = mapping.get(prop_name, "") or getattr(scene, prop_name, "")
            if xps_name:
                matched.append(f"  {mmd_name:10s} ← {xps_name}")
            else:
                unmatched.append(f"  {mmd_name:10s} ← (未匹配)")
        print(f"--- 已匹配 ({len(matched)}) ---")
        for line in matched:
            print(line)
        if unmatched:
            print(f"--- 未匹配 ({len(unmatched)}) — 需手动设置或由补全骨骼创建 ---")
            for line in unmatched:
                print(line)

        from collections import Counter
        counts = Counter(cls.values())
        print("\n--- Helper 骨分类 ---")
        for cat in ("twist", "pelvis", "preserve", "merge", "other"):
            names = sorted([k for k, v in cls.items() if v == cat])
            if names:
                preview = ", ".join(names[:5])
                if len(names) > 5:
                    preview += f" (+{len(names) - 5})"
                print(f"  {cat:10s} {len(names):3d}  {preview}")
        print("=" * 50)

        total = sum(1 for v in mapping.values() if v)
        self.report({'INFO'}, f"自动识别完成: {total} 个骨骼角色已填充")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_auto_identify_skeleton)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_auto_identify_skeleton)
