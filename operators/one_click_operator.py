"""One-click full XPS→MMD conversion: auto-identify + entire pipeline.

Ported from xps_to_mmd-main; adapted to Convert_to_MMD operator ids and
prefix-less scene props. Steps that don't apply (e.g. XPS fixes when the
operators aren't present) fail gracefully and are reported as warnings.
"""

import bpy
import time


# (step_num, operator_id, label, critical)
PIPELINE_PRE_D = [
    ("0.5", "object.correct_bones", "归正骨架位置", False),
    ("1", "object.rename_to_mmd", "重命名为 MMD", True),
    ("1.4", "object.transfer_unused_weights", "转移 unused 权重 (第一次)", False),
    ("1.5", "object.fix_forearm_bend", "修正前腕弯曲", False),
    ("1.6", "object.align_arms_to_canonical", "对齐上臂到 A-pose", False),
    ("1.7", "object.align_fingers_to_canonical", "对齐手指到 A-pose", False),
    ("2", "object.complete_missing_bones", "补全缺失骨骼", True),
    ("2.5", "object.transfer_unused_weights", "清理控制骨权重 (第二次)", False),
    # 2.7 fix_shoulder_weights 已停用：三角肌(肩盖)现由 transfer_unused 整组路由到 肩
    # (用户决定 2026-06-06: 忠实复用 XPS 三角肌权重，不再启发式 腕→肩 切分/欠补)。
    ("3", "object.add_mmd_ik", "添加 MMD IK", True),
    ("4", "object.create_bone_group", "创建骨骼集合", True),
    ("5", "object.use_mmd_tools_convert", "mmd_tools 转换", True),
]

PIPELINE_POST_D = [
    ("6", "object.add_leg_d_bones", "添加腿部 D 骨", False),
    ("7", "object.add_twist_bone", "添加捩骨", False),
    # 7.5 在 add_twist 之后：拇指渗出还给手首 + 手首手掌段分给掌骨(先让捩骨回收手首前臂侧)。
    ("7.5", "object.fix_palm_weights", "手部权重修正(拇指+掌骨)", False),
    ("8", "object.add_shoulder_p_bones", "添加肩P骨", False),
]


def _find_armature():
    for o in bpy.data.objects:
        if o.type == 'ARMATURE' and 'backup' not in o.name.lower():
            return o
    return None


def _call(op_id):
    parts = op_id.split('.')
    return getattr(getattr(bpy.ops, parts[0]), parts[1])()


class OBJECT_OT_one_click_convert(bpy.types.Operator):
    """一键完成 XPS→MMD 全流程转换（自动识别 + 全部步骤）"""
    bl_idname = "object.one_click_convert"
    bl_label = "一键转换 XPS→MMD"
    bl_description = "自动识别骨架并依次执行重命名/修正/补全/IK/集合/mmd_tools转换/次标准骨"
    bl_options = {'REGISTER', 'UNDO'}

    auto_identify: bpy.props.BoolProperty(name="自动识别骨架", default=True)  # type: ignore

    def execute(self, context):
        t_start = time.time()
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选中骨架")
            return {'CANCELLED'}

        results = []

        # Step 0: auto identify (optional — skip to use the panel's manual/preset mapping)
        if self.auto_identify:
            try:
                bpy.ops.object.auto_identify_skeleton()
                results.append(("0", "自动识别骨架", "OK"))
            except Exception as e:
                results.append(("0", "自动识别骨架", f"WARN: {e}"))

        # Save XPS→MMD name map BEFORE rename (for post-convert VG cleanup)
        from ..bone_map_and_group import mmd_bone_map
        scene = context.scene
        xps_to_mmd_map = {}
        for prop_name, mmd_name in mmd_bone_map.items():
            xps_name = getattr(scene, prop_name, None)
            if xps_name and xps_name != mmd_name:
                xps_to_mmd_map[xps_name] = mmd_name

        for step_num, op_id, label, critical in PIPELINE_PRE_D:
            arm = obj if (obj and obj.name in bpy.data.objects) else _find_armature()
            if arm:
                context.view_layer.objects.active = arm
                arm.select_set(True)
            try:
                t = time.time()
                result = _call(op_id)
                dt = time.time() - t
                status = "OK" if result == {'FINISHED'} else str(result)
                results.append((step_num, label, f"{status} ({dt:.1f}s)"))
            except Exception as e:
                results.append((step_num, label, f"FAIL: {e}"))
                if critical:
                    self._print_summary(results, time.time() - t_start)
                    self.report({'ERROR'}, f"Step {step_num} {label} 失败: {e}")
                    return {'CANCELLED'}

        # VG cleanup: merge stranded old-name vertex groups before D-bone copy
        arm = obj if (obj and obj.name in bpy.data.objects) else _find_armature()
        if arm and xps_to_mmd_map:
            meshes = [o for o in bpy.data.objects if o.type == 'MESH'
                      and any(m.type == 'ARMATURE' and m.object == arm for m in o.modifiers)]
            merged = 0
            for mesh in meshes:
                for old_name, new_name in xps_to_mmd_map.items():
                    old_vg = mesh.vertex_groups.get(old_name)
                    if not old_vg:
                        continue
                    new_vg = mesh.vertex_groups.get(new_name)
                    if not new_vg:
                        old_vg.name = new_name
                        merged += 1
                        continue
                    for v in mesh.data.vertices:
                        for g in v.groups:
                            if g.group == old_vg.index and g.weight > 0.001:
                                new_vg.add([v.index], g.weight, 'ADD')
                                break
                    mesh.vertex_groups.remove(old_vg)
                    merged += 1
            if merged:
                results.append(("5.5", f"VG 残留清理 ({merged})", "OK"))

        for step_num, op_id, label, critical in PIPELINE_POST_D:
            arm = obj if (obj and obj.name in bpy.data.objects) else _find_armature()
            if arm:
                context.view_layer.objects.active = arm
                arm.select_set(True)
            try:
                t = time.time()
                result = _call(op_id)
                dt = time.time() - t
                status = "OK" if result == {'FINISHED'} else str(result)
                results.append((step_num, label, f"{status} ({dt:.1f}s)"))
            except Exception as e:
                results.append((step_num, label, f"FAIL: {e}"))

        # 设置标准付与 (肩C/腕捩/手捩/D骨) 到 mmd_bone，导出 PMX 时生效
        arm = obj if (obj and obj.name in bpy.data.objects) else _find_armature()
        if arm:
            try:
                from .mmd_grant_operator import apply_standard_grants
                ng = apply_standard_grants(arm)
                results.append(("8.4", f"设置标准付与 ({ng})", "OK"))
            except Exception as e:
                results.append(("8.4", "设置标准付与", f"WARN: {e}"))

        # apply additional transform (付与) so D/twist/腰キャンセル take effect
        try:
            bpy.ops.mmd_tools.apply_additional_transform()
            results.append(("8.5", "apply_additional_transform", "OK"))
        except Exception as e:
            results.append(("8.5", "apply_additional_transform", f"WARN: {e}"))

        total = time.time() - t_start
        self._print_summary(results, total)
        ok = sum(1 for _, _, s in results if s.startswith("OK"))
        self.report({'INFO'}, f"一键转换完成: {ok}/{len(results)} 步成功 ({total:.1f}s)")
        return {'FINISHED'}

    def _print_summary(self, results, total):
        print("\n" + "=" * 60)
        print("[Convert_to_MMD] 一键转换结果")
        print("=" * 60)
        for step, label, status in results:
            mark = "✓" if status.startswith("OK") else ("⚠" if "WARN" in status else "✗")
            print(f"  {mark} Step {step:<5} {label:<28} {status}")
        print(f"\n  总耗时: {total:.1f}s")
        print("=" * 60)


def register():
    bpy.utils.register_class(OBJECT_OT_one_click_convert)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_one_click_convert)
