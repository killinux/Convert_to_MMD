import bpy
from mathutils import Vector
from .. import bone_utils


def _split_chain_weights(obj, src_name, dst_name, seg_from_name, seg_to_name,
                         perp_threshold=1.5, src_keep_floor=0.0):
    """PMXEditor 风格：沿 seg_from→seg_to 段按 t∈[0,1] 线性把 src 权重分配给 dst。

    用于自动补全的中间骨（上半身1/上半身3/首1）权重分割，以及腋窝平滑
    (src_keep_floor=1.0 时只追加不削减源骨权重)。
    返回 (moved_verts, filtered_verts)。
    """
    src_keep_floor = max(0.0, min(1.0, src_keep_floor))
    src_b = obj.data.bones.get(seg_from_name)
    dst_b = obj.data.bones.get(seg_to_name)
    if not src_b or not dst_b:
        return (0, 0)
    seg_from = src_b.head_local
    seg_to = dst_b.head_local
    seg = seg_to - seg_from
    if seg.length_squared < 1e-9:
        return (0, 0)
    meshes = [
        m for m in bpy.data.objects
        if m.type == 'MESH' and any(
            mod.type == 'ARMATURE' and mod.object == obj for mod in m.modifiers
        )
    ]
    arm_mw = obj.matrix_world
    seg_from_w = arm_mw @ seg_from
    seg_to_w = arm_mw @ seg_to
    seg_w = seg_to_w - seg_from_w
    seg_len_sq_w = seg_w.length_squared
    if seg_len_sq_w < 1e-9:
        return (0, 0)
    perp_limit_sq = (perp_threshold * perp_threshold) * seg_len_sq_w
    moved = 0
    filtered = 0
    for m in meshes:
        src_vg = m.vertex_groups.get(src_name)
        if not src_vg:
            continue
        if dst_name not in m.vertex_groups:
            m.vertex_groups.new(name=dst_name)
        dst_vg = m.vertex_groups[dst_name]
        mesh_mw = m.matrix_world
        plans = []
        for v in m.data.vertices:
            src_w = 0.0
            existing_dst = 0.0
            for g in v.groups:
                if g.group == src_vg.index:
                    src_w = g.weight
                elif g.group == dst_vg.index:
                    existing_dst = g.weight
            if src_w <= 0:
                continue
            v_w = mesh_mw @ v.co
            rel = v_w - seg_from_w
            t = rel.dot(seg_w) / seg_len_sq_w
            t = max(0.0, min(1.0, t))
            if t <= 0:
                continue
            perp_sq = rel.length_squared - t * t * seg_len_sq_w
            if perp_sq > perp_limit_sq:
                filtered += 1
                continue
            k = t
            src_factor = 1.0 - k * (1.0 - src_keep_floor)
            new_src = src_w * src_factor
            new_dst = existing_dst + src_w * k
            plans.append((v.index, new_src, new_dst))
        for v_idx, new_src, new_dst in plans:
            if new_src > 1e-6:
                src_vg.add([v_idx], new_src, 'REPLACE')
            else:
                src_vg.remove([v_idx])
            if new_dst > 1e-6:
                dst_vg.add([v_idx], new_dst, 'REPLACE')
            moved += 1
    return (moved, filtered)


def _transfer_shoulder_weight(obj, arm_name, elbow_name, shoulder_name,
                              zone=0.15, max_frac=0.85):
    """把 arm 顶点组中位于肩侧（沿 腕→ひじ 轴 t<0，即上臂头之上的三角肌/肩部）
    的权重按平滑比例转移到 shoulder。

    XPS/XNALara 把三角肌绑到"上臂"(shoulder2→腕)，而 MMD/目标绑到锁骨(肩)。
    不转移会导致手臂弯曲时整块肩部/三角肌跟着上臂转 → 肩膀变形。
    t∈(-zone,0) 平滑过渡，t<=-zone 转移 max_frac，避免硬缝。返回受影响顶点数。
    """
    arm_b = obj.data.bones.get(arm_name)
    el_b = obj.data.bones.get(elbow_name)
    sh_b = obj.data.bones.get(shoulder_name)
    if not (arm_b and el_b and sh_b):
        return 0
    mw = obj.matrix_world
    o = mw @ arm_b.head_local
    e = mw @ el_b.head_local
    axis = e - o
    L2 = axis.length_squared
    if L2 < 1e-9:
        return 0
    meshes = [m for m in bpy.data.objects
              if m.type == 'MESH' and any(md.type == 'ARMATURE' and md.object == obj for md in m.modifiers)]
    moved = 0
    for m in meshes:
        avg = m.vertex_groups.get(arm_name)
        if not avg:
            continue
        svg = m.vertex_groups.get(shoulder_name) or m.vertex_groups.new(name=shoulder_name)
        mesh_mw = m.matrix_world
        plans = []
        for v in m.data.vertices:
            w = 0.0
            for g in v.groups:
                if g.group == avg.index:
                    w = g.weight
                    break
            if w <= 0:
                continue
            t = ((mesh_mw @ v.co) - o).dot(axis) / L2
            if t >= 0:
                continue  # 只处理肩侧
            f = min(1.0, (-t) / zone) * max_frac
            if f <= 1e-4:
                continue
            plans.append((v.index, w * (1.0 - f), w * f))
        for vi, new_arm, to_sh in plans:
            if new_arm > 1e-6:
                avg.add([vi], new_arm, 'REPLACE')
            else:
                avg.remove([vi])
            svg.add([vi], to_sh, 'ADD')
            moved += 1
    return moved


class OBJECT_OT_complete_missing_bones(bpy.types.Operator):
    """补充缺失的 MMD 格式骨骼"""
    bl_idname = "object.complete_missing_bones"
    bl_label = "Complete Missing Bones"

    def connect_finger_bones(self, edit_bones):
        """连接手指骨骼的头尾"""
        finger_chains = [
            ["左親指０", "左親指１", "左親指２"],
            ["左人指１", "左人指２", "左人指３"],
            ["左中指１", "左中指２", "左中指３"],
            ["左薬指１", "左薬指２", "左薬指３"],
            ["左小指１", "左小指２", "左小指３"],
            ["右親指０", "右親指１", "右親指２"],
            ["右人指１", "右人指２", "右人指３"],
            ["右中指１", "右中指２", "右中指３"],
            ["右薬指１", "右薬指２", "右薬指３"],
            ["右小指１", "右小指２", "右小指３"]
        ]
        for chain in finger_chains:
            if all(bone in edit_bones for bone in chain):
                for i in range(len(chain) - 1):
                    current_bone = edit_bones[chain[i]]
                    next_bone = edit_bones[chain[i + 1]]
                    current_bone.tail = next_bone.head

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "没有选择骨架")
            return {'CANCELLED'}

        if context.mode != 'EDIT_ARMATURE':
            bpy.ops.object.mode_set(mode='EDIT')

        edit_bones = obj.data.edit_bones
        left_foot_bone = edit_bones.get("左足")
        right_foot_bone = edit_bones.get("右足")
        upper_body_bone = edit_bones.get("上半身")
        lower_body_bone = edit_bones.get("下半身")
        if left_foot_bone:
            left_foot_bone.use_connect = False
            left_foot_bone.parent = None
        if right_foot_bone:
            right_foot_bone.use_connect = False
            right_foot_bone.parent = None
        if upper_body_bone and upper_body_bone.parent:
            upper_body_bone.use_connect = False
            upper_body_bone.parent = None
        if lower_body_bone and lower_body_bone.parent:
            lower_body_bone.use_connect = False
            lower_body_bone.parent = None
        if not upper_body_bone:
            self.report({'ERROR'}, "上半身骨骼不存在")
            return {'CANCELLED'}
        upper_body_head = upper_body_bone.head.copy()
        upper_body_tail = upper_body_bone.tail.copy()

        bone_length = bone_utils.calculate_bone_length(edit_bones)

        # 先检测上半身链骨骼（上半身2, 上半身3, ...）
        upper_chain_bones = []
        for i in range(2, 6):
            name = f"上半身{i}"
            if edit_bones.get(name):
                upper_chain_bones.append(name)

        last_upper_body = upper_chain_bones[-1] if upper_chain_bones else "上半身"

        # 是否有左右足，决定腰キャンセル/足父级
        has_left_leg = bool(edit_bones.get("左足"))
        has_right_leg = bool(edit_bones.get("右足"))
        left_leg_parent = "腰キャンセル.L" if has_left_leg else "下半身"
        right_leg_parent = "腰キャンセル.R" if has_right_leg else "下半身"

        bone_properties = {
            "全ての親": {"head": Vector((0, 0, 0)), "tail": Vector((0, 0, bone_length)), "parent": None, "use_deform": False, "use_connect": False},
            "センター": {"head": Vector((0, 0, bone_length * 2)), "tail": Vector((0, 0, bone_length*1.1)), "parent": "全ての親", "use_deform": False, "use_connect": False},
            "グルーブ": {"head": Vector((0, 0, bone_length * 3.2)), "tail": Vector((0, 0, bone_length * 4)), "parent": "センター", "use_deform": False, "use_connect": False},
            "腰": {"head": Vector((0, upper_body_head.y + bone_length * 0.5, upper_body_head.z - bone_length * 0.5)), "tail": Vector((0, upper_body_head.y, upper_body_head.z)),
                "parent": "グルーブ", "use_deform": False, "use_connect": False},
            "上半身": {"head": Vector((0, upper_body_head.y, upper_body_head.z)),
                "tail": Vector((0, upper_body_tail.y, upper_body_head.z+bone_length)),
                "parent": "腰", "use_connect": False},
            "首": {
                "head": edit_bones["首"].head,
                "tail": edit_bones["頭"].head,
                "parent": last_upper_body,
                "use_connect": False
            },
            "頭": {
                "head": edit_bones["頭"].head,
                "tail": Vector((0, edit_bones["頭"].head.y, edit_bones["頭"].head.z+bone_length * 0.25)),
                "parent": "首",
                "use_connect": False
            },
            # 上肢骨骼链
            "左肩": {
                "head": edit_bones["左肩"].head,
                "tail": edit_bones["左腕"].head,
                "parent": last_upper_body,
                "use_connect": False
            },
            "左腕": {
                "head": edit_bones["左腕"].head,
                "tail": edit_bones["左ひじ"].head,
                "parent": "左肩",
                "use_connect": True
            },
            "左ひじ": {
                "head": edit_bones["左ひじ"].head,
                "tail": edit_bones["左手首"].head if edit_bones["左手首"]else edit_bones["左ひじ"].tail,
                "parent": "左腕",
                "use_connect": True
            },
            "左手首": {
                "head": edit_bones["左手首"].head,
                "tail": edit_bones["左中指１"].head.copy() if edit_bones.get("左中指１") else edit_bones["左手首"].tail,
                "parent": "左ひじ",
                "use_connect": False
            },

            "右肩": {
                "head": edit_bones["右肩"].head,
                "tail": edit_bones["右腕"].head,
                "parent": last_upper_body,
                "use_connect": False
            },
            "右腕": {
                "head": edit_bones["右腕"].head,
                "tail": edit_bones["右ひじ"].head,
                "parent": "右肩",
                "use_connect": True
            },
            "右ひじ": {
                "head": edit_bones["右ひじ"].head,
                "tail": edit_bones["右手首"].head if edit_bones["右手首"]else edit_bones["右ひじ"].tail,
                "parent": "右腕",
                "use_connect": True
            },
            "右手首": {
                "head": edit_bones["右手首"].head,
                "tail": edit_bones["右中指１"].head.copy() if edit_bones.get("右中指１") else edit_bones["右手首"].tail,
                "parent": "右ひじ",
                "use_connect": False
            },

            "下半身": {"head": Vector((0, upper_body_head.y, upper_body_head.z)), "tail": Vector((0, upper_body_head.y, upper_body_head.z - bone_length)), "parent": "腰", "use_connect": False},
        }

        # 腰キャンセル: 抵消腰旋转的控制骨 (付与親=腰, influence=-1.0, 在 OBJECT mode 设置)
        if has_left_leg:
            bone_properties["腰キャンセル.L"] = {
                "head": edit_bones["左足"].head.copy(),
                "tail": edit_bones["左足"].head + Vector((0, 0, bone_length * 0.5)),
                "parent": "下半身", "use_connect": False, "use_deform": False,
            }
        if has_right_leg:
            bone_properties["腰キャンセル.R"] = {
                "head": edit_bones["右足"].head.copy(),
                "tail": edit_bones["右足"].head + Vector((0, 0, bone_length * 0.5)),
                "parent": "下半身", "use_connect": False, "use_deform": False,
            }

        bone_properties.update({
            "左足": {
                "head": edit_bones["左足"].head,
                "tail": edit_bones["左ひざ"].head,
                "parent": left_leg_parent,
                "use_connect": False
            },
            "右足": {
                "head": edit_bones["右足"].head,
                "tail": edit_bones["右ひざ"].head,
                "parent": right_leg_parent,
                "use_connect": False
            },
            "左ひざ": {
                "head": edit_bones["左ひざ"].head,
                "tail": edit_bones["左足首"].head,
                "parent": "左足",
                "use_connect": False
            },
            "右ひざ": {
                "head": edit_bones["右ひざ"].head,
                "tail": edit_bones["右足首"].head,
                "parent": "右足",
                "use_connect": False
            },
            "左足首": {
                "head": edit_bones["左足首"].head,
                "tail": Vector((edit_bones["左足首"].head.x, edit_bones["左足首"].head.y - bone_length*0.3, 0)),
                "parent": "左ひざ",
                "use_connect": False
            },
            "右足首": {
                "head": edit_bones["右足首"].head,
                "tail": Vector((edit_bones["右足首"].head.x, edit_bones["右足首"].head.y - bone_length*0.3, 0)),
                "parent": "右ひざ",
                "use_connect": False
            },
            "左足先EX": {
                "head": edit_bones["左足首"].tail,
                "tail": Vector((edit_bones["左足首"].tail.x, edit_bones["左足首"].tail.y - bone_length*0.5, 0)),
                "parent": "左足首",
                "use_connect": False
            },
            "右足先EX": {
                "head": edit_bones["右足首"].tail,
                "tail": Vector((edit_bones["右足首"].tail.x, edit_bones["右足首"].tail.y - bone_length*0.5, 0)),
                "parent": "右足首",
                "use_connect": False
            }
        })

        # 上半身链 (上半身2..5): 尾部指向下一节, 首/肩 已挂到 last_upper_body
        if upper_chain_bones:
            for idx, bone_name in enumerate(upper_chain_bones):
                next_bone_name = upper_chain_bones[idx + 1] if idx + 1 < len(upper_chain_bones) else None
                if next_bone_name:
                    bone_properties[bone_name] = {
                        "head": Vector((0, edit_bones[bone_name].head.y, edit_bones[bone_name].head.z)),
                        "tail": Vector((0, edit_bones[next_bone_name].head.y, edit_bones[next_bone_name].head.z)),
                        "parent": upper_chain_bones[idx - 1] if idx > 0 else "上半身",
                        "use_connect": False
                    }
                else:
                    bone_properties[bone_name] = {
                        "head": Vector((0, edit_bones[bone_name].head.y, edit_bones[bone_name].head.z)),
                        "tail": Vector((0, edit_bones["首"].head.y, edit_bones["首"].head.z)),
                        "parent": upper_chain_bones[idx - 1] if idx > 0 else "上半身",
                        "use_connect": False
                    }

        # 上半身1 自动补全 + 权重分割: 在 上半身 与 第一节上半身链(上半身2) 之间
        first_upper_chain = upper_chain_bones[0] if upper_chain_bones else None
        upper1_just_created = False
        if (first_upper_chain and not edit_bones.get("上半身1")):
            ub_head = bone_properties["上半身"]["head"].copy()
            ub2_head = bone_properties[first_upper_chain]["head"].copy()
            mid = (ub_head + ub2_head) * 0.5
            if (ub2_head - ub_head).length > bone_length * 0.2:
                bone_properties["上半身"]["tail"] = mid.copy()
                bone_properties["上半身1"] = {
                    "head": mid.copy(), "tail": ub2_head.copy(),
                    "parent": "上半身", "use_connect": False, "use_deform": True,
                }
                bone_properties[first_upper_chain]["parent"] = "上半身1"
                upper1_just_created = True

        # 首1 自动补全 + 权重分割: 在 首 与 頭 之间
        neck1_just_created = False
        if (edit_bones.get("首") and edit_bones.get("頭") and not edit_bones.get("首1")):
            neck_head = bone_properties["首"]["head"].copy()
            head_head = bone_properties["頭"]["head"].copy()
            neck_mid = (neck_head + head_head) * 0.5
            if (head_head - neck_head).length > bone_length * 0.2:
                bone_properties["首"]["tail"] = neck_mid.copy()
                bone_properties["首1"] = {
                    "head": neck_mid.copy(), "tail": head_head.copy(),
                    "parent": "首", "use_connect": False, "use_deform": True,
                }
                bone_properties["頭"]["parent"] = "首1"
                neck1_just_created = True

        # 指根骨 (人指０/中指０/薬指０/小指０): pass-through, 不切权重
        finger_root_defs = [
            ("人指０", "人指１"), ("中指０", "中指１"),
            ("薬指０", "薬指１"), ("小指０", "小指１"),
        ]
        for side in ("左", "右"):
            wrist = edit_bones.get(f"{side}手首")
            if not wrist:
                continue
            for root_base, first_base in finger_root_defs:
                root_name = f"{side}{root_base}"
                first_name = f"{side}{first_base}"
                if edit_bones.get(root_name) or not edit_bones.get(first_name):
                    continue
                first_eb = edit_bones[first_name]
                bone_properties[root_name] = {
                    "head": (wrist.head + first_eb.head) * 0.5,
                    "tail": first_eb.head.copy(),
                    "parent": f"{side}手首", "use_connect": False, "use_deform": True,
                }
                bone_properties[first_name] = {
                    "head": first_eb.head.copy(), "tail": first_eb.tail.copy(),
                    "parent": root_name, "use_connect": False,
                }

        # 按顺序检查并创建或更新骨骼
        for bone_name, properties in bone_properties.items():
            if bone_name in ["左足先EX", "右足先EX"] and bone_name in edit_bones:
                original_head = edit_bones[bone_name].head.copy()
                bone_utils.create_or_update_bone(edit_bones, bone_name, original_head, properties["tail"], properties.get("use_connect", False), properties["parent"], properties.get("use_deform", True))
            else:
                bone_utils.create_or_update_bone(edit_bones, bone_name, properties["head"], properties["tail"], properties.get("use_connect", False), properties["parent"], properties.get("use_deform", True))

        if "左足先EX" in edit_bones:
            edit_bones["左足首"].tail = edit_bones["左足先EX"].head
        if "右足先EX" in edit_bones:
            edit_bones["右足首"].tail = edit_bones["右足先EX"].head

        # 二次 pass 修复 parent 依赖顺序问题 (字典中子骨可能先于父骨创建)
        for bone_name, properties in bone_properties.items():
            parent_name = properties.get("parent")
            if parent_name and bone_name in edit_bones:
                parent_bone = edit_bones.get(parent_name)
                if parent_bone and edit_bones[bone_name].parent != parent_bone:
                    edit_bones[bone_name].parent = parent_bone

        # unused bip001 pelvis → reparent 到 下半身 (XPS pelvis helper 跟随下半身)
        pelvis_bone = edit_bones.get("unused bip001 pelvis")
        lower_body = edit_bones.get("下半身")
        if pelvis_bone and lower_body:
            pelvis_bone.parent = lower_body

        # 设置 roll 值
        bone_utils.set_roll_values(edit_bones, bone_utils.DEFAULT_ROLL_VALUES)

        # 连接手指骨骼的头尾
        self.connect_finger_bones(edit_bones)

        # 中间骨权重分割 (回 OBJECT mode 改 vertex group)
        if upper1_just_created and first_upper_chain:
            bpy.ops.object.mode_set(mode='OBJECT')
            try:
                _split_chain_weights(obj, "上半身", "上半身1", "上半身", first_upper_chain)
            except Exception as e:
                print(f"[Convert_to_MMD] 上半身1 权重分割失败: {e}")
            bpy.ops.object.mode_set(mode='EDIT')

        if neck1_just_created:
            bpy.ops.object.mode_set(mode='OBJECT')
            try:
                _split_chain_weights(obj, "首", "首1", "首", "頭")
            except Exception as e:
                print(f"[Convert_to_MMD] 首1 权重分割失败: {e}")
            bpy.ops.object.mode_set(mode='EDIT')

        bpy.ops.object.mode_set(mode='OBJECT')
        # 腋窝平滑：肩→腕 追加权重 (additive, src_keep_floor=1.0 不削肩权重)
        # (肩部三角肌权重纠正改由独立步骤 object.fix_shoulder_weights 在流程后段执行，
        #  因为在 complete 内部此刻 肩/腕 骨的 data.bones 状态不稳定)
        for side_jp in ("左", "右"):
            shoulder = f"{side_jp}肩"
            arm_bone = f"{side_jp}腕"
            if obj.data.bones.get(shoulder) and obj.data.bones.get(arm_bone):
                try:
                    _split_chain_weights(obj, shoulder, arm_bone, shoulder, arm_bone, src_keep_floor=1.0)
                except Exception as e:
                    print(f"[Convert_to_MMD] 腋窝平滑 {shoulder} 失败: {e}")
        bpy.ops.object.mode_set(mode='EDIT')

        # 腰キャンセル 付与親设置 (OBJECT mode, 需要 mmd_tools)
        bpy.ops.object.mode_set(mode='OBJECT')
        for side in (".L", ".R"):
            name = f"腰キャンセル{side}"
            pb = obj.pose.bones.get(name)
            if pb and obj.pose.bones.get("腰") and hasattr(pb, "mmd_bone"):
                try:
                    pb.mmd_bone.has_additional_rotation = True
                    pb.mmd_bone.has_additional_location = False
                    pb.mmd_bone.additional_transform_bone = "腰"
                    pb.mmd_bone.additional_transform_influence = -1.0
                    pb.mmd_bone.is_tip = True
                except Exception as e:
                    print(f"[Convert_to_MMD] {name} 付与设置失败: {e}")
            bone = obj.data.bones.get(name)
            if bone:
                bone.hide = True
        bpy.ops.object.mode_set(mode='EDIT')

        return {'FINISHED'}


class OBJECT_OT_fix_shoulder_weights(bpy.types.Operator):
    """把上臂(腕)肩侧的三角肌/肩部权重转移到 肩，对齐目标 PMX 的肩部权重。

    XPS/XNALara 把三角肌绑到上臂(shoulder2→腕)，导致弯臂时肩膀跟着上臂变形。
    本步骤把 腕 顶点组中沿 腕→ひじ 轴 t<0(上臂头之上)的权重平滑转给 肩。
    """
    bl_idname = "object.fix_shoulder_weights"
    bl_label = "纠正肩部三角肌权重"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        if obj and obj.type != 'ARMATURE':
            obj = next((o for o in bpy.data.objects
                        if o.type == 'ARMATURE' and 'backup' not in o.name.lower()), None)
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "未找到骨架")
            return {'CANCELLED'}
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        total = 0
        for side in ("左", "右"):
            if obj.data.bones.get(f"{side}肩") and obj.data.bones.get(f"{side}腕") and obj.data.bones.get(f"{side}ひじ"):
                try:
                    n = _transfer_shoulder_weight(obj, f"{side}腕", f"{side}ひじ", f"{side}肩")
                    total += n
                    print(f"[Convert_to_MMD] 三角肌→{side}肩: {n} verts")
                except Exception as e:
                    print(f"[Convert_to_MMD] 三角肌转移 {side}肩 失败: {e}")
        self.report({'INFO'}, f"肩部三角肌权重纠正：转移 {total} 顶点")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_complete_missing_bones)
    bpy.utils.register_class(OBJECT_OT_fix_shoulder_weights)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_fix_shoulder_weights)
    bpy.utils.unregister_class(OBJECT_OT_complete_missing_bones)


if __name__ == "__main__":
    register()
