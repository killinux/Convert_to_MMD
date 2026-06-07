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
                              t_hi=0.35, max_frac=0.85):
    """把 arm 顶点组中肩部/三角肌区域的权重按沿 腕→ひじ 轴的位置平滑转移到 shoulder。

    XPS/XNALara 把三角肌绑到"上臂"(shoulder2→腕)，而 MMD/目标绑到锁骨(肩)。
    不转移会导致：① 手臂弯曲时整块肩部跟着上臂转；② 关节以下的三角肌随后被
    add_twist 切分吃到 腕捩1（25%付与）上 → 旋转 腕捩 时肩膀仍变形（本次修复点）。

    转移比例 f(t)（t=沿 腕→ひじ 轴，0=腕头/肩关节，1=肘）：
      t<=0      : f=max_frac              —— 肩盖/三角肌上部整体归 肩
      0<t<t_hi  : f=max_frac*(1-t/t_hi)   —— 关节以下三角肌线性递减，t_hi 处归零
      t>=t_hi   : 不转移                   —— 肘侧交给捩骨系统
    f 在 t=0 处连续（两侧皆为 max_frac），不会在肩关节留硬缝。曲线按目标 PMX 标定：
    肩占(肩+腕+腕捩1) 沿 t 约 86%/46%/28%/5%（t=0/0.1/0.2/0.3）。本步骤在 add_twist
    之前执行，先减薄 腕 再切分捩骨，使三角肌不落到 腕捩1。返回受影响顶点数。
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
            if t >= t_hi:
                continue  # 肘侧：保留在 腕，交给捩骨系统
            if t <= 0.0:
                f = max_frac           # 肩盖/三角肌上部：整体归 肩
            else:
                f = max_frac * (1.0 - t / t_hi)  # 关节以下：线性递减至 t_hi 归零
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
    """把上臂(腕)肩部/三角肌权重转移到 肩，对齐目标 PMX 的肩部权重。

    XPS/XNALara 把三角肌绑到上臂(shoulder2→腕)，导致弯臂时肩膀跟着上臂变形，
    且关节以下的三角肌会被后续 add_twist 切分到 腕捩1 上 → 旋转腕捩时肩膀仍变形。
    本步骤把 腕 顶点组中沿 腕→ひじ 轴 t<t_hi(肩关节到三角肌止点)的权重按目标曲线
    平滑转给 肩；必须在 add_twist 之前执行（流程 step 2.7）。
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


# 掌部权重重分配常量（按目标 PMX 手掌剖面标定）。
# 沿 手首→中指 的手深 d（0=腕关节，1=中指根）：d<LO 全留 手首，d>HI 全归掌骨，
# 之间线性退让。横向用「点到各指 指０→指１ 线段距离」的反 P 次方在四指间软混合。
PALM_D_LO = 0.10
PALM_D_HI = 0.80
PALM_LATERAL_POW = 2
PALM_FINGERS = ("人指", "中指", "薬指", "小指")


def _pt_seg_dist(p, a, b):
    ab = b - a
    L2 = ab.length_squared
    if L2 < 1e-12:
        return (p - a).length
    t = max(0.0, min(1.0, (p - a).dot(ab) / L2))
    return (p - (a + ab * t)).length


def _redistribute_palm_to_metacarpals(obj):
    """把 手首 顶点组落在手掌段的权重按就近手指列分给 指０ 掌骨（人指０/中指０/薬指０/小指０）。

    XPS 源无掌骨顶点组，complete_missing_bones 建的 指０ 为 pass-through 空骨 → 手掌权重
    全压在 手首 上、整掌随手腕刚性转。本步骤复刻目标 PMX 的手掌分布：沿手深 d 让 手首
    退让、掌骨接管（守恒，逐顶点总权重不变），横向在相邻掌骨间按距离软混合避免硬缝。
    亲指(親指０)已由 XPS 绑定，不参与。返回受影响顶点数。"""
    meshes = [m for m in bpy.data.objects
              if m.type == 'MESH' and any(md.type == 'ARMATURE' and md.object == obj for md in m.modifiers)]
    mw = obj.matrix_world
    total = 0
    for side in ("左", "右"):
        bw = obj.data.bones.get(f"{side}手首")
        bmid = obj.data.bones.get(f"{side}中指１")
        cols = [(f, obj.data.bones.get(f"{side}{f}０")) for f in PALM_FINGERS]
        cols = [(f, b) for f, b in cols if b]
        if not bw or not bmid or len(cols) < 2:
            continue
        W = mw @ bw.head_local
        H = (mw @ bmid.head_local) - W
        Hlen = H.length
        if Hlen < 1e-6:
            continue
        Hn = H / Hlen
        rays = [(f, mw @ b.head_local, mw @ obj.data.bones[f"{side}{f}１"].head_local)
                for f, b in cols if obj.data.bones.get(f"{side}{f}１")]
        if len(rays) < 2:
            continue
        for m in meshes:
            wvg = m.vertex_groups.get(f"{side}手首")
            if not wvg:
                continue
            mvg = {f: (m.vertex_groups.get(f"{side}{f}０") or m.vertex_groups.new(name=f"{side}{f}０"))
                   for f, _, _ in rays}
            mmw = m.matrix_world
            plans = []
            for v in m.data.vertices:
                w = 0.0
                for g in v.groups:
                    if g.group == wvg.index:
                        w = g.weight
                        break
                if w <= 1e-6:
                    continue
                vp = mmw @ v.co
                d = (vp - W).dot(Hn) / Hlen
                if d <= PALM_D_LO:
                    continue
                frac = min(1.0, (d - PALM_D_LO) / (PALM_D_HI - PALM_D_LO))
                if frac <= 1e-4:
                    continue
                moved = w * frac
                wt = {}
                sw = 0.0
                for f, a, b in rays:
                    ww = 1.0 / (_pt_seg_dist(vp, a, b) ** PALM_LATERAL_POW + 1e-9)
                    wt[f] = ww
                    sw += ww
                plans.append((v.index, w - moved, {f: moved * wt[f] / sw for f in wt}))
            for vidx, new_w, add in plans:
                if new_w > 1e-6:
                    wvg.add([vidx], new_w, 'REPLACE')
                else:
                    wvg.remove([vidx])
                for f, aw in add.items():
                    if aw > 1e-6:
                        mvg[f].add([vidx], aw, 'ADD')
                total += 1
    return total


# 拇指根渗出修正常量：親指０(拇指掌骨)落在拇指根之后(沿 親指０→親指１ 轴 u<0)的权重
# 视为渗到手腕内侧，按 u 渐变还给 手首。U_HI 以上保留(拇指本体/thenar)，U_LO 以下全还。
# 标定自目标 PMX：目标 親指０ 仅 ~1% 在拇指根之后，XPS 源高达 29–41%(拇指弯曲会拉扯腕皮)。
THUMB_U_HI = -0.1
THUMB_U_LO = -0.5


def _debleed_thumb_to_wrist(obj):
    """把 親指０ 渗到手腕内侧的权重还给 手首（守恒，逐顶点总权重不变）。

    XPS 把拇指掌骨绑得过宽，親指０ 顶点组有 29–41% 的权重落在拇指根之后的手腕上
    (目标仅 ~1%)。沿拇指轴 u(0=親指０头,1=親指１)把 u<THUMB_U_HI 的 親指０ 权重按渐变
    转给 手首。在掌部分配前执行。返回受影响顶点数。"""
    meshes = [m for m in bpy.data.objects
              if m.type == 'MESH' and any(md.type == 'ARMATURE' and md.object == obj for md in m.modifiers)]
    mw = obj.matrix_world
    span = max(THUMB_U_HI - THUMB_U_LO, 1e-6)
    total = 0
    for side in ("左", "右"):
        b0 = obj.data.bones.get(f"{side}親指０")
        b1 = obj.data.bones.get(f"{side}親指１")
        if not b0 or not b1 or not obj.data.bones.get(f"{side}手首"):
            continue
        A = mw @ b0.head_local
        axis = (mw @ b1.head_local) - A
        Lt = axis.length
        if Lt < 1e-6:
            continue
        axn = axis / Lt
        for m in meshes:
            t0 = m.vertex_groups.get(f"{side}親指０")
            if not t0:
                continue
            wf = m.vertex_groups.get(f"{side}手首") or m.vertex_groups.new(name=f"{side}手首")
            mmw = m.matrix_world
            plans = []
            for v in m.data.vertices:
                w = 0.0
                for g in v.groups:
                    if g.group == t0.index:
                        w = g.weight
                        break
                if w <= 1e-6:
                    continue
                u = ((mmw @ v.co) - A).dot(axn) / Lt
                if u >= THUMB_U_HI:
                    continue
                moved = w * min(1.0, (THUMB_U_HI - u) / span)
                if moved > 1e-6:
                    plans.append((v.index, w - moved, moved))
            for vidx, neww, moved in plans:
                if neww > 1e-6:
                    t0.add([vidx], neww, 'REPLACE')
                else:
                    t0.remove([vidx])
                wf.add([vidx], moved, 'ADD')
                total += 1
    return total


class OBJECT_OT_fix_palm_weights(bpy.types.Operator):
    """手部权重修正：先把 親指０ 渗到手腕的权重还给 手首，再把 手首 手掌段分给 指０ 掌骨，复刻目标 PMX。

    XPS 源没有掌骨权重，补全骨骼建的 人指０/中指０/薬指０/小指０ 为空 pass-through 骨，
    导致整个手掌刚性跟随 手首。本步骤在 add_twist（手首前臂侧回收）之后执行（流程 7.5），
    对最终 手首 的手掌段做守恒重分配。"""
    bl_idname = "object.fix_palm_weights"
    bl_label = "掌部权重分给掌骨"
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
        try:
            nt = _debleed_thumb_to_wrist(obj)
            n = _redistribute_palm_to_metacarpals(obj)
        except Exception as e:
            self.report({'ERROR'}, f"手部权重修正失败: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"手部权重修正: 拇指渗出 {nt} 顶点→手首, 掌部 {n} 顶点→掌骨")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_complete_missing_bones)
    bpy.utils.register_class(OBJECT_OT_fix_shoulder_weights)
    bpy.utils.register_class(OBJECT_OT_fix_palm_weights)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_fix_palm_weights)
    bpy.utils.unregister_class(OBJECT_OT_fix_shoulder_weights)
    bpy.utils.unregister_class(OBJECT_OT_complete_missing_bones)


if __name__ == "__main__":
    register()
