import bpy
from mathutils import Vector

# 捩骨子骨的标准付与跟随系数（沿肩→肘递增）
TWIST_SUB_INFLUENCE = {1: 0.25, 2: 0.50, 3: 0.75}


class OBJECT_OT_add_twist_bone(bpy.types.Operator):
    """对腕部和手部骨骼进行捩骨设置（统一走 mmd_tools 付与，不再建 _shadow_/_dummy_ 约束）"""
    bl_idname = "object.add_twist_bone"
    bl_label = "添加腕捩骨骼"
    bl_options = {'REGISTER', 'UNDO'}

    # 主捩骨 + 三个细分捩骨；base 是被切分的源骨
    twist_bones_def = [
        ("左腕",  ["左腕捩", "左腕捩1", "左腕捩2", "左腕捩3"]),
        ("左ひじ", ["左手捩", "左手捩1", "左手捩2", "左手捩3"]),
        ("右腕",  ["右腕捩", "右腕捩1", "右腕捩2", "右腕捩3"]),
        ("右ひじ", ["右手捩", "右手捩1", "右手捩2", "右手捩3"]),
    ]

    # 主捩骨 head 在 base 上的位置；细分骨 head 位置（沿 base 轴 0=头 1=尾）
    _POS = {0: 0.80, 1: 0.20, 2: 0.40, 3: 0.60}

    def execute(self, context):
        obj = context.active_object
        if obj is None or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架对象")
            return {'CANCELLED'}

        bpy.ops.object.mode_set(mode='EDIT')
        edit_bones = obj.data.edit_bones

        for bone_name, twist_names in self.twist_bones_def:
            if bone_name not in edit_bones:
                continue
            base_bone = edit_bones[bone_name]
            children_bones = [c for c in edit_bones if c.parent == base_bone]

            bone_head = base_bone.head.copy()
            bone_vec = base_bone.tail - base_bone.head
            length = bone_vec.length
            if length < 1e-6:
                continue
            dirn = bone_vec.normalized()
            seg_len = max(length * 0.12, 1e-4)
            roll = base_bone.roll  # 关键：所有捩骨与 base 同 roll，付与才不会扭歪

            created = {}
            for i, tname in enumerate(twist_names):
                tb = edit_bones.get(tname) or edit_bones.new(tname)
                h = bone_head + bone_vec * self._POS[i]
                tb.head = h
                tb.tail = h + dirn * seg_len   # 与 base 同方向（共线）
                tb.roll = roll                 # 与 base 同 roll
                tb.use_connect = False
                tb.parent = base_bone
                tb.use_deform = True
                created[tname] = tb

            # base 的原子骨（如 ひじ / 手首）挂到主捩骨下，使其跟随捩转
            main_tb = created[twist_names[0]]
            for child in children_bones:
                oh, ot = child.head.copy(), child.tail.copy()
                child.parent = main_tb
                child.use_connect = False
                child.head, child.tail = oh, ot

        bpy.ops.object.mode_set(mode='OBJECT')
        self.setup_weights(obj)          # 切分权重（守恒，scale=1.0）
        self.setup_grants(obj)           # 设置 mmd_bone 付与 + 锁主捩骨轴
        bpy.context.view_layer.objects.active = obj
        try:
            bpy.ops.object.create_bone_group()
        except Exception as e:
            print(f"[add_twist] create_bone_group: {e}")
        self.report({'INFO'}, "已设置捩骨：权重守恒切分 + mmd_tools 付与（单一驱动）")
        return {'FINISHED'}

    # ------------------------------------------------------------------
    # 主捩骨的軸固定方向来源：腕捩沿 腕→ひじ；手捩沿 ひじ→手首
    _AXIS_SRC = {"腕捩": ("腕", "ひじ"), "手捩": ("ひじ", "手首")}

    def setup_grants(self, obj):
        """统一走 mmd_tools 付与 + 軸固定（与目标 PMX 一致）：
        - 主捩骨设 軸固定=手臂方向（锁定扭转轴）+ 锁 X/Z 旋转，仅绕轴扭转；
        - 细分捩骨用 mmd_bone 付与跟随主捩骨 0.25/0.5/0.75。
        軸固定是目标模型有、原实现缺失的关键项——没有它 VMD 给腕捩的大角度旋转
        会带非扭转分量，经付与传到上臂 → 扭曲。
        """
        if bpy.context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        pose_bones = obj.pose.bones
        bones = obj.data.bones
        for side in ('左', '右'):
            for stem in ('腕捩', '手捩'):
                main = f"{side}{stem}"
                main_pb = pose_bones.get(main)
                if main_pb is None:
                    continue
                # 主捩骨：軸固定 = 手臂方向（armature 局部空间，归一化）
                a_name, b_name = self._AXIS_SRC[stem]
                ba = bones.get(f"{side}{a_name}")
                bb = bones.get(f"{side}{b_name}")
                if ba and bb:
                    axis = (bb.head_local - ba.head_local)
                    if axis.length > 1e-6:
                        axis = axis.normalized()
                        # mmd_bone.fixed_axis 以 MMD 坐标系存储 = Blender (x, z, -y)。
                        # 直接写 Blender 向量会导致导出 PMX 的扭转轴指向错误。
                        mb = main_pb.mmd_bone
                        mb.enabled_fixed_axis = True
                        mb.fixed_axis = (axis.x, axis.z, -axis.y)
                # 仅允许绕骨向（局部Y）旋转，作为扭转控制
                main_pb.lock_location = (True, True, True)
                main_pb.lock_rotation = (True, False, True)
                # 细分捩骨：mmd_bone 付与跟随主捩骨
                for i, infl in TWIST_SUB_INFLUENCE.items():
                    sub = f"{side}{stem}{i}"
                    pb = pose_bones.get(sub)
                    if pb is None or not hasattr(pb, 'mmd_bone'):
                        continue
                    mb = pb.mmd_bone
                    mb.has_additional_rotation = True
                    mb.has_additional_location = False
                    mb.additional_transform_bone = main
                    mb.additional_transform_influence = infl
                    pb.lock_location = (True, True, True)

    # ------------------------------------------------------------------
    def setup_weights(self, obj):
        """沿 base 轴 5 段线性切分权重（守恒，不缩水）。"""
        mesh_objects = [o for o in bpy.context.scene.objects
                        if o.type == 'MESH' and o.parent == obj]
        for mesh in mesh_objects:
            vgroups = mesh.vertex_groups
            for side in ("左", "右"):
                self._split_segment(obj, mesh, vgroups, f"{side}腕",
                                    [f"{side}腕捩", f"{side}腕捩1", f"{side}腕捩2", f"{side}腕捩3"])
                self._split_segment(obj, mesh, vgroups, f"{side}ひじ",
                                    [f"{side}手捩", f"{side}手捩1", f"{side}手捩2", f"{side}手捩3"])

    # 扭转目标曲线：沿骨轴 t∈[0,1] → 扭转比例 τ∈[0,1]
    # 肩侧 (t<TAU_LO) 不扭转；肘侧 (t>TAU_HI) 全扭转；中间平滑过渡。
    TAU_LO = 0.20
    TAU_HI = 0.80

    def _split_segment(self, obj, mesh, vgroups, base_name, twist_names):
        """沿骨轴把 base 权重按"扭转目标 τ"在相邻两根捩骨间线性插值分配。

        捩骨扭转档位: 腕=0, 捩1=0.25, 捩2=0.5, 捩3=0.75, 主捩=1.0。
        顶点 τ(t) 落在哪两档之间，就按比例分给那两根 → 平滑重叠、扭转量连续、权重守恒。
        这与参考 PMX 的捩骨权重分布一致，避免硬分段造成的折痕/扭曲。
        """
        if base_name not in vgroups:
            return
        main_name, t1, t2, t3 = twist_names
        base_group = vgroups[base_name]
        for n in twist_names:
            if n not in vgroups:
                vgroups.new(name=n)
        # 扭转档位（升序）：(顶点组, 扭转比例)
        levels = [
            (base_group, 0.0),
            (vgroups[t1], 0.25),
            (vgroups[t2], 0.50),
            (vgroups[t3], 0.75),
            (vgroups[main_name], 1.0),
        ]

        pb = obj.pose.bones.get(base_name)
        if pb is None:
            return
        head_w = obj.matrix_world @ pb.head
        tail_w = obj.matrix_world @ pb.tail
        axis = tail_w - head_w
        L = axis.length
        if L < 1e-6:
            return
        axis_n = axis / L
        span = max(self.TAU_HI - self.TAU_LO, 1e-6)

        for v in mesh.data.vertices:
            w = 0.0
            for g in v.groups:
                if g.group == base_group.index:
                    w = g.weight
                    break
            if w <= 0:
                continue
            base_group.remove([v.index])

            vw = mesh.matrix_world @ v.co
            t = (vw - head_w).dot(axis_n) / L
            tau = max(0.0, min(1.0, (t - self.TAU_LO) / span))

            # 找到 τ 落入的相邻两档 [k, k+1]，线性插值
            for k in range(len(levels) - 1):
                f0 = levels[k][1]
                f1 = levels[k + 1][1]
                if (f0 <= tau <= f1) or (k == len(levels) - 2):
                    a = (tau - f0) / (f1 - f0) if f1 > f0 else 0.0
                    a = max(0.0, min(1.0, a))
                    # 守恒：两档权重之和 == 原 base 权重 w；'ADD' 叠加到既有
                    if (1.0 - a) * w > 1e-6:
                        levels[k][0].add([v.index], (1.0 - a) * w, 'ADD')
                    if a * w > 1e-6:
                        levels[k + 1][0].add([v.index], a * w, 'ADD')
                    break


def register():
    bpy.utils.register_class(OBJECT_OT_add_twist_bone)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_add_twist_bone)


if __name__ == "__main__":
    register()
