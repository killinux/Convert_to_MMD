"""物理刚体 / 关节自动生成 (身体主干 + 头发 + 胸部)。

3 个独立 operator，都走 mmd_tools Model API：
- generate_body_rigid_bodies: MMD 标准骨胶囊静态刚体 (用于碰撞)
- generate_hair_physics: 头发 bone 链 dynamic 刚体 + joint
- generate_breast_physics: 胸部球形 dynamic_bone 刚体 + joint

R16: mmd_tools API 运行时 guard。
R17: 胸部不做参数自动 loop，preset 默认写死 mass=1.0/damp=0.5。
R2/R3: 只读 use_deform，不碰权重；_dummy_/_shadow_ 前缀过滤。
"""
import bpy
import math
from mathutils import Vector, Matrix, Euler

try:
    from mmd_tools.core.model import Model as _MMDModel
    from mmd_tools.core import rigid_body as _rb_mod
    MMD_TOOLS_OK = (
        hasattr(_MMDModel, 'createRigidBody') and
        hasattr(_MMDModel, 'createJoint') and
        hasattr(_MMDModel, 'findRoot')
    )
except ImportError:
    _MMDModel = None
    _rb_mod = None
    MMD_TOOLS_OK = False


# mmd_tools constants
SHAPE_SPHERE = 0
SHAPE_BOX = 1
SHAPE_CAPSULE = 2
MODE_STATIC = 0
MODE_DYNAMIC = 1
MODE_DYNAMIC_BONE = 2

# Body Builder / Oomary 明文基线
DEFAULT_MASS = 1.0
DEFAULT_DAMP = 0.5
DEFAULT_FRICTION = 0.5
DEFAULT_BOUNCE = 0.0
JOINT_ROT_LIMIT_DEG = 10.0
BREAST_JOINT_ROT_X_DEG = 10.0
BREAST_JOINT_ROT_Y_DEG = 3.0
BREAST_JOINT_ROT_Z_DEG = 5.0
BREAST_SPRING_ANGULAR = 100.0

# Naming prefixes for repeat-run cleanup
PREFIX_BODY = "auto_rb_body_"
PREFIX_HAIR = "auto_rb_hair_"
PREFIX_BREAST = "auto_rb_breast_"
PREFIX_JOINT_BODY = "J.auto_rb_body_"  # mmd_tools prefixes joint name with 'J.'
PREFIX_JOINT_HAIR = "J.auto_rb_hair_"
PREFIX_JOINT_BREAST = "J.auto_rb_breast_"


D_BONE_ORIGINAL = {
    '左足D': '左足',   '右足D': '右足',
    '左ひざD': '左ひざ', '右ひざD': '右ひざ',
    '左足首D': '左足首', '右足首D': '右足首',
    '足D.L': '足.L', '足D.R': '足.R',
    'ひざD.L': 'ひざ.L', 'ひざD.R': 'ひざ.R',
    '足首D.L': '足首.L', '足首D.R': '足首.R',
}

TWIST_BONE_FALLBACK = {
    'ひじ.L': ['手捩.L', '手捩1.L', '手捩2.L', '手捩3.L'],
    'ひじ.R': ['手捩.R', '手捩1.R', '手捩2.R', '手捩3.R'],
    '腕.L': ['腕捩.L', '腕捩1.L', '腕捩2.L', '腕捩3.L'],
    '腕.R': ['腕捩.R', '腕捩1.R', '腕捩2.R', '腕捩3.R'],
    '足D.L': ['足.L'], '足D.R': ['足.R'],
    'ひざD.L': ['ひざ.L'], 'ひざD.R': ['ひざ.R'],
    '左足D': ['左足'], '右足D': ['右足'],
    '左ひざD': ['左ひざ'], '右ひざD': ['右ひざ'],
}

HAIR_KEYWORDS = ('hair', '髪', 'kami', 'ponytail', 'fringe', 'bang')
BREAST_KEYWORDS = ('bust', 'breast', 'boob', 'chest', '胸', '乳', 'oppai', 'pectoral')
# TODO(generalize): parent 候选列表，不同 rig 可能不同
BREAST_PARENT_CANDIDATES = ('上半身2', '上半身', '上半身3', 'Chest', 'chest',
                            'chestLower', 'chestUpper')  # valid parents


# ============================================================
# Shared helpers
# ============================================================

def _get_model(armature_obj, operator=None):
    """Find MMDModel wrapping this armature. Return None if missing."""
    if not MMD_TOOLS_OK:
        if operator:
            operator.report({'ERROR'}, "mmd_tools 未装或版本过旧 (缺 Model.createRigidBody)")
        return None
    root = _MMDModel.findRoot(armature_obj)
    if root is None:
        if operator:
            operator.report(
                {'ERROR'},
                "未找到 MMD root。请先运行 '5. 使用mmd_tools转换格式' 之后再点此",
            )
        return None
    return _MMDModel(root)


def _bone_world(armature_obj, bone_name):
    """返回 (head_world, tail_world, length, y_axis_world)."""
    b = armature_obj.data.bones[bone_name]
    mw = armature_obj.matrix_world
    head_world = mw @ b.head_local
    tail_world = mw @ b.tail_local
    length = (tail_world - head_world).length
    y_axis = (tail_world - head_world).normalized() if length > 1e-6 else Vector((0, 1, 0))
    return head_world, tail_world, length, y_axis


def _bone_world_matrix(armature_obj, bone_name):
    """Return world-space matrix of bone's local axes (for rigid body rotation)."""
    b = armature_obj.data.bones[bone_name]
    return (armature_obj.matrix_world @ b.matrix_local)


def pick_deform_bone(armature_obj, preferred_names):
    """按优先级返回第一个存在且 use_deform=True 的骨名；跳过 _dummy_/_shadow_ 前缀。"""
    bones = armature_obj.data.bones
    for name in preferred_names:
        if name.startswith(('_dummy_', '_shadow_')):
            continue
        b = bones.get(name)
        if b is not None and b.use_deform:
            return name
    return None


def _euler_from_bone(armature_obj, bone_name):
    """Bone's world-space rotation as Euler (for rigid body placement)."""
    mat = _bone_world_matrix(armature_obj, bone_name)
    return mat.to_euler()


def _mask_all_false():
    return [False] * 16


def _mask_only_self(group_idx):
    """mask blocks only its own group (avoid self-collisions within dynamic chain)."""
    m = [False] * 16
    m[group_idx] = True
    return m


def _mask_block_body_and_self(group_idx):
    """Block body (group 0) AND own group to avoid Bullet overlap explosion."""
    m = [False] * 16
    m[0] = True
    m[group_idx] = True
    return m


def _clear_by_prefix(prefixes):
    """Remove all rigid bodies / joints whose name starts with any of prefixes."""
    to_remove = []
    for obj in bpy.data.objects:
        name = obj.name
        if obj.mmd_type in ('RIGID_BODY', 'JOINT'):
            for p in prefixes:
                if name.startswith(p):
                    to_remove.append(obj)
                    break
    for obj in to_remove:
        try:
            bpy.data.objects.remove(obj, do_unlink=True)
        except Exception as e:
            print(f"[xps_physics] clear {obj.name} 失败: {e}")
    return len(to_remove)


def _find_mesh_objects(armature_obj):
    """Return all mesh objects using this armature (via modifier or parenting)."""
    result = []
    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        for mod in obj.modifiers:
            if mod.type == 'ARMATURE' and mod.object is armature_obj:
                result.append(obj)
                break
        else:
            if obj.parent is armature_obj:
                result.append(obj)
    return result


def _compute_capsule_radius(armature_obj, bone_name, mesh_objects,
                            percentile=0.60, weight_threshold=0.2):
    """Compute capsule radius from vertex group perpendicular distance to bone axis."""
    b = armature_obj.data.bones.get(bone_name)
    if b is None:
        return None
    mw = armature_obj.matrix_world
    head_w = mw @ b.head_local
    tail_w = mw @ b.tail_local
    bone_vec = tail_w - head_w
    bone_len = bone_vec.length
    if bone_len < 1e-6:
        return None
    bone_dir = bone_vec / bone_len

    vg_names = [bone_name]
    if bone_name in TWIST_BONE_FALLBACK:
        vg_names.extend(TWIST_BONE_FALLBACK[bone_name])

    radii = []
    for mesh_obj in mesh_objects:
        mesh_mw = mesh_obj.matrix_world
        vg_indices = set()
        for vg_name in vg_names:
            vg = mesh_obj.vertex_groups.get(vg_name)
            if vg is not None:
                vg_indices.add(vg.index)
        if not vg_indices:
            continue
        for v in mesh_obj.data.vertices:
            for g in v.groups:
                if g.group in vg_indices and g.weight >= weight_threshold:
                    to_v = mesh_mw @ v.co - head_w
                    perp = to_v - bone_dir * to_v.dot(bone_dir)
                    radii.append(perp.length)
                    break

    if not radii:
        return None
    radii.sort()
    idx = min(int(len(radii) * percentile), len(radii) - 1)
    return radii[idx]


def _auto_snap_soft_tissue(armature_obj):
    """Before generating body/breast physics, snap soft-tissue bones (乳奶.L/R)
    to vg center so rigid bodies don't get placed at backbone/offset positions.
    Silently swallow errors — physics generation should not be blocked."""
    try:
        from .xps_fixes_operator import snap_bone_to_vg_center, DEFAULT_SNAP_BONES
    except ImportError:
        return
    prev_mode = bpy.context.mode
    if prev_mode != 'OBJECT':
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception:
            pass
    for bn in DEFAULT_SNAP_BONES:
        try:
            res = snap_bone_to_vg_center(armature_obj, bn, threshold_m=0.05)
            if res.get('status') == 'snapped':
                print(f"[xps_physics auto-snap] {bn}: delta={res.get('delta_m')}m")
        except Exception as e:
            print(f"[xps_physics auto-snap] {bn} failed: {e}")


# ============================================================
# Body main skeleton — static collision capsules
# ============================================================

# Each entry: (name_j, [preferred_bone_names], shape_type)
# Optional 4th element: radius_override (absolute meters) for bones where
# mesh-based computation fails (head too big, shoulder vg too narrow, etc.)
BODY_BONE_SPEC = [
    # Spine
    ('上半身', ['上半身'], SHAPE_CAPSULE),
    ('上半身2', ['上半身2'], SHAPE_CAPSULE),
    ('上半身3', ['上半身3'], SHAPE_CAPSULE),
    ('下半身', ['下半身'], SHAPE_CAPSULE),
    # Head / neck
    ('首', ['首'], SHAPE_CAPSULE),
    ('頭', ['頭'], SHAPE_SPHERE),
    # Shoulders
    ('肩.L', ['左肩', '肩.L'], SHAPE_CAPSULE),
    ('肩.R', ['右肩', '肩.R'], SHAPE_CAPSULE),
    # Arms (prefer raw 腕 not 腕捩)
    ('腕.L', ['左腕', '腕.L'], SHAPE_CAPSULE),
    ('腕.R', ['右腕', '腕.R'], SHAPE_CAPSULE),
    ('ひじ.L', ['左ひじ', 'ひじ.L'], SHAPE_CAPSULE),
    ('ひじ.R', ['右ひじ', 'ひじ.R'], SHAPE_CAPSULE),
    ('手首.L', ['左手首', '手首.L'], SHAPE_CAPSULE),
    ('手首.R', ['右手首', '手首.R'], SHAPE_CAPSULE),
    # Legs (D-bone preferred over raw 足)
    ('足.L', ['左足D', '左足', '足.L'], SHAPE_CAPSULE),
    ('足.R', ['右足D', '右足', '足.R'], SHAPE_CAPSULE),
    ('ひざ.L', ['左ひざD', '左ひざ', 'ひざ.L'], SHAPE_CAPSULE),
    ('ひざ.R', ['右ひざD', '右ひざ', 'ひざ.R'], SHAPE_CAPSULE),
    ('足首.L', ['左足首D', '左足首', '足首.L'], SHAPE_CAPSULE),
    ('足首.R', ['右足首D', '右足首', '足首.R'], SHAPE_CAPSULE),
]


class OBJECT_OT_generate_body_rigid_bodies(bpy.types.Operator):
    """生成 MMD 身体主干静态刚体（胶囊）。先跑第 5 步 mmd_tools 转换 + D 骨/捩骨/肩P（如需）。"""
    bl_idname = "object.generate_body_rigid_bodies"
    bl_label = "生成身体刚体"
    bl_description = "按 MMD 标准骨名生成身体胶囊刚体（静态 collision），需先跑完主流水线第 5 步"

    def execute(self, context):
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选中骨架")
            return {'CANCELLED'}

        model = _get_model(arm, self)
        if model is None:
            return {'CANCELLED'}

        _auto_snap_soft_tissue(arm)

        cleared = _clear_by_prefix([PREFIX_BODY, PREFIX_JOINT_BODY])
        if cleared:
            print(f"[xps_physics body] cleared {cleared} previous objects")

        mesh_objects = _find_mesh_objects(arm)

        created = 0
        skipped = []
        for spec in BODY_BONE_SPEC:
            name_j, candidates, shape = spec[0], spec[1], spec[2]
            radius_override = spec[3] if len(spec) > 3 else None

            bone_name = pick_deform_bone(arm, candidates)
            if bone_name is None:
                skipped.append(name_j)
                continue
            head_w, tail_w, length, _ = _bone_world(arm, bone_name)
            if length < 1e-4:
                skipped.append(f"{name_j}(len=0)")
                continue

            eff_length = length
            orig_name = D_BONE_ORIGINAL.get(bone_name)
            if orig_name and orig_name in arm.data.bones:
                _, _, orig_len, _ = _bone_world(arm, orig_name)
                eff_length = orig_len

            mid = (head_w + tail_w) * 0.5
            rot = _euler_from_bone(arm, bone_name)

            if radius_override is not None:
                radius = radius_override
            else:
                vg_r = _compute_capsule_radius(arm, bone_name, mesh_objects)
                bl_r = eff_length * 0.2
                if vg_r is not None:
                    radius = max(vg_r, bl_r)
                else:
                    radius = bl_r

            if shape == SHAPE_SPHERE:
                size = (radius, radius, radius)
            else:
                size = (radius, eff_length, radius)

            try:
                rb = model.createRigidBody(
                    shape_type=shape,
                    location=mid,
                    rotation=rot,
                    size=size,
                    dynamics_type=MODE_STATIC,
                    collision_group_number=0,
                    collision_group_mask=_mask_all_false(),
                    name=f"{PREFIX_BODY}{name_j}",
                    name_e=name_j,
                    bone=bone_name,
                    friction=DEFAULT_FRICTION,
                    mass=DEFAULT_MASS,
                    angular_damping=DEFAULT_DAMP,
                    linear_damping=DEFAULT_DAMP,
                    bounce=DEFAULT_BOUNCE,
                )
                created += 1
                print(f"[xps_physics body] {name_j} → bone={bone_name} len={eff_length:.3f} r={radius:.3f}")
            except Exception as e:
                print(f"[xps_physics body] {name_j} 失败: {e}")
                skipped.append(name_j)

        # Restore armature as active (mmd_tools.createRigidBody changes it)
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception:
            pass
        for o in bpy.context.view_layer.objects:
            o.select_set(False)
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        try:
            bpy.ops.mmd_tools.build_rig()
        except Exception as e:
            print(f"[xps_physics body] build_rig 失败 (可忽略): {e}")

        msg = f"生成身体刚体 {created} 个; 跳过 {len(skipped)}"
        if skipped:
            msg += f" ({', '.join(skipped[:6])}{'...' if len(skipped)>6 else ''})"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


# ============================================================
# Hair physics — dynamic chains
# ============================================================

def _matches_keyword(name, keywords):
    low = name.lower()
    return any(k.lower() in low for k in keywords)


def _find_hair_root_bones(armature_obj):
    """Bones whose name matches hair keyword. Filter out _dummy_/_shadow_/_deform=False."""
    roots = []
    for b in armature_obj.data.bones:
        name = b.name
        if name.startswith(('_dummy_', '_shadow_')):
            continue
        if not b.use_deform:
            continue
        if not _matches_keyword(name, HAIR_KEYWORDS):
            continue
        # Also require parent is NOT itself a hair bone (to identify roots only)
        if b.parent and _matches_keyword(b.parent.name, HAIR_KEYWORDS):
            continue
        roots.append(b)
    return roots


def _walk_chain(bone, max_len=10):
    """Walk down a single-child chain (first deform child) up to max_len."""
    chain = [bone]
    cur = bone
    for _ in range(max_len - 1):
        # Pick first child that is deform + not _dummy_/_shadow_
        candidates = [
            c for c in cur.children
            if c.use_deform and not c.name.startswith(('_dummy_', '_shadow_'))
        ]
        if not candidates:
            break
        cur = candidates[0]
        chain.append(cur)
    return chain


def _create_joint_between(model, name, rigid_a, rigid_b, loc, rot,
                          max_rot_deg=10.0, spring_angular=0.0, spring_linear=0.0):
    """Wrapper for Model.createJoint with sensible defaults."""
    r = math.radians(max_rot_deg)
    return model.createJoint(
        name=name,
        name_e=name,
        location=loc,
        rotation=rot,
        rigid_a=rigid_a,
        rigid_b=rigid_b,
        maximum_location=(0, 0, 0),
        minimum_location=(0, 0, 0),
        maximum_rotation=(r, r, r),
        minimum_rotation=(-r, -r, -r),
        spring_angular=(spring_angular, spring_angular, spring_angular),
        spring_linear=(spring_linear, spring_linear, spring_linear),
    )


class OBJECT_OT_generate_hair_physics(bpy.types.Operator):
    """生成头发物理刚体链 + 关节。按关键词检测发根（hair/髪/kami/ponytail/fringe/bang）。"""
    bl_idname = "object.generate_hair_physics"
    bl_label = "生成头发物理"
    bl_description = "检测头发骨链自动生成 dynamic 刚体 + joint"

    MAX_CHAIN_LEN = 10

    def execute(self, context):
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选中骨架")
            return {'CANCELLED'}

        model = _get_model(arm, self)
        if model is None:
            return {'CANCELLED'}

        cleared = _clear_by_prefix([PREFIX_HAIR, PREFIX_JOINT_HAIR])
        if cleared:
            print(f"[xps_physics hair] cleared {cleared} previous objects")

        roots = _find_hair_root_bones(arm)
        if not roots:
            self.report({'INFO'}, "未检测到头发骨（关键词: hair/髪/kami/ponytail/fringe/bang）")
            return {'FINISHED'}

        total_rbs = 0
        total_jts = 0
        for root in roots:
            chain = _walk_chain(root, self.MAX_CHAIN_LEN)
            if len(chain) < 1:
                continue
            n = len(chain)
            prev_rb = None
            for i, b in enumerate(chain):
                head_w, tail_w, length, _ = _bone_world(arm, b.name)
                if length < 1e-4:
                    continue
                mid = (head_w + tail_w) * 0.5
                rot = _euler_from_bone(arm, b.name)
                radius = length * 0.4  # hair capsule radius
                size = (radius, length, radius)

                # gradient params: first node anchors (STATIC — follows bone exactly,
                # no physics response). DYNAMIC_BONE would still drift under gravity.
                if i == 0:
                    mode = MODE_STATIC
                    mass = 0.5
                    ang_damp = 0.999999
                    lin_damp = 0.5555
                else:
                    mode = MODE_DYNAMIC
                    t = i / max(1, n - 1)
                    mass = 0.5 - 0.45 * t
                    lin_damp = 0.5555 + (0.9 - 0.5555) * t
                    ang_damp = 0.999999 if i < n - 2 else 1.0

                name = f"{PREFIX_HAIR}{b.name}"
                try:
                    rb = model.createRigidBody(
                        shape_type=SHAPE_CAPSULE,
                        location=mid,
                        rotation=rot,
                        size=size,
                        dynamics_type=mode,
                        collision_group_number=1,
                        collision_group_mask=_mask_only_self(1),
                        name=name,
                        name_e=b.name,
                        bone=b.name,
                        friction=0.0,
                        mass=mass,
                        angular_damping=ang_damp,
                        linear_damping=lin_damp,
                        bounce=0.0,
                    )
                    total_rbs += 1
                except Exception as e:
                    print(f"[xps_physics hair] {b.name} rb 失败: {e}")
                    rb = None

                if rb is not None and prev_rb is not None:
                    # Joint at prev_bone.tail = this_bone.head = head_w
                    try:
                        _create_joint_between(
                            model,
                            name=f"{PREFIX_HAIR}{b.name}",  # joint gets 'J.' prefix auto
                            rigid_a=prev_rb,
                            rigid_b=rb,
                            loc=head_w,
                            rot=rot,
                            max_rot_deg=JOINT_ROT_LIMIT_DEG,
                        )
                        total_jts += 1
                    except Exception as e:
                        print(f"[xps_physics hair] joint {b.name} 失败: {e}")

                prev_rb = rb

        # Restore armature as active (mmd_tools.createRigidBody changes it)
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception:
            pass
        for o in bpy.context.view_layer.objects:
            o.select_set(False)
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        try:
            bpy.ops.mmd_tools.build_rig()
        except Exception as e:
            print(f"[xps_physics hair] build_rig 失败 (可忽略): {e}")

        self.report({'INFO'}, f"头发物理: {total_rbs} 刚体, {total_jts} 关节 ({len(roots)} 链)")
        return {'FINISHED'}


# ============================================================
# Breast physics — sphere dynamic_bone with spring joint to 上半身2
# ============================================================

def _find_breast_bones(armature_obj):
    results = []
    for b in armature_obj.data.bones:
        name = b.name
        if name.startswith(('_dummy_', '_shadow_')):
            continue
        if not b.use_deform:
            continue
        if not _matches_keyword(name, BREAST_KEYWORDS):
            continue
        # Parent must be a torso bone (exclude e.g. chest-as-spine-label)
        parent_name = b.parent.name if b.parent else ""
        if not any(p in parent_name for p in BREAST_PARENT_CANDIDATES):
            continue
        results.append(b)
    return results


class OBJECT_OT_generate_breast_physics(bpy.types.Operator):
    """生成胸部物理：球形 dynamic_bone 刚体 + joint 到上半身2。R17: 不做参数自动调 loop。"""
    bl_idname = "object.generate_breast_physics"
    bl_label = "生成胸部物理"
    bl_description = "对称胸部骨生成球形刚体 + 弹簧 joint"

    def execute(self, context):
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选中骨架")
            return {'CANCELLED'}

        model = _get_model(arm, self)
        if model is None:
            return {'CANCELLED'}

        _auto_snap_soft_tissue(arm)

        cleared = _clear_by_prefix([PREFIX_BREAST, PREFIX_JOINT_BREAST])
        if cleared:
            print(f"[xps_physics breast] cleared {cleared} previous objects")

        bones = _find_breast_bones(arm)
        if not bones:
            self.report({'INFO'}, "未检测到胸部骨（关键词 + 父骨在 上半身2/上半身 等）")
            return {'FINISHED'}

        # Find anchor bone (上半身2 preferred, fallback 上半身)
        anchor_name = pick_deform_bone(arm, ['上半身2', '上半身'])
        if anchor_name is None:
            self.report({'ERROR'}, "未找到 上半身2 / 上半身 作为锚骨")
            return {'CANCELLED'}
        anchor_rb = None
        for obj in bpy.data.objects:
            if obj.mmd_type == 'RIGID_BODY' and obj.mmd_rigid.bone == anchor_name:
                anchor_rb = obj
                break
        if anchor_rb is None:
            self.report(
                {'ERROR'},
                f"未找到 {anchor_name} 对应的刚体。请先点 '生成身体刚体'",
            )
            return {'CANCELLED'}

        total_rbs = 0
        total_jts = 0
        for b in bones:
            head_w, tail_w, length, _ = _bone_world(arm, b.name)
            if length < 1e-4:
                continue
            radius = max(length * 0.9, 0.03)
            size = (radius, radius, radius)
            mid = (head_w + tail_w) * 0.5
            rot = _euler_from_bone(arm, b.name)

            name = f"{PREFIX_BREAST}{b.name}"
            try:
                rb = model.createRigidBody(
                    shape_type=SHAPE_SPHERE,
                    location=mid,
                    rotation=rot,
                    size=size,
                    dynamics_type=MODE_DYNAMIC,
                    collision_group_number=2,
                    collision_group_mask=_mask_only_self(2),
                    name=name,
                    name_e=b.name,
                    bone=b.name,
                    friction=0.0,
                    mass=1.0,
                    angular_damping=0.5,
                    linear_damping=0.5,
                    bounce=0.0,
                )
                total_rbs += 1
            except Exception as e:
                print(f"[xps_physics breast] {b.name} rb 失败: {e}")
                continue

            try:
                rx = math.radians(BREAST_JOINT_ROT_X_DEG)
                ry = math.radians(BREAST_JOINT_ROT_Y_DEG)
                rz = math.radians(BREAST_JOINT_ROT_Z_DEG)
                model.createJoint(
                    name=f"{PREFIX_BREAST}{b.name}",
                    name_e=b.name,
                    location=head_w,
                    rotation=rot,
                    rigid_a=anchor_rb,
                    rigid_b=rb,
                    maximum_location=(0, 0, 0),
                    minimum_location=(0, 0, 0),
                    maximum_rotation=(rx, ry, rz),
                    minimum_rotation=(-rx, -ry, -rz),
                    spring_angular=(BREAST_SPRING_ANGULAR,) * 3,
                    spring_linear=(0, 0, 0),
                )
                total_jts += 1
            except Exception as e:
                print(f"[xps_physics breast] joint {b.name} 失败: {e}")

        # Restore armature as active (mmd_tools.createRigidBody changes it)
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception:
            pass
        for o in bpy.context.view_layer.objects:
            o.select_set(False)
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        try:
            bpy.ops.mmd_tools.build_rig()
        except Exception as e:
            print(f"[xps_physics breast] build_rig 失败 (可忽略): {e}")

        self.report({'INFO'}, f"胸部物理: {total_rbs} 刚体, {total_jts} 关节")
        return {'FINISHED'}


class OBJECT_OT_toggle_rigid_body_visibility(bpy.types.Operator):
    bl_idname = "object.toggle_rigid_body_visibility"
    bl_label = "显示/隐藏刚体"
    bl_description = "切换所有刚体和关节对象的视口可见性"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        targets = [obj for obj in bpy.data.objects
                   if getattr(obj, 'mmd_type', '') in ('RIGID_BODY', 'JOINT')]
        if not targets:
            self.report({'WARNING'}, "未找到刚体/关节对象")
            return {'CANCELLED'}
        currently_hidden = targets[0].hide_get()
        for obj in targets:
            obj.hide_set(not currently_hidden)
        state = "显示" if currently_hidden else "隐藏"
        self.report({'INFO'}, f"已{state} {len(targets)} 个刚体/关节")
        return {'FINISHED'}


_CLASSES = (
    OBJECT_OT_generate_body_rigid_bodies,
    OBJECT_OT_generate_hair_physics,
    OBJECT_OT_generate_breast_physics,
    OBJECT_OT_toggle_rigid_body_visibility,
)


def register():
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
