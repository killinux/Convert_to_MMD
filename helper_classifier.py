"""Helper bone classifier — position+parent based classification of non-standard bones.

Takes the skeleton identifier's mapping and classifies all remaining bones
into categories used by the pipeline (twist candidates, pelvis, preserve, etc.).

Categories:
    mapped   — already identified by skeleton_identifier
    twist    — on arm segment, handled by twist scanner
    pelvis   — map directly to 下半身
    preserve — keep XPS weights (thigh/hip/breast helpers)
    control  — non-deform control bones, transfer weights out
    ignore   — _dummy_/_shadow_/leaf with no significance
    other    — unclassified, transfer weights to nearest
"""

from mathutils import Vector


def classify_helpers(armature_data, skeleton_map):
    """Classify all non-standard bones by position + parent relationship.

    Args:
        armature_data: bpy.types.Armature
        skeleton_map: dict from identify_skeleton()

    Returns:
        dict: {bone_name: category_string}
    """
    bones = armature_data.bones
    mapped = set(v for v in skeleton_map.values() if v)

    segments = _build_segments(bones, skeleton_map)
    center_name = skeleton_map.get("center_bone", "")
    thigh_names = {skeleton_map.get("left_thigh_bone", ""),
                   skeleton_map.get("right_thigh_bone", "")} - {""}
    spine_names = {skeleton_map.get(k, "") for k in (
        "upper_body_bone", "upper_body1_bone", "upper_body2_bone",
        "upper_body3_bone")} - {""}

    head_name = skeleton_map.get("head_bone", "")
    hand_names = {skeleton_map.get("left_hand_bone", ""),
                  skeleton_map.get("right_hand_bone", "")} - {""}

    result = {}
    for bone in bones:
        name = bone.name
        if name in mapped:
            result[name] = "mapped"
            continue
        if name.startswith(("_dummy_", "_shadow_")):
            result[name] = "ignore"
            continue

        # Head descendants (hair/face) — before segment check to avoid
        # hair near arms being classified as twist
        if head_name and _is_descendant_of(bone, head_name):
            result[name] = "other"
            continue

        # Hand descendants (carpals, extra finger bones)
        if hand_names and _is_descendant_of_any(bone, hand_names):
            result[name] = "other"
            continue

        ancestor = _find_mapped_ancestor(bone, mapped)

        # Pelvis: DIRECT child of center, centered
        if ancestor == center_name and center_name:
            is_direct = bone.parent and bone.parent.name == center_name
            if is_direct and abs(bone.head_local.x) < 0.02:
                result[name] = "pelvis"
            elif abs(bone.head_local.x) >= 0.02:
                result[name] = "preserve"
            else:
                result[name] = "other"
            continue

        seg_type = _closest_segment_type(bone, segments)
        if seg_type in ("upper_arm", "forearm"):
            result[name] = "twist"
            continue
        if seg_type == "thigh":
            result[name] = "preserve"
            continue

        if ancestor in thigh_names:
            result[name] = "preserve"
            continue
        if ancestor in spine_names:
            if abs(bone.head_local.x) >= 0.02:
                result[name] = "preserve"  # breast/chest helper
            else:
                result[name] = "merge"  # intermediate spine segment — merge to nearest
            continue

        result[name] = "other"

    return result


def summary(classification):
    """Print a summary of the classification."""
    from collections import Counter
    counts = Counter(classification.values())
    lines = []
    for cat in ("mapped", "twist", "pelvis", "preserve", "merge", "control", "ignore", "other"):
        n = counts.get(cat, 0)
        if n:
            names = [k for k, v in classification.items() if v == cat]
            preview = ", ".join(names[:5])
            if len(names) > 5:
                preview += f" ... (+{len(names) - 5})"
            lines.append(f"  {cat:10s} {n:3d}  {preview}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------

def _build_segments(bones, smap):
    """Build body segment definitions from skeleton map."""
    segments = []
    for side in ("left", "right"):
        pairs = [
            ("upper_arm", f"{side}_upper_arm_bone", f"{side}_lower_arm_bone"),
            ("forearm", f"{side}_lower_arm_bone", f"{side}_hand_bone"),
            ("thigh", f"{side}_thigh_bone", f"{side}_calf_bone"),
        ]
        for seg_type, from_key, to_key in pairs:
            from_name = smap.get(from_key, "")
            to_name = smap.get(to_key, "")
            if not from_name or not to_name:
                continue
            from_bone = bones.get(from_name)
            to_bone = bones.get(to_name)
            if not from_bone or not to_bone:
                continue
            seg_vec = to_bone.head_local - from_bone.head_local
            seg_len = seg_vec.length
            if seg_len < 1e-5:
                continue
            segments.append((seg_type, from_bone.head_local, to_bone.head_local, seg_len))
    return segments


def _closest_segment_type(bone, segments):
    """Find which body segment a bone is closest to (if any)."""
    best_type = None
    best_perp = float("inf")
    pos = bone.head_local

    for seg_type, seg_from, seg_to, seg_len in segments:
        seg = seg_to - seg_from
        L_sq = seg.length_squared
        if L_sq < 1e-8:
            continue
        t = (pos - seg_from).dot(seg) / L_sq
        if not (-0.15 <= t <= 1.15):
            continue
        t_c = max(0.0, min(1.0, t))
        proj = seg_from + t_c * seg
        perp = (pos - proj).length
        if perp < seg_len * 0.35 and perp < best_perp:
            best_perp = perp
            best_type = seg_type

    return best_type


def _find_mapped_ancestor(bone, mapped_names):
    """Walk parent chain to find the first mapped ancestor."""
    cur = bone.parent
    while cur:
        if cur.name in mapped_names:
            return cur.name
        cur = cur.parent
    return None


def _is_descendant_of(bone, ancestor_name):
    """Check if bone is a descendant of the named bone."""
    cur = bone.parent
    while cur:
        if cur.name == ancestor_name:
            return True
        cur = cur.parent
    return False


def _is_descendant_of_any(bone, ancestor_names):
    """Check if bone is a descendant of any of the named bones."""
    cur = bone.parent
    while cur:
        if cur.name in ancestor_names:
            return True
        cur = cur.parent
    return False
