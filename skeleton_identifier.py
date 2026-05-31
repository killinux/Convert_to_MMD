"""Auto skeleton identifier — pure topology+geometry bone role detection.

Analyzes any humanoid armature and produces a bone role mapping dict
(same format as preset JSON files). No bone name dependency.

Algorithm:
1. Find spine chain: trace from highest center bone to root via parents
2. Find fork points: where arms/legs branch off laterally
3. Map spine segment bones to MMD roles
4. Trace arm chains: shoulder/upper_arm/forearm/hand + fingers
5. Trace leg chains: thigh/shin/foot/toe
6. Find eye bones near head (symmetric pair)
"""

from mathutils import Vector


def identify_skeleton(armature_data):
    """Analyze armature topology and geometry to identify bone roles.

    Args:
        armature_data: bpy.types.Armature (armature.data)

    Returns:
        dict matching preset JSON format with bone names filled in.
    """
    bones = armature_data.bones
    if not bones:
        return _empty_result()

    result = _empty_result()

    spine = _find_spine_chain(bones)
    if len(spine) < 2:
        return result

    leg_idx, arm_idx = _find_fork_points(spine)

    _map_spine(spine, leg_idx, arm_idx, result)

    if arm_idx is not None:
        _map_arms(spine, arm_idx, result)

    if leg_idx is not None:
        _map_legs(spine, leg_idx, result)

    if result["head_bone"]:
        _map_eyes(bones, result["head_bone"], result)

    return result


# ---------------------------------------------------------------------------
# Spine chain detection
# ---------------------------------------------------------------------------

def _find_spine_chain(bones):
    """Find the spine chain by tracing from the highest center bone to root."""
    all_z = [b.head_local.z for b in bones]
    height = max(all_z) - min(all_z)
    if height < 0.001:
        return list(bones)[:1]

    x_thresh = height * 0.1

    center = [b for b in bones if abs(b.head_local.x) < x_thresh]
    if not center:
        center = sorted(bones, key=lambda b: abs(b.head_local.x))[:5]

    # The head bone has bilateral children (both +X and -X sides: eyes, jaw, etc.)
    # Score by Z position + children bonus (head typically has many more children than jaw/eye)
    bilateral = [b for b in center
                 if any(c.head_local.x > 0.01 for c in b.children)
                 and any(c.head_local.x < -0.01 for c in b.children)]
    if bilateral:
        top = max(bilateral,
                  key=lambda b: b.head_local.z + 0.01 * min(len(b.children), 50))
    else:
        with_ch = [b for b in center if b.children]
        top = max(with_ch or center, key=lambda b: b.head_local.z)

    chain = []
    cur = top
    while cur:
        chain.append(cur)
        cur = cur.parent
    chain.reverse()
    return chain


def _find_fork_points(chain):
    """Find arm and leg fork indices on the spine chain.

    Returns (leg_fork_idx, arm_fork_idx).
    Uses two passes: direct children first, then grandchildren.
    """
    if len(chain) < 2:
        return None, None

    all_z = [b.head_local.z for b in chain]
    height = max(all_z) - min(all_z)
    x_thresh = max(height * 0.01, 0.01)

    chain_set = {b.name for b in chain}
    min_depth = 3
    forks = []

    for i, bone in enumerate(chain):
        if i >= len(chain) - 2:
            continue
        off = [c for c in bone.children if c.name not in chain_set]
        left = [c for c in off
                if c.head_local.x > x_thresh and _subtree_depth(c) >= min_depth]
        right = [c for c in off
                 if c.head_local.x < -x_thresh and _subtree_depth(c) >= min_depth]
        if left and right:
            forks.append(i)

    if len(forks) < 2:
        for i, bone in enumerate(chain):
            if i in forks or i >= len(chain) - 2:
                continue
            off = [c for c in bone.children if c.name not in chain_set]
            for oc in off:
                gc_left = [c for c in oc.children
                           if c.head_local.x > x_thresh and _subtree_depth(c) >= min_depth]
                gc_right = [c for c in oc.children
                            if c.head_local.x < -x_thresh and _subtree_depth(c) >= min_depth]
                if gc_left and gc_right:
                    forks.append(i)
                    break

    if not forks:
        return None, None

    if len(forks) == 1:
        idx = forks[0]
        mid_z = (chain[0].head_local.z + chain[-1].head_local.z) / 2
        if chain[idx].head_local.z > mid_z:
            return None, idx
        else:
            return idx, None

    forks.sort(key=lambda i: chain[i].head_local.z)
    return forks[0], forks[-1]


# ---------------------------------------------------------------------------
# Spine role mapping
# ---------------------------------------------------------------------------

def _map_spine(chain, leg_idx, arm_idx, result):
    """Assign spine chain bones to MMD roles."""
    if leg_idx is None and arm_idx is None:
        # Fallback: first = root, last = head, second-to-last = neck
        if len(chain) >= 2:
            result["all_parents_bone"] = chain[0].name
            result["head_bone"] = chain[-1].name
        if len(chain) >= 3:
            result["neck_bone"] = chain[-2].name
        return

    # Root / ground: bones before leg fork
    if leg_idx is not None and leg_idx > 0:
        result["all_parents_bone"] = chain[0].name

    # Center (hips): if the leg fork bone's parent is center-ish with only 1 child,
    # prefer the parent (e.g., "root hips" over "unused bip001 pelvis")
    if leg_idx is not None:
        fork = chain[leg_idx]
        if (leg_idx > 1
                and len(chain[leg_idx - 1].children) == 1
                and abs(chain[leg_idx - 1].head_local.x) < 0.01):
            result["center_bone"] = chain[leg_idx - 1].name
        else:
            result["center_bone"] = fork.name

    if arm_idx is None:
        # Only leg fork found — map remaining chain above legs
        above = chain[leg_idx + 1:]
        if len(above) >= 1:
            result["upper_body2_bone"] = above[0].name
        if len(above) >= 3:
            result["upper_body_bone"] = above[0].name
            result["upper_body2_bone"] = above[1].name
            result["neck_bone"] = above[-2].name if len(above) >= 3 else ""
            result["head_bone"] = above[-1].name
        elif len(above) == 2:
            result["neck_bone"] = above[0].name
            result["head_bone"] = above[1].name
        elif len(above) == 1:
            result["head_bone"] = above[0].name
        return

    if leg_idx is None:
        # Only arm fork found — set center as root
        result["center_bone"] = chain[0].name
        arm_idx_eff = arm_idx
    else:
        arm_idx_eff = arm_idx

    # Spine segments between hips and arm fork
    start = (leg_idx + 1) if leg_idx is not None else 1
    spine_seg = chain[start:arm_idx_eff]

    if len(spine_seg) >= 1:
        result["upper_body_bone"] = spine_seg[0].name
    if len(spine_seg) >= 2:
        result["upper_body1_bone"] = spine_seg[1].name
    # arm fork bone = upper_body2 (chest level)
    result["upper_body2_bone"] = chain[arm_idx_eff].name

    # Neck and head: above arm fork (head = last, neck = first above fork)
    above = chain[arm_idx_eff + 1:]
    if len(above) >= 2:
        result["head_bone"] = above[-1].name
        result["neck_bone"] = above[0].name
    elif len(above) == 1:
        result["head_bone"] = above[0].name


# ---------------------------------------------------------------------------
# Arm detection
# ---------------------------------------------------------------------------

def _map_arms(chain, arm_idx, result):
    """Identify arm chains from the arm fork point."""
    fork_bone = chain[arm_idx]
    chain_set = {b.name for b in chain}
    all_z = [b.head_local.z for b in chain]
    height = max(all_z) - min(all_z)
    x_thresh = max(height * 0.01, 0.01)

    off = [c for c in fork_bone.children if c.name not in chain_set]
    left = [c for c in off if c.head_local.x > x_thresh]
    right = [c for c in off if c.head_local.x < -x_thresh]

    # If no direct lateral children, check grandchildren
    if not left or not right:
        for oc in off:
            for gc in oc.children:
                if gc.head_local.x > x_thresh and not left:
                    left = [oc]
                elif gc.head_local.x < -x_thresh and not right:
                    right = [oc]

    left_start = _pick_arm_start(left)
    right_start = _pick_arm_start(right)

    if left_start:
        _assign_arm(left_start, True, result)
    if right_start:
        _assign_arm(right_start, False, result)


def _pick_arm_start(candidates):
    """Among lateral children, pick the one most likely to be the arm chain start."""
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    with_hand = [c for c in candidates if _has_hand_descendant(c)]
    if with_hand:
        return max(with_hand, key=lambda c: _subtree_depth(c))
    return max(candidates, key=lambda c: _subtree_depth(c))


def _assign_arm(start, is_left, result):
    """Trace arm chain and assign shoulder/upper_arm/forearm/hand/fingers."""
    side = "left" if is_left else "right"
    arm = _trace_arm_chain(start)

    if len(arm) >= 4:
        result[f"{side}_shoulder_bone"] = arm[0].name
        result[f"{side}_upper_arm_bone"] = arm[1].name
        result[f"{side}_lower_arm_bone"] = arm[2].name
        result[f"{side}_hand_bone"] = arm[3].name
        hand = arm[3]
    elif len(arm) == 3:
        l0 = (arm[0].head_local - arm[0].tail_local).length
        l1 = (arm[1].head_local - arm[1].tail_local).length
        if l0 < l1 * 0.7:
            result[f"{side}_shoulder_bone"] = arm[0].name
            result[f"{side}_upper_arm_bone"] = arm[1].name
            result[f"{side}_hand_bone"] = arm[2].name
            hand = arm[2]
        else:
            result[f"{side}_upper_arm_bone"] = arm[0].name
            result[f"{side}_lower_arm_bone"] = arm[1].name
            result[f"{side}_hand_bone"] = arm[2].name
            hand = arm[2]
    elif len(arm) == 2:
        result[f"{side}_upper_arm_bone"] = arm[0].name
        result[f"{side}_hand_bone"] = arm[1].name
        hand = arm[1]
    else:
        return

    _identify_fingers(hand, is_left, result)


def _trace_arm_chain(start, max_depth=8):
    """Trace arm chain from shoulder/upper_arm, stopping at hand."""
    chain = [start]
    cur = start
    for _ in range(max_depth):
        if _is_hand_bone(cur) and len(chain) >= 2:
            break
        children = list(cur.children)
        if not children:
            break
        if len(children) == 1:
            chain.append(children[0])
            cur = children[0]
        else:
            best = max(children, key=lambda c: _subtree_depth(c))
            chain.append(best)
            cur = best
    return chain


def _is_hand_bone(bone):
    """True if bone has 3+ children with finger-like chains (depth >= 3)."""
    children = list(bone.children)
    if len(children) < 3:
        return False
    deep = sum(1 for c in children if _subtree_depth(c) >= 3)
    return deep >= 3


def _has_hand_descendant(bone, depth=6):
    """Check if bone or any descendant within depth is a hand bone."""
    if _is_hand_bone(bone):
        return True
    if depth <= 0:
        return False
    return any(_has_hand_descendant(c, depth - 1) for c in bone.children)


# ---------------------------------------------------------------------------
# Finger detection
# ---------------------------------------------------------------------------

def _identify_fingers(hand_bone, is_left, result):
    """Classify finger chains branching from the hand bone."""
    children = list(hand_bone.children)
    chains = []
    for child in children:
        ch = [child]
        cur = child
        while len(cur.children) == 1:
            ch.append(cur.children[0])
            cur = cur.children[0]
        if len(ch) >= 2:
            chains.append(ch)

    if len(chains) < 2:
        return

    # Strip leading carpal bones (4+ bone chain where first bone is a pass-through)
    for i, ch in enumerate(chains):
        if len(ch) >= 4:
            chains[i] = ch[1:]

    side = "left" if is_left else "right"

    # Identify thumb: most deviated direction from average
    hand_pos = hand_bone.head_local
    dirs = [(ch[0].head_local - hand_pos).normalized() for ch in chains]
    avg = Vector((0, 0, 0))
    for d in dirs:
        avg += d
    avg /= len(dirs)

    deviations = [(dirs[i] - avg).length for i in range(len(chains))]
    thumb_i = deviations.index(max(deviations))
    thumb = chains.pop(thumb_i)

    for i, bone in enumerate(thumb[:3]):
        result[f"{side}_thumb_{i}"] = bone.name

    # Sort remaining by distance from thumb root (closest = index)
    thumb_pos = thumb[0].head_local
    chains.sort(key=lambda ch: (ch[0].head_local - thumb_pos).length)

    names = ["index", "middle", "ring", "pinky"]
    for fi, ch in enumerate(chains[:4]):
        for si, bone in enumerate(ch[:3]):
            result[f"{side}_{names[fi]}_{si + 1}"] = bone.name


# ---------------------------------------------------------------------------
# Leg detection
# ---------------------------------------------------------------------------

def _map_legs(chain, leg_idx, result):
    """Identify leg chains from the leg fork point."""
    fork_bone = chain[leg_idx]
    chain_set = {b.name for b in chain}
    all_z = [b.head_local.z for b in chain]
    height = max(all_z) - min(all_z)
    x_thresh = max(height * 0.01, 0.01)

    off = [c for c in fork_bone.children if c.name not in chain_set]
    left = [c for c in off if c.head_local.x > x_thresh]
    right = [c for c in off if c.head_local.x < -x_thresh]

    # If no direct lateral children, check grandchildren
    if not left or not right:
        for oc in off:
            gc_left = [c for c in oc.children if c.head_local.x > x_thresh]
            gc_right = [c for c in oc.children if c.head_local.x < -x_thresh]
            if gc_left and not left:
                left = gc_left
            if gc_right and not right:
                right = gc_right

    if left:
        best = max(left, key=lambda c: _subtree_depth(c))
        _assign_leg(best, True, result)
    if right:
        best = max(right, key=lambda c: _subtree_depth(c))
        _assign_leg(best, False, result)


def _assign_leg(start, is_left, result):
    """Trace leg chain and assign thigh/shin/foot/toe."""
    side = "left" if is_left else "right"
    chain = _trace_limb_chain(start, max_depth=6)

    # Skip control bones (腰キャンセル) at chain start: head overlaps next bone
    if len(chain) >= 2:
        d = (chain[0].head_local - chain[1].head_local).length
        if d < 0.01:
            chain = chain[1:]

    if len(chain) >= 4:
        result[f"{side}_thigh_bone"] = chain[0].name
        result[f"{side}_calf_bone"] = chain[1].name
        result[f"{side}_foot_bone"] = chain[2].name
        result[f"{side}_toe_bone"] = chain[3].name
    elif len(chain) == 3:
        result[f"{side}_thigh_bone"] = chain[0].name
        result[f"{side}_calf_bone"] = chain[1].name
        result[f"{side}_foot_bone"] = chain[2].name
    elif len(chain) == 2:
        result[f"{side}_thigh_bone"] = chain[0].name
        result[f"{side}_calf_bone"] = chain[1].name


def _trace_limb_chain(start, max_depth=6):
    """Trace a limb chain, following the child with the deepest subtree."""
    chain = [start]
    cur = start
    for _ in range(max_depth):
        children = list(cur.children)
        if not children:
            break
        if len(children) == 1:
            chain.append(children[0])
            cur = children[0]
        else:
            best = max(children, key=lambda c: _subtree_depth(c))
            chain.append(best)
            cur = best
    return chain


# ---------------------------------------------------------------------------
# Eye detection
# ---------------------------------------------------------------------------

def _is_eye_name(name):
    """True if a bone name looks like an eyeball (not eyelid/eyebrow/eyelash)."""
    n = name.lower()
    if any(bad in n for bad in ("eyelid", "eyebrow", "eyelash", "brow", "lash")):
        return False
    return ("eyeball" in n) or ("eye" in n) or ("目" in n)


def _map_eyes(bones, head_name, result):
    """Find symmetric eye bones among head's children/grandchildren."""
    head = None
    for b in bones:
        if b.name == head_name:
            head = b
            break
    if not head:
        return

    candidates = []
    for child in head.children:
        candidates.append(child)
        for gc in child.children:
            candidates.append(gc)

    # Find symmetric pairs by matching |X| and Z. Order-independent: pair every
    # +X (left) candidate with every -X (right) candidate (the old [i+1:] form
    # missed pairs whose right bone was listed before the left one — e.g. eyes).
    best_pair = None
    best_score = -float('inf')
    x_min = 0.02
    sym_tol = 0.01
    lefts = [c for c in candidates if c.head_local.x > x_min]
    rights = [c for c in candidates if c.head_local.x < -x_min]
    for c1 in lefts:
        for c2 in rights:
            dx = abs(abs(c1.head_local.x) - abs(c2.head_local.x))
            dz = abs(c1.head_local.z - c2.head_local.z)
            dy = abs(c1.head_local.y - c2.head_local.y)
            if dx < sym_tol and dz < sym_tol and dy < sym_tol:
                # Prefer the highest pair (eyes are above lips/nose);
                # strongly prefer bones whose names look like eyes (avoid
                # picking symmetric hair bones over real eyeball bones).
                score = c1.head_local.z + c2.head_local.z
                if _is_eye_name(c1.name) and _is_eye_name(c2.name):
                    score += 1000.0
                if score > best_score:
                    best_score = score
                    best_pair = (c1, c2)

    if best_pair:
        left = best_pair[0] if best_pair[0].head_local.x > 0 else best_pair[1]
        right = best_pair[1] if best_pair[0].head_local.x > 0 else best_pair[0]
        result["left_eye_bone"] = left.name
        result["right_eye_bone"] = right.name


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_depth_cache = {}


def _subtree_depth(bone, max_depth=20):
    """Depth of subtree rooted at bone (cached)."""
    key = bone.name
    if key in _depth_cache:
        return _depth_cache[key]
    if not bone.children or max_depth <= 0:
        _depth_cache[key] = 1
        return 1
    d = 1 + max(_subtree_depth(c, max_depth - 1) for c in bone.children)
    _depth_cache[key] = d
    return d


def clear_cache():
    """Clear the subtree depth cache (call between different armatures)."""
    _depth_cache.clear()


def _empty_result():
    """Return preset dict template with all keys empty."""
    return {
        "all_parents_bone": "",
        "center_bone": "",
        "groove_bone": "",
        "hip_bone": "",
        "upper_body_bone": "",
        "upper_body1_bone": "",
        "upper_body2_bone": "",
        "upper_body3_bone": "",
        "neck_bone": "",
        "head_bone": "",
        "left_shoulder_bone": "",
        "right_shoulder_bone": "",
        "left_upper_arm_bone": "",
        "right_upper_arm_bone": "",
        "left_lower_arm_bone": "",
        "right_lower_arm_bone": "",
        "left_hand_bone": "",
        "right_hand_bone": "",
        "lower_body_bone": "",
        "left_thigh_bone": "",
        "right_thigh_bone": "",
        "left_calf_bone": "",
        "right_calf_bone": "",
        "left_foot_bone": "",
        "right_foot_bone": "",
        "left_toe_bone": "",
        "right_toe_bone": "",
        "control_center_bone": "",
        "left_eye_bone": "",
        "right_eye_bone": "",
        "left_thumb_0": "",
        "left_thumb_1": "",
        "left_thumb_2": "",
        "right_thumb_0": "",
        "right_thumb_1": "",
        "right_thumb_2": "",
        "left_index_1": "",
        "left_index_2": "",
        "left_index_3": "",
        "right_index_1": "",
        "right_index_2": "",
        "right_index_3": "",
        "left_middle_1": "",
        "left_middle_2": "",
        "left_middle_3": "",
        "right_middle_1": "",
        "right_middle_2": "",
        "right_middle_3": "",
        "left_ring_1": "",
        "left_ring_2": "",
        "left_ring_3": "",
        "right_ring_1": "",
        "right_ring_2": "",
        "right_ring_3": "",
        "left_pinky_1": "",
        "left_pinky_2": "",
        "left_pinky_3": "",
        "right_pinky_1": "",
        "right_pinky_2": "",
        "right_pinky_3": "",
    }
