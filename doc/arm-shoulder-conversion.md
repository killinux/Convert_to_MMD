# 手臂与肩膀转换规范（XPS → MMD）

> 目的：把 XPS/XNALara 的手臂+肩膀骨骼/权重转换成符合 MMD 惯例、且与参考 PMX 行为一致的结果。
> 本文是**实现规范**——若重写本插件的手臂/肩膀部分，照此即可。每条都给出 **做什么 / 为什么 / 参数 / 验证**。
> 标定基准（target）：`Purifier Inase 18 None.pmx`（mmd_tools 导入）。源：`xps-b.xps`（XNALaraMesh 导入，109 骨）。

---

## 0. 总原则

1. **忠实复用 XPS 权重，不凭空造**。需要细分（如捩骨）时用**守恒切分**（切完总权重不变），不向目标"补"权重。源网格本身和目标的差异（见 §11）予以保留。
2. **按拓扑识别骨骼，不靠名字**——流水线前段骨头还是 XPS 原名（见 §2 时机）。用 `skeleton_identifier.identify_skeleton()`。
3. **捩骨复用 mmd_tools 的付与机制**（`_dummy_`/`_shadow_` 影子骨由 mmd_tools 自动建），不手写约束。
4. 每一步都要能**量化验证**对齐目标（见 §10）。

---

## 1. 骨链与命名映射（右侧示例，左侧同理 右→左）

XPS 源手臂骨（英文名）→ MMD 名：

| XPS 源骨 | MMD | 说明 |
|---|---|---|
| `arm right shoulder 1` | 右肩 | 肩 |
| `arm right shoulder 2` | 右腕 | 上臂 |
| `arm right elbow` | 右ひじ | 前臂 |
| `arm right wrist` | 右手首 | 手腕/手掌根 |
| `arm right finger 1a/1b/1c` | 右親指０/１/２ | 拇指（3 段） |
| `arm right finger 2a/2b/2c` | 右人指１/２/３ | 食指 |
| `arm right finger 3/4/5 a/b/c` | 右中指/薬指/小指 1/2/3 | 中/无名/小指 |
| `unused bip001 xtra07`(右)/`xtra07pp`(左) | → 肩+腕 | **三角肌(肩盖)辅助骨**，按位置分（§3） |
| `unused bip001 r/l foretwist`,`foretwist1` | → ひじ/手捩系统 | **前臂捩辅助骨**（§5 注） |

> 细分指根（人指０/中指０/…）MMD 有、XPS 无 → 由 `complete_missing_bones` 创建。
> 精确映射见 `bone_map_and_group.py`。

MMD 手臂目标骨集（每侧）：
`肩 → 腕 → [腕捩 + 腕捩1/2/3] → ひじ → [手捩 + 手捩1/2/3] → 手首 → 指`，外加肩 P/C（肩P→肩C 付与）。

---

## 2. 流水线顺序与时机（`one_click_operator.py`）

```
0    auto_identify_skeleton      拓扑识别 → 填命名映射
0.5  correct_bones               归正骨架位置
1    rename_to_mmd               按映射重命名（手指等改名；★见下）
1.4  transfer_unused_weights     第1趟：unused 骨权重转移 → ★三角肌路由在此
1.5  fix_forearm_bend            前臂弯曲修正（烘焙，§7）
1.6  align_arms_to_canonical     上臂对齐 A-pose（烘焙，§7）
1.7  align_fingers_to_canonical  手指对齐 A-pose（烘焙，§7）
2    complete_missing_bones      ★把臂骨改成 MMD 名 + 创建缺失骨（人指０等）
2.5  transfer_unused_weights     第2趟
3    add_mmd_ik
4    create_bone_group
5    use_mmd_tools_convert       mmd_tools 转 MMD 模型 → ★双修改器去重在此(§6)
5.5  (VG 残留清理)
6    add_leg_d_bones
7    add_twist_bone              ★建捩骨 + 切权重 + 设付与(§4,§5)
8    add_shoulder_p_bones        肩P/肩C
8.4  apply_standard_grants       写 mmd_bone 付与(肩C/腕捩/手捩/D骨)
8.5  mmd_tools.apply_additional_transform  ★实现付与 → 自动建捩骨 _dummy_/_shadow_(§4)
```

**★关键时机：臂骨（肩/腕/ひじ）直到 step 2 `complete_missing_bones` 才有 MMD 名；step 1 `rename_to_mmd` 不重命名臂骨**（手指等会改）。所以 step 1.4 的三角肌路由**必须用拓扑识别**解析 肩/腕/ひじ，不能查日文名（查不到→落空）。路由到**肩骨当前名**，complete 改名时顶点组随骨头一起改名、权重归位。

---

## 3. 肩 + 三角肌（肩盖）

**问题**：XPS 把肩盖三角肌绑在独立辅助骨（`xtra07`/`xtra07pp`）上，质心在上臂近端。若按"最近骨头"转移会整片倒进腕（肩关节头最近）→ 三角肌跟着上臂扭转，糖纸。

**做法（`xps_fixes_operator.py: _detect_arm_deltoid` + `transfer_unused_weights`）**：
1. **识别**三角肌辅助骨：对每个 unused/候选骨，计算其权重质心，沿该侧 `腕→ひじ` 轴参数 `t`；若 `0.05 ≤ t ≤ 0.55` 且横向距离 `< 0.5×上臂长` → 判为三角肌。
2. **按位置分肩/腕**（不整组、不按最近骨头）：每个三角肌顶点按 `t`（沿 腕→ひじ）用 ramp：
   - `肩占比 sf = 1.0 (t≤DELTOID_SH_T_LO) → 0.0 (t≥DELTOID_SH_T_HI)`，线性过渡；
   - 顶点权重 `× sf → 肩`，`× (1-sf) → 腕基部`。
3. 常量：`DELTOID_SH_T_LO=0.0`，`DELTOID_SH_T_HI=0.25`（沿 腕→ひじ 轴；经标定使交界落在 肩→ひじ 轴 ~t0.4）。

**为什么安全**：下端落在**腕基部低 t 段**，那里几乎不参与腕捩扭转（`setup_weights TAU_LO=0.20`），所以三角肌下半不会糖纸；上端肩盖顶仍在肩。这复刻目标——目标在 肩→ひじ 轴 t≈0.4 处 肩 归零、腕 接管。

**历史**：早期(s3)是"整组→肩"（提交 2c29515/d23636e），导致肩过重、拖到 t≈0.6；改为按位置分（260fda6 + 调参 5ec4f25）。

---

## 4. 捩骨（腕捩/手捩 + 细分 1/2/3）

### 4.1 结构（与目标 PMX 一致）

| | 主捩骨 腕捩/手捩 | 子捩骨 1/2/3 |
|--|--|--|
| 朝向 | **沿手臂**（与 base 同向/同 roll） | **竖直朝上 `(0,0,1)`** |
| 驱动 | VMD 直接驱动，`fixed_axis`（轴固定），`lock_rotation=(T,F,T)` | 付与跟随主捩骨（回転付与） |
| influence | 1.0 | 0.25 / 0.5 / 0.75 |
| 约束 | 无（VMD） | 经 `_dummy_/_shadow_` 影子骨链中转 |

子骨 head 沿 base 轴位置 `_POS = {主:0.80, 1:0.20, 2:0.40, 3:0.60}`（注：主捩骨落肘端，子骨递增）。`seg_len = max(base长×0.12, 1e-4)`。

### 4.2 子骨为什么朝上 + 影子骨链（核心）

子骨朝上是 MMD 显示惯例。**关键洞察**：`_dummy_`/`_shadow_` 影子骨**不需要手写**——mmd_tools 的 `apply_additional_transform`（流水线 step 8.5）会自动建，触发条件是 `FnBone._AT_ShadowBoneCreate.__is_well_aligned`：

```
子骨与付与目标(主捩骨) x/y 轴点积 >0.99 → 对齐 → 直连 LOCAL→LOCAL 约束
否则（不对齐）→ 建 _dummy_/_shadow_ 世界空间中转链
```

所以**只要把子骨掰朝上**（与沿臂的主捩骨不对齐），step 8.5 就自动建出和目标逐字一致的链路。建子骨：
```python
tb.tail = tb.head + Vector((0,0,seg_len))   # 朝上
tb.align_roll(Vector((0,-1,0)))              # 复刻目标朝向 x=+X, z=-Y
```
主捩骨保持沿臂、同 roll（VMD 驱动 + fixed_axis）。

自动建出的链路（每子骨一对，共 +24 根；以 腕捩1 为例）：
```
腕捩(主, 沿臂, VMD驱动) ──父── _dummy_腕捩1(朝上, 无约束)
                                   │ COPY_TRANSFORMS(POSE→POSE, REPLACE)
                                   ▼
腕(上臂) ──父── _shadow_腕捩1(朝上) ──约束──┐
                                            │
腕(上臂) ──父── 腕捩1(可见变形骨, 朝上) ──TRANSFORM(LOCAL→LOCAL, ADD)──> 取 _shadow_腕捩1
```
- `_dummy_` 父 = 主捩骨；`_shadow_` 父 = 变形骨的父（腕/ひじ）；二者朝向 = 子骨朝向(朝上)。
- 经 POSE/世界空间走一遍 → 解耦"显示朝向"与"扭转"，扭转始终正确。

### 4.3 影子骨链各字段（标定自目标，必须复刻）

- 变形骨 `腕捩i` 约束 `TRANSFORM`（mmd_tools 名 `mmd_additional_rotation`）：
  - `owner_space=LOCAL, target_space=LOCAL, subtarget=_shadow_腕捩i`
  - `map_from=ROTATION, map_to=ROTATION`（x→x,y→y,z→z）
  - `from_*_rot = ±π`（±180°，全轴）
  - **`to_*_rot = ±(influence×π)`** → i=1/2/3 即 **±45° / ±90° / ±135°**（influence 比例藏在这里！）
  - `mix_mode_rot = ADD`
- `_shadow_腕捩i` 约束 `COPY_TRANSFORMS`（mmd_tools 名 `mmd_tools_at_dummy`）：`owner/target_space=POSE, REPLACE, subtarget=_dummy_腕捩i`
- `_dummy_腕捩i`：无约束。

### 4.4 主捩骨 fixed_axis

`setup_grants`：主捩骨设 `mmd_bone.enabled_fixed_axis=True`，`fixed_axis = 手臂方向`。源轴：腕捩沿 腕→ひじ、手捩沿 ひじ→手首。**存 MMD 坐标 = Blender (x, z, −y)**（直接写 Blender 向量会导致导出 PMX 扭转轴错）。主捩骨 `lock_location=(T,T,T)`，`lock_rotation=(T,F,T)`（只许绕骨向扭）。

### 4.5 付与元数据

子骨 `mmd_bone`：`has_additional_rotation=True`，`additional_transform_bone=主捩骨`，`additional_transform_influence=0.25/0.5/0.75`，`lock_location=(T,T,T)`。`add_twist_bone` 和 `apply_standard_grants`(step 8.4) 都会设（值一致，幂等）。

---

## 5. 权重切分（`add_twist_bone_operator.py: setup_weights`）

把 base 骨（腕/ひじ）的权重沿骨轴**守恒**切分到 [base, 捩1, 捩2, 捩3, 主捩]，按"扭转目标 τ"在相邻两档间线性插值（重叠平滑、扭转量连续、总权重不变）。档位扭转比例：base=0, 捩1=0.25, 捩2=0.5, 捩3=0.75, 主捩=1.0。

- 顶点沿骨轴 `t∈[0,1]` → `τ = clamp((t−TAU_LO)/(TAU_HI−TAU_LO))` → 落在哪两档之间就按比例分给那两根。
- **上臂(腕)**：`TAU_LO=0.20, TAU_HI=0.80`（主腕捩落肘端 t≈1.0，与目标一致）。
- **前臂(ひじ)**：`TAU_LO_FOREARM=0.05, TAU_HI_FOREARM=0.90`（目标标定：手捩1/2/3/主 峰值 t≈0.3/0.5/0.7/0.9）。

**前臂 reclaim（关键）**：XPS 把前臂**远端半段**绑在"手"骨(→手首)上，ひじ 顶点组只覆盖肘侧半段。若只切 ひじ，手捩(主)/手捩3 拿不到权重 → 前臂远端不扭转、糖纸。故把 `手首` 落在前臂段的权重按 ramp 并入切分池：`RECLAIM_LO=0.90, RECLAIM_HI=1.10`（t≤LO 全收、t≥HI 不收、中间线性），并等量从 手首 扣除。

> **权重与子骨显示朝向无关**：切分按 base 骨轴 + 顶点位置，影子骨 `use_deform=False`。改子骨朝向（§4.2）不影响权重。

---

## 6. 蒙皮：双 armature 修改器（必须去重）

**陷阱**：XNALaraMesh 导入给网格加一个 armature 修改器；mmd_tools `convert_to_mmd_model`（step 5）**又加一个**指向同一骨架 → 双重蒙皮。rest 姿势是恒等（看似正常），但一旦有姿势就把骨骼变换叠加两次 → 顶点炸飞（frame250 包围盒 ~4×）。

**做法（`preset_operator.py: use_mmd_tools_convert`，convert 之后）**：每个网格只保留第一个指向该骨架的 armature 修改器，删掉其余。
```python
for m in meshes:
    arm_mods=[md for md in m.modifiers if md.type=='ARMATURE' and md.object==arm]
    for md in arm_mods[1:]: m.modifiers.remove(md)
```
提交 67e559c。

---

## 7. 烘焙到 rest（fix_forearm / align_arms / align_fingers）与陷阱

这三步把骨骼摆到 canonical 姿势再**烘焙为新 rest**（`_bake_pose_delta_to_rest`）：
1. 找带 armature 修改器的网格（**跳过有 shape key 的**）；
2. 给每个网格加临时 `_copy` armature 修改器；
3. 摆姿势（清 pose → 按 plans 旋转骨）；
4. `modifier_apply` 临时 `_copy`（把姿势烘进网格）；
5. `armature_apply`（姿势变 rest）。

**★陷阱（本会话踩到）**：第 4 步内部 `view_layer.objects.active = 网格` 然后 `bpy.ops.object.modifier_apply()`。若**外层用 `temp_override(active_object=...)` 钉死了 active object**（脚本/自动化/MCP 常见），`modifier_apply` 会作用到被钉的对象（骨架）→ 空操作 → **骨头转了、网格没烘焙 → 手指(拇指尤甚)/手臂与网格错位**（拇指尖偏移 0.034 vs 正常 0.008），且留下 `_copy` 修改器。**UI 点按钮不钉 active_object → 正常。**
- **调用方修法**：脚本里用**轻量 override**（只给 window/area/region/screen，先设 `view_layer.objects.active` + 选中，**不要**钉 active_object/object/selected_objects）。
- **防御性插件修法（建议重写时采用）**：在 `_bake_pose_delta_to_rest` 内部给 `modifier_apply` 包**自己的** `temp_override(active_object=网格, object=网格, selected_objects=[网格])`，使其不受外层上下文影响。

> 另一陷阱：跳过 shape-key 网格——若变形网格带 shape key 会被跳过、烘焙失效。重写时考虑临时移除/合并 shape key 后烘焙再恢复，或对带 shape key 的网格走 `Mesh.from_object(evaluated)` 回写。

---

## 8. 手指

- 由 `align_fingers_to_canonical`（step 1.7）把每条手指根段方向对齐到内置 canonical（`canonical_finger_dirs.json`），绕根骨 head 旋转、烘焙（见 §7 陷阱——这步对 override 敏感）。
- 缺失的指根（人指０/中指０/…）由 `complete_missing_bones` 创建。
- 拇指：`右親指０/１/２` 由 XPS `finger 1a/1b/1c` 映射。`親指０` 是掌内掌骨，紧贴手腕（head 距手腕 head ~3-4cm），所以它**天然覆盖鱼际/手腕根**一点点（目标也有，~0.86%）；XPS 源覆盖略多（~8.6%）——属源差异（§11）。

---

## 9. 目标 PMX 参考值（要对齐的规格）

| 项 | 目标值（右侧，frame 250 用 yaoxiang.vmd） |
|---|---|
| 子捩骨朝向 | `(0,0,1)`，x=(1,0,0)，z=(0,-1,0) |
| 捩骨影子链字段 | §4.3（to_*_rot ±45/90/135°, POSE/LOCAL, ADD/REPLACE） |
| 前臂扭转梯度 | 主手捩≈76.8°，子骨/主 = 0.28/0.54/0.78（≈0.25/0.5/0.75，欧拉缩放在大角略超） |
| 肩↔腕交界（沿肩→ひじ） | t0.4：肩≈腕≈8%；t0.5：肩→0、腕接管 |
| 上臂捩骨质心（沿腕→ひじ） | 腕捩1/2/3 ≈ t 0.34/0.52/0.65 |
| 手指骨↔网格偏移 | < ~0.011（对齐） |
| 动作下网格包围盒 | ≈ rest 尺寸（无炸开） |

---

## 10. 验证方法与指标

1. **捩骨结构**：dump `腕捩i/手捩i` 及 `_dummy_/_shadow_` 的 父子、朝向(matrix_local 列)、约束全字段，逐项对比目标(§9, §4.3)。
2. **扭转梯度**：加 VMD，frame 250，对每根捩骨算"相对父级的局部旋转增量"角度（`(rest_local⁻¹ @ pose_local).to_quaternion().angle`），看主≈77°、子骨 0.25/0.5/0.75 比例。
3. **肩/腕权重剖面**：沿 肩→ひじ 轴把顶点按 t 分箱，统计 肩/腕/捩 各占比，对比目标交界(§9)。
4. **手指对齐**：每指骨 `bone_mid` vs `权重>0.5 顶点的世界质心` 的距离；应 <~0.01。
5. **炸开检测**：比较网格在 frame 0(rest) 与 frame 250 的世界包围盒尺寸；若 frame250 ≫ rest（如 4×）→ 双修改器/蒙皮问题。另：扫 pose bone 矩阵有无非单位缩放/NaN。
6. **权重守恒**：切分前后 base+捩 总权重应相等。

---

## 11. 保留的 XPS 源差异（评估后不修，避免凭空造权重）

- 上臂扭转覆盖偏低（~58% vs 目标 76%）：XPS 上臂权重偏近肩端。质心位置对得上、只是占比低。
- 前臂扭转权重偏远端集中（t≈0.7/0.9 尖峰 vs 目标平滑）：XPS 前臂权重绑在 手首/foretwist 上、近端 ひじ 很轻。
- `親指０` 比目标多覆盖一点手腕（8.6% vs 0.86%）。
- 主捩骨扭转轴差 ~7°：源自两模型上臂 rest 朝向差 ~8°。
- 这些动作下都不产生可见异常（已渲染确认）。**复用 XPS 原貌是刻意决定**。

可选未来项：直接复用 XPS `foretwist`/`foretwist1` 的自带前臂扭转梯度（而非 transfer+τ重切），但 XPS 只有 2 档、MMD 要 4 档，非 1:1。

---

## 12. 常量速查

| 常量 | 值 | 位置 | 含义 |
|---|---|---|---|
| `TWIST_SUB_INFLUENCE` | {1:.25,2:.5,3:.75} | add_twist_bone | 子捩骨付与比例 |
| `_POS` | {0:.80,1:.20,2:.40,3:.60} | add_twist_bone | 捩骨 head 沿 base 轴位置 |
| `TAU_LO/HI` (上臂) | 0.20 / 0.80 | add_twist_bone | 上臂扭转过渡区间 |
| `TAU_LO/HI_FOREARM` | 0.05 / 0.90 | add_twist_bone | 前臂扭转过渡区间 |
| `RECLAIM_LO/HI` | 0.90 / 1.10 | add_twist_bone | 前臂从手首回收的 ramp |
| `DELTOID_SH_T_LO/HI` | 0.0 / 0.25 | xps_fixes | 三角肌肩/腕分配 ramp(沿腕→ひじ) |
| 三角肌判定 t 区间 | 0.05–0.55 | xps_fixes | 质心沿腕→ひじ 的接受范围 |
| 三角肌横向阈值 | <0.5×上臂长 | xps_fixes | 贴轴判定 |

---

## 13. 关键陷阱清单（重写时务必处理）

1. **臂骨直到 complete_missing_bones(step2) 才有 MMD 名** → 前段(如三角肌路由 1.4)用拓扑识别，路由到当前名。
2. **双 armature 修改器** → convert 后去重（§6）。
3. **`_bake_pose_delta_to_rest` 对 active_object override 敏感** → 内部 modifier_apply 自带 temp_override；调用方用轻量 override（§7）。
4. **烘焙跳过 shape-key 网格** → 带 morph 的网格要特殊处理（§7）。
5. **捩骨子骨必须与主骨不对齐** 才能触发 mmd_tools 建影子骨（§4.2）；对齐则只得直连约束、显示沿臂。
6. **fixed_axis 存 MMD 坐标 (x,z,−y)**，非 Blender 向量（§4.4）。
7. **前臂不 reclaim 手首** → 远端不扭、糖纸（§5）。
8. **权重切分要守恒**，不向目标补权重（§0,§11）。

---

## 14. 相关文件 / 提交

- `operators/add_twist_bone_operator.py` — 捩骨创建/朝向/权重切分/付与（提交 3854540 子骨朝上）
- `operators/xps_fixes_operator.py` — 三角肌识别+按位置分、烘焙修正（提交 260fda6+5ec4f25 三角肌分肩腕）
- `operators/preset_operator.py: use_mmd_tools_convert` — 双修改器去重（提交 67e559c）
- `operators/mmd_grant_operator.py` — 标准付与（肩C/腕捩/手捩/D骨）
- `operators/complete_bones_operator.py` — 补全臂骨/改名/创建缺失指根
- `operators/one_click_operator.py` — 流水线顺序
- `skeleton_identifier.py` / `helper_classifier.py` — 拓扑识别/辅助骨分类
- 捩骨影子骨原理详解：`doc/twist-bone-shadow-refactor.md`
- 测试用例/远程环境/部署：见记忆 `conversion-test-task`、`remote-blender-connection`、`dev-deploy-workflow`
