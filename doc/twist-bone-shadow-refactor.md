# 捩骨（旋转骨）重构：_dummy_/_shadow_ 影子骨方案

> 目标：让转换模型的细分捩骨（腕捩1/2/3、手捩1/2/3）像参考 PMX 那样**竖直朝上显示**，同时不破坏扭转变形。
> 状态：✅ **已实现并验证（2026-06-07）**。最终方案比本文原设计（手建影子骨）更简单——见下方"实现结果"。本文 §1/§2 的原理分析仍然成立，§3 的手建方案被更优的"方案A"取代。

---

## 实现结果（2026-06-07）

**最终采用「方案A：复用 mmd_tools 自动建链」，而非本文 §3 原计划的手建 24 根影子骨。**

动手前扒数据（§4）时读了 mmd_tools 源码 `core/bone.py`，发现 `_dummy_`/`_shadow_` 链是 mmd_tools 自己建的，触发条件是 `_AT_ShadowBoneCreate.__is_well_aligned`（子骨与付与目标的 x/y 轴点积 >0.99 才算对齐）：**对齐→直连约束；不对齐→建 dummy/shadow 中转**。上次记的"无依赖环所以不建"是错的。流水线第 8.5 步本来就调 `mmd_tools.apply_additional_transform()`。

所以只需把子捩骨建成**竖直朝上**（与沿臂的主捩骨不对齐），第 8.5 步就会**自动**建出与目标 PMX 逐字一致的 `_dummy_`/`_shadow_` 链。

**改动（唯一功能改动）**：`operators/add_twist_bone_operator.py` 创建子捩骨时，朝向从"沿 base 轴"改为 `(0,0,seg_len)` 朝上 + `align_roll((0,-1,0))`（主捩骨不变，仍沿臂带 fixed_axis）。`setup_weights`/`setup_grants` 未动——权重与子骨显示朝向无关。提交 `3854540`。

**验证（转换 vs 目标，全部一致）**：
- 子骨朝向 `(0,0,1)`、`_shadow_`父=腕/ひじ、`_dummy_`父=主捩骨、TRANSFORM `to_*_rot`=±45/90/135°、POSE/LOCAL 空间、influence 0.25/0.5/0.75 —— 与目标逐字相同。
- frame 250 扭转梯度：手捩主 76.8°、子骨 0.28/0.54/0.78 —— 与目标完全相同。

**附带修掉一个独立老 bug（与捩骨无关）**：转换网格挂了**两个** armature 修改器（XNALaraMesh 导入加一个 + mmd_tools `convert_to_mmd_model` 又加一个），双重蒙皮导致 rest 正常、一动就炸（frame 250 网格爆开）。在 `operators/preset_operator.py` 的 `use_mmd_tools_convert` 里转换后去重（每网格只留一个）。提交 `67e559c`。验证：全新转换 8 个网格各剩 1 个修改器，frame 250 网格尺寸 2.44→0.61（与目标 0.6 一致）。

---

## 1. 背景与问题

转换模型（XPS→MMD）和参考目标 PMX 的捩骨，**功能结构其实已经一致**：

| | 主捩骨 腕捩/手捩 | 子捩骨 1/2/3 |
|--|--|--|
| 驱动 | VMD 直接驱动，有 `fixed_axis`（轴固定） | 付与（回転付与）跟随主捩骨 |
| influence | 1.0 | 0.25 / 0.5 / 0.75 |

差异只有两点：

1. **主捩骨扭转轴** ~7°：本模型 `[0.71,-0.71,0]` vs 目标 `[0.80,-0.60]`（根因是两模型上臂 rest 朝向差 ~8°）。
2. **子捩骨显示朝向**：本模型**沿手臂**，目标**竖直朝上 `(0,0,1)`**。

用户要的是第 2 点：**子捩骨改成朝上、和目标一致**。

### 为什么不能直接把子骨掰朝上

本模型的付与是**直连约束**：`左腕捩1 ──TRANSFORM(LOCAL→LOCAL)── 左腕捩`。
LOCAL→LOCAL 是把源骨的**局部旋转值直接塞进目标骨的局部坐标系**——只有两骨**朝向相同**才对。
所以本模型的子捩骨**必须沿臂**（与主骨同向）。代码里也写明了：

> `operators/add_twist_bone_operator.py` 创建子骨时 `dir=base、roll=base`，注释：“所有捩骨与 base 同 roll，付与才不会扭歪”。

**实验证据（决定性）**：把左侧子捩骨掰朝上、保留直连约束，frame 250（左手捩≈77°）下，左前臂顶点世界坐标位移 **max 4.8cm / 均值 2.0cm**（模型高 1.75m，>1cm 即肉眼可见）→ 确认会扭歪，**直接掰不可行**。

---

## 2. 解决方案：_dummy_/_shadow_ 影子骨

参考目标 PMX 的细分捩骨用的就是这套（mmd_tools 的付与实现）。**本质：把"源骨转了多少"经过世界空间中转后再传给变形骨，使变形与变形骨自身朝向无关。**

### 2.1 目标的链路（以 `腕捩1.L` 为例）

```
腕捩.L  （主捩骨，VMD 驱动，沿臂，有 fixed_axis）
   │ 父子（parent）
   ▼
_dummy_腕捩1.L     父 = 腕捩.L      无约束
   │ COPY_TRANSFORMS（POSE→POSE, REPLACE, 整个世界变换）
   ▼
_shadow_腕捩1.L    父 = 腕.L
   │ TRANSFORM（LOCAL→LOCAL, ADD, ×influence）
   ▼
腕捩1.L            父 = 腕.L        可见变形骨，朝上 (0,0,1)
```

### 2.2 各骨分工

- **`_dummy_`（替身）**：父挂在**源骨（主捩骨 腕捩.L）**下，无约束。源骨一转，它作为刚性子骨跟着转 = **在世界空间录下源骨的运动**。
- **`_shadow_`（影子）**：父挂在**变形骨的父级（腕.L）**下，用 **POSE（世界）空间** `COPY_TRANSFORMS` 复制 dummy = **把源骨的世界运动换算进变形骨所在坐标系**。
- **变形骨 `腕捩1.L`**：用 LOCAL 的 `TRANSFORM`（ADD、×influence 0.25/0.5/0.75）叠加 shadow 的局部旋转。

### 2.3 为什么这样能解耦

旋转经过 **POSE/世界空间**走一遍（dummy→shadow），是**坐标系无关**的换算。带来三个好处：

1. **解耦显示与变形**：变形骨爱朝哪朝哪（朝上=MMD 显示惯例），扭转轴始终正确。← 这正是我们要的。
2. **打破依赖环**：直连约束在"源骨又被目标骨影响"时会成环；dummy/shadow 把求值分两步错开。（这也是为什么 mmd_tools 给**腿 D 骨**自动建了影子骨、给无环的**捩骨**没建——`apply_additional_transform` 对捩骨只生成直连约束。）
3. **干净地施加 influence 比例**。

---

## 3. 重构方案（实现顺序）

> 用户指定顺序（2026-06-06）：“先建 dummy、shadow，然后再调权重，要重构这个旋转骨的逻辑”。

改 `operators/add_twist_bone_operator.py`，**重构**而非打补丁：

1. **先建 `_dummy_` + `_shadow_` 影子骨链**（按上面 2.1 复刻目标）：
   - 每根子捩骨 `{side}{stem}{i}`（腕捩1/2/3、手捩1/2/3 × 左右 = 12 根）建一对 `_dummy_` / `_shadow_`。
   - `_dummy_` 父挂主捩骨；`_shadow_` 父挂变形骨的父级（腕/ひじ），加 `COPY_TRANSFORMS`(POSE→POSE,REPLACE) 取 dummy。
   - 变形骨的约束从现在的直连 `→主捩骨` 改成 `TRANSFORM(LOCAL→LOCAL,ADD)→_shadow_`。
   - 替换掉现有的直连付与（即去掉 `setup_grants` 里直接产生直连约束的路径，或保留 mmd_bone 元数据但改走影子骨）。
2. **子捩骨掰朝上** `(0,0,1)`（与目标一致）。主捩骨保持沿臂、保留 `fixed_axis`。
3. **再调权重** `setup_weights`：为新结构重做（守恒切分，参考既有 τ 系数；确认朝上后权重分布仍对应正确的捩骨档位）。

---

## 4. 动手前还缺的数据（必须先扒）

目标的变形骨 `腕捩1.L` 上那条 `TRANSFORM` 约束，**只查了 X 轴**（`from/to_x = 0`）。还需要：

- **Y、Z 轴的完整 from/to 映射**（`from_min/max_{y,z}`、`to_min/max_{y,z}`、`map_from`、`map_to`、`from_rotation_mode`、`mix_mode`）。
- **0.25/0.5/0.75 的 influence 比例到底藏在哪**：目标的 `_dummy_` 是父挂主骨拿**全量**变换，所以比例很可能在变形骨 `TRANSFORM` 的 from/to 区间里（例如 from ±180° → to ±45° = 0.25）。需确认。

扒法：在远程 Blender 里 dump 目标 `腕捩1.L / 腕捩2.L / 腕捩3.L` 的 TRANSFORM 约束全字段（三根的 to 区间比例应分别对应 0.25/0.5/0.75），即可反推规则。

---

## 5. 验证清单（改完后）

1. 子捩骨显示**朝上** `(0,0,1)`，和目标一致。
2. **扭转梯度不变**：frame 250（左手捩≈77°）下，左前臂顶点位移相对重构前 ≈ 0（不像实验那样 2–5cm）。
3. 影子骨 `_shadow_/_dummy_` 数量正确（每子骨一对，共 +24 根）。
4. 并排渲染 frame 250 前臂特写，与重构前/目标对比无异常。

---

## 6. 相关引用

- **代码**：`operators/add_twist_bone_operator.py`
  - 子骨创建（dir=base/roll=base）：约 40–55 行
  - `setup_grants`（设 fixed_axis + mmd_bone 付与元数据）：约 84–128 行
  - `setup_weights`（沿轴 5 段守恒切分）：约 131 行起
- **测试用例 / 远程环境**：见记忆 `conversion-test-task`、`remote-blender-connection`、`dev-deploy-workflow`
  - 源 XPS：`E:\mywork\mymodel\inase (purifier)_lezisell-A\xps-b.xps`
  - 目标 PMX：`E:\mywork\mymodel\Purifier Inase 18\Purifier Inase 18 None.pmx`（导入 `scale=0.083956`，与转换模型等高 1.749m）
  - 动作 VMD：`E:\mywork\mymodel\yaoxiang\yaoxiang.vmd`（frame 250 左手捩≈77°，适合验证）
- **部署流程**（改代码后）：本地 push → 远程 `git -C E:\code\othercode\convert_to_mmd pull` → `Copy-Item operators\*.py` 进 addon 目录 → 在运行的 Blender 里 `addon_disable` + 清 `sys.modules` + `addon_enable` 重载 → 全新转换验证。
