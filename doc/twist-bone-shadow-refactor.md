# 捩骨（旋转骨）重构：_dummy_/_shadow_ 影子骨方案

> 目标：让转换模型的细分捩骨（腕捩1/2/3、手捩1/2/3）像参考 PMX 那样**竖直朝上显示**，同时不破坏扭转变形。
> 状态：调查完成，**待实现**（2026-06-06）。本文是动手前的设计依据。

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
