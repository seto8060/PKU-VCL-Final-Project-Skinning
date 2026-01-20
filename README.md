# BVH loader & Skinning

实现了北京大学可视计算与交互概论（VCI）课程大作业的E.2. Skinning二星项目。本项目以 **BVH 动作捕捉数据** 为输入，通过 **骨架构建、前向运动学（FK）与动作重定向**，将动作迁移到 **FBX 角色模型** 上，并结合 **线性混合蒙皮（LBS）/ 对偶四元数蒙皮（DQS）** 实现角色动画驱动与可视化渲染。

---

## 功能概览

- **BVH 动作加载**
  - 递归解析骨架层级（HIERARCHY）与帧数据（MOTION）
  - 构建完整骨架结构与动作序列

- **前向运动学（FK）**
  - 基于层级结构自顶向下计算关节全局位姿
  - 使用四元数表示旋转，支持位移通道

- **动作重定向（BVH → FBX）**
  - 基于“语义关节映射”对齐 BVH 与 Mixamo FBX 骨架
  - 计算参考帧旋转偏移，逐帧迁移关节局部旋转
  - 根节点位移按腿长比例进行尺度修正

- **权重重映射与蒙皮**
  - 将 FBX 原始权重对齐到动画骨架
  - 非语义关节权重自动 fallback 到最近有效父关节
  - 支持：
    - 线性混合蒙皮（LBS）
    - 对偶四元数蒙皮（DQS，实验性）

- **可视化渲染**
  - 骨架火柴人渲染（调试 FK / 动作结构）
  - FBX 网格蒙皮渲染（最终角色动画）

---

## 使用方式

```bash
xmake run final
```

启动后通过 **ImGui 界面**进行交互：

- 选择 BVH 动作文件
- 选择 FBX 模型文件
- 切换渲染模式：
  - BVH 骨架火柴人
  - FBX 蒙皮网格
- 可选启用 **对偶四元数蒙皮（DQS）**

---

## 项目核心结构在 `./src/VCX/Labs/Final-Animation` 中，核心文件及其作用如下：

```text
Animation/
├── BVH/
│   └── BVHLoader.cpp          # BVH 文件解析
├── Pose/
│   └── FK.cpp                 # 前向运动学
├── Retarget/
│   └── Retargeter.cpp         # 动作重定向逻辑
├── FBX/
│   ├── FBXAnimSkeleton.cpp    # 动画骨架与 bind pose
│   └── SkinCPU.cpp            # 蒙皮（LBS / DQS）
CaseFinalAnimation.cpp         # 主程序
```

---

## 说明

- BVH 数据来源于 **CMU Motion Capture Database**
- FBX 模型来源于 **Mixamo**
- 当前条件下 **LBS 表现更稳定**，DQS 受绑定姿态与权重质量影响较大

---

## 补充

这个项目似乎相对于其他二星题还是有一定难度的（也可能是我代码能力不够），但扩展到三星应当不难，只是我来不及做了。
