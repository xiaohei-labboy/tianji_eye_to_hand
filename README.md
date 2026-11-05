# 天机双臂 + RealSense 眼在手外标定

固定相机，法兰上刚性安装标定板，逐个姿态停稳后采样。本程序负责图像识别、机器人反馈读取、正运动学、数据记录及离线标定。摆位由现场已有拖动方式完成；程序没有自动运动、使能、模式切换或清错功能。

**默认运动学配置已经按你的选择设置为：**

```text
/media/zhh/D44E3D434E3D2022/TJ_FX_ROBOT_CONTRL_SDK/CommonConfig/ccs_m6_40.MvKDCfg
```

SDK 根目录也已经配置。A 为左臂，B 为右臂，运行采集时必须明确选择。输出基坐标系是所选手臂的 SDK 运动学基座，不自动假定左右臂共用一个全局坐标系。

## 1. 已完成与实际限制

- 提供独立 Python 程序，不修改厂家 SDK 源代码。
- 原图测试自动识别到 **56 个外围内角点**，无需二维码解码。见 `detection_preview.jpg`。
- 棋盘按 **10×10 方格、5 mm 格距**建模，内部统一使用米。原点在逻辑棋盘外角 `(0,0)`，点坐标是 `(列×0.005, 行×0.005, 0)`。
- 自动初始化先寻找白色板框，再拟合外围角点；白边约一格仅作为搜索初值，不用于计算实际尺度。初始化至少要求 40 个实测角点，并检查黑白排列及四边覆盖。
- 初始化后用光流和几何约束固定角点编号。中央 `3≤列≤7、3≤行≤7` 的 25 个内交点完全不用于计算，剩余 56 个才是候选点。
- 跟踪丢失不会自动重设原点。需要按初始化参考图重新选定同四个物理角点，防止 180° 对称造成编号混乱。
- 这是经照片和合成数据验证的采集原型；实机相机图像、网络反馈和最终标定精度仍需现场测试。相机硬件型号、序列号、固件从连接设备读取，不将输入中的“D43i”硬编码为某个型号。

## 2. Python 环境

当前工作区已在 `work/tianji-venv` 准备好隔离环境，安装了 `pyrealsense2 2.59.1.11053`，并复用系统的 OpenCV/NumPy/SciPy。**在当前目录直接使用 `./run.sh` 即可**，例如：

```bash
./run.sh inspect-sdk --arm A
./run.sh capture --arm A --ip 192.168.1.190 --session session_A_01
./run.sh solve --session session_A_01
```

`run.sh` 优先使用本目录 `.venv`，其次使用本工作区已准备的隔离环境，也可通过 `TIANJI_PYTHON` 指定 Python。移动/解压到其他位置后请按下面方法重新准备环境。

Python 3.10 或更高版本，需要带图形界面的 OpenCV。不要使用 `opencv-python-headless` 运行采集窗口。

通常可在本目录执行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

如果 Ubuntu 缺少 `ensurepip`，而系统已有 `pip`、OpenCV、NumPy、SciPy，可以使用：

```bash
python3 -m venv --without-pip --system-site-packages .venv
.venv/bin/python -m pip install pyrealsense2
```

以下示例统一使用 `.venv/bin/python`。若使用已经装好依赖的 Python 环境，请相应替换。Windows 需调整 Python 路径、SDK 路径以及 `robot_adapter.py` 中库文件哈希记录的 `.so` 文件名；这一交付版本面向你当前的 Linux 环境。

## 3. 先做不连接设备的检查

```bash
.venv/bin/python main.py inspect-sdk --arm A
```

该命令只加载本地动态库、检查 SDK ABI、读取选定模型配置，并对示例关节角做一次 FK。**打印的是离线示例位姿，不是真实机械臂当前位姿。** 可用 `--arm B` 检查右臂。

对一张标定板照片测试识别：

```bash
.venv/bin/python main.py detect --image /绝对路径/标定板照片.jpg --out detection_preview.jpg
```

照片测试会将最长边缩小到至多 1280 像素；报告的网格拟合 RMS 是该处理图像中的像素误差，不是手眼标定误差或毫米精度。正式采集使用相机所选分辨率，不使用照片缩放。

## 4. 现场采集

固定相机和机器人基座，把板刚性固定到所选机械臂。提前用现场熟悉的方式准备好可手动摆位/拖动的状态，退出占用同一控制连接的 MarvinPlatform。厂家 SDK 固定使用 UDP 端口 4730，不可与另一个 SDK/上位机控制进程同时占用。程序连接时核对活动帧、控制器版本以及机械臂 Type；模型的具体尺寸仍以你选定的配置文件为准。

下面 `192.168.1.190` 是厂家示例地址，请替换成实机地址：

```bash
.venv/bin/python main.py capture \
  --arm A \
  --ip 192.168.1.190 \
  --session session_A_01
```

默认 RGB 为 1280×720、30 FPS。可以指定 `--width`、`--height`、`--fps`；设备不支持的组合会报错，不会静默改变标定参数。有多个 RealSense 时使用 `--serial 实机序列号` 指定设备。

窗口按键：

| 按键 | 操作 |
|---|---|
| `S` | 保存一组已停稳、角点及 PnP 合格的数据 |
| `I` | 冻结图像，手动选择/恢复四个内角点编号 |
| `Q` 或 Esc | 退出并释放连接，已保存的数据保留 |

初次识别成功会保存 `initial_reference.png`，图中 1、2、3、4 是以后恢复跟踪的物理角点依据。按 `I` 后选 **内角点网格的四个最外角**，不是白色板框的四角，也不是二维码四角：

- 1：逻辑 `(1,1)`；2：`(9,1)`；3：`(9,9)`；4：`(1,9)`。
- 首次可按画面左上、右上、右下、左下依次选择。以后必须对应初始化时的同四个物理点，不能随着棋盘旋转重新按画面方位编号。
- 点满四个后按 Enter 确认，Backspace 撤销，Esc 取消。
- 如果已无法确认原来的物理方向，请重新采集一个新会话，不要把猜测的编号混入旧会话。

建议采集 20～30 组。覆盖不同位置和距离，并绕至少两条不平行轴改变板的倾斜角；保持板正面可见、尽量完整占据视野、避免反光和模糊。连续慢速移动，避免每帧跨过半个格子或突然大幅旋转。5 mm 格子比较小，程序要求投影后每格至少约 8 像素；若距离太远，应靠近或提高有效图像分辨率。

每次按 S 会检查：

- 近期约 0.5 秒内有持续更新的反馈，图像处理未明显超时。
- 各关节速度不超过 0.15 度/秒，窗口内变化范围不超过 0.06 度。
- 法兰位姿在该窗口内变化不超过 0.5 mm / 0.1°。
- 机器人无报错且不处于状态切换过程。
- PnP 重投影 RMS ≤1 像素；平面 PnP 两解接近且姿态差明显时拒绝保存，提示改变倾斜角。
- 与已有样本不是非常接近的重复位姿。

使用静止窗口、图像到达前后的反馈和时间记录进行采样，**不等于相机与控制器有硬件同步**。不要在连续运动中保存。棋盘固定姿态只平移的数据不足以进行这里的完整手眼标定。

会话目录必须是新目录，避免不同初始化方向、左右臂或相机设置混用。程序每次保存都会立即写入：

```text
metadata.json          # 设备参数、配置及库哈希、坐标约定
samples.json           # 机器人/相机矩阵、关节反馈窗口、角点、时间戳
initial_reference.png  # 首帧角点编号参考图
color_0000.png ...     # 去畸变后的 RGB 图像
```

## 5. 离线求解与验收

```bash
.venv/bin/python main.py solve --session session_A_01
```

至少 12 组才求解，建议 20～30 组；每第 5 个样本留作验证，不参与求解器选择或参数优化。结果写到会话中的 `calibration_result.json`：

- `T_base_camera`：**相机彩色光学坐标 → 所选机械臂 SDK 基座坐标**。
- `T_camera_base`：逆变换。
- `T_flange_board`：板坐标 → 法兰坐标，作为同时求出的固定安装关系。
- `training` / `validation`：拟合样本、独立留出样本的平移及旋转残差。
- `all_samples.per_sample`：逐样本误差，便于定位编号错误或不稳定样本。
- `quality_passed`：默认要求所有样本固定安装关系的一致性误差 ≤5 mm、≤2°，可以用 `--max-mm`、`--max-deg` 修改。失败时返回退出码 2，并保存诊断结果。

一致性通过并不代表绝对定位精度达到 5 mm。实际误差还受打印/加工格距、板平整度、相机内参、机器人几何标定、安装稳定性影响。应使用未参与采集的现场点再验证。

对以米表示的彩色相机三维点：

```python
import json
import numpy as np
result = json.load(open("session_A_01/calibration_result.json"))
T = np.array(result["T_base_camera"])
p_camera = np.array([0.02, -0.01, 0.50, 1.0])  # 示例三维点，单位米
p_base = (T @ p_camera)[:3]
```

若三维点来自原始深度光学坐标系，需先使用 RealSense 的深度到彩色外参转换到彩色光学系，再用本结果。不要把像素 `[u,v,depth]` 直接当成三维 XYZ，也不要直接混用深度和彩色坐标系。

## 6. 坐标关系与 SDK 接口依据

采用 `T_A_B` 将 B 系点映射至 A 系。对每个样本：

```text
T_base_camera @ T_camera_board_i
    = T_base_flange_i @ T_flange_board
```

从 `DCSS.m_Out[0/1].m_FB_Joint_Pos` 读取七个关节角（度），与 SDK `subscribe()` 返回的 `outputs[0/1].fb_joint_pos` 对应。使用选定模型的 `Marvin_Kine.fk()` 求法兰矩阵，平移 mm 除以 1000。`remove_tool_kine()` 只清除本进程运动学库的工具偏置，不调用控制器的工具配置接口。状态中的 `est_cart_fn` 是力估计，不是位姿。

向 `cv2.calibrateHandEye` 传入机器人矩阵的逆 `T_flange_base`，视觉部分传 `T_camera_board`，得到 `T_base_camera`。随后联合优化相机外参与板安装矩阵。用已知真值合成测试检查了这个方向，避免把眼在手上公式直接套入。

相机使用当前 RGB 流的内参和对应畸变模型。Brown-Conrady 通过 OpenCV 去畸变；modified Brown-Conrady 等通过 RealSense 前向投影建立去畸变映射。inverse Brown-Conrady 彩色流会明确报不支持。所有保存图像均使用原 K 的去畸变像素坐标，PnP 的 D 为零。求解不读取角点深度，不使用 IMU。

SDK 包装还显式匹配了原生函数签名：当前 C++ `OnGetIntPara` 的输出是 `long*`，本程序使用 `ctypes.c_long`，不依赖厂家 Python 包装中的 `c_int` 输出指针。

本地接口依据：`SDK_PYTHON/fx_robot.py`、`SDK_PYTHON/fx_kine.py`、`python_doc_kine.md`、`DEMO_PYTHON/showcase_kine_two_arms.py`、`contrlSDK100343/MarvinSDK.cpp`。视觉与标定参考 OpenCV calib3d/PnP 官方接口和 RealSense SDK 的 projection 接口。

## 7. 测试

```bash
.venv/bin/python -m unittest discover -s tests -v
```

测试覆盖：外围角点检测、忽略中央图案、连续跟踪及丢失处理、PnP 尺度、眼在手外方向、带噪声数据、留出样本隔离、退化运动拒绝，以及采样时反馈稳定性检查。测试不连接机器人、不移动机械臂。
