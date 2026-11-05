# Snowbot

Snowbot 是基于 [Bindu](https://github.com/WXB1870/bindu) 开发的扫雪机器人应用，提供机器人模型、双臂遥操作、底层驱动适配、仿真和部署工具。通用运动求解与执行能力由 Bindu 提供，配对版本通过 [dependencies.repos](dependencies.repos) 固定。

## 快速开始

完整部署环境为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12 和 x86_64。依赖安装见[运行指南](运行与验证.md#支持条件)。

```bash
git clone https://github.com/WXB1870/snowbot.git
cd snowbot
python3 -m tools.deployment.deploy prepare
```

`prepare` 获取匹配的 Bindu，生成底层源码并构建底层和上层软件。后续更新使用 `python3 -m tools.deployment.deploy build`。

完成构建后，按[现场配置](运行与验证.md#首次现场配置)设置设备、标定和 PICO 网络，再按[启动顺序](运行与验证.md#日常启动)运行。开发和仿真见[仿真与浏览器操作台](运行与验证.md#仿真与浏览器操作台)。

本地与远程 PICO 的运行参数集中在 [runtime_parameters.json](src/snowbot_bringup/config/robots/snowbot/runtime_parameters.json)。本地直连使用顶层参数，远程调试在此基础上读取 `remote_vr` 中的输入有效期、换源等待、发送频率、双手时间差和告警间隔。远程现场的 `arguments.parameters_config` 可继续指向[远程入口](src/snowbot_bringup/config/robots/snowbot/runtime_parameters_remote_vr.json)；已有现场参数文件时，在其顶层加 `"vr_connection": "remote"` 即可选择远程模式，需要覆盖的远程值写在该文件的 `remote_vr` 中。下次启动时读取。

## 功能

- **双臂遥操作**：PICO 手柄输入、独立双臂与工具耦合模式、准备位和夹爪操作。
- **运动执行**：通过 Bindu 统一处理运动求解、控制权、目标期限、动态限制和停止反馈。
- **设备适配**：底层位置会话、关节与设备反馈、夹爪及型号配置。
- **仿真与可视化**：轻量运动学显示、Isaac 物理后端和浏览器操作台。
- **部署工具**：依赖锁定、底层补丁生成、增量构建和配置清单。

## 目录结构

```text
snowbot/
├── src/
│   ├── snowbot_bringup/       launch、配置、仿真工具与浏览器页面
│   ├── snowbot_description/   URDF、CAD 与模型资源
│   ├── snowbot_hardware/      设备协议、位置会话与反馈适配
│   └── snowbot_teleop/        PICO 操作流程与协调节点
├── vendor/snowbot/            底层驱动源码与基线清单
├── tools/deployment/         部署入口、源码准备器与补丁
├── tests/                    单元测试与集成检查
└── dependencies.repos        Bindu 版本锁
```

运行工具安装在 `snowbot_bringup.teleop` 和 `snowbot_bringup.simulation`，页面位于该包的 `share/web/`。

## 配置与模型

配置模板位于 [src/snowbot_bringup/config/](src/snowbot_bringup/config/)：

- `deployments/`：当前实机与位置仿真两份运行配方。
- `robots/snowbot/`：当前本体、动态限制、工具、PICO、夹爪和可选底盘参数；实机与位置仿真共用本体和工具配置。
- `examples/snowbot/`：单臂、独立适配和较早仿真入口使用的示例配置。
- `robots/snowbot/teleop.json`：操作流程、准备姿态与握持确认。
- `robots/snowbot/grippers.json`：夹爪开闭、增益、速度、标定与到位参数。
- `robots/snowbot/runtime_parameters.json`：输入／反馈有效期、求解超时、命令时序、执行、诊断与显示参数。

`teleop.json` 和 `grippers.json` 的公共字段位于顶层，`modes.real`／`modes.simulation` 保存各环境差异，运行时按配方的 `runtime_mode` 合并。

资源使用 `package://snowbot_bringup/...` 和 `package://snowbot_description/...` 路径。机器配置保存在部署目录的 `upper/site.json`，底层参数保存在 `vendor-position.yaml`；配置方法见[运行指南](运行与验证.md#首次现场配置)。

[D 型把手雪铲](src/snowbot_description/urdf/d_handle_snow_shovel.urdf)包含 28×45 cm 浅铲、22 cm 横把和名义间距 34 cm 的左右握点，右手握杆位置可用于不同握距评估。`left_grip_frame` 的 +Z 沿横把，`right_grip_frame` 的 +Z 沿杆。固定握点坐标用于标定，离线双轴握持通过 Bindu 的 `left_axial_grasp`、`right_axial_grasp` 求解，工具跟踪完整六维目标；CAD 与物理形状共用该 URDF。

## 开发

通用契约、求解与执行逻辑在 Bindu 维护；型号配置、底层适配和操作流程在 Snowbot 维护。底层代码以 `vendor/snowbot/original/` 为基线，在 `tools/deployment/snowbot_vendor/position_session/` 维护补丁与公共头，通过 [prepare_position_session.py](tools/deployment/prepare_position_session.py) 生成构建源码。

应用测试从仓库根目录执行，例如 `python -m unittest tests.unit.test_pico_control`。ROS 与原生依赖的加载方式、仿真构建及集成检查见[运行指南](运行与验证.md#仿真与浏览器操作台)。

## 文档

- [运行与验证](运行与验证.md)：依赖、构建、配置、PICO 操作与仿真。
- [Bindu](https://github.com/WXB1870/bindu)：通用框架与模块文档。
