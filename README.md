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

## 遥操作代码

PICO 配方由 `vr_input → pico_control → execution → 厂商 upper/middle/servo` 连接输入与设备。`pico_control` 在进程内组合应用流程、模式选择和 Bindu 会话，共享一个执行客户端及反馈缓存。QP 和准备路径校验使用各自的数值 worker，电脑显示通过观察器读取状态。

| 阅读位置 | 职责 |
|---|---|
| [assembly.py](src/snowbot_bringup/snowbot_bringup/assembly.py) | 启动装配与参数传递 |
| [node.py](src/snowbot_teleop/snowbot_teleop/node.py) | ROS 发布、订阅、服务和组件装配 |
| [application.py](src/snowbot_teleop/snowbot_teleop/application.py) | 准备、模式切换、跟随、取消及操作交接 |
| [control.py](src/snowbot_teleop/snowbot_teleop/control.py) | PICO 按键状态机与操作意图 |
| [preparation.py](src/snowbot_teleop/snowbot_teleop/preparation.py) | 准备路径、目标生成及实测到位检查 |
| [position.py](src/snowbot_teleop/snowbot_teleop/position.py) | 夹爪操作及夹爪反馈 |
| [status.py](src/snowbot_teleop/snowbot_teleop/status.py) | 面向操作者的状态视图 |
| Bindu `tool_session.py` / `tool_follow.py` | 会话就绪、输入映射、QP 事务、执行租约与清理 |
| Bindu `execution_client.py` | 执行服务客户端和唯一执行反馈缓存 |

模式选择和会话启动在应用内直接调用，`teleop/mode_state` 与 `teleop/state` 提供观察出口。`teleop/vr/input` 在存在观察或记录订阅者时发布处理后输入，独立夹爪通过 `teleop/vr/gripper_input` 接收扳机。执行层继续通过租约、修订号和实测反馈确认命令与停止。

日常 QP 诊断和稳定会话反馈以约 10 Hz 更新，阶段变化及时报告。`--diagnostics` 启用逐次 QP 诊断，采集与浏览器显示由独立观察进程处理。

## 配置与模型

[配置目录](src/snowbot_bringup/config/)随 `snowbot_bringup` 安装。按要调整的内容选择入口。

| 要调整的内容 | 配置位置 |
|---|---|
| 真机或位置仿真启动哪些组件 | [deployments/](src/snowbot_bringup/config/deployments/) 下的 real、sim 两份配方 |
| VR 有效期、双手配对、求解超时和执行时序 | [runtime_parameters.json](src/snowbot_bringup/config/robots/snowbot/runtime_parameters.json)。顶层是默认值，`remote_vr` 保存远程覆盖；同目录的 `runtime_parameters_remote_vr.json` 仅供已有部署选择远程模式 |
| 准备姿态、按钮流程、握持确认；夹爪和底盘参数 | [teleop.json](src/snowbot_bringup/config/robots/snowbot/teleop.json)、[grippers.json](src/snowbot_bringup/config/robots/snowbot/grippers.json)、[snowbot_base.json](src/snowbot_bringup/config/robots/snowbot/snowbot_base.json) |
| 本体、关节限制、双臂工具几何和求解权重 | [本体入口](src/snowbot_bringup/config/robots/snowbot/robot_snowbot_real.json)引用关节配置和[双臂工具配置](src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json)。独立加载双臂工具时，从同目录的运行参数读取默认时序 |
| 动作回放数据、雪铲物理仿真参数 | [primitive-library/](src/snowbot_bringup/config/robots/snowbot/primitive-library/) 和 [snowbot_shovel_physics.json](src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_physics.json) |
| 单臂、独立适配和早期仿真示例 | [examples/snowbot/](src/snowbot_bringup/config/examples/snowbot/) |

`teleop.json` 和 `grippers.json` 的公共字段位于顶层，`modes.real`／`modes.simulation` 保存各环境差异，运行时按配方的 `runtime_mode` 合并。

资源使用 `package://snowbot_bringup/...` 和 `package://snowbot_description/...` 路径。机器配置保存在部署目录的 `upper/site.json`，底层参数保存在 `vendor-position.yaml`；配置方法见[运行指南](运行与验证.md#首次现场配置)。

[D 型把手雪铲](src/snowbot_description/urdf/d_handle_snow_shovel.urdf)包含 28×45 cm 浅铲、22 cm 横把和名义间距 34 cm 的左右握点，右手握杆位置可用于不同握距评估。`left_grip_frame` 的 +Z 沿横把，`right_grip_frame` 的 +Z 沿杆。固定握点坐标用于标定，离线双轴握持通过 Bindu 的 `left_axial_grasp`、`right_axial_grasp` 求解，工具跟踪完整六维目标；CAD 与物理形状共用该 URDF。

## 开发

通用契约、求解与执行逻辑在 Bindu 维护；型号配置、底层适配和操作流程在 Snowbot 维护。底层代码以 `vendor/snowbot/original/` 为基线，在 `tools/deployment/snowbot_vendor/position_session/` 维护补丁与公共头，通过 [prepare_position_session.py](tools/deployment/prepare_position_session.py) 生成构建源码。

应用测试从仓库根目录执行，例如 `python -m unittest tests.unit.test_pico_control`。ROS 与原生依赖的加载方式、仿真构建及集成检查见[运行指南](运行与验证.md#仿真与浏览器操作台)。

## 文档

- [运行与验证](运行与验证.md)：依赖、构建、配置、PICO 操作与仿真。
- [Bindu](https://github.com/WXB1870/bindu)：通用框架与模块文档。
