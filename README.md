# Snowbot

Snowbot 是基于 [Bindu](https://github.com/WXB1870/bindu) 的扫雪机器人应用，维护机器人模型、PICO 双臂遥操作、预设动作回放、设备适配、仿真和部署工具。通用运动求解、控制权与执行契约由 Bindu 提供，配对版本固定在 [dependencies.repos](dependencies.repos)。

[准备与构建](#准备与构建) · [现场运行](#现场运行) · [配置入口](#配置入口) · [代码与模型](#代码与模型) · [开发与版本](#开发与版本) · [本地文档与记录](#本地文档与记录)

## 准备与构建

完整 x86 部署环境为 **Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12**，需预先准备 Pinocchio C++ 环境和设备权限。部署入口位于 [tools/deployment/deploy.py](tools/deployment/deploy.py)。

```bash
git clone https://github.com/WXB1870/snowbot.git
cd snowbot
python3 -m tools.deployment.deploy prepare --with-impedance
```

`prepare` 按依赖锁获取 Bindu、生成底层源码并构建两仓软件；后续增量更新使用：

```bash
python3 -m tools.deployment.deploy build
```

上面的准备命令编入阻抗后端，与下文 `base` 的默认启动模式配套；仅位置控制可省略 `--with-impedance`，并以 `base --no-impedance` 启动。Orin 使用 `prepare --with-impedance --machine orin`。机器选择会被保存，后续构建复用。x86 与 Orin 的环境及验证结果分别核对。

默认部署产物在 `artifacts/robot/`，现场上层配置为 `upper/site.json`，底层参数为 `vendor-position.yaml`，配对版本与配置见 `deployment-manifest.json`。准备、构建与设备启动分开。

## 现场运行

已完成构建、设备配置与标定后，由现场人员从同一部署目录运行 [tools/run_robot.sh](tools/run_robot.sh)。两个终端使用同一 ROS 域和配套安装版本：

| 终端 | 命令 | 用途 |
|---|---|---|
| 1 | `bash tools/run_robot.sh base` | 启动底层、设备反馈和上层通信，默认选择阻抗模式 |
| 2，按需选一 | `bash tools/run_robot.sh vr` | PICO 遥操作、执行器、浏览器与录制 |
| 2，按需选一 | `bash tools/run_robot.sh replay` | Prepare 后进入动作回放菜单 |
| 2，按需选一 | `bash tools/run_robot.sh menu` | 已装工具时直接进入回放菜单 |
| 2，按需选一 | `bash tools/run_robot.sh prepare` | 仅执行准备流程 |

位置模式由终端 1 的 `base --no-impedance` 选择，上层读取 base 模式。阻抗模式需已构建对应后端并完成现场标定。位置会话统一使用 `session_v3`。

电脑打开终端打印的 `http://机器IP:8013/`，PICO 使用打印的 HTTPS 地址。VR 支持独立双臂和工具耦合模式；详细按键、准备位、停止恢复、可选 `--home` 回零及 `open-grippers` 单独开爪流程见本地运行文档。**这些入口可能使真实硬件运动，需由现场人员确认条件并操作；离线测试不代表真机验收。**

## 配置入口

配置随 `snowbot_bringup` 安装。按修改目标选择维护源，生成文件用于部署核对。

| 要调整的内容 | 维护入口 |
|---|---|
| 真机／仿真装配、启动组件 | [config/deployments/](src/snowbot_bringup/config/deployments/) |
| Orin 网卡、ROS 域、VR 端口与 TLS 路径 | [machines/orin.json](src/snowbot_bringup/config/deployments/machines/orin.json) |
| 输入有效期、双手配对、求解超时、执行时序与诊断 | [runtime_parameters.json](src/snowbot_bringup/config/robots/snowbot/runtime_parameters.json) |
| 准备姿态、按钮流程与握持确认 | [teleop.json](src/snowbot_bringup/config/robots/snowbot/teleop.json) |
| 夹爪、底盘 | [grippers.json](src/snowbot_bringup/config/robots/snowbot/grippers.json)、[snowbot_base.json](src/snowbot_bringup/config/robots/snowbot/snowbot_base.json) |
| 本体、关节限制与工具求解 | [robot_snowbot_real.json](src/snowbot_bringup/config/robots/snowbot/robot_snowbot_real.json)、[bimanual_snowbot_real.json](src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json) |
| 预设动作与转场 | [primitive-library/](src/snowbot_bringup/config/robots/snowbot/primitive-library/) |
| 雪铲物理仿真 | [snowbot_shovel_physics.json](src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_physics.json) |
| 底层公共参数与驱动补丁 | [position_session/](tools/deployment/snowbot_vendor/position_session/) |

`teleop.json` 和 `grippers.json` 的公共字段在顶层，环境差异在 `modes.real`／`modes.simulation`。直连与远程 VR 共用输入时序参数。单臂、独立适配和演示配置在 [examples/snowbot/](src/snowbot_bringup/config/examples/snowbot/)，部分默认 launch 和测试仍会引用。

机器配方部署时，从真机模板、底层公共参数和机器覆盖生成 `upper/site.json`、`vendor-position.yaml`。长期修改应进入对应维护源；证书、私钥与生成产物留在机器本地。

## 代码与模型

| 位置 | 职责 |
|---|---|
| [snowbot_bringup](src/snowbot_bringup/) | launch、配置及运行入口；`runtime` 管启动，`visualization` 管显示与诊断，`simulation` 管仿真，`web/` 放浏览器页面 |
| [snowbot_teleop](src/snowbot_teleop/) | PICO 操作流程、协调节点与 `replay` 预设规划／回放 |
| [snowbot_hardware](src/snowbot_hardware/) | 设备协议、位置会话和反馈适配 |
| [snowbot_description](src/snowbot_description/) | URDF、CAD、显示／控制模型和雪铲握点；资源来源见 [SNOWBOT_ASSETS_NOTICE](src/snowbot_description/SNOWBOT_ASSETS_NOTICE) |
| [tools/deployment](tools/deployment/) | 部署入口、底层源码准备器与补丁 |
| [vendor/snowbot](vendor/snowbot/) | 原始底层基线及来源清单 |
| [tests](tests/) | 单元测试、原生检查与集成验证 |

当前真机／仿真配方共用 `urdf/snowbot_control.urdf`，轻量显示使用 `snowbot_display.urdf`；D 型把手雪铲及左右握点见 [d_handle_snow_shovel.urdf](src/snowbot_description/urdf/d_handle_snow_shovel.urdf)。Isaac 物理后端需要独立的 GPU 与软件环境，轻量显示可用于日常仿真和实机。

## 开发与版本

通用契约、求解与执行逻辑在 Bindu 维护；型号配置、设备适配和操作流程在 Snowbot 维护。底层以 `vendor/snowbot/original/` 为基线，通过 [prepare_position_session.py](tools/deployment/prepare_position_session.py) 应用本仓补丁并生成构建源码；阻抗来源与哈希见 [impedance_118.json](vendor/snowbot/impedance_118.json)。

在加载配套 Bindu、ROS 和原生依赖后，从 Snowbot 根目录运行相关测试，例如：

```bash
python -m unittest tests.unit.test_pico_control
```

源码测试通过 `BINDU_TEST_SOURCE` 选择配对 Bindu。构建前需要阻抗专项检查时，已有入口为 `build --validate session`（另支持 `rt`、`nominal`、`all`）；普通增量构建使用 `build`。

两仓按配对版本维护：

| 分支 | 用途与边界 |
|---|---|
| `main` | 真机部署维护线，按 `dependencies.repos` 选择配套 Bindu；部署与运动验收由现场执行 |
| `xavier/session-v3-position-impedance` | 通信链路、模块分层与会话精简的重构候选，尚未经过真机测试 |

共享功能与修复先进入 `main`，再同步到精简分支；重构完成真机验收后再考虑合入。其他历史专题、部署和实验入口保留为本地 `archive/` 标签，远端旧引用未随本地整理删除。

## 本地文档与记录

以下内容按当前仓库约定仅在本地保留，**新克隆不会包含**；源码、配置和命令入口见上文。

| 本地位置 | 查找内容 |
|---|---|
| [docs/运行与验证.md](docs/运行与验证.md) | 环境依赖、现场配置、PICO 操作、仿真、日志与专项诊断 |
| [docs/当前进度.md](docs/当前进度.md) | 当前选定版本、验收边界、阻塞和下一步 |
| [docs/TODO.md](docs/TODO.md) | 待完成事项 |
| [docs/底层驱动问题记录.md](docs/底层驱动问题记录.md) | 已知问题、措施与证据 |
| [docs/位置遥操作修改目标.md](docs/位置遥操作修改目标.md) | 遥操作需求与目标 |
| [docs/过程记录/](docs/过程记录/) | 按日期追溯修改和验证 |
| `artifacts/field/<设备>/<批次>/` | 现场版本回执、日志与分析 |
| `artifacts/experiments/`、`artifacts/validation/` | 离线实验与软件验证结果 |
| `artifacts/exports/`、`artifacts/archive/`、`releases/` | 导出材料、历史归档与生成的版本快照 |

框架设计与模块说明见 [Bindu](https://github.com/WXB1870/bindu)。
