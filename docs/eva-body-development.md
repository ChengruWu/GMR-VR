# GMR Body 1.0.0 开发与待验证事项

本包为 EVA Body 接口的实验实现，分发名 `gmr-body`，导入名 `gmr_body`，工厂 `gmr_body:create_backend`。使用当前仓库的 GMR 求解器，Luna 模型、关节表与 IK 任务参数来自旧 GMR 仓库的 PICO 全身实验。构建时将当前求解器复制到包内私有命名空间，不覆盖旧 `gmr_m2` 或 `general_motion_retargeting`。旧 M2 0.4.0 和 Body 应分别选择，不应依靠同时安装解决版本切换。

`resources/body_config.json` 配置 24→17 的命名映射、坐标轴、时间预算、单步限制及手柄扳机映射；`resources/luna_ik.json` 配置任务权重、偏移与人体缩放；`resources/gento_luna/joint_limits.json` 显式配置 EVA 名称、顺序、MuJoCo 名称与限位。人体输入的 24 个关节全部检查，17 个参与规范化；实际 IK 任务只使用 Hips、Chest、Head、双肩、双肘、双腕。支撑链由胸部任务间接驱动，不是人体髋膝逐关节映射。

首次有效帧使用真实 24 项反馈设置模型并建立一次站姿空间参考，返回 idle；后续每次以当前反馈为求解初值，固定底盘、车轮和夹爪自由度，由触发器产生夹爪目标。可选输出底盘目标 `base_target_xyyaw`：取 GMR 中 Hips 到 base_Link 的任务目标（已应用空间对齐和配置偏移），位于机器人反馈的世界坐标系。该目标并非由可移动底盘自由度求解得到；EVA 单独比较目标和实测底盘位姿并输出速度。失败、输入过期、会话/参考空间变化或序号回退后要求 reset；重复样本返回 idle。时钟是 PC 接收单调时钟，不声称为 PICO 采样时间。每次关节目标相对实测反馈的变化受配置限制，解算超预算或返回后输入过期会被拒绝；同步调用无法强行中断卡住的求解。

默认 `allow_unvalidated_output` 为 false，工厂可加载但拒绝输出。仅用于受控仿真时，复制配置到独立文件，将该值改为 true，并通过进程环境变量 `GMR_BODY_CONFIG` 指向该文件。不要把该开关作为实机安全认证。默认配置采用旧实验假设的 XR 轴、global 和米，尚未在目标固件上验证。

底盘目标另需在该文件设置 `output_base_target: true`，同时在 EVA 配置启用 `body_gmr.base_target_control.enabled`；两边都默认关闭。启用时要求真实底盘位姿反馈，EVA 根据世界坐标误差限速并转换为机器人坐标系速度。阈值是待验证实验值；尚无加速度限制、避障或硬件停车保证。

当前 EVA-VR 源码发送 `units=metres` 和 `joint_space=base_space`；配置将它们解释为米和相对于同一参考空间的全局姿态。这是待验证的约定，不代表已验证设备输出。元数据有值时，若与配置不符会拒绝。

构建：`uv run --no-project --python 3.11 --with build --with setuptools --with wheel python packaging/body/build.py`。wheel 位于 `packaging/body/dist/`，需在运行 EVA 的 Python 环境安装；EVA 提供 `eva_body_contract` 类型，包不复制该模块。构建产物包含私有 GMR 求解器、Luna XML、49 个网格、配置、关节表和仓库 MIT 许可证；Luna 模型及网格的来源和再分发许可仍须另行核验。

本次 Windows Python 3.11 验证中，仓库 `tests` 13 项通过，独立安装 wheel 后的 EVA 契约和真实模型合成测试 3 项通过；EVA 相关接口与执行测试 13 项通过。依赖版本尚未为该独立包生成锁文件，构建可复现性和其他依赖版本组合仍需验证。

## 必须继续验证

1. PICO 实机固件的 24 关节名称、顺序、flags、姿态坐标轴、单位、全局空间、追踪丢失与重定位；Body 与手柄同步及设备时钟尚不确定。
2. EVA 与 GMR 的 24 项顺序、夹爪米单位与 trigger 方向、实际反馈来源和接收时效，及 34 项中被动轮和 mimic 的约定。
3. 自然站姿参考、Xsens 姿态偏移、身高比例与轴映射在多姿态和不同使用者上的合理性；第一帧不是已验证的人体标定。重定位通知当前未进入候选契约，需协议扩展或操作流程支持。
4. 离线真实录制回放、动作方向与可达性；MuJoCo 下左右抬手、躯干旋转、弯腰、下蹲、头部和夹爪动作、断流与恢复。当前仅做合成 smoke，不证明追踪精度。
5. 当前实现没有完整自碰撞/场景碰撞、加速度约束、关节跟踪误差监控或硬超时隔离；底盘的最大位姿误差检查、单步限制和模型关节限位不构成实机安全系统。必须先完成动力学、独立急停、watchdog 和现场验收。
6. 确认求解耗时及端到端频率并确定数值门槛；特别是首次运行、CPU 差异及超时恢复。必要时改为隔离进程。
7. 安装、旧 0.4.0 切换回滚、真实 EVA 网络链路和旧模式回归；目标 Windows 3.11 的完整上游锁文件安装目前被 `proxsuite` 源码构建路径限制阻止，本包采用 daqp，不依赖 proxsuite。
