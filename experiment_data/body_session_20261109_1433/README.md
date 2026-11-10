# 2026-11-09 14:33 木偶模式实时测试

- `vr_capture.jsonl`：头显原始与标准化输入；session `4c202741a6ae4e2caa85726e832b7eef`，本地记录无丢弃、session_end 完整。包含人体姿态数据，请按个人数据妥善保存。
- `control.jsonl`：隔离探针每周期的来源帧、反馈关节和发送目标；用 `(session_id, frame_seq)` 对应原始记录。反馈在发布本周期目标之前采样，并非同一物理时刻。
- `probe.py`：本次运行脚本；仅用于本机仿真，探针自行处理 ARM 事件，不是完整控制台 Operation。
- 三个 `*.diff`：对应仓库未提交修改的快照；三个基准提交分别为 EVA-VR `f6d1ea8`、eva-mobile `5b5e575`、GMR-VR `2dcad59`。`python-packages.txt` 记录 EVA 环境包版本；这不是可复现锁文件。
- MuJoCo 节点 `--kinematic --stand-on-start`，使用本地独立端口 15555/15556/15557；bridge 握把长按阈值 600 ms。GMR 测试进程启用 `allow_unvalidated_output=true`、`relative_task_alignment=true`、`relative_position_scale=0.8`；底盘目标关闭，反馈容差默认关闭，夹爪目标无额外余量。头显内是第三人称二维相机画面。
- 443 控制周期：414 次发送、29 次 idle、无记录的拒绝；发送持续约 23.1 秒。发送不等于动作精度验收。用户要求保留数据后续调整映射和 IK。
- SHA-256：`vr_capture.jsonl` `a0f284807d97098a3bf3055d0611ce5ad87f9526e1aeb55a2fd81cd6cebdb996`；`control.jsonl` `581248dd6e7c46a3aec847f33f40bb48172be1bfa99f9e7eac96df926c6d885f`。
- 使用的 GMR wheel SHA-256 为 `4c4d94252991130779daf313f9b2425a96740744c876929d5af0c7e9acaa6a8a`；头显 APK SHA-256 为 `7e8f840b8d0181a617884df50e5128f0ce31bda6aacaa57fa0ce29a56ae4b6f7`。`headset_view.py` 保存本次使用的只读画面服务源码。
