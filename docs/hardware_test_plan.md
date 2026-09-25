# 真机测试清单（FR20）— 2026-09-25 版

目标：
1. 确认整条链路在真机上可靠：Houdini → Pre-Flight → CSV → 播放器 → FR20。
2. 用实测数据替换软件里的三个假设：房间、加速度上限、URDF 和控制器运动学是否一致。
3. 记录控制器现有的安全设置，为下一步“控制器安全设置当运行时防线”做准备（见 `docs/standard_tools_eval.md` 第 3 节）。

**每一步会动的，先确认三件事：工作区清空、手在急停上、知道 WebUI 的全局速度。**
表里标 🟢 的不动，🟡 小幅动，🔴 大幅动。
所有命令都在仓库根目录运行，`<IP>` 换成真机 IP。

---

## 0. 准备（不动）

1. 跑自测，每条最后都应输出 OK：

```bash
python scripts/fairino_player.py --self-test
```

```bash
python scripts/collision.py
```

2. **改播放设置。** 你本地的 `playback.toml` 现在是 `speed = 1.0`、`acc_limit = 300`，这是给 SimMachine 用的，真机不要用。打开 `python scripts/play_ui.py`，在界面里改成：

| 项 | 真机值 | 原因 |
|---|---|---|
| Target | Hardware | 界面会显示红字警告 |
| Controller IP | 真机 IP | |
| Speed (0-1) | **0.3** | 先慢速；真机上每一步动作前界面都会先问你 |
| Acc limit deg/s² | **150** | FR20 profile 的值；Houdini 的 Retime 和 Pre-Flight 都按 150 规划 |
| MoveJ % | **10** | 去起点、回 HOME 用 |
| Wiggle | J1, 3°, 4 s, 1 次 | J6 转 5° 看不出来 |

3. **WebUI 安全设置拍照记录**（不要改，只记）：
   - 软限位
   - 碰撞检测等级和碰撞策略
   - 速度限制
   - 安全墙、干涉区
   - 奇异点保护
   - 如果有“安全参数校验和”，也记下来

   以后播放器会读回这些设置核对，对不上就拒绝播放。

## 1. 连接检查 🟢

play_ui 按 **1 Check**，或者：

```bash
python scripts/fairino_player.py --check --hardware --ip <IP>
```

看四项：型号、错误码为 0、当前关节角、**FK 与 URDF 的差**（控制器 TCP 和我们 URDF 算出的 TCP 之差）。
SimMachine 上这个差是 0.004 mm，但 SimMachine 实际是 FR5 的模型。**真机的数才是第一次验证 FR20 的 URDF。** 差超过 1 mm 先停，把输出发给我。

## 2. 回 HOME 🔴（MoveJ）

HOME = `[0, -90, 90, -90, -90, 0]`：大臂朝上、小臂朝前、工具朝下，TCP 在底座前约 0.85 m、高约 1.1 m。
**不要回全零**：全零时 FR20 平躺，TCP 离底板只有约 8 cm。
播放器这边的路径检查已暂停，所以 MoveJ % 用 10，眼睛盯着。

play_ui 按 **6 Go HOME**。到位后再按一次 **1 Check**，记下 HOME 位置的 FK 差。

**FK 交叉验证（建议做）**：用拖动示教把手臂摆到 3 个明显不同的姿态，每个姿态按一次 Check，记下 FK 差。
- 3 个姿态：伸远、贴近底座、手腕转大角度。
- 如果差随姿态变化，说明 URDF 连杆长度和控制器不一致。

## 3. Wiggle 🟡

play_ui 按 **4 Wiggle**（J1 3°），或者：

```bash
python scripts/fairino_player.py --hardware --ip <IP> --wiggle 1 3 4 1
```

整条手臂会左右轻摆。日志会写 `wiggle: J1 moved 3.00 deg`。这一步验证 ServoJ 流式链路通不通：之前 J6 5° 在真机上看不出动。

## 4. 测量房间 🟡（拖动示教）

`envs/volvox_lab.json` 现在是照片估计的，朝向也是假设的：假设机械臂正前方（J1=0 时手臂伸出的方向）朝电视墙。Houdini 里 Pre-Flight 的 Cell 检查、Show Cell 显示的房间，都基于这个估计。

1. WebUI 打开拖动示教。
2. 打开探点窗口（只读，不会让机械臂动）：

```bash
python scripts/probe_ui.py
```

   选物体（下拉里有常用的），把工具尖端贴到点上，按 **Record point** 或回车。
   - 表格会列出每个点，以及“URDF vs controller mm”：我们 URDF 算的 TCP 和控制器 TCP 差多少。这就是 FK 交叉验证，每个点都顺带做了。
   - 下面一行显示每个物体已有几个点、还需要几个。
   - 命令行版本也还在：`python scripts/probe_env.py --ip <IP> --tool-len 0.0`，输入名字回车，`u` 撤销，`q` 退出。

| 名字 | 点 |
|---|---|
| `floor:level` | 地面 3 个点，散开；法兰面尽量放平（倾斜时边缘先着地，读数偏高） |
| `wall_tv:wall` | 电视墙，沿墙水平方向散开 2–3 个点（按竖直墙拟合） |
| `partition_left:wall` | 左侧隔断，同上 |
| `control_cart:box` | 红色小车台面 4 个角 |
| `operator:cylinder` | 操作员站位的地面，绕一圈 4–5 个点 |
| `stage:point` | 表演区域的几个角，仅作参考 |

装了工具就用 `--tool-len` 填工具长度（米）。更准的做法是先用控制器的工具标定（4 点或 6 点）量出 TCP，再把长度填进来。

3. 在窗口里先按 **Preview fit** 看会改什么，再按 **Write env** 写入（旧文件存成 `.bak`，会列出每个物体移动了多少）。命令行版本：

```bash
python scripts/env_from_points.py envs/volvox_lab_points.json
```

4. 在 Houdini 的 robot_arm 上打开 **Display > Cell**，看房间和实际是否对得上。
   - 区域和高墙现在有实线轮廓，不用选中节点也能看到。
   - 如果朝向反了（电视墙跑到机械臂背后），告诉我，我来改坐标系。

## 5. 测加速度上限 🟡

现在所有规划都用 150 deg/s²，这是手册里 20–25 kg 扩展负载的值。空载时大概率能更高。这个数决定动作能有多“快”：
- 在 150 下，大幅动作不可能快，“突然”的动作只能是 7–10° 的小刺；
- 在 450 下，同样的 punch 段落，TCP 速度从 0.6 m/s 提到 1.3 m/s。

**先确认底座固定**（法奥手册：6 颗 M10、强度 ≥ 8.8 级螺栓，扭矩 ≥ 45 N·m，装在刚性、无共振的底座上，FR20 建议直接固定在地面）。大关节测试会把反作用力传到底板上。

用窗口测（只动一个关节，出发前会检查关节限位和房间；真机上每一级都会先问你）：

```bash
python scripts/accel_ui.py
```

先测 J6、J5、J4（3°，手腕和姿态基本无关），再在 HOME 测 J3、J2、J1（2°，结果只适用于和测试姿态相近的姿态）。先按 **1 Check** 看预检，再按 **2 Run levels**。结果存在 `tests/accel/`。命令行版本：

```bash
python scripts/accel_probe.py --hardware --ip <IP> --joint 6 --amp 3 --report accel_j6.json
```

记下每个关节 “clean up to” 的值。有抖动、异响或报错就停，那一级不算。**这些数先别改进 profile，发给我。**

## 6. 播放片段 🔴

每个片段按 0.3 → 0.6 → 1.0 的速度播放，都勾上 Record 和 Report。

**每个片段的流程**：
1. 在 play_ui 里选好 CSV。
2. **2 Dry run**：`time_scale` 应该是 1.0，大于 1 表示播放器会放慢。
3. **3 Go to start**。
4. **5 Play**。
5. 播完按 **6 Go HOME**。

| 顺序 | 片段 | 内容 |
|---|---|---|
| a | `tests/csv/fr20_test.csv` | 你的曲线，18 s，之前在 0.3 真机跑过 |
| b | `tests/csv/dance_d01_punch-punch.csv` | 单一动作：punch，14 s |
| c | `tests/csv/dance_d17_punch-float-punch.csv` | 对比段落 ABA，16 s |
| d | `tests/csv/dance_d32_float-slash-press-float.csv` | 四小节混合，38 s |
| e | 当前 Houdini 场景的曲线，新导出 | 验证今天修的 Retime |

四个 CSV 我今天都 dry-run 过，在 1.0 下 `time_scale` 都是 1.0。

**e 的做法**：在 Houdini 里依次按 Retime → Pre-Flight（Robot playback 应该是 OK）→ Export。然后在 play_ui 里选这个新 CSV，照上面的流程播。

**报告里看三项**：
- `tracking_after_lag_max_deg`：跟踪误差，越小越好；
- `controller_error_after`：应该是 0；
- 有没有出现 `skipped`。

## 7. 回看真机轨迹（不动，Houdini）

Record 生成的 `*_actual_*.csv` 可以直接回放：在 robot_arm 的 **Output > Import CSV** 里选它，按 Import。现在锁定的实例也能导入，文件是实时读的。

看点：
- 真机轨迹和设计的曲线差在哪里；
- 哪一段落后最多。

## 8. OAK-D（如果带了）🟢

1. 先告诉我相机型号，以及能否 `pip install depthai`。
2. 采集脚本我会用 depthai 加 MediaPipe 写，不自己造姿态估计。
3. 相机到机械臂底座的标定用 OpenCV ChArUco 手眼标定。
4. 数据格式参考 `tests/keypoints/synthetic_float_punch_float.json`：每帧肩、肘、腕的三维点，单位米。

---

## 今天不要做

- **不要在真机上测“软限位或干涉区能不能在 ServoJ 播放时触发”。** 先在 SimMachine 上测。
- 不要修改 WebUI 的安全设置，只拍照记录。
- 不要用 `acc_limit = 300` 或 `speed = 1.0` 作为第一遍。

## 需要发给我的

| 文件或记录 | 用途 |
|---|---|
| 第 0 步的 WebUI 安全设置照片 | 控制器安全设置当运行时防线 |
| 第 1、2 步 Check 的 FK 差（HOME + 3 个姿态） | URDF 和控制器运动学对照 |
| `envs/volvox_lab_points.json` | 房间实测点 |
| `accel_*.json` | 新的加速度上限 |
| `*_actual_*.csv` 和报告 JSON | 跟踪误差、播放表现 |
| 任何报错码和当时的操作 | 排查 |
