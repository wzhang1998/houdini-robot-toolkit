# 真机测试清单（FR20）

目标：用真机把软件里的三个假设换成实测，再用新数值重跑生成器。
每一步都会让机械臂动的，先确认：工作区清空、手在急停上、WebUI 全局速度已知。

命令里的 `192.168.58.2` 是法奥出厂默认 IP，换成你真机的。所有命令在仓库根目录运行。先把分支拉到最新，跑一遍自测：

```bash
python scripts/fairino_player.py --self-test
```

```bash
python scripts/collision.py
```

---

## 1. 连接检查（不动）

```bash
python scripts/fairino_player.py --check --hardware --ip 192.168.58.2
```

看型号、错误码、当前关节、FK 与 URDF 是否一致。IP 换成真机的。

## 1b. 回 HOME（会动，MoveJ）

HOME = `[0, -90, 90, -90, -90, 0]`：大臂朝上、小臂朝前、工具朝下，TCP 在底座前约 0.85 m、高约 1.1 m。**不要回全零**：全零时 FR20 平躺，TCP 离底板只有约 8 cm。路径检查暂停（碰撞只在 Houdini 的 Pre-Flight 里查），所以第一次回 HOME 时 MoveJ % 调低，盯着看。UI 里是"6 Go HOME"：

```bash
python scripts/fairino_player.py --hardware --ip 192.168.58.2 --goto-home
```

建议同时在 WebUI 里把这个姿态存成一个示教点。

## 2. 测量现场（不动，拖动示教）

`envs/volvox_lab.json` 现在是**照片估计**的，而且朝向是假设的：假设机械臂的正前方（J1=0 时手臂伸出的方向，URDF −X）朝向电视墙。先用机械臂本身把房间量出来：

1. WebUI 打开拖动示教（手引导）。
2. 运行探点工具，把工具尖端贴到点上，在提示处输入名字回车；输 `u` 撤销，`q` 退出：

```bash
python scripts/probe_env.py --ip 192.168.58.2 --tool-len 0.0
```

建议点位（名字:类型）：

| 名字 | 点 |
|---|---|
| `floor:plane` | 地面 3 个点，散开 |
| `wall_tv:plane` | 电视墙 3 个点 |
| `partition_left:plane` | 左侧隔断 3 个点 |
| `control_cart:box` | 红色小车台面 4 个角 |
| `operator:cylinder` | 操作员站位的地面，绕一圈 4–5 个点 |
| `stage:point` | 表演区域的几个角，仅作参考 |

装了工具就用 `--tool-len` 填工具长度（米），否则按法兰面计算。

3. 拟合，更新环境文件（旧的存为 `.bak`，会打印每个物体被移动了多少）：

```bash
python scripts/env_from_points.py envs/volvox_lab_points.json
```

4. 在 Houdini 里打开 `scenes/FR20_cell.hiplc` 看房间对不对，或者重新出图：

```bash
hython scripts/render_previews.py --only=cell_overview
```

## 3. 测加速度上限（会动，小幅）

现在所有规划都用 150 deg/s²，这是手册里 20–25 kg 扩展负载的值。空载时大概率能更高，这个数值决定舞蹈能有多"快"：

- 在 150 下，大幅动作不可能快，所以"突然"的动作只能是 7–10° 的小刺；
- 在 450 下，同样的 punch 段落 TCP 速度从 0.6 m/s 提到 1.3 m/s。

从最轻的 J6 开始，小幅度，逐级加速，每一级都会询问：

```bash
python scripts/accel_probe.py --hardware --ip 192.168.58.2 --joint 6 --amp 3 --report accel_j6.json
```

然后依次测 J5、J4，最后 J1–J3（幅度改成 2°）：

```bash
python scripts/accel_probe.py --hardware --ip 192.168.58.2 --joint 2 --amp 2 --levels 150 225 300 450 --report accel_j2.json
```

记下每个关节"clean up to"的值。把 `profiles/fr20.json` 的 `robot.max_acceleration_deg_s2` 改成每关节一个数，取干净上限的约 70%。然后重跑舞蹈工厂：

```bash
hython scripts/build_factory_scene.py --cook-dance
```

## 4. 播放片段（会动）

每个片段先 0.3，再 0.6，最后 1.0，都带 record。播放器默认速度就是 0.3：

| 片段 | 内容 |
|---|---|
| `tests/csv/fr20_test.csv` | 你的曲线，retime 后 18 s |
| `tests/csv/dance_d01_punch-punch.csv` | 单一动作：punch |
| `tests/csv/dance_d17_punch-float-punch.csv` | 对比段落 ABA |
| `tests/csv/dance_d32_float-slash-press-float.csv` | 四小节混合，38 s |

```bash
python scripts/fairino_player.py tests/csv/dance_d17_punch-float-punch.csv --hardware --ip 192.168.58.2 --speed 0.3 --record actual_d17.csv
```

看报告里的 `tracking_after_lag_max_deg` 和 `controller_error_after`。播放前 Houdini 的 Pre-Flight 已包含 Cell 检查；但现场量过之前，那个检查只基于估计的房间。

## 5. 可见的 wiggle

J6 转 5° 几乎看不出。想确认链路，用 J1 转 3°（整条手臂会摆）：

```bash
python scripts/fairino_player.py --hardware --ip 192.168.58.2 --wiggle 1 3 4 1
```

日志会写 `wiggle: J1 moved 3.00 deg`。

## 6. OAK-D（如果带了）

1. 格式：`tests/keypoints/synthetic_float_punch_float.json`。每帧一只手臂的肩、肘、腕（可选：手）三维点，单位米，坐标是表演者自己的方向（x 前、y 左、z 上）。
2. 采集：告诉我相机型号和能不能 `pip install depthai`，我写采集脚本（人体姿态加深度，输出这个格式）。
3. 转成机械臂片段，两种方法都会做碰撞检查和标注：
   - `direct`：照搬轨迹，慢的动作能 1:1；
   - `effort`：只保留 Laban 动态特征，快动作用这个。

```bash
python -c "import sys,json; sys.path.insert(0,'scripts'); import retarget as R, motion_clip as M, collision as C; kp=json.load(open('take.json')); env=C.load_env('envs/volvox_lab.json'); c=R.effort(kp, env); M.save(c,'take_effort.json'); M.to_csv(c,'take_effort.csv'); print(c['safety'], c['labels']['sequence'])"
```

## 记录

每一步的报告 JSON 和 `actual_*.csv` 留着，明天发给我。尤其是：

- `accel_*.json`：决定新的加速度上限；
- `envs/volvox_lab_points.json`：房间的实测点。
