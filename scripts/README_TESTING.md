# Planning / Lattice 测试说明

> 以下命令假设你已经自己创建并激活了虚拟环境(venv/conda 均可),并用根目录的 `requirements.txt` 装好了依赖(`pip install -r requirements.txt`)。本项目不提供预置的虚拟环境。

## 脚本一览

| 脚本 | 用途 |
|------|------|
| `run_lattice_component_checks.py` | 主回归：Lattice、参考线、交通规则、PathAssessment、PathBounds、OnLanePlanning 等 |
| `run_extended_planner_checks.py` | 扩展回归：Frenet 导数、障碍 scope、大横向 PathData、借道边界、阻塞+backup、legacy 开关 |
| `run_lattice_scenario_cases.py` | **全场景测试（34 项）**：lattice / decider / on_lane / stress / overtake |
| `run_lattice_demo_cases.py` | 精简演示（6 项，全场景的子集） |
| `run_minimal_lattice_plan.py` | 最小端到端 smoke（单参考线 + LatticePlanner.Plan） |
| `run_overtake_animations.py` | **超车场景** 测试 + 动画（`--tag overtake --animate` 的快捷入口） |
| `run_all_scenario_animations.py` | 全部场景（含 overtake）动画 GIF |
| `run_all_planner_checks.py` | 依次执行回归脚本（不含动画） |

## 快速体验（推荐先看这个）

```bash
# 全场景列表（34 项，分 lattice / decider / on_lane / stress / overtake）
python scripts/run_lattice_scenario_cases.py --list

# 跑全部场景
python scripts/run_lattice_scenario_cases.py

# 只跑 Lattice 核心（18 项，含曲线车道）
python scripts/run_lattice_scenario_cases.py --tag lattice

# 只跑某一个 / 几个
python scripts/run_lattice_scenario_cases.py open_road mid_lane_start stress_obstacle_distance_sweep

# 场景动画（推荐）：地图上自车每 0.1s 一帧，输出 GIF
python scripts/run_lattice_scenario_cases.py open_road far_obstacle_stop --animate

# 同时导出逐帧 PNG（frame_0000.png ...）
python scripts/run_lattice_scenario_cases.py open_road --animate --animate-frames

# macOS 打开 GIF
open scripts/output/animations/open_road.gif

# 精简 6 项演示（入门）
python scripts/run_lattice_demo_cases.py --animate

# 一键：全部场景动画 GIF（推荐批量出图）
python scripts/run_all_scenario_animations.py
python scripts/run_all_scenario_animations.py --frames   # 另存逐帧 PNG
python scripts/run_all_scenario_animations.py --list    # 预览哪些会出 GIF
```

场景类别：

| `--tag` | 内容 |
|---------|------|
| `lattice` | 纯 LatticePlanner：空旷、障碍、backup、初态变化等 |
| `decider` | PathBounds / Assessment / Combine / 巡航速度 |
| `on_lane` | OnLanePlanning.RunOnce 端到端，含 PathBounds 借道超车分支 |
| `overtake` | **超车**：PathBounds 借道 S 形（推荐）、Cartesian 参考、Lattice 跟停对照 |

**超车动画（推荐先看 `overtake_path_bounds_left`）**：

```bash
python scripts/run_lattice_scenario_cases.py overtake_path_bounds_left --animate
# 或全部超车场景
python scripts/run_overtake_animations.py
```

说明：纯 **Lattice** 在 blocking 前车场景只能跟停；超车走 **OnLane/PathBoundsDecider 借道 + PathAssessment + S 形 l(s)**，与 C++ lane-follow 任务链一致。

**`--animate`**：在 XY 地图上按 **0.1s** 一帧播放自车矩形 + 已走轨迹尾迹 + 障碍 + 参考线，默认输出 `scripts/output/animations/<场景名>.gif`。加 `--animate-frames` 可另存 `scripts/output/frames/<场景名>/frame_0000.png` …

```bash
# 一键跑完全部
python scripts/run_all_planner_checks.py

# 或分步
python scripts/run_lattice_component_checks.py
python scripts/run_extended_planner_checks.py
python scripts/run_minimal_lattice_plan.py
```

## 已覆盖（主脚本）

- 1D 轨迹外推、Backup、动态障碍 PathTimeGraph
- HDMap / PncMap / ReferenceLineProvider / 相对地图
- TrafficDecider（停止线、让行、KeepClear）
- PathDecider 静态 nudge、横向 lattice 后 PathDecider
- QP 参考线平滑、轨迹拼接
- PathAssessment / PathBounds / XY 距离 / Debug 记录（decider 模块单测）
- OnLanePlanning 输出
- 纯 Lattice 主路径（对齐 `lattice_planner.cc`）

## 已覆盖（扩展脚本）

- `CalculateLateralDerivative` / `GetFrenetPoint` 一致性
- `IsWithinPathDeciderScopeObstacle` 过滤逻辑
- 大横向 `BuildLatticeCandidatePath` 的 XY 回退
- 偏离参考线 / 碰撞路径无效性
- `ComparePathData` 自车道长度择优
- `PathBoundary.boundary()` API
- PlanningContext 借道 → 左/右边界候选
- 前方阻塞 + backup 仍可规划（LatticePlanner 核心）
- 默认纯 Lattice 路径（`check_lattice_default_plan`）
- PathDecider + PathAssessment 联调
- 空 discretized 时 `SetPathInfo` 安全跳过

## 仍值得后续补测（需更多场景或数据）

1. **多参考线变道**：`OnLanePlanning` + `FLAGS_cost_non_priority_reference_line` + 并行 lane
2. **PathDecider after lateral**：Lattice 全链路 + `FLAGS_enable_path_decider=True`（扩展测已覆盖 assessment 后单次 PathDecider）
3. **仅 boundary 候选、无 lattice 可行对**：PathBounds + PathAssessment + `CombinePathAndSpeedProfile`（扩展测 `boundary_only_path_assessment_combine`）
4. **阻塞且无 backup**：纯 Lattice 应失败（扩展测 `lattice_fails_when_blocked_without_backup`）；借道边界见 `path_bounds_lane_borrow_from_context`
5. **曲线路 / 大曲率参考线**：`SetFrenetPath` 与 `CombinePathAndSpeedProfile` 稳定性
6. **OnLanePlanning 末尾聚合**：`AggregateReferenceLineTrajectory`（扩展测 `on_lane_aggregate_path_speed`）
7. **pullover / open-space**：`PlanningContext` 其他 scenario 状态机
8. **交通规则全集**：REFERENCE_LINE_END、CREEP、REROUTING 等规则逐条单测

## 新增用例建议

在 `run_extended_planner_checks.py` 的 `CHECKS` 列表末尾追加 `(name, fn)`，并在 `planner_test_fixtures.py` 复用构造器，避免复制 `_build_reference_line`。
