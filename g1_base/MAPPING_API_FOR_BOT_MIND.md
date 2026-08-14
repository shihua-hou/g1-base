# 建图 API（给 bot_mind 用）

本会话在 `g1_base` 侧实现了「建图过程中可观测、可流式拉取 2D 地图」的能力。
本文档面向 bot_mind 维护者，说明：bot_mind 应当如何对接、有哪些约定不可破坏。

`g1_base` 与 `bot_mind` 部署在同一台 PC2，**通过文件系统共享 manifest + ROS2 service 触发**通信，不引入额外 RPC。

---

## 1. 关键路径与文件命名约定

所有建图产物落到（PC2 上）：

```
/home/unitree/g1_maps/
  ├── current_map.json                        ← 「当前最新地图」指针（bot_mind 主要消费的就是它）
  ├── <base_name>_map.pcd                     ← 3D 点云（Super-LIO 原始 map.pcd 的复制改名件）
  ├── <base_name>_exhibit_2d_map.pgm          ← 2D 占据栅格图（Nav2 兼容）
  └── <base_name>_exhibit_2d_map.yaml         ← Nav2 map_server 配置
```

实际运行时目录由 `G1_MAPS_DIR` 决定；PC2 的 `robot_env.sh` 默认设为 `/home/unitree/g1_maps`，作为代码目录之外的运行时数据目录。bot_mind 读取时也会优先使用 `G1_MAPS_DIR`；未配置该变量时，会兼容历史目录 `/home/unitree/g1_base/config/maps` 和 `/home/unitree/g1_base/install/g1_base/share/g1_base/config/maps`。

- `<base_name>` 格式严格固定为 `YYYYMMDD_HHMMSS`（24 小时制本地时区）。
- 文件名后缀 `_exhibit_2d_map` **由服务器统一下发**，bot_mind 不要自行拼接，直接从 manifest 取。
- 仅保留最近 **100** 份三件套，超出的最旧会在下次 `start_mapping` 自动删除。

---

## 2. `current_map.json` Schema

bot_mind 暴露的 GET 接口、前端轮询逻辑、所有「最新地图是哪份」的判断都应**只读这一个文件**，不要 glob 目录。

### 2.1 字段定义

```json
{
  "base_name": "20260430_120000",
  "status": "mapping",
  "pgm": "20260430_120000_exhibit_2d_map.pgm",
  "yaml": "20260430_120000_exhibit_2d_map.yaml",
  "pcd": "20260430_120000_map.pcd",
  "started_at": "2026-04-30T12:00:00",
  "finished_at": null,
  "last_snapshot_at": "2026-04-30T12:01:20",
  "error": null
}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `base_name` | string | 当前/最近一轮建图的时间戳 ID |
| `status` | enum | `"mapping"` / `"ready"` / `"failed"`（见 §2.2 状态机） |
| `pgm` | string | 文件名（不含路径），与 `base_name` 一致 |
| `yaml` | string | 同上 |
| `pcd` | string | 同上 |
| `started_at` | ISO8601 string | 本轮 `start_mapping` 时刻 |
| `finished_at` | ISO8601 \| null | 终态时刻；`mapping` 中为 `null` |
| `last_snapshot_at` | ISO8601 \| null | 最近一次 20s 周期快照落盘时刻；从未刷新过则 `null` |
| `error` | string \| null | 仅 `status="failed"` 时填，给运维看的人类可读原因 |

### 2.2 状态机

```
              start_mapping                          stop_mapping (success)
   (无)  ──────────────────►  mapping  ────────────────────────────────────►  ready
                                  │
                                  │ stop_mapping (LIO map.pcd missing
                                  │   或 PCD→2D 转换异常)
                                  ▼
                                failed
```

**单向不回退**：一旦写入 `ready` / `failed`，本轮不会再回到 `mapping`。
下一次 `start_mapping` 才会写入新的 `base_name` + `status="mapping"`。

### 2.3 文件存在性保证

| status | `<base>_exhibit_2d_map.pgm` 存在？ | `<base>_map.pcd` 存在？ |
|---|---|---|
| `mapping` 且 `last_snapshot_at != null` | ✅ 是 | ❌ 否（PCD 仅在 stop 时落盘） |
| `mapping` 且 `last_snapshot_at == null` | ❌ 否（首张快照还没出） | ❌ 否 |
| `ready` | ✅ 是 | ✅ 是 |
| `failed` | 不保证 | 不保证 |

**bot_mind 在返回 PGM 前必须 stat 一次 `pgm` 文件，避免文件未生成或被中途清理**。

#### 「manifest 不存在」也是合法状态

机器人**从未跑过 `start_mapping`** 时，`current_map.json` 文件根本不存在。bot_mind 在 status 接口中把这种情况映射为 `map: null` 是合理的（如：

```json
{"mode":"navigation","state":"READY","map":null}
```

）。**只要跑过一次 `start_mapping`，manifest 文件就会持续存在**（即使后来变成 `failed`），此后 `map` 字段不会再回到 `null`。

### 2.4 原子性与并发

- manifest 写入用 tmp + `os.replace()` 原子替换，bot_mind 任意时刻读到的都是完整 JSON。
- PGM/YAML 也用 tmp + `os.replace()` 原子替换，**不会读到半张图**。
- bot_mind 侧建议读 manifest 时一次性 `read_text()` + `json.loads()`，不要多次 stat 比对。

---

## 3. ROS2 Service 接口

g1_base 暴露三个相关服务。bot_mind **应当**用 ROS2 service 触发流程切换；**不要**绕过 navigation_manager 直接动 Super-LIO 进程或地图文件。

| Service | Type | 作用 |
|---|---|---|
| `/navigation_manager/start_mapping` | `std_srvs/srv/Trigger` | 切到建图模式：停导航栈 → 启 Super-LIO → 启动周期快照 → 写 manifest `mapping` |
| `/navigation_manager/stop_mapping` | `std_srvs/srv/Trigger` | 停建图：停快照 → SIGINT Super-LIO（等 map.pcd 落盘）→ 转 2D → 写 manifest `ready` 或 `failed` |
| `/navigation_manager/generate_2d_map` | `std_srvs/srv/Trigger` | 单独触发一次 PCD→2D 转换（不依赖 start/stop 流程，用于已有 map.pcd 时手动重转） |

### 3.1 调用语义

- **Trigger.Response**：
  - `success: bool` — 该次操作是否被接受/成功
  - `message: string` — 人类可读结果或失败原因
- 三个服务都通过 `_with_operation` 串行化，**bot_mind 同时多次调用是安全的**（后到的会等前面跑完或被拒），但仍建议串行。
- `start_mapping` 在导航栈正在运行时会先把它停掉，**机器人会短暂失去导航能力**——bot_mind 在用户层面应有明确确认提示。

### 3.2 超时与失败语义

| 场景 | success | manifest 状态 |
|---|---|---|
| `start_mapping` 启动脚本失败 | false | failed (error 描述启动失败) |
| `start_mapping` LIO topic 60s 内没起来 | false | failed (mapping LIO did not start in time) |
| `stop_mapping` 时 `map.pcd` 不存在 | false | failed (map.pcd not found) |
| `stop_mapping` 时 PCD→2D 转换异常 | true（service 已返回成功）| failed（**异步**写入，bot_mind 需要 poll manifest 才能感知） |

**注意第 4 行**：`stop_mapping` service 立即返回 `success=true`，但最终 PGM 是后台线程生成的。bot_mind 不能仅凭 service response 就认为地图就绪，必须 poll `current_map.json.status == "ready"`。

---

## 4. bot_mind 标准消费模式

### 4.1 「拿当前最新地图」GET 接口实现

```python
def get_current_map():
    manifest_path = "/home/unitree/g1_maps/current_map.json"
    if not os.path.isfile(manifest_path):
        return 404, "no map yet"
    m = json.loads(open(manifest_path).read())
    pgm_path = os.path.join(os.path.dirname(manifest_path), m["pgm"])
    if not os.path.isfile(pgm_path):
        return 503, f"manifest says {m['status']} but pgm not on disk"
    return 200, {
        "status": m["status"],          # 让客户端知道是中间快照还是终版
        "base_name": m["base_name"],
        "pgm_bytes": open(pgm_path, "rb").read(),
        "yaml": open(pgm_path.replace(".pgm", ".yaml")).read(),
        "last_snapshot_at": m["last_snapshot_at"],
        "finished_at": m["finished_at"],
    }
```

### 4.2 建图轮询

前端实时展示「正在建什么样」时，建议：

- 轮询周期 **5–10s**（g1_base 侧每 20s 出一张快照，更高频纯浪费）
- 客户端保留 `last_seen_at = manifest.last_snapshot_at`，若不变则直接 304/不刷
- `status="ready"` 后客户端就可以停止轮询

### 4.3 触发建图全流程

```
bot_mind  →  ros2 service call /navigation_manager/start_mapping
   ↓ (success=true)
bot_mind  →  poll current_map.json
              status == "mapping" 即可开始展示快照
              last_snapshot_at 每 ~20s 更新一次
   ↓
用户遥控完成
   ↓
bot_mind  →  ros2 service call /navigation_manager/stop_mapping
   ↓ (success=true ≠ 终态完成)
bot_mind  →  继续 poll 直到 status ∈ {"ready", "failed"}
              (典型 PCD→2D 耗时 5–30 秒，取决于 PCD 大小)
```

---

## 5. 不要做的事

- **不要**自己 glob `G1_MAPS_DIR/*.pgm` 取最新 mtime——manifest 是唯一权威。
- **不要**写入 `G1_MAPS_DIR` 任何文件，除非是你新建图功能的产物（即将由 g1_base 管理）。
- **不要**在 `status="mapping"` 期间假定 `<base>_map.pcd` 存在；它**只在 `stop_mapping` 之后**才会出现。
- **不要**绕过 `/navigation_manager/start_mapping` 直接拉起 Super-LIO，会丢 manifest、丢命名时间戳、丢清理策略。
- **不要**修改 manifest schema 字段名以适配 bot_mind；如需新字段请反馈到 g1_base 侧加，避免单边改名导致解析破裂。

---

## 6. 当前实现位置（运维参考）

- 状态机入口：[`g1_base/navigation_manager.py`](g1_base/navigation_manager.py) `_start_mapping_impl` / `_stop_mapping_impl`
- Manifest 写入：`_write_map_manifest`（同上文件）
- 周期快照：[`g1_base/mapping_snapshotter.py`](g1_base/mapping_snapshotter.py) `MappingSnapshotter`
- PCD→2D 转换：[`g1_base/pcd_to_2d_map.py`](g1_base/pcd_to_2d_map.py) `convert_pcd_to_2d_map`
- 单测：[`tests/test_mapping_snapshotter.py`](tests/test_mapping_snapshotter.py), [`tests/test_pcd_to_2d_map.py`](tests/test_pcd_to_2d_map.py)
