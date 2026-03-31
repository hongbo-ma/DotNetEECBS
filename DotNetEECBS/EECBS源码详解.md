# EECBS 源码详解

## 目录
1. [概述](#1-概述)
2. [项目结构](#2-项目结构)
3. [核心概念](#3-核心概念)
4. [核心类详解](#4-核心类详解)
5. [算法流程](#5-算法流程)
6. [关键模块详解](#6-关键模块详解)
7. [使用方法](#7-使用方法)

---

## 1. 概述

**EECBS (Explicit Estimation Conflict-Based Search)** 是一个用于解决**多智能体路径规划 (MAPF)** 问题的有界次优算法。

### 1.1 什么是 MAPF 问题？

MAPF 问题的目标是为一组智能体找到一组无冲突的路径：
- 每个智能体从起点移动到终点
- 避免智能体之间的碰撞（同一时间不能在同一位置）
- 避免边冲突（两个智能体不能同时穿过同一条边）
- 目标：最小化总成本（通常是路径长度总和）

### 1.2 CBS 算法基础

CBS (Conflict-Based Search) 是一个两层搜索算法：
- **高层**：在约束树(CT)上搜索，处理冲突
- **底层**：为单个智能体在约束条件下寻找最优路径

### 1.3 EECBS 的改进

EECBS 在 CBS 基础上做了以下改进：
1. **高层使用 EES (Explicit Estimation Search)**：使用 f̂ 值（不可容许启发式）进行节点选择
2. **底层使用 Focal Search**：在次优范围内快速找到路径
3. **多种 CBS 优化技术**：
   - Bypass 策略
   - 冲突优先级排序
   - 高层启发式（WDG等）
   - 对称性推理（矩形、走廊、目标）

---

## 2. 项目结构

```
EECBS-main/
├── inc/                          # 头文件目录
│   ├── common.h                  # 通用定义和类型
│   ├── Instance.h                # 问题实例类
│   ├── CBS.h                     # CBS基类
│   ├── CBSNode.h                 # CBS节点类
│   ├── ECBS.h                    # ECBS类（继承CBS）
│   ├── ECBSNode.h                # ECBS节点类
│   ├── Conflict.h                # 冲突类
│   ├── ConstraintTable.h         # 约束表
│   ├── SingleAgentSolver.h       # 单智能体求解器基类
│   ├── SpaceTimeAStar.h          # 时空A*算法
│   ├── SIPP.h                    # SIPP算法
│   ├── MDD.h                     # MDD（多值决策图）
│   ├── CBSHeuristic.h            # CBS启发式
│   ├── RectangleReasoning.h      # 矩形推理
│   ├── CorridorReasoning.h       # 走廊推理
│   └── MutexReasoning.h          # 互斥推理
├── src/                          # 源文件目录
│   ├── driver.cpp                # 主程序入口
│   ├── CBS.cpp                   # CBS实现
│   ├── ECBS.cpp                  # ECBS实现
│   └── ...                       # 其他实现文件
├── random-32-32-20.map           # 示例地图
├── random-32-32-20-random-1.scen # 示例场景文件
└── CMakeLists.txt                # CMake配置
```

---

## 3. 核心概念

### 3.1 约束 (Constraint)

约束用五元组表示：
```cpp
typedef std::tuple<int, int, int, int, constraint_type> Constraint;
// <agent, loc, -1, t, VERTEX>           顶点约束：智能体不能在时间t位于位置loc
// <agent, from, to, t, EDGE>            边约束：智能体不能在时间t从from移动到to
// <agent, loc, -1, t, LEQLENGTH>        长度约束：路径长度不超过t
// <agent, loc, -1, t, GLENGTH>          长度约束：路径长度至少t+1
// <agent, loc, -1, t, POSITIVE_VERTEX>  正顶点约束：智能体必须在时间t位于loc
// <agent, from, to, t, POSITIVE_EDGE>   正边约束：智能体必须在时间t从from移动到to
```

### 3.2 冲突 (Conflict)

冲突类型定义在 `Conflict.h` 中：

```cpp
enum conflict_type { 
    MUTEX,      // 互斥冲突
    TARGET,     // 目标冲突
    CORRIDOR,   // 走廊冲突
    RECTANGLE,  // 矩形冲突
    STANDARD    // 标准冲突（顶点/边冲突）
};
```

冲突优先级：
```cpp
enum conflict_priority { 
    CARDINAL,       // 基数冲突（必须增加成本）
    PSEUDO_CARDINAL,// 伪基数冲突
    SEMI,           // 半基数冲突
    NON,            // 非基数冲突
    UNKNOWN         // 未知
};
```

### 3.3 高层节点 (HLNode)

高层节点是约束树的节点，包含：
- `constraints`: 新增的约束
- `g_val`: 当前成本
- `h_val`: 容许启发式值
- `cost_to_go`: 不可容许启发式值
- `distance_to_go`: 到目标的距离（冲突数）
- `conflicts`: 已分类的冲突列表
- `paths`: 当前各智能体的路径

### 3.4 MDD (Multi-Value Decision Diagram)

MDD 是一个有向无环图，表示在给定约束和成本限制下，智能体所有可能的路径。

```
level 0: [start]
level 1: [loc1, loc2]  
level 2: [loc3, loc4, loc5]
...
level n: [goal]
```

---

## 4. 核心类详解

### 4.1 Instance 类 (`Instance.h`)

**功能**：表示 MAPF 问题实例

```cpp
class Instance {
    vector<bool> my_map;           // 地图（障碍物标记）
    vector<int> start_locations;   // 各智能体起点
    vector<int> goal_locations;    // 各智能体终点
    int num_of_agents;             // 智能体数量
    
    // 关键方法
    list<int> getNeighbors(int curr);  // 获取邻居位置
    int getManhattanDistance(int loc1, int loc2);  // 曼哈顿距离
    bool isObstacle(int loc);          // 是否是障碍物
};
```

### 4.2 ConstraintTable 类 (`ConstraintTable.h`)

**功能**：存储和查询约束

```cpp
class ConstraintTable {
    CT ct;                    // 约束表：location -> time range
    CAT cat;                  // 冲突避免表
    map<int, size_t> landmarks; // 里程碑约束
    
    // 关键方法
    bool constrained(size_t loc, int t);        // 检查顶点约束
    bool constrained(size_t curr, size_t next, int t);  // 检查边约束
    void insert2CT(const HLNode& node, int agent);  // 从节点构建约束表
    void insert2CAT(int agent, const vector<Path*>& paths);  // 构建CAT
};
```

### 4.3 SingleAgentSolver 类 (`SingleAgentSolver.h`)

**功能**：单智能体路径规划求解器基类

```cpp
class SingleAgentSolver {
    int start_location;
    int goal_location;
    vector<int> my_heuristic;  // 预计算的启发式值
    
    // 纯虚函数
    virtual Path findOptimalPath(...) = 0;      // 找最优路径
    virtual pair<Path, int> findSuboptimalPath(...) = 0;  // 找次优路径
};
```

### 4.4 SpaceTimeAStar 类 (`SpaceTimeAStar.h`)

**功能**：时空 A* 算法实现

**底层搜索节点**：
```cpp
class LLNode {
    int location;          // 当前位置
    int g_val;             // 从起点到当前的代价
    int h_val;             // 启发式值
    int timestep;          // 时间步
    int num_of_conflicts;  // 冲突数量
    LLNode* parent;        // 父节点
};
```

**搜索过程**：
1. 使用 OPEN 列表（按 f 值排序）
2. 使用 FOCAL 列表（在次优范围内的节点，按冲突数排序）
3. 时间-空间扩展：在每个时间步考虑所有可能的移动

### 4.5 CBS 类 (`CBS.h` / `CBS.cpp`)

**功能**：CBS 算法的基类实现

**核心数据结构**：
```cpp
class CBS {
    // 三个列表用于 EES
    pairing_heap<CBSNode*, compare<...>> cleanup_list;  // 按 f 值排序
    pairing_heap<CBSNode*, compare<...>> open_list;     // 按 f̂ 值排序  
    pairing_heap<CBSNode*, compare<...>> focal_list;    // 按 d 值排序
    
    vector<Path*> paths;                // 当前路径
    vector<SingleAgentSolver*> search_engines;  // 底层求解器
    
    // 辅助模块
    MDDTable mdd_helper;
    RectangleReasoning rectangle_helper;
    CorridorReasoning corridor_helper;
    MutexReasoning mutex_helper;
    CBSHeuristic heuristic_helper;
};
```

**核心方法**：
```cpp
// 主求解函数
bool solve(double time_limit, int cost_lowerbound, int cost_upperbound);

// 生成根节点
bool generateRoot();

// 选择要展开的节点
CBSNode* selectNode();

// 生成子节点
bool generateChild(CBSNode* node, CBSNode* parent);

// 查找冲突
void findConflicts(HLNode& curr);

// 选择要处理的冲突
shared_ptr<Conflict> chooseConflict(const HLNode& node);
```

### 4.6 ECBS 类 (`ECBS.h` / `ECBS.cpp`)

**功能**：EECBS 的主要实现类

**额外数据成员**：
```cpp
class ECBS : public CBS {
    vector<int> min_f_vals;  // 每个智能体的最小 f 值（下界）
    // 使用 ECBSNode 而非 CBSNode
};
```

**ECBSNode 的关键区别**：
```cpp
class ECBSNode : public HLNode {
    int sum_of_costs;  // 路径成本总和（而非 g_val）
    list<pair<int, pair<Path, int>>> paths;  // 存储 <智能体ID, <路径, min_f>>
};
```

---

## 5. 算法流程

### 5.1 整体流程图

```
                    ┌─────────────────┐
                    │   开始 solve()  │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  generateRoot() │  生成根节点
                    │  为每个智能体   │
                    │  计算初始路径   │
                    └────────┬────────┘
                             │
              ┌──────────────▼──────────────┐
              │   while (!cleanup_list.empty())  │
              └──────────────┬──────────────┘
                             │
                    ┌────────▼────────┐
                    │   selectNode()  │  选择要展开的节点
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   terminate()?  │  检查终止条件
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
     无冲突   │                             │ 有冲突
              ▼                             ▼
        ┌──────────┐              ┌─────────────────┐
        │ 找到解!  │              │ classifyConflicts│
        └──────────┘              │  冲突分类        │
                                  └────────┬────────┘
                                           │
                                  ┌────────▼────────┐
                                  │ chooseConflict  │
                                  │  选择要处理的   │
                                  │  冲突           │
                                  └────────┬────────┘
                                           │
                                  ┌────────▼────────┐
                                  │ generateChild   │
                                  │ 生成两个子节点  │
                                  │ (添加不同约束)  │
                                  └────────┬────────┘
                                           │
                                  ┌────────▼────────┐
                                  │   pushNode      │
                                  │  将子节点加入   │
                                  │  各个列表       │
                                  └─────────────────┘
```

### 5.2 节点选择策略 (EES)

EECBS 使用三个列表进行节点选择：

```
1. cleanup_list: 按 f = g + h 排序（容许启发式）
2. open_list: 按 f̂ = g + cost_to_go 排序（不可容许启发式）
3. focal_list: 按 distance_to_go 排序（冲突数估计）

选择逻辑：
┌─────────────────────────────────────────────────────┐
│ if focal_list.top().cost <= w * cost_lowerbound:    │
│     选择 focal_list.top()  (最优 d)                 │
│ elif open_list.top().cost <= w * cost_lowerbound:   │
│     选择 open_list.top()   (最优 f̂)                │
│ else:                                               │
│     选择 cleanup_list.top() (最优 f)               │
└─────────────────────────────────────────────────────┘
```

### 5.3 底层搜索 (Focal Search)

```cpp
// SpaceTimeAStar::findSuboptimalPath

1. 初始化：
   - OPEN 列表：包含起点节点
   - FOCAL 列表：f 值在 w * min_f 范围内的节点

2. 搜索循环：
   while (!open_list.empty()) {
       // 更新 focal 列表
       if (open_list.top()->f_val > min_f_val) {
           min_f_val = open_list.top()->f_val;
           focal_list_threshold = w * min_f_val;
           // 将 f <= focal_list_threshold 的节点加入 focal
       }
       
       // 从 focal 列表选择节点（最少冲突）
       curr = focal_list.top();
       
       // 检查是否到达目标
       if (curr->location == goal && curr->满足约束) {
           return 重构路径;
       }
       
       // 扩展节点
       for (每个邻居 next) {
           if (!constrained(curr, next, time)) {
               创建新节点;
               加入 OPEN;
               if (f <= focal_threshold) 加入 FOCAL;
           }
       }
   }
```

### 5.4 冲突检测

```cpp
// CBS::findConflicts

对于每对智能体 (a1, a2):
    for t = 0 to min(path[a1].size, path[a2].size):
        // 顶点冲突
        if path[a1][t].location == path[a2][t].location:
            创建顶点冲突
        
        // 边冲突（跟随冲突）
        if t < min_length - 1:
            if path[a1][t] == path[a2][t+1] && 
               path[a2][t] == path[a1][t+1]:
                创建边冲突
    
    // 目标冲突（一个智能体到达终点后）
    if path[a1].size != path[a2].size:
        检查较早到达终点的智能体位置
        与较晚移动的智能体的碰撞
```

### 5.5 Bypass 策略

```cpp
// 在生成子节点后检查是否可以 bypass

if (bypass && child->g_val == curr->g_val && 
    child->distance_to_go < curr->distance_to_go):
    // 子节点成本相同但冲突更少
    // 可以用子节点的信息更新当前节点
    curr->conflicts = child->conflicts;
    curr->distance_to_go = child->distance_to_go;
    // 不需要展开这个节点，继续处理
    foundBypass = true;
```

---

## 6. 关键模块详解

### 6.1 MDD (Multi-Value Decision Diagram)

**位置**：`MDD.h` / `MDD.cpp`

**作用**：表示在给定成本限制下的所有可能路径

**结构**：
```cpp
class MDD {
    vector<list<MDDNode*>> levels;  // 每个时间步的节点集合
};

class MDDNode {
    int location;                   // 位置
    int level;                      // 时间层
    list<MDDNode*> children;        // 子节点
    list<MDDNode*> parents;         // 父节点
};
```

**构建过程**：
1. 从目标位置反向 BFS，标记可达节点
2. 从起点前向构建，只保留可达目标的路径

**用途**：
- 判断冲突是否为基数冲突
- 合并两个智能体的 MDD 判断依赖性
- 用于高层启发式计算

### 6.2 高层启发式

**位置**：`CBSHeuristic.h` / `CBSHeuristic.cpp`

**类型**：
```cpp
enum heuristics_type { 
    ZERO,   // h = 0
    CG,     // 冲突图
    DG,     // 依赖图
    WDG     // 加权依赖图
};
```

**WDG (Weighted Dependency Graph)**：
1. 对每对有冲突的智能体 (a1, a2)：
   - 构建合并的 MDD
   - 运行两人 CBS 子问题
   - h 值 = 子问题的最优成本增量

2. 构建 WDG 图，边权重为 h 值

3. 求解最小加权顶点覆盖 (MWVC)：
   ```cpp
   h_total = minimumWeightedVertexCover(WDG);
   ```

### 6.3 矩形推理 (Rectangle Reasoning)

**位置**：`RectangleReasoning.h` / `RectangleReasoning.cpp`

**原理**：当两个智能体在网格中相向移动形成矩形区域时，可以利用对称性推理避免展开多个节点。

```
智能体1: S1 ----→
              ↓ 冲突区域 (矩形)
智能体2: ←---- S2
```

**效果**：一次处理多个等价的冲突，减少搜索空间。

### 6.4 走廊推理 (Corridor Reasoning)

**位置**：`CorridorReasoning.h` / `CorridorReasoning.cpp`

**原理**：在狭窄走廊中，智能体无法互相穿过。可以通过添加范围约束一次性解决走廊中的所有冲突。

```
走廊: [ ] → [ ] → [X] → [ ] → [ ]
              冲突点
```

### 6.5 目标推理 (Target Reasoning)

**原理**：当一个智能体到达目标后停留在那里，其他智能体不能经过该位置。

**约束生成**：
```cpp
// LEQLENGTH: 该智能体路径长度不超过 t
// GLENGTH: 该智能体路径长度至少 t+1（必须等待）
```

---

## 7. 使用方法

### 7.1 编译

```bash
# 安装依赖 (Ubuntu)
sudo apt install libboost-all-dev

# 编译
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

### 7.2 运行

```bash
./eecbs -m random-32-32-20.map \
        -a random-32-32-20-random-1.scen \
        -o test.csv \
        --outputPaths=paths.txt \
        -k 50 \
        -t 60 \
        --suboptimality=1.2
```

### 7.3 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-m, --map` | 地图文件 | 必需 |
| `-a, --agents` | 场景文件 | 必需 |
| `-o, --output` | 输出统计文件 | 可选 |
| `--outputPaths` | 输出路径文件 | 可选 |
| `-k, --agentNum` | 智能体数量 | 0 (使用文件中全部) |
| `-t, --cutoffTime` | 时间限制(秒) | 7200 |
| `--suboptimality` | 次优因子 w | 1.2 |
| `--highLevelSolver` | 高层求解器 | EES |
| `--heuristics` | 启发式类型 | WDG |
| `--bypass` | 启用 bypass | true |
| `--rectangleReasoning` | 矩形推理 | true |
| `--corridorReasoning` | 走廊推理 | true |
| `--targetReasoning` | 目标推理 | true |
| `--sipp` | 使用 SIPP 底层 | false |

### 7.4 文件格式

**地图文件格式**：
```
type octile
height 32
width 32
map
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@..............................@
@..............................@
...
```

**场景文件格式**：
```
version 1
0   random-32-32-20.map  32  32   5   5   1   1   10
1   random-32-32-20.map  32  32  10  10   5   5   15
...
# 版本 地图 行数 列数 起点行 起点列 终点行 终点列 最优成本
```

---

## 8. 代码调用关系

```
main() [driver.cpp]
    │
    ├── Instance()                    // 加载问题实例
    │
    └── ECBS::solve()                 // 开始求解
            │
            ├── generateRoot()        // 生成根节点
            │       └── findSuboptimalPath() × N  // 为每个智能体找初始路径
            │
            └── while (running)
                    │
                    ├── selectNode()           // 选择节点 (EES)
                    │
                    ├── terminate()            // 检查终止
                    │
                    ├── classifyConflicts()    // 分类冲突
                    │       ├── corridor_reasoning
                    │       ├── rectangle_reasoning
                    │       └── computeConflictPriority (MDD)
                    │
                    ├── chooseConflict()       // 选择冲突
                    │
                    ├── generateChild() × 2    // 生成两个子节点
                    │       └── findSuboptimalPath()  // 重规划路径
                    │
                    └── pushNode()             // 加入列表
```

---

## 9. 性能统计

算法运行后会输出以下统计信息：

| 统计项 | 说明 |
|--------|------|
| `num_HL_expanded` | 高层展开节点数 |
| `num_HL_generated` | 高层生成节点数 |
| `num_LL_expanded` | 底层展开节点数 |
| `num_LL_generated` | 底层生成节点数 |
| `num_cardinal_conflicts` | 基数冲突数 |
| `num_rectangle_conflicts` | 矩形冲突数 |
| `num_corridor_conflicts` | 走廊冲突数 |
| `num_adopt_bypass` | bypass 采用次数 |
| `num_cleanup/open/focal` | 从各列表选择的次数 |

---

## 10. 参考文献

1. **EECBS 论文**：
   Jiaoyang Li, Wheeler Ruml and Sven Koenig. 
   "EECBS: Bounded-Suboptimal Search for Multi-Agent Path Finding."
   AAAI 2021.

2. **CBS 论文**：
   Sharon et al. "Conflict-Based Search For Optimal Multi-Agent Pathfinding."
   Artificial Intelligence, 2015.

3. **SIPPS**：
   Li et al. "MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search."
   AAAI 2022.

---

## 附录：关键代码片段

### A.1 节点比较器

```cpp
// ECBSNode::compare_node_by_f (用于 cleanup_list)
bool operator()(const ECBSNode* n1, const ECBSNode* n2) const {
    // 按 f = g + h 排序
    return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
}

// ECBSNode::compare_node_by_d (用于 focal_list)
bool operator()(const ECBSNode* n1, const ECBSNode* n2) const {
    // 按 distance_to_go 排序（冲突数估计）
    return n1->distance_to_go >= n2->distance_to_go;
}
```

### A.2 路径验证

```cpp
bool CBS::validateSolution() const {
    // 检查成本是否在次优范围内
    if (solution_cost > cost_lowerbound * suboptimality)
        return false;
    
    // 检查所有智能体对之间是否有冲突
    for (int a1 = 0; a1 < num_of_agents; a1++) {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
            // 检查顶点冲突和边冲突
            // ...
        }
    }
    return true;
}
```

---

*文档生成时间：2024*
*基于 EECBS 源码版本*
