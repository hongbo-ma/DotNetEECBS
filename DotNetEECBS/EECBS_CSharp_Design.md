# EECBS C# 版本类签名设计

> 基于 C++ 官方实现移植，保留原始算法结构

---

## 枚举类型

### ConflictType — 冲突类型

两个智能体之间发生冲突的类型，决定了生成约束的方式。

```csharp
public enum ConflictType
{
    /// <summary>互斥冲突：两个智能体在 MDD 层面互斥，无法同时存在于某些状态，优先级自动为 Cardinal</summary>
    Mutex,

    /// <summary>目标冲突：两个智能体竞争同一目标位置，一个智能体到达目标后另一个不能占用该位置</summary>
    Target,

    /// <summary>走廊冲突：两个智能体在狭窄走廊（度为2的节点序列）中相向而行，使用 RANGE 约束解决</summary>
    Corridor,

    /// <summary>矩形冲突：两个智能体在矩形区域内发生冲突，使用屏障约束（BARRIER）解决，可大幅剪枝</summary>
    Rectangle,

    /// <summary>标准冲突：普通的顶点冲突（同一时刻同一位置）或边冲突（同一时刻交换位置）</summary>
    Standard,
}
```

---

### ConflictPriority — 冲突优先级

CBS 在选择要解决的冲突时，优先处理高优先级冲突，以减少搜索树的分支。

```csharp
public enum ConflictPriority
{
    /// <summary>基数冲突（Cardinal）：解决该冲突必然导致总代价增加，两个智能体的最优路径长度都会增加，必须优先处理</summary>
    Cardinal,

    /// <summary>伪基数冲突（Pseudo-Cardinal）：依赖智能体之间的半/非基数冲突，优先级高于普通半/非基数冲突</summary>
    PseudoCardinal,

    /// <summary>半基数冲突（Semi-Cardinal）：只对其中一个智能体是基数的，解决后只有一方代价增加</summary>
    Semi,

    /// <summary>非基数冲突（Non-Cardinal）：对两个智能体都不是基数的，解决后两方代价均不增加</summary>
    Non,

    /// <summary>未知优先级：初始状态，尚未通过 MDD 分析计算出优先级</summary>
    Unknown,
}
```

---

### ConstraintType — 约束类型

CBS 节点中施加给智能体的约束形式，格式为 `(agent, loc1, loc2, timestep, type)`。

```csharp
public enum ConstraintType
{
    /// <summary>路径长度上限约束（≤ t）：智能体路径长度不能超过 t，用于目标冲突，防止一方过早到达目标</summary>
    LeqLength,

    /// <summary>路径长度下限约束（≥ t+1）：智能体路径长度至少为 t+1，用于目标冲突，强制一方延迟到达目标</summary>
    GLength,

    /// <summary>范围约束：智能体在时间段 [t1, t2] 内不能经过某位置，用于走廊冲突</summary>
    Range,

    /// <summary>屏障约束：沿矩形边界的一组顶点约束，用于矩形冲突推理，一次性施加多个位置限制</summary>
    Barrier,

    /// <summary>顶点约束（负）：智能体在时刻 t 不能出现在位置 loc，标准冲突的基本约束形式</summary>
    Vertex,

    /// <summary>边约束（负）：智能体在时刻 t 不能从 loc1 移动到 loc2，用于边冲突（交换位置）</summary>
    Edge,

    /// <summary>正顶点约束：智能体在时刻 t 必须出现在位置 loc，用于析取分裂（Disjoint Splitting）</summary>
    PositiveVertex,

    /// <summary>正边约束：智能体在时刻 t 必须从 loc1 移动到 loc2，用于析取分裂（Disjoint Splitting）</summary>
    PositiveEdge,
}
```

---

### HighLevelSolverType — 高层求解器类型

CBS 高层搜索使用的节点展开策略，影响解的最优性保证和搜索效率。

```csharp
public enum HighLevelSolverType
{
    /// <summary>标准 A* 搜索：保证最优解，使用 OPEN 列表按 f 值排序展开节点</summary>
    AStar,

    /// <summary>A*-ε 搜索：允许次优解（bounded suboptimal），在 FOCAL 列表中选择冲突最少的节点，CBS-D 使用此策略</summary>
    AStarEps,

    /// <summary>新型求解器：实验性高层搜索策略</summary>
    New,

    /// <summary>增强指数搜索（Enhanced Exponential Search）：ECBS 使用的三列表策略（CLEANUP / OPEN / FOCAL），在次优界内优先展开冲突少的节点</summary>
    EES,
}
```

---

### HeuristicsType — 高层启发式类型

CBS 高层节点的 h 值计算方式，用于估计当前节点到最优解还需增加多少代价。

```csharp
public enum HeuristicsType
{
    /// <summary>零启发式：h = 0，退化为 Dijkstra，不使用任何启发信息</summary>
    Zero,

    /// <summary>冲突图（Conflict Graph）：以冲突图的最小顶点覆盖（MVC）作为 h 值下界</summary>
    CG,

    /// <summary>依赖图（Dependency Graph）：在冲突图基础上，只保留存在依赖关系的智能体对，MVC 更精确</summary>
    DG,

    /// <summary>加权依赖图（Weighted Dependency Graph）：对依赖图中每条边赋予代价权重，MVC 权重之和作为 h 值，最精确但最慢</summary>
    WDG,

    /// <summary>全局启发式：使用全局信息计算的非容许启发式，用于 FOCAL 列表的次优搜索</summary>
    Global,

    /// <summary>路径启发式：基于路径信息的非容许启发式</summary>
    Path,

    /// <summary>本地启发式：基于局部冲突信息的非容许启发式</summary>
    Local,

    /// <summary>冲突数启发式：直接以当前节点的冲突数量作为非容许启发式，计算最快</summary>
    Conflict,
}
```

---

### ConflictSelection — 冲突选择策略

当一个 CBS 节点存在多个冲突时，选择哪个冲突进行分裂的策略。

```csharp
public enum ConflictSelection
{
    /// <summary>随机选择：从所有冲突中随机挑选一个</summary>
    Random,

    /// <summary>最早冲突：选择发生时间戳最小的冲突</summary>
    Earliest,

    /// <summary>冲突数最多：选择涉及冲突数量最多的智能体对</summary>
    Conflicts,

    /// <summary>约束数最多（M）：选择会产生最多约束的冲突</summary>
    MConstraints,

    /// <summary>约束数最少（F）：选择会产生最少约束的冲突</summary>
    FConstraints,

    /// <summary>宽度（Width）：基于冲突图宽度选择冲突</summary>
    Width,

    /// <summary>单例优先（Singletons）：优先选择只涉及一对智能体的冲突</summary>
    Singletons,
}
```

---

### NodeSelection — 节点选择策略

在 FOCAL 列表中选择下一个展开的 CBS 节点的策略（用于次优搜索）。

```csharp
public enum NodeSelection
{
    /// <summary>随机选择：从 FOCAL 列表中随机选取节点</summary>
    NodeRandom,

    /// <summary>最小 h 值：选择启发式值 h 最小的节点，倾向于接近最优解的节点</summary>
    NodeH,

    /// <summary>最小深度：选择搜索树深度最浅的节点，倾向于约束少的节点</summary>
    NodeDepth,

    /// <summary>最少冲突数：选择当前冲突总数最少的节点，倾向于接近无冲突解的节点</summary>
    NodeConflicts,

    /// <summary>最少冲突对数：选择冲突智能体对数量最少的节点</summary>
    NodeConflictPairs,

    /// <summary>最小顶点覆盖（MVC）：选择冲突图 MVC 值最小的节点，与 WDG 启发式配合使用效果最好</summary>
    NodeMVC,
}
```

---

## 核心数据结构

```csharp
public record Constraint(int Agent, int Loc1, int Loc2, int Timestep, ConstraintType Type);
public struct PathEntry { public int Location; public bool IsGoal; }
public class Path : List<PathEntry> { }
public record Interval(int TMin, int TMax, bool HasCollision);
public static class Constants
{
    public const int MaxTimestep = int.MaxValue / 2;
    public const int MaxCost    = int.MaxValue / 2;
}
```

---

## Instance

```csharp
public class Instance
{
    public int NumOfCols   { get; private set; }
    public int NumOfRows   { get; private set; }
    public int MapSize     { get; private set; }
    public int NumOfAgents { get; private set; }
    public IReadOnlyList<int> StartLocations { get; private set; }
    public IReadOnlyList<int> GoalLocations  { get; private set; }

    public Instance(string mapFile, string agentFile,
                    int numAgents = 0, int numRows = 0, int numCols = 0,
                    int numObstacles = 0, int warehouseWidth = 0);

    public bool IsObstacle(int loc);
    public bool ValidMove(int curr, int next);
    public IList<int> GetNeighbors(int curr);
    public int LinearizeCoordinate(int row, int col);
    public int GetRowCoordinate(int id);
    public int GetColCoordinate(int id);
    public (int Row, int Col) GetCoordinate(int id);
    public int GetManhattanDistance(int loc1, int loc2);
    public int GetDegree(int loc);
    public int GetDefaultNumberOfAgents();
}
```

---

## LLNode

```csharp
public class LLNode
{
    public int     Location;
    public int     GVal;
    public int     HVal;
    public LLNode? Parent;
    public int     Timestep;
    public int     NumOfConflicts;
    public bool    InOpenList;
    public bool    WaitAtGoal;
    public bool    IsGoal;
    public int     FVal => GVal + HVal;

    public LLNode(int location, int g, int h, LLNode? parent, int timestep,
                  int numConflicts = 0, bool waitAtGoal = false);
    public void Copy(LLNode other);

    public class CompareByF         : IComparer<LLNode> { }
    public class CompareByConflicts : IComparer<LLNode> { }
}
```

---

## SingleAgentSolver（抽象基类）

```csharp
public abstract class SingleAgentSolver
{
    public ulong  NumExpanded;
    public ulong  NumGenerated;
    public double RuntimeBuildCT;
    public double RuntimeBuildCAT;
    public int    StartLocation  { get; protected set; }
    public int    GoalLocation   { get; protected set; }
    public int    MinFVal        { get; protected set; }
    public double W              { get; set; }

    protected List<int>      MyHeuristic;
    protected readonly Instance Instance;

    protected SingleAgentSolver(Instance instance, int start, int goal);

    public abstract Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound);
    public abstract (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w);
    public abstract int    GetTravelTime(int start, int end, ConstraintTable ct, int upperBound);
    public abstract string GetName();

    public IList<int> GetNextLocations(int curr);
    public IList<int> GetNeighbors(int curr);
    public int        ComputeHeuristic(int from, int to);
    public void       ComputeHeuristics();
}
```

---

## ConstraintTable

```csharp
public class ConstraintTable
{
    public int LengthMin;
    public int LengthMax;
    public int CtMaxTimestep  { get; private set; }
    public int CatMaxTimestep { get; private set; }

    public ConstraintTable(int numCol, int mapSize);

    public int  GetHoldingTime(int location, int earliestTimestep);
    public int  GetMaxTimestep();
    public int  GetLastCollisionTimestep(int location);
    public bool Constrained(int loc, int t);
    public bool Constrained(int currLoc, int nextLoc, int nextT);
    public int  GetNumOfConflictsForStep(int currId, int nextId, int nextTimestep);
    public bool HasConflictForStep(int currId, int nextId, int nextTimestep);
    public bool HasEdgeConflict(int currId, int nextId, int nextTimestep);
    public int  GetFutureNumOfCollisions(int loc, int t);
    public void Insert2CT(HLNode node, int agent);
    public void Insert2CT(IList<Constraint> constraints, int agent);
    public void Insert2CT(Path path);
    public void Insert2CAT(int agent, IList<Path?> paths);
}
```

---

## Conflict

```csharp
public class Conflict
{
    public int              A1;
    public int              A2;
    public List<Constraint> Constraint1;
    public List<Constraint> Constraint2;
    public ConflictType     Type;
    public ConflictPriority Priority;
    public double           SecondaryPriority;

    public int  GetConflictId();
    public void SetVertexConflict(int a1, int a2, int v, int t);
    public void SetEdgeConflict(int a1, int a2, int v1, int v2, int t);
    public void SetCorridorConflict(int a1, int a2, int v1, int v2, int t1, int t2);
    public bool SetRectangleConflict(int a1, int a2, (int, int) rs, (int, int) rg, int rgT,
                                      IList<Constraint> c1, IList<Constraint> c2);
    public void SetTargetConflict(int a1, int a2, int v, int t);
    public void SetMutexConflict(int a1, int a2);
}
```

---

## HLNode（抽象基类）

```csharp
public abstract class HLNode
{
    public List<Constraint> Constraints;
    public int     GVal;
    public int     HVal;
    public int     CostToGo;
    public int     DistanceToGo;
    public int     Depth;
    public int     Makespan;
    public bool    HComputed;
    public ulong   TimeExpanded;
    public ulong   TimeGenerated;
    public List<Conflict> Conflicts;
    public List<Conflict> UnknownConflicts;
    public Conflict?      ChosenConflict;
    public int     DistanceError;
    public int     CostError;
    public bool    FullyExpanded;
    public HLNode? Parent;
    public List<HLNode> Children;
    public int FVal => GVal + HVal;

    public abstract int          GetFHatVal();
    public abstract int          GetNumNewPaths();
    public abstract IList<int>   GetReplannedAgents();
    public abstract string       GetName();

    public void Clear();
    public void UpdateDistanceToGo();
    public void PrintConstraints(int id);
}
```

---

## CBSNode

```csharp
public class CBSNode : HLNode
{
    public List<(int AgentId, Path Path)> Paths;

    public override int        GetFHatVal();
    public override int        GetNumNewPaths();
    public override string     GetName();
    public override IList<int> GetReplannedAgents();

    public class CompareByF             : IComparer<CBSNode> { }
    public class CompareByD             : IComparer<CBSNode> { }
    public class CompareByInadmissibleF : IComparer<CBSNode> { }
}
```

---

## ECBSNode

```csharp
public class ECBSNode : HLNode
{
    public int SumOfCosts;
    public List<(int AgentId, Path Path, int MinF)> Paths;

    public override int        GetFHatVal();
    public override int        GetNumNewPaths();
    public override string     GetName();
    public override IList<int> GetReplannedAgents();

    public class CompareByF             : IComparer<ECBSNode> { }
    public class CompareByD             : IComparer<ECBSNode> { }
    public class CompareByInadmissibleF : IComparer<ECBSNode> { }
}
```

---

## MDDNode / MDD / SyncMDD / MDDTable

```csharp
public class MDDNode
{
    public int Location;
    public int Level;
    public int Cost;
    public List<MDDNode> Children;
    public List<MDDNode> Parents;

    public MDDNode(int location, int level);
}

public class MDD
{
    public List<List<MDDNode>> Levels;

    public MDD(SingleAgentSolver solver);
    public bool     BuildMDD(ConstraintTable ct, int numLevels, SingleAgentSolver solver);
    public MDDNode? Find(int location, int level);
    public void     DeleteNode(MDDNode node);
    public void     Clear();
    public void     IncreaseBy(ConstraintTable ct, int dLevel, SingleAgentSolver solver);
    public MDDNode? GoalAt(int level);
}

public class SyncMDDNode
{
    public int Location;
    public List<SyncMDDNode> Children;
    public List<SyncMDDNode> Parents;
    public List<MDDNode>     CoexistingNodesFromOtherMdds;
}

public class SyncMDD
{
    public List<List<SyncMDDNode>> Levels;

    public SyncMDDNode? Find(int location, int level);
    public void         DeleteNode(SyncMDDNode node, int level);
    public void         Clear();
}

public class MDDTable
{
    public double AccumulatedRuntime;
    public ulong  NumReleasedMdds;
    public int    MaxNumOfMdds;

    public MDDTable(int numAgents);
    public MDD? FindMDD(HLNode node, int agent);
    public MDD  GetMDD(HLNode node, int agent, int mddLevels = -1);
    public void Clear();
}
```

---

## ReservationTable

```csharp
public class ReservationTable
{
    public ReservationTable(ConstraintTable ct, int goalLocation);

    public IList<(int TMin, int TMax, int NumCollisions, bool IsGoal, bool IsFirst)>
        GetSafeIntervals(int from, int to, int lowerBound, int upperBound);
    public Interval GetFirstSafeInterval(int location);
    public bool     FindSafeInterval(ref Interval interval, int location, int tMin);
    public void     UpdateSIT(int location);
}
```

---

## SIPPNode / SIPP

```csharp
public class SIPPNode : LLNode
{
    public int  HighGeneration;
    public int  HighExpansion;
    public bool CollisionV;

    public SIPPNode(int location, int g, int h, LLNode? parent, int timestep,
                    int highGen, int highExp, bool collisionV, int numConflicts = 0);
}

public class SIPP : SingleAgentSolver
{
    public SIPP(Instance instance, int start, int goal);

    public override Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound);
    public override (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w);
    public override int    GetTravelTime(int start, int end, ConstraintTable ct, int upperBound);
    public override string GetName();
}
```

---

## AStarNode / SpaceTimeAStar

```csharp
public class AStarNode : LLNode
{
    public AStarNode(int location, int g, int h, LLNode? parent, int timestep,
                     int numConflicts = 0, bool waitAtGoal = false);
}

public class SpaceTimeAStar : SingleAgentSolver
{
    public SpaceTimeAStar(Instance instance, int start, int goal);

    public override Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound);
    public override (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w);
    public override int    GetTravelTime(int start, int end, ConstraintTable ct, int upperBound);
    public override string GetName();
}
```

---

## ConstraintPropagation

```csharp
public class ConstraintPropagation
{
    public ConstraintPropagation(MDD mdd0, MDD mdd1);

    public void InitMutex();
    public void FwdMutexProp();
    public void BwdMutexProp();
    public bool HasMutex(MDDNode a, MDDNode b);
    public bool HasFwdMutex(MDDNode a, MDDNode b);
    public bool Mutexed(int level0, int level1);
    public bool Feasible(int level0, int level1);
    public bool SemiCardinal(int level, int loc);
    public (List<Constraint> C1, List<Constraint> C2) GenerateConstraints(int a1, int a2);
}
```

---

## RectangleReasoning

```csharp
public class RectangleReasoning
{
    public double AccumulatedRuntime;

    public RectangleReasoning(Instance instance);

    public Conflict? Run(IList<Path?> paths, int timestep, int a1, int a2, MDD mdd1, MDD mdd2);
    public Conflict? FindRectangleConflictByRM(IList<Path?> paths, int timestep,
                                               int a1, int a2, MDD mdd1, MDD mdd2);
    public Conflict? FindRectangleConflictByGR(IList<Path?> paths, int timestep,
                                               int a1, int a2, MDD mdd1, MDD mdd2);
}
```

---

## CorridorReasoning

```csharp
public class CorridorReasoning
{
    public double AccumulatedRuntime;

    public CorridorReasoning(IList<SingleAgentSolver> searchEngines,
                              IList<ConstraintTable> initialConstraints);

    public Conflict? Run(Conflict conflict, IList<Path?> paths, HLNode node);
    public Conflict? FindCorridorConflict(Conflict conflict, IList<Path?> paths, HLNode node);
}
```

---

## MutexReasoning

```csharp
public class MutexReasoning
{
    public double AccumulatedRuntime;

    public MutexReasoning(Instance instance, IList<ConstraintTable> initialConstraints);

    public Conflict? Run(int a1, int a2, CBSNode node, MDD mdd1, MDD mdd2);
}
```

---

## ConstraintsHasher

```csharp
public struct ConstraintsHasher
{
    public int    Agent;
    public HLNode Node;

    public class EqNode : IEqualityComparer<ConstraintsHasher> { }
    public class Hasher : IEqualityComparer<ConstraintsHasher> { }
}
```

---

## HTableEntry / CBSHeuristic

```csharp
public struct HTableEntry
{
    public int    A1;
    public int    A2;
    public HLNode Node;

    public class EqNode : IEqualityComparer<HTableEntry> { }
    public class Hasher : IEqualityComparer<HTableEntry> { }
}

public class CBSHeuristic
{
    public HeuristicsType   Type;
    public bool             RectangleReasoning;
    public bool             CorridorReasoning;
    public bool             TargetReasoning;
    public bool             MutexReasoning;
    public bool             DisjointSplitting;
    public bool             PC;
    public bool             SaveStats;
    public double           RuntimeBuildDependencyGraph;
    public double           RuntimeSolveMVC;
    public ulong            NumSolveMVC;
    public ulong            NumMergeMDDs;
    public ulong            NumSolve2AgentProblems;
    public ulong            NumMemoization;

    public CBSHeuristic(int numAgents, Instance instance,
                         IList<SingleAgentSolver> solvers,
                         IList<Path?> paths,
                         MDDTable mddHelper);

    public void   Init();
    public void   SetInadmissibleHeuristics(HeuristicsType h);
    public bool   ComputeInformedHeuristics(CBSNode curr, double timeLimit);
    public bool   ComputeInformedHeuristics(ECBSNode curr, IList<int> minFVals, double timeLimit);
    public void   ComputeQuickHeuristics(HLNode curr);
    public void   UpdateOnlineHeuristicErrors(CBSNode curr);
    public void   UpdateOnlineHeuristicErrors(ECBSNode curr);
    public void   UpdateInadmissibleHeuristics(HLNode curr);
    public double GetCostError(int i = 0);
    public double GetDistanceError(int i = 0);
    public void   Clear();
}
```

---

## CBS

```csharp
public class CBS
{
    // 统计
    public double Runtime, RuntimeGenerateChild, RuntimeBuildCT, RuntimeBuildCAT;
    public double RuntimePathFinding, RuntimeDetectConflicts, RuntimePreprocessing;
    public ulong  NumCardinalConflicts, NumCorridorConflicts, NumRectangleConflicts;
    public ulong  NumTargetConflicts, NumMutexConflicts, NumStandardConflicts;
    public ulong  NumHLExpanded, NumHLGenerated, NumLLExpanded, NumLLGenerated;

    // 配置
    public bool                RandomRoot, RectangleReasoning, CorridorReasoning;
    public bool                TargetReasoning, DisjointSplitting, MutexReasoning;
    public bool                Bypass, PC, SaveStats;
    public HighLevelSolverType SolverType;
    public ConflictSelection   ConflictSelectionRule;
    public NodeSelection       NodeSelectionRule;

    protected MDDTable           MddHelper;
    protected RectangleReasoning RectangleHelper;
    protected CorridorReasoning  CorridorHelper;
    protected MutexReasoning     MutexHelper;
    protected CBSHeuristic       HeuristicHelper;

    protected HLNode? DummyStart;
    protected HLNode? GoalNode;
    public    bool    SolutionFound;
    public    int     SolutionCost;

    protected readonly Instance        Instance;
    protected IList<Path?>             Paths;
    protected IList<SingleAgentSolver> SearchEngines;
    protected IList<ConstraintTable>   InitialConstraints;

    public CBS(Instance instance, bool sipp, int screen);

    public virtual bool Solve(double timeLimit, int costLowerBound = 0,
                               int costUpperBound = int.MaxValue);
    public int  GetLowerBound();
    public void SaveResults(string fileName, string instanceName);
    public void SaveStats(string fileName, string instanceName);
    public void SaveCT(string fileName);
    public void SavePaths(string fileName);
    public virtual void Clear();

    protected bool FindPathForSingleAgent(HLNode node, int ag, int lowerBound = 0);
    protected void ClassifyConflicts(HLNode node);
    protected bool GenerateChild(HLNode child, HLNode curr);
    protected bool GenerateRoot();
    protected bool IsTerminal(HLNode curr);
    protected void ChooseConflict(HLNode node);
}
```

---

## ECBS

```csharp
public class ECBS : CBS
{
    private List<int>                   MinFVals;
    private List<(Path Path, int MinF)> PathsFoundInitially;

    public ECBS(Instance instance, bool sipp, int screen);

    public override bool Solve(double timeLimit, int costLowerBound = 0);
    public override void Clear();
}
```
