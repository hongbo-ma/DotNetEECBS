namespace DotNetEECBS;
/// <summary>
///两个智能体之间发生冲突的类型，决定了生成约束的方式。
/// </summary>
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
/// <summary>
/// CBS 在选择要解决的冲突时，优先处理高优先级冲突，以减少搜索树的分支。
/// </summary>
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
/// <summary>
/// 高层求解器类型 CBS 高层搜索使用的节点展开策略，影响解的最优性保证和搜索效率
/// </summary>
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
/// <summary>
/// 高层启发式类型 CBS 高层节点的 h 值计算方式，用于估计当前节点到最优解还需增加多少代价。
/// </summary>
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
/// <summary>
/// 当一个 CBS 节点存在多个冲突时，选择哪个冲突进行分裂的策略。
/// </summary>
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
/// <summary>
/// 在 FOCAL 列表中选择下一个展开的 CBS 节点的策略（用于次优搜索）。
/// </summary>
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
public class Conflict
{
    public int              A1;
    public int              A2;
    public List<Constraint> Constraint1 = new();
    public List<Constraint> Constraint2 = new();
    public ConflictType     Type;
    public ConflictPriority Priority = ConflictPriority.Unknown;
    /// <summary>冲突选择时的次级排序依据</summary>
    public double           SecondaryPriority;

    /// <summary>返回冲突类型的整数 ID，用于排序和统计</summary>
    public int GetConflictId() => (int)Type;

    /// <summary>
    /// 设置顶点冲突：两个智能体在时刻 t 同时占据位置 v。
    /// 为每个智能体生成一条 VERTEX 负约束。
    /// </summary>
    public void SetVertexConflict(int a1, int a2, int v, int t)
    {
        Constraint1.Clear();
        Constraint2.Clear();
        A1 = a1;
        A2 = a2;
        Constraint1.Add(new Constraint(a1, v, -1, t, ConstraintType.Vertex));
        Constraint2.Add(new Constraint(a2, v, -1, t, ConstraintType.Vertex));
        Type = ConflictType.Standard;
    }

    /// <summary>
    /// 设置边冲突：两个智能体在时刻 t 交换位置（a1: v1→v2，a2: v2→v1）。
    /// 为每个智能体生成一条 EDGE 负约束，方向相反。
    /// </summary>
    public void SetEdgeConflict(int a1, int a2, int v1, int v2, int t)
    {
        Constraint1.Clear();
        Constraint2.Clear();
        A1 = a1;
        A2 = a2;
        Constraint1.Add(new Constraint(a1, v1, v2, t, ConstraintType.Edge));
        Constraint2.Add(new Constraint(a2, v2, v1, t, ConstraintType.Edge));
        Type = ConflictType.Standard;
    }

    /// <summary>
    /// 设置走廊冲突：两个智能体在走廊中相向而行。
    /// v1/t1 是 a1 的入口端点，v2/t2 是 a2 的入口端点，使用 RANGE 约束限制各自的通行时间段。
    /// </summary>
    public void SetCorridorConflict(int a1, int a2, int v1, int v2, int t1, int t2)
    {
        Constraint1.Clear();
        Constraint2.Clear();
        A1 = a1;
        A2 = a2;
        Constraint1.Add(new Constraint(a1, v1, 0, t1, ConstraintType.Range));
        Constraint2.Add(new Constraint(a2, v2, 0, t2, ConstraintType.Range));
        Type = ConflictType.Corridor;
    }

    /// <summary>
    /// 设置矩形冲突：直接使用外部计算好的屏障约束列表（BARRIER）。
    /// rs/rg 为矩形的起始和目标角点（仅记录，不参与约束生成）。
    /// </summary>
    public bool SetRectangleConflict(int a1, int a2, (int, int) rs, (int, int) rg, int rgT,
        IList<Constraint> c1, IList<Constraint> c2)
    {
        A1 = a1;
        A2 = a2;
        Constraint1 = new List<Constraint>(c1);
        Constraint2 = new List<Constraint>(c2);
        Type = ConflictType.Rectangle;
        return true;
    }

    /// <summary>
    /// 设置目标冲突：a1 已到达目标位置 v（时刻 t），a2 不能在 t 之后占用该位置。
    /// 为 a1 生成 LEQLENGTH 约束（路径长度 ≤ t），为 a1 生成 GLENGTH 约束（路径长度 ≥ t+1）作为备选分支。
    /// 注意：与 C++ 原版一致，两条约束的 agent 字段均为 a1。
    /// </summary>
    public void SetTargetConflict(int a1, int a2, int v, int t)
    {
        Constraint1.Clear();
        Constraint2.Clear();
        A1 = a1;
        A2 = a2;
        Constraint1.Add(new Constraint(a1, v, -1, t, ConstraintType.LeqLength));
        Constraint2.Add(new Constraint(a1, v, -1, t, ConstraintType.GLength));
        Type = ConflictType.Target;
    }

    /// <summary>
    /// 设置互斥冲突：由 MDD 互斥推理得出，两个智能体的状态集合互斥。
    /// 约束由互斥推理模块单独填充，此处仅设置类型和优先级。
    /// 互斥冲突优先级自动为 Cardinal。
    /// </summary>
    public void SetMutexConflict(int a1, int a2)
    {
        Constraint1.Clear();
        Constraint2.Clear();
        A1 = a1;
        A2 = a2;
        Type     = ConflictType.Mutex;
        Priority = ConflictPriority.Cardinal;
    }
}
/// <summary>
/// 一个 CBS 节点上的约束，用于限制智能体在特定时间步长上的移动。
/// </summary>
/// <param name="Agent">约束涉及的智能体 ID</param>
/// <param name="Loc1">约束涉及的第一个位置</param>
/// <param name="Loc2">约束涉及的第二个位置</param>
/// <param name="Timestep">约束生效的时间步长</param>
/// <param name="Type">约束的类型</param>
public record Constraint(int Agent, int Loc1, int Loc2, int Timestep, ConstraintType Type);

/// <summary>
/// 一个时间步长区间，用于表示智能体在该区间内发生冲突。
/// </summary>
/// <param name="TMin">时间步长区间最小值</param>
/// <param name="TMax">时间步长区间最大值</param>
/// <param name="HasCollision">是否发生冲突</param>
public record Interval(int TMin, int TMax, bool HasCollision);