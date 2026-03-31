namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 单智能体求解器抽象基类。
/// 提供启发式预计算（反向 Dijkstra）和邻居查询，
/// 具体搜索算法由子类实现。
/// </summary>
public abstract class SingleAgentSolver
{
    public ulong  NumExpanded;
    public ulong  NumGenerated;
    public double RuntimeBuildCT;
    public double RuntimeBuildCAT;

    public int    StartLocation { get; protected set; }
    public int    GoalLocation  { get; protected set; }
    public int    MinFVal       { get; protected set; }
    public double W             { get; set; } = 1.0;

    /// <summary>预计算的启发式值：MyHeuristic[loc] = loc 到终点的最短距离</summary>
    public List<int> MyHeuristic = new();
    protected readonly Instance Instance;

    protected SingleAgentSolver(Instance instance, int agent)
    {
        Instance      = instance;
        StartLocation = instance.StartLocations[agent];
        GoalLocation  = instance.GoalLocations[agent];
        ComputeHeuristics();
    }

    // -------------------------------------------------------------------------
    // 抽象方法（由子类实现）
    // -------------------------------------------------------------------------

    public abstract Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound);

    public abstract (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w);

    public abstract int    GetTravelTime(int start, int end, ConstraintTable ct, int upperBound);
    public abstract string GetName();

    // -------------------------------------------------------------------------
    // 公共方法
    // -------------------------------------------------------------------------

    /// <summary>返回 curr 的所有邻居位置（不含自身）</summary>
    public IList<int> GetNeighbors(int curr) => Instance.GetNeighbors(curr);

    /// <summary>返回 curr 的所有邻居位置加上自身（用于生成下一步候选，含原地等待）</summary>
    public IList<int> GetNextLocations(int curr)
    {
        var result = new List<int>(Instance.GetNeighbors(curr)) { curr };
        return result;
    }

    /// <summary>
    /// 计算 from 到 to 的容许启发式值：
    /// 取差分启发式（|h[from] - h[to]|）和曼哈顿距离的最大值。
    /// </summary>
    public int ComputeHeuristic(int from, int to)
    {
        int dh = Math.Abs(MyHeuristic[from] - MyHeuristic[to]);
        return Math.Max(dh, Instance.GetManhattanDistance(from, to));
    }

    /// <summary>
    /// 从终点出发做反向 Dijkstra，预计算所有位置到终点的最短距离。
    /// 结果存入 MyHeuristic，不可达位置值为 MaxTimestep。
    /// </summary>
    public void ComputeHeuristics()
    {
        MyHeuristic = new List<int>(new int[Instance.MapSize]);
        for (int i = 0; i < Instance.MapSize; i++)
            MyHeuristic[i] = Common.MaxTimestep;

        // 最小堆：(距离, 位置)
        var heap = new PriorityQueue<int, int>();
        MyHeuristic[GoalLocation] = 0;
        heap.Enqueue(GoalLocation, 0);

        while (heap.Count > 0)
        {
            heap.TryDequeue(out int loc, out int dist);
            // 惰性跳过过期条目
            if (dist > MyHeuristic[loc]) continue;

            foreach (int next in Instance.GetNeighbors(loc))
            {
                int newDist = dist + 1;
                if (newDist < MyHeuristic[next])
                {
                    MyHeuristic[next] = newDist;
                    heap.Enqueue(next, newDist);
                }
            }
        }
    }
}
