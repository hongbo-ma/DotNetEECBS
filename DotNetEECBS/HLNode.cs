namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 高层搜索节点抽象基类（CBS 约束树节点）。
/// </summary>
public abstract class HLNode
{
    /// <summary>当前节点新增的约束列表</summary>
    public List<Constraint> Constraints = new();

    /// <summary>路径总代价（CBS 为路径长度之和，ECBS 为 min-f 之和）</summary>
    public int GVal;

    /// <summary>容许启发式值</summary>
    public int HVal;

    /// <summary>非容许启发式值（cost_to_go），用于 FOCAL 排序</summary>
    public int CostToGo;

    /// <summary>到目标状态的距离估计（冲突数），用于 FOCAL 排序</summary>
    public int DistanceToGo;

    /// <summary>在约束树中的深度</summary>
    public int Depth;

    /// <summary>所有智能体路径的最大时间步</summary>
    public int Makespan;

    /// <summary>是否已计算过启发式值</summary>
    public bool HComputed;

    public ulong TimeExpanded;
    public ulong TimeGenerated;

    /// <summary>已分类的冲突列表（每对智能体最多保留一个最高优先级冲突）</summary>
    public List<Conflict> Conflicts = new();

    /// <summary>尚未分类的冲突列表</summary>
    public List<Conflict> UnknownConflicts = new();

    /// <summary>本次展开选择的冲突</summary>
    public Conflict? ChosenConflict;

    /// <summary>在线学习：代价误差</summary>
    public int DistanceError;

    /// <summary>在线学习：距离误差</summary>
    public int CostError;

    /// <summary>是否已完全展开</summary>
    public bool FullyExpanded;

    public HLNode? Parent;
    public List<HLNode> Children = new();

    /// <summary>f 值 = g + h</summary>
    public int FVal => GVal + HVal;

    // -------------------------------------------------------------------------
    // 抽象方法
    // -------------------------------------------------------------------------

    /// <summary>非容许 f 帽值 = g + cost_to_go，用于 FOCAL/OPEN 排序</summary>
    public abstract int GetFHatVal();

    /// <summary>本节点新规划的路径数量</summary>
    public abstract int GetNumNewPaths();

    /// <summary>本节点重新规划了路径的智能体 ID 列表</summary>
    public abstract IList<int> GetReplannedAgents();

    public abstract string GetName();

    // -------------------------------------------------------------------------
    // 具体方法
    // -------------------------------------------------------------------------

    /// <summary>清空冲突列表（展开后释放内存）</summary>
    public void Clear()
    {
        Conflicts.Clear();
        UnknownConflicts.Clear();
    }

    /// <summary>
    /// 更新 DistanceToGo：
    /// = 已分类冲突数 + UnknownConflicts 中不重复的智能体对数。
    /// </summary>
    public void UpdateDistanceToGo()
    {
        var conflictingPairs = new HashSet<(int, int)>();
        foreach (var c in UnknownConflicts)
        {
            var pair = (Math.Min(c.A1, c.A2), Math.Max(c.A1, c.A2));
            conflictingPairs.Add(pair);
        }
        DistanceToGo = Conflicts.Count + conflictingPairs.Count;
    }

    /// <summary>打印从根到当前节点路径上属于 agent id 的所有约束（调试用）</summary>
    public void PrintConstraints(int id)
    {
        var curr = this;
        while (curr.Parent != null)
        {
            foreach (var c in curr.Constraints)
            {
                switch (c.Type)
                {
                    case ConstraintType.LeqLength:
                    case ConstraintType.PositiveVertex:
                    case ConstraintType.PositiveEdge:
                        Console.Write($"({c.Agent},{c.Loc1},{c.Loc2},{c.Timestep},{c.Type}), ");
                        break;
                    default:
                        if (c.Agent == id)
                            Console.Write($"({c.Agent},{c.Loc1},{c.Loc2},{c.Timestep},{c.Type}), ");
                        break;
                }
            }
            curr = curr.Parent;
        }
    }
}
