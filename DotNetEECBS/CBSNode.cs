namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// CBS 高层搜索节点。
/// 每个节点存储本次新规划的路径，以及在三个堆（CLEANUP/OPEN/FOCAL）中的排序逻辑。
/// </summary>
public class CBSNode : HLNode
{
    /// <summary>是否在 CLEANUP 列表中（用于 AStarEps FOCAL 更新遍历）</summary>
    public bool InOpenList;

    /// <summary>本节点新规划的路径列表：(智能体ID, 路径)</summary>
    public List<(int AgentId, Path Path)> Paths = new();

    // -------------------------------------------------------------------------
    // HLNode 抽象方法实现
    // -------------------------------------------------------------------------

    /// <summary>非容许 f 帽值 = g + cost_to_go</summary>
    public override int GetFHatVal() => GVal + CostToGo;

    /// <summary>本节点新规划的路径数量</summary>
    public override int GetNumNewPaths() => Paths.Count;

    public override string GetName() => "CBS Node";

    /// <summary>返回本节点重新规划了路径的智能体 ID 列表</summary>
    public override IList<int> GetReplannedAgents()
    {
        var result = new List<int>(Paths.Count);
        foreach (var (agentId, _) in Paths)
            result.Add(agentId);
        return result;
    }

    // -------------------------------------------------------------------------
    // 比较器（用于三个堆）
    // -------------------------------------------------------------------------

    /// <summary>
    /// CLEANUP 堆比较器：按 f 值升序。
    /// f 相同时依次按 distance_to_go、f_hat、h 值排序。
    /// </summary>
    public class CompareByF : IComparer<CBSNode>
    {
        public int Compare(CBSNode? x, CBSNode? y)
        {
            if (x is null || y is null) return 0;
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            int fhx = x.GVal + x.CostToGo, fhy = y.GVal + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            return x.HVal.CompareTo(y.HVal);
        }
    }

    /// <summary>
    /// FOCAL 堆比较器：按 distance_to_go 升序。
    /// 相同时依次按 f、f_hat、h 值排序。
    /// </summary>
    public class CompareByD : IComparer<CBSNode>
    {
        public int Compare(CBSNode? x, CBSNode? y)
        {
            if (x is null || y is null) return 0;
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            int fhx = x.GVal + x.CostToGo, fhy = y.GVal + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            return x.HVal.CompareTo(y.HVal);
        }
    }

    /// <summary>
    /// OPEN 堆比较器（EES 使用）：按非容许 f_hat 升序。
    /// 相同时依次按 f、distance_to_go、h 值排序。
    /// </summary>
    public class CompareByInadmissibleF : IComparer<CBSNode>
    {
        public int Compare(CBSNode? x, CBSNode? y)
        {
            if (x is null || y is null) return 0;
            int fhx = x.GVal + x.CostToGo, fhy = y.GVal + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            return x.HVal.CompareTo(y.HVal);
        }
    }
}
