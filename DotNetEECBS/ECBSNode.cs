namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// ECBS 高层搜索节点。
/// 与 CBSNode 的关键区别：g_val 存储 sum-of-min-f（下界之和），
/// sum_of_costs 存储实际路径代价之和，f_hat = sum_of_costs + cost_to_go。
/// </summary>
public class ECBSNode : HLNode
{
    /// <summary>实际路径代价之和（sum of costs）</summary>
    public int SumOfCosts;

    /// <summary>本节点新规划的路径：(智能体ID, 路径, 该智能体的 min-f 值)</summary>
    public List<(int AgentId, Path Path, int MinF)> Paths = new();

    // -------------------------------------------------------------------------
    // HLNode 抽象方法实现
    // -------------------------------------------------------------------------

    /// <summary>非容许 f 帽值 = sum_of_costs + cost_to_go</summary>
    public override int GetFHatVal() => SumOfCosts + CostToGo;

    public override int GetNumNewPaths() => Paths.Count;

    public override string GetName() => "ECBS Node";

    public override IList<int> GetReplannedAgents()
    {
        var result = new List<int>(Paths.Count);
        foreach (var (agentId, _, _) in Paths)
            result.Add(agentId);
        return result;
    }

    // -------------------------------------------------------------------------
    // 比较器
    // -------------------------------------------------------------------------

    /// <summary>
    /// CLEANUP 堆：按 f（g+h）升序。
    /// 相同时依次按 distance_to_go、f_hat（sum_of_costs+cost_to_go）、h 排序。
    /// </summary>
    public class CompareByF : IComparer<ECBSNode>
    {
        public int Compare(ECBSNode? x, ECBSNode? y)
        {
            if (x is null || y is null) return 0;
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            int fhx = x.SumOfCosts + x.CostToGo, fhy = y.SumOfCosts + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            return x.HVal.CompareTo(y.HVal);
        }
    }

    /// <summary>
    /// FOCAL 堆：按 distance_to_go 升序。
    /// 相同时依次按 f、f_hat、h 排序。
    /// </summary>
    public class CompareByD : IComparer<ECBSNode>
    {
        public int Compare(ECBSNode? x, ECBSNode? y)
        {
            if (x is null || y is null) return 0;
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            int fhx = x.SumOfCosts + x.CostToGo, fhy = y.SumOfCosts + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            return x.HVal.CompareTo(y.HVal);
        }
    }

    /// <summary>
    /// OPEN 堆（EES 使用）：按 f_hat（sum_of_costs+cost_to_go）升序。
    /// 相同时依次按 f、distance_to_go、h 排序。
    /// </summary>
    public class CompareByInadmissibleF : IComparer<ECBSNode>
    {
        public int Compare(ECBSNode? x, ECBSNode? y)
        {
            if (x is null || y is null) return 0;
            int fhx = x.SumOfCosts + x.CostToGo, fhy = y.SumOfCosts + y.CostToGo;
            if (fhx != fhy) return fhx.CompareTo(fhy);
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);
            if (x.DistanceToGo != y.DistanceToGo) return x.DistanceToGo.CompareTo(y.DistanceToGo);
            return x.HVal.CompareTo(y.HVal);
        }
    }
}
