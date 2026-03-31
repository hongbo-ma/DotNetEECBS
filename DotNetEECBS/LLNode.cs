namespace DotNetEECBS;

/// <summary>
/// 低层搜索节点（Low-Level Node），用于单智能体路径规划（A* / SIPP）。
/// </summary>
public class LLNode
{
    /// <summary>当前所在位置的线性化索引</summary>
    public int     Location;

    /// <summary>从起点到当前节点的实际代价</summary>
    public int     GVal;

    /// <summary>从当前节点到终点的启发式估计代价</summary>
    public int     HVal;

    /// <summary>父节点，用于回溯路径</summary>
    public LLNode? Parent;

    /// <summary>当前节点对应的时间步</summary>
    public int     Timestep;

    /// <summary>当前路径与其他智能体路径的冲突数量，用于 FOCAL 列表排序</summary>
    public int     NumOfConflicts;

    /// <summary>该节点是否在 OPEN 列表中</summary>
    public bool    InOpenList;

    /// <summary>当前动作是否为在目标位置等待，用于处理路径长度上限约束（LeqLength）</summary>
    public bool    WaitAtGoal;

    /// <summary>该节点是否为目标节点</summary>
    public bool    IsGoal;

    /// <summary>f 值 = g 值 + h 值</summary>
    public int FVal => GVal + HVal;

    /// <summary>默认构造函数，所有字段初始化为零/false/null</summary>
    public LLNode()
    {
        Location       = 0;
        GVal           = 0;
        HVal           = 0;
        Parent         = null;
        Timestep       = 0;
        NumOfConflicts = 0;
        InOpenList     = false;
        WaitAtGoal     = false;
        IsGoal         = false;
    }

    /// <summary>带参数的构造函数</summary>
    /// <param name="location">位置线性化索引</param>
    /// <param name="g">g 值</param>
    /// <param name="h">h 值</param>
    /// <param name="parent">父节点</param>
    /// <param name="timestep">时间步</param>
    /// <param name="numConflicts">冲突数量</param>
    /// <param name="inOpenList">是否在 OPEN 列表中</param>
    public LLNode(int location, int g, int h, LLNode? parent, int timestep,
                  int numConflicts = 0, bool inOpenList = false)
    {
        Location       = location;
        GVal           = g;
        HVal           = h;
        Parent         = parent;
        Timestep       = timestep;
        NumOfConflicts = numConflicts;
        InOpenList     = inOpenList;
        WaitAtGoal     = false;
        IsGoal         = false;
    }

    /// <summary>将另一个节点的所有字段复制到当前节点（浅拷贝 Parent 引用）</summary>
    public void Copy(LLNode other)
    {
        Location       = other.Location;
        GVal           = other.GVal;
        HVal           = other.HVal;
        Parent         = other.Parent;
        Timestep       = other.Timestep;
        NumOfConflicts = other.NumOfConflicts;
        WaitAtGoal     = other.WaitAtGoal;
        IsGoal         = other.IsGoal;
    }

    /// <summary>
    /// OPEN 列表比较器：按 f 值升序排列（最小堆）。
    /// f 值相同时优先选 h 值更小的节点（更接近目标）；
    /// h 值也相同时随机打破平局。
    /// </summary>
    public class CompareByF : IComparer<LLNode>
    {
        private static readonly Random Rng = new();

        /// <returns>
        /// 负数表示 x 优先于 y（x 的 f 值更小，或 f 相同时 h 更小）
        /// </returns>
        public int Compare(LLNode? x, LLNode? y)
        {
            if (x is null || y is null) return 0;

            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);

            // f 值相同：h 值更小的优先
            if (x.HVal != y.HVal) return x.HVal.CompareTo(y.HVal);

            // h 值也相同：随机打破平局
            return Rng.Next(2) == 0 ? -1 : 1;
        }
    }

    /// <summary>
    /// FOCAL 列表比较器：按冲突数升序排列（最小堆）。
    /// 冲突数相同时退化为 CompareByF 的逻辑（f 值 → h 值 → 随机）。
    /// </summary>
    public class CompareByConflicts : IComparer<LLNode>
    {
        private static readonly Random Rng = new();

        /// <returns>
        /// 负数表示 x 优先于 y（x 的冲突数更少）
        /// </returns>
        public int Compare(LLNode? x, LLNode? y)
        {
            if (x is null || y is null) return 0;

            // 首先按冲突数排序
            if (x.NumOfConflicts != y.NumOfConflicts)
                return x.NumOfConflicts.CompareTo(y.NumOfConflicts);

            // 冲突数相同：按 f 值排序
            int fx = x.FVal, fy = y.FVal;
            if (fx != fy) return fx.CompareTo(fy);

            // f 值相同：按 h 值排序
            if (x.HVal != y.HVal) return x.HVal.CompareTo(y.HVal);

            // 全部相同：随机打破平局
            return Rng.Next(2) == 0 ? -1 : 1;
        }
    }
}
