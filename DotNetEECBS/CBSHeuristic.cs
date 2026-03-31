namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// CBS 高层启发式计算器，支持 CG / DG / WDG 三种容许启发式。
/// CG：基数冲突图的最小顶点覆盖（MVC）。
/// DG：依赖图的 MVC（通过 MDD 合并判断依赖关系）。
/// WDG：加权依赖图的最小加权顶点覆盖（MWVC），边权 = 双智能体子问题的最优代价增量。
/// </summary>
public class CBSHeuristic
{
    // -------------------------------------------------------------------------
    // 公开配置
    // -------------------------------------------------------------------------
    public HeuristicsType  Type                       = HeuristicsType.Zero;
    public bool            RectangleReasoning;
    public bool            CorridorReasoning;
    public bool            TargetReasoning;
    public bool            MutexReasoning;
    public bool            DisjointSplitting;
    public bool            PC;
    public bool            SaveStats;
    public ConflictSelection ConflictSelectionRule    = ConflictSelection.Earliest;
    public NodeSelection     NodeSelectionRule        = NodeSelection.NodeH;

    // -------------------------------------------------------------------------
    // 统计
    // -------------------------------------------------------------------------
    public double RuntimeBuildDependencyGraph;
    public double RuntimeSolveMVC;
    public ulong  NumSolveMVC;
    public ulong  NumMergeMDDs;
    public ulong  NumSolve2AgentProblems;
    public ulong  NumMemoization;

    // -------------------------------------------------------------------------
    // 私有字段
    // -------------------------------------------------------------------------
    private readonly int                    _numAgents;
    private readonly IList<Path?>           _paths;
    private readonly IList<SingleAgentSolver> _searchEngines;
    private readonly IList<ConstraintTable> _initialConstraints;
    private readonly MDDTable               _mddHelper;

    // DG/WDG 查找表：lookupTable[a1][a2] -> (h, a1_f, a2_f)
    private List<List<Dictionary<HTableEntry, (int H, int F1, int F2)>>> _lookupTable = new();

    private const int NodeLimit = 4;   // 子 CBS 节点数上限
    private double    _timeLimit;
    private System.Diagnostics.Stopwatch _sw = new();

    // -------------------------------------------------------------------------
    // 构造函数
    // -------------------------------------------------------------------------

    public CBSHeuristic(int numAgents, IList<Path?> paths,
                         IList<SingleAgentSolver> searchEngines,
                         IList<ConstraintTable> initialConstraints,
                         MDDTable mddHelper)
    {
        _numAgents          = numAgents;
        _paths              = paths;
        _searchEngines      = searchEngines;
        _initialConstraints = initialConstraints;
        _mddHelper          = mddHelper;
    }

    public void Init()
    {
        if (Type == HeuristicsType.DG || Type == HeuristicsType.WDG)
        {
            _lookupTable = new List<List<Dictionary<HTableEntry, (int, int, int)>>>();
            for (int i = 0; i < _numAgents; i++)
            {
                var row = new List<Dictionary<HTableEntry, (int, int, int)>>();
                for (int j = 0; j < _numAgents; j++)
                    row.Add(new Dictionary<HTableEntry, (int, int, int)>(new HTableEntry.EqNode()));
                _lookupTable.Add(row);
            }
        }
    }

    public void Clear() => _lookupTable.Clear();

    // -------------------------------------------------------------------------
    // 快速启发式（pathmax + distance_to_go）
    // -------------------------------------------------------------------------

    /// <summary>
    /// 快速启发式：用父节点的 h 值做 pathmax，更新 distance_to_go。
    /// 在每次生成子节点后立即调用，不需要 MDD。
    /// </summary>
    public void ComputeQuickHeuristics(HLNode node)
    {
        if (node.Parent != null)
            node.HVal = Math.Max(0, node.Parent.GVal + node.Parent.HVal - node.GVal);
        node.UpdateDistanceToGo();
    }

    // -------------------------------------------------------------------------
    // 容许启发式（CG / DG / WDG）
    // -------------------------------------------------------------------------

    /// <summary>为 CBS 节点计算容许 h 值（CG/DG/WDG）</summary>
    public bool ComputeInformedHeuristics(CBSNode curr, double timeLimit)
    {
        curr.HComputed = true;
        _sw = System.Diagnostics.Stopwatch.StartNew();
        _timeLimit = timeLimit;

        var hg = new int[_numAgents * _numAgents];
        int numEdges = 0;
        int h = -1;

        switch (Type)
        {
            case HeuristicsType.Zero:
                h = 0;
                break;
            case HeuristicsType.CG:
                BuildCardinalConflictGraph(curr, hg, ref numEdges);
                h = MinimumVertexCover(hg);
                break;
            case HeuristicsType.DG:
                if (!BuildDependenceGraph(curr, hg, ref numEdges)) return false;
                h = MinimumVertexCover(hg);
                break;
            case HeuristicsType.WDG:
                if (!BuildWeightedDependencyGraph(curr, hg)) return false;
                h = MinimumWeightedVertexCover(hg);
                break;
            default:
                h = 0;
                break;
        }

        if (h < 0) return false;
        curr.HVal = Math.Max(h, curr.HVal);
        return true;
    }

    /// <summary>为 ECBS 节点计算容许 h 值（WDG）</summary>
    public bool ComputeInformedHeuristics(ECBSNode curr, IList<int> minFVals, double timeLimit)
    {
        curr.HComputed = true;
        _sw = System.Diagnostics.Stopwatch.StartNew();
        _timeLimit = timeLimit;

        if (Type == HeuristicsType.WDG)
        {
            var hg = new int[_numAgents * _numAgents];
            int deltaG = 0;
            if (!BuildWeightedDependencyGraphECBS(curr, minFVals, hg, ref deltaG)) return false;
            int h = MinimumWeightedVertexCover(hg) + deltaG;
            if (h < 0) return false;
            curr.HVal = Math.Max(h, curr.HVal);
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // 在线启发式误差更新（用于非容许启发式，暂时空实现）
    // -------------------------------------------------------------------------

    public void UpdateOnlineHeuristicErrors(CBSNode curr) { }
    public void UpdateOnlineHeuristicErrors(ECBSNode curr) { }
    public void UpdateInadmissibleHeuristics(HLNode curr) { }
    public double GetCostError(int i = 0) => 0;
    public double GetDistanceError(int i = 0) => 0;

    // -------------------------------------------------------------------------
    // 图构建：CG（基数冲突图）
    // -------------------------------------------------------------------------

    private void BuildCardinalConflictGraph(CBSNode node, int[] cg, ref int numEdges)
    {
        numEdges = 0;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        foreach (var c in node.Conflicts)
        {
            if (c.Priority != ConflictPriority.Cardinal) continue;
            int a1 = c.A1, a2 = c.A2;
            if (cg[a1 * _numAgents + a2] == 0)
            {
                cg[a1 * _numAgents + a2] = 1;
                cg[a2 * _numAgents + a1] = 1;
                numEdges++;
            }
        }
        RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds;
    }

    // -------------------------------------------------------------------------
    // 图构建：DG（依赖图）
    // -------------------------------------------------------------------------

    private bool BuildDependenceGraph(CBSNode node, int[] cg, ref int numEdges)
    {
        numEdges = 0;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        foreach (var conflict in node.Conflicts)
        {
            int a1 = Math.Min(conflict.A1, conflict.A2);
            int a2 = Math.Max(conflict.A1, conflict.A2);
            int idx = a1 * _numAgents + a2;
            if (cg[idx] > 0) continue;

            if (conflict.Priority == ConflictPriority.Cardinal)
            {
                cg[idx] = 1; cg[a2 * _numAgents + a1] = 1; numEdges++;
                continue;
            }

            var key = new HTableEntry(a1, a2, node);
            if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2 &&
                _lookupTable[a1][a2].TryGetValue(key, out var cached))
            {
                cg[idx] = cached.H > 0 ? 1 : 0;
                cg[a2 * _numAgents + a1] = cg[idx];
                NumMemoization++;
            }
            else
            {
                bool dep = Dependent(a1, a2, node);
                cg[idx] = dep ? 1 : 0;
                cg[a2 * _numAgents + a1] = cg[idx];
                if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2)
                    _lookupTable[a1][a2][key] = (cg[idx], 1, 0);
                if (_sw.Elapsed.TotalSeconds > _timeLimit)
                { RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds; return false; }
            }
            if (cg[idx] > 0) { numEdges++; conflict.Priority = ConflictPriority.PseudoCardinal; }
        }
        RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds;
        return true;
    }

    // -------------------------------------------------------------------------
    // 图构建：WDG（CBS 版本）
    // -------------------------------------------------------------------------

    private bool BuildWeightedDependencyGraph(CBSNode node, int[] cg)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        foreach (var conflict in node.Conflicts)
        {
            int a1 = Math.Min(conflict.A1, conflict.A2);
            int a2 = Math.Max(conflict.A1, conflict.A2);
            int idx = a1 * _numAgents + a2;

            var key = new HTableEntry(a1, a2, node);
            if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2 &&
                _lookupTable[a1][a2].TryGetValue(key, out var cached))
            {
                cg[idx] = cached.H; cg[a2 * _numAgents + a1] = cached.H;
                NumMemoization++;
            }
            else
            {
                bool cardinal = conflict.Priority == ConflictPriority.Cardinal;
                if (!cardinal && !MutexReasoning) cardinal = Dependent(a1, a2, node);

                int h = 0;
                if (cardinal)
                {
                    var (hVal, _, _) = Solve2Agents(a1, a2, node, cardinal);
                    h = hVal;
                }
                if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2)
                    _lookupTable[a1][a2][key] = (h, 1, 0);
                cg[idx] = h; cg[a2 * _numAgents + a1] = h;

                if (_sw.Elapsed.TotalSeconds > _timeLimit)
                { RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds; return false; }
            }
            if (cg[idx] == Common.MaxCost) return false;
            if (conflict.Priority != ConflictPriority.Cardinal && cg[idx] > 0)
                conflict.Priority = ConflictPriority.PseudoCardinal;
        }
        RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds;
        return true;
    }

    // -------------------------------------------------------------------------
    // 图构建：WDG（ECBS 版本）
    // -------------------------------------------------------------------------

    private bool BuildWeightedDependencyGraphECBS(ECBSNode node, IList<int> minFVals,
                                                   int[] cg, ref int deltaG)
    {
        deltaG = 0;
        var sw      = System.Diagnostics.Stopwatch.StartNew();
        var counted = new bool[_numAgents];
        foreach (var conflict in node.Conflicts.Concat(node.UnknownConflicts))
        {
            int a1 = Math.Min(conflict.A1, conflict.A2);
            int a2 = Math.Max(conflict.A1, conflict.A2);
            int idx = a1 * _numAgents + a2;
            var key = new HTableEntry(a1, a2, node);
            if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2 &&
                _lookupTable[a1][a2].TryGetValue(key, out var cached))
            {
                cg[idx] = cached.H; cg[a2 * _numAgents + a1] = cached.H;
                NumMemoization++;
                if (!counted[a1]) { deltaG += Math.Max(0, cached.F1 - minFVals[a1]); counted[a1] = true; }
                if (!counted[a2]) { deltaG += Math.Max(0, cached.F2 - minFVals[a2]); counted[a2] = true; }
            }
            else
            {
                var (h, f1, f2) = Solve2AgentsECBS(a1, a2, node);
                if (_lookupTable.Count > a1 && _lookupTable[a1].Count > a2)
                    _lookupTable[a1][a2][key] = (h, f1, f2);
                cg[idx] = h; cg[a2 * _numAgents + a1] = h;
                if (!counted[a1]) { deltaG += Math.Max(0, f1 - minFVals[a1]); counted[a1] = true; }
                if (!counted[a2]) { deltaG += Math.Max(0, f2 - minFVals[a2]); counted[a2] = true; }
                if (_sw.Elapsed.TotalSeconds > _timeLimit)
                { RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds; return false; }
            }
            if (cg[idx] == Common.MaxCost) return false;
        }
        RuntimeBuildDependencyGraph += sw.Elapsed.TotalSeconds;
        return true;
    }

    // -------------------------------------------------------------------------
    // 双智能体子 CBS
    // -------------------------------------------------------------------------

    private (int H, int Nodes, int Unused) Solve2Agents(int a1, int a2, CBSNode node, bool cardinal)
    {
        var engines = new List<SingleAgentSolver> { _searchEngines[a1], _searchEngines[a2] };
        var initPaths = new List<Path> { new Path(_paths[a1]!), new Path(_paths[a2]!) };
        var constraints = new List<ConstraintTable>
        {
            new ConstraintTable(_initialConstraints[a1]),
            new ConstraintTable(_initialConstraints[a2])
        };
        constraints[0].Insert2CT(node, a1);
        constraints[1].Insert2CT(node, a2);

        var sub = new CBS2Agent(engines, constraints, initPaths);
        sub.PC = PC; sub.RectangleReasoning = RectangleReasoning;
        sub.CorridorReasoning = CorridorReasoning; sub.TargetReasoning = TargetReasoning;

        int rootG = initPaths[0].Count - 1 + initPaths[1].Count - 1;
        int lb    = cardinal ? rootG + 1 : rootG;
        double rem = _timeLimit - _sw.Elapsed.TotalSeconds;
        sub.Solve(rem, lb, Common.MaxCost);
        NumSolve2AgentProblems++;

        int h = sub.Runtime >= rem || sub.NumHLExpanded > (ulong)NodeLimit
            ? sub.GetLowerBound() - rootG
            : sub.SolutionCost < 0 ? Common.MaxCost : sub.SolutionCost - rootG;
        return (h, (int)sub.NumHLExpanded, 0);
    }

    private (int H, int F1, int F2) Solve2AgentsECBS(int a1, int a2, ECBSNode node)
    {
        var engines = new List<SingleAgentSolver> { _searchEngines[a1], _searchEngines[a2] };
        var constraints = new List<ConstraintTable>
        {
            new ConstraintTable(_initialConstraints[a1]),
            new ConstraintTable(_initialConstraints[a2])
        };
        constraints[0].Insert2CT(node, a1);
        constraints[1].Insert2CT(node, a2);

        var sub = new CBS2Agent(engines, constraints, new List<Path>());
        sub.PC = PC;
        double rem = _timeLimit - _sw.Elapsed.TotalSeconds;
        sub.Solve(rem, 0, Common.MaxCost);
        NumSolve2AgentProblems++;

        if (sub.Runtime >= rem) return (0, 0, 0);
        int f1 = sub.GetInitialPathLength(0), f2 = sub.GetInitialPathLength(1);
        if (sub.NumHLExpanded > (ulong)NodeLimit)
            return (sub.GetLowerBound() - sub.DummyStartGVal, f1, f2);
        if (sub.SolutionCost < 0) return (Common.MaxCost, f1, f2);
        return (sub.SolutionCost - sub.DummyStartGVal, f1, f2);
    }

    // -------------------------------------------------------------------------
    // MDD 依赖判断 + SyncMDD
    // -------------------------------------------------------------------------

    private bool Dependent(int a1, int a2, HLNode node)
    {
        var mdd1 = _mddHelper.GetMDD(node, a1, _paths[a1]!.Count);
        var mdd2 = _mddHelper.GetMDD(node, a2, _paths[a2]!.Count);
        if (mdd1.Levels.Count > mdd2.Levels.Count) (mdd1, mdd2) = (mdd2, mdd1);
        NumMergeMDDs++;
        return !SyncMDDs(mdd1, mdd2);
    }

    private static bool SyncMDDs(MDD mdd, MDD other)
    {
        if (other.Levels.Count <= 1) return false;
        var copy = new SyncMDD(mdd);

        // 扩展到 other 的层数（终点等待）
        while (copy.Levels.Count < other.Levels.Count)
        {
            int i = copy.Levels.Count;
            copy.Levels.Add(new List<SyncMDDNode>());
            var parent   = copy.Levels[i - 1][0];
            var waitNode = new SyncMDDNode(parent.Location);
            parent.Children.Add(waitNode);
            waitNode.Parents.Add(parent);
            copy.Levels[i].Add(waitNode);
        }

        copy.Levels[0][0].CoexistingNodesFromOtherMdds.Add(other.Levels[0][0]);

        for (int i = 1; i < copy.Levels.Count; i++)
        {
            for (int ni = copy.Levels[i].Count - 1; ni >= 0; ni--)
            {
                var node = copy.Levels[i][ni];
                foreach (var parent in node.Parents)
                    foreach (var pCoex in parent.CoexistingNodesFromOtherMdds)
                        foreach (var cCoex in pCoex.Children)
                        {
                            if (node.Location == cCoex.Location) continue;
                            if (node.Location == pCoex.Location && parent.Location == cCoex.Location) continue;
                            if (!node.CoexistingNodesFromOtherMdds.Contains(cCoex))
                                node.CoexistingNodesFromOtherMdds.Add(cCoex);
                        }

                if (node.CoexistingNodesFromOtherMdds.Count == 0)
                    copy.DeleteNode(node, i);
            }
            if (copy.Levels[i].Count == 0) { copy.Clear(); return false; }
        }
        copy.Clear();
        return true;
    }

    // -------------------------------------------------------------------------
    // MVC（无权，CG/DG）
    // -------------------------------------------------------------------------

    private int MinimumVertexCover(int[] cg)
    {
        var sw   = System.Diagnostics.Stopwatch.StartNew();
        int rst  = 0;
        var done = new bool[_numAgents];

        for (int i = 0; i < _numAgents; i++)
        {
            if (done[i]) continue;
            var indices = new List<int>();
            var queue   = new Queue<int>();
            queue.Enqueue(i); done[i] = true;
            while (queue.Count > 0)
            {
                int j = queue.Dequeue();
                indices.Add(j);
                for (int k = 0; k < _numAgents; k++)
                    if (!done[k] && (cg[j * _numAgents + k] > 0 || cg[k * _numAgents + j] > 0))
                    { queue.Enqueue(k); done[k] = true; }
            }
            if (indices.Count == 1) continue;
            if (indices.Count == 2) { rst++; continue; }

            int n = indices.Count;
            var sub = new int[n * n];
            int numEdges = 0;
            for (int j = 0; j < n; j++)
                for (int k = j + 1; k < n; k++)
                {
                    sub[j * n + k] = cg[indices[j] * _numAgents + indices[k]];
                    sub[k * n + j] = cg[indices[k] * _numAgents + indices[j]];
                    if (sub[j * n + k] > 0) numEdges++;
                }

            if (numEdges > 10)
                rst += GreedyMatching(sub, n);
            else
            {
                for (int k = 1; k < n; k++)
                {
                    if (KVertexCover(sub, n, numEdges, k, n)) { rst += k; break; }
                    if (_sw.Elapsed.TotalSeconds > _timeLimit) return -1;
                }
            }
        }
        NumSolveMVC++;
        RuntimeSolveMVC += sw.Elapsed.TotalSeconds;
        return rst;
    }

    private bool KVertexCover(int[] cg, int numNodes, int numEdges, int k, int cols)
    {
        if (_sw.Elapsed.TotalSeconds > _timeLimit) return true;
        if (numEdges == 0) return true;
        if (numEdges > k * numNodes - k) return false;

        int u = -1, v = -1;
        for (int i = 0; i < cols - 1 && u < 0; i++)
            for (int j = i + 1; j < cols && u < 0; j++)
                if (cg[i * cols + j] > 0) { u = i; v = j; }

        foreach (int node in new[] { u, v })
        {
            var copy = (int[])cg.Clone();
            int removed = 0;
            for (int j = 0; j < cols; j++)
                if (copy[node * cols + j] > 0) { copy[node * cols + j] = 0; copy[j * cols + node] = 0; removed++; }
            if (KVertexCover(copy, numNodes - 1, numEdges - removed, k - 1, cols)) return true;
        }
        return false;
    }

    private static int GreedyMatching(int[] cg, int cols)
    {
        int rst = 0;
        var used = new bool[cols];
        for (int i = 0; i < cols; i++)
        {
            if (used[i]) continue;
            for (int j = i + 1; j < cols; j++)
                if (!used[j] && cg[i * cols + j] > 0) { rst++; used[i] = used[j] = true; break; }
        }
        return rst;
    }

    // -------------------------------------------------------------------------
    // MWVC（加权，WDG）
    // -------------------------------------------------------------------------

    private int MinimumWeightedVertexCover(int[] hg)
    {
        var sw  = System.Diagnostics.Stopwatch.StartNew();
        int rst = WeightedVertexCover(hg);
        NumSolveMVC++;
        RuntimeSolveMVC += sw.Elapsed.TotalSeconds;
        return rst;
    }

    private int WeightedVertexCover(int[] cg)
    {
        int rst  = 0;
        var done = new bool[_numAgents];
        for (int i = 0; i < _numAgents; i++)
        {
            if (done[i]) continue;
            var indices = new List<int>();
            var range   = new List<int>();
            var queue   = new Queue<int>();
            queue.Enqueue(i); done[i] = true;
            while (queue.Count > 0)
            {
                int j = queue.Dequeue();
                int maxW = 0;
                for (int k = 0; k < _numAgents; k++)
                {
                    int w = Math.Max(cg[j * _numAgents + k], cg[k * _numAgents + j]);
                    if (w > 0) { maxW = Math.Max(maxW, w); if (!done[k]) { queue.Enqueue(k); done[k] = true; } }
                }
                indices.Add(j); range.Add(maxW);
            }
            int n = indices.Count;
            if (n == 1) continue;
            if (n == 2) { rst += Math.Max(cg[indices[0] * _numAgents + indices[1]], cg[indices[1] * _numAgents + indices[0]]); continue; }

            var g = new int[n * n];
            for (int j = 0; j < n; j++)
                for (int k = j + 1; k < n; k++)
                    g[j * n + k] = Math.Max(cg[indices[j] * _numAgents + indices[k]], cg[indices[k] * _numAgents + indices[j]]);

            if (n > 5)
                rst += GreedyWeightedMatching(g, n);
            else
            {
                var x = new int[n]; int best = Common.MaxCost;
                rst += DPForWMVC(x, 0, 0, g, range, ref best);
            }
            if (_sw.Elapsed.TotalSeconds > _timeLimit) return -1;
        }
        return rst;
    }

    private int DPForWMVC(int[] x, int i, int sum, int[] cg, List<int> range, ref int bestSoFar)
    {
        if (sum >= bestSoFar) return Common.MaxCost;
        if (_sw.Elapsed.TotalSeconds > _timeLimit) return -1;
        int n = x.Length;
        if (i == n) { bestSoFar = sum; return sum; }
        if (range[i] == 0) return DPForWMVC(x, i + 1, sum, cg, range, ref bestSoFar);

        int minCost = 0;
        for (int j = 0; j < i; j++)
            if (minCost + x[j] < cg[j * n + i]) minCost = cg[j * n + i] - x[j];

        int bestCost = -1;
        for (int cost = minCost; cost <= range[i]; cost++)
        {
            x[i] = cost;
            int rst = DPForWMVC(x, i + 1, sum + cost, cg, range, ref bestSoFar);
            if (rst < bestSoFar) { bestSoFar = rst; bestCost = cost; }
        }
        if (bestCost >= 0) x[i] = bestCost;
        return bestSoFar;
    }

    private static int GreedyWeightedMatching(int[] cg, int cols)
    {
        int rst = 0;
        var used = new bool[cols];
        while (true)
        {
            int maxW = 0, ep1 = -1, ep2 = -1;
            for (int i = 0; i < cols; i++)
            {
                if (used[i]) continue;
                for (int j = i + 1; j < cols; j++)
                    if (!used[j] && cg[i * cols + j] > maxW) { maxW = cg[i * cols + j]; ep1 = i; ep2 = j; }
            }
            if (maxW == 0) return rst;
            rst += maxW; used[ep1] = used[ep2] = true;
        }
    }
}
