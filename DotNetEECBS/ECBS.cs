namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// ECBS（Enhanced CBS）：次优多智能体路径规划。
/// 在 CBS 基础上引入次优界 w，低层搜索使用 FindSuboptimalPath，
/// 高层使用 AStarEps（CLEANUP + FOCAL）策略。
/// </summary>
public class ECBS : CBS
{
    private List<int> _minFVals = new();

    private readonly UpdatablePriorityQueue<ECBSNode> _ecbsCleanup
        = new(new ECBSNode.CompareByF());
    private readonly UpdatablePriorityQueue<ECBSNode> _ecbsOpen
        = new(new ECBSNode.CompareByInadmissibleF());
    private readonly UpdatablePriorityQueue<ECBSNode> _ecbsFocal
        = new(new ECBSNode.CompareByD());
    private readonly List<ECBSNode> _ecbsAllNodes = new();

    private double _w = 1.5;

    public ECBS(Instance instance, double suboptimality = 1.5, int screen = 0)
        : base(instance, sipp: false, screen)
    {
        _w = suboptimality;
        SolverType = HighLevelSolverType.AStarEps;
    }

    // -------------------------------------------------------------------------
    // 公开接口
    // -------------------------------------------------------------------------

    public override bool Solve(double timeLimit, int costLowerBound = 0, int costUpperBound = int.MaxValue)
    {
        _costLowerBound             = costLowerBound;
        _inadmissibleCostLowerBound = 0;
        _timeLimit                  = timeLimit;
        _sw                         = System.Diagnostics.Stopwatch.StartNew();

        if (!EcbsGenerateRoot()) return false;

        while (!_ecbsCleanup.IsEmpty && !SolutionFound)
        {
            var curr = EcbsSelectNode();
            if (EcbsTerminate(curr)) return SolutionFound;

            ClassifyEcbsConflicts(curr);

            NumHLExpanded++;
            curr.TimeExpanded = NumHLExpanded;

            var child0 = new ECBSNode();
            var child1 = new ECBSNode();

            curr.ChosenConflict = ChooseEcbsConflict(curr);
            AddEcbsConstraints(curr, child0, child1);

            bool[] solved    = { false, false };
            var    pathsCopy = new List<Path?>(_paths);
            var    fminCopy  = new List<int>(_minFVals);

            for (int i = 0; i < 2; i++)
            {
                if (i > 0) { _paths = pathsCopy; _minFVals = fminCopy; }
                var child = i == 0 ? child0 : child1;
                solved[i] = EcbsGenerateChild(child, curr);
                if (!solved[i]) continue;
                EcbsPushNode(child);
                curr.Children.Add(child);
            }

            if (curr.ChosenConflict != null)
            {
                switch (curr.ChosenConflict.Type)
                {
                    case ConflictType.Rectangle: NumRectangleConflicts++; break;
                    case ConflictType.Corridor:  NumCorridorConflicts++;  break;
                    case ConflictType.Target:    NumTargetConflicts++;    break;
                    case ConflictType.Standard:  NumStandardConflicts++;  break;
                    case ConflictType.Mutex:     NumMutexConflicts++;     break;
                }
                if (curr.ChosenConflict.Priority == ConflictPriority.Cardinal)
                    NumCardinalConflicts++;
            }
            curr.Clear();
        }

        return SolutionFound;
    }

    public override void Clear()
    {
        _ecbsCleanup.Clear();
        _ecbsOpen.Clear();
        _ecbsFocal.Clear();
        _ecbsAllNodes.Clear();
        _paths.Clear();
        _minFVals.Clear();
        DummyStart    = null;
        GoalNode      = null;
        SolutionFound = false;
        SolutionCost  = -2;
    }

    public Path? GetEcbsPath(int agent) => _paths[agent];

    // -------------------------------------------------------------------------
    // 根节点生成
    // -------------------------------------------------------------------------

    private bool EcbsGenerateRoot()
    {
        var root = new ECBSNode { GVal = 0, SumOfCosts = 0, Depth = 0 };
        _paths               = new List<Path?>(new Path?[_numAgents]);
        _minFVals            = new List<int>(new int[_numAgents]);
        _pathsFoundInitially = new List<Path>(new Path[_numAgents]);

        var agents = Enumerable.Range(0, _numAgents).ToList();
        if (RandomRoot) agents = agents.OrderBy(_ => Guid.NewGuid()).ToList();

        foreach (int i in agents)
        {
            var (path, minF) = _searchEngines[i].FindSuboptimalPath(
                root, _initialConstraints[i], _paths, i, 0, _w);
            if (path.Count == 0) return false;

            _pathsFoundInitially[i] = path;
            _paths[i]               = path;
            _minFVals[i]            = minF;
            root.Makespan           = Math.Max(root.Makespan, path.Count - 1);
            root.GVal              += minF;
            root.SumOfCosts        += path.Count - 1;
            NumLLExpanded          += _searchEngines[i].NumExpanded;
            NumLLGenerated         += _searchEngines[i].NumGenerated;
        }

        root.HVal = 0;
        FindEcbsConflicts(root);
        root.UpdateDistanceToGo();
        EcbsPushNode(root);
        DummyStart = root;
        return true;
    }

    // -------------------------------------------------------------------------
    // 节点操作
    // -------------------------------------------------------------------------

    private void EcbsPushNode(ECBSNode node)
    {
        NumHLGenerated++;
        node.TimeGenerated = NumHLGenerated;
        _ecbsCleanup.Enqueue(node);
        if (node.SumOfCosts <= _w * _costLowerBound)
            _ecbsFocal.Enqueue(node);
        _ecbsAllNodes.Add(node);
    }

    private ECBSNode EcbsSelectNode()
    {
        if (_ecbsCleanup.Peek().FVal > _costLowerBound)
        {
            double oldThr = _w * _costLowerBound;
            _costLowerBound = Math.Max(_costLowerBound, _ecbsCleanup.Peek().FVal);
            double newThr = _w * _costLowerBound;
            foreach (var n in _ecbsAllNodes)
                if (n.SumOfCosts > oldThr && n.SumOfCosts <= newThr)
                    _ecbsFocal.Enqueue(n);
        }

        ECBSNode curr;
        if (!_ecbsFocal.IsEmpty)
        {
            curr = _ecbsFocal.Dequeue();
            _ecbsCleanup.Remove(curr);
            NumFocal++;
        }
        else
        {
            curr = _ecbsCleanup.Dequeue();
            NumCleanup++;
        }

        EcbsUpdatePaths(curr);
        return curr;
    }

    private void EcbsUpdatePaths(ECBSNode curr)
    {
        for (int i = 0; i < _numAgents; i++)
            _paths[i] = _pathsFoundInitially[i];

        var updated = new bool[_numAgents];
        var node    = curr;
        while (node != null)
        {
            foreach (var (agId, path, minF) in node.Paths)
            {
                if (!updated[agId])
                {
                    _paths[agId]    = path;
                    _minFVals[agId] = minF;
                    updated[agId]   = true;
                }
            }
            node = node.Parent as ECBSNode;
        }

        for (int i = 0; i < _numAgents; i++)
            if (!updated[i] && _paths[i] != null)
                _minFVals[i] = _paths[i]!.Count - 1;
    }

    // -------------------------------------------------------------------------
    // 子节点生成
    // -------------------------------------------------------------------------

    private bool EcbsGenerateChild(ECBSNode child, ECBSNode parent)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        child.Parent     = parent;
        child.GVal       = parent.GVal;
        child.SumOfCosts = parent.SumOfCosts;
        child.Makespan   = parent.Makespan;
        child.Depth      = parent.Depth + 1;

        foreach (int agent in GetEcbsInvalidAgents(child.Constraints))
        {
            if (!EcbsFindPath(child, agent))
            {
                RuntimeGenerateChild += sw.Elapsed.TotalSeconds;
                return false;
            }
        }

        FindEcbsConflicts(child);
        child.UpdateDistanceToGo();
        RuntimeGenerateChild += sw.Elapsed.TotalSeconds;
        return true;
    }

    private bool EcbsFindPath(ECBSNode node, int ag)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var (path, minF) = _searchEngines[ag].FindSuboptimalPath(
            node, _initialConstraints[ag], _paths, ag, _minFVals[ag], _w);
        NumLLExpanded      += _searchEngines[ag].NumExpanded;
        NumLLGenerated     += _searchEngines[ag].NumGenerated;
        RuntimeBuildCT     += _searchEngines[ag].RuntimeBuildCT;
        RuntimeBuildCAT    += _searchEngines[ag].RuntimeBuildCAT;
        RuntimePathFinding += sw.Elapsed.TotalSeconds;

        if (path.Count == 0) return false;

        node.Paths.Add((ag, path, minF));
        node.GVal        = node.GVal - _minFVals[ag] + minF;
        node.SumOfCosts  = node.SumOfCosts - _paths[ag]!.Count + path.Count;
        _paths[ag]       = path;
        _minFVals[ag]    = minF;
        node.Makespan    = Math.Max(node.Makespan, path.Count - 1);
        return true;
    }

    // -------------------------------------------------------------------------
    // 冲突检测
    // -------------------------------------------------------------------------

    private void FindEcbsConflicts(ECBSNode curr)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();

        if (curr.Parent != null)
        {
            var newAgents = curr.GetReplannedAgents();
            CopyEcbsConflicts(curr.Parent.Conflicts,        curr.Conflicts,        newAgents);
            CopyEcbsConflicts(curr.Parent.UnknownConflicts, curr.UnknownConflicts, newAgents);

            for (int i = 0; i < newAgents.Count; i++)
            {
                int a1 = newAgents[i];
                for (int a2 = 0; a2 < _numAgents; a2++)
                {
                    if (a1 == a2) continue;
                    bool skip = false;
                    for (int j = 0; j < i; j++)
                        if (newAgents[j] == a2) { skip = true; break; }
                    if (!skip) FindEcbsConflictsBetween(curr, a1, a2);
                }
            }
        }
        else
        {
            for (int a1 = 0; a1 < _numAgents; a1++)
                for (int a2 = a1 + 1; a2 < _numAgents; a2++)
                    FindEcbsConflictsBetween(curr, a1, a2);
        }

        RuntimeDetectConflicts += sw.Elapsed.TotalSeconds;
    }

    private void FindEcbsConflictsBetween(ECBSNode curr, int a1, int a2)
    {
        var p1 = _paths[a1]!;
        var p2 = _paths[a2]!;
        int minLen = Math.Min(p1.Count, p2.Count);

        for (int t = 0; t < minLen; t++)
        {
            int loc1 = p1[t].location, loc2 = p2[t].location;
            if (loc1 == loc2)
            {
                var c = new Conflict();
                if (TargetReasoning && p1.Count == t + 1)      c.SetTargetConflict(a1, a2, loc1, t);
                else if (TargetReasoning && p2.Count == t + 1) c.SetTargetConflict(a2, a1, loc1, t);
                else                                            c.SetVertexConflict(a1, a2, loc1, t);
                curr.UnknownConflicts.Add(c);
            }
            else if (t < minLen - 1 &&
                     loc1 == p2[t + 1].location &&
                     loc2 == p1[t + 1].location)
            {
                var c = new Conflict();
                c.SetEdgeConflict(a1, a2, loc1, loc2, t + 1);
                curr.UnknownConflicts.Add(c);
            }
        }

        if (p1.Count != p2.Count)
        {
            var pShort = p1.Count < p2.Count ? p1 : p2;
            var pLong  = p1.Count < p2.Count ? p2 : p1;
            int shortA = p1.Count < p2.Count ? a1 : a2;
            int longA  = p1.Count < p2.Count ? a2 : a1;
            int goal   = pShort[^1].location;
            for (int t = minLen; t < pLong.Count; t++)
            {
                if (pLong[t].location == goal)
                {
                    var c = new Conflict();
                    if (TargetReasoning) c.SetTargetConflict(shortA, longA, goal, t);
                    else                 c.SetVertexConflict(shortA, longA, goal, t);
                    curr.UnknownConflicts.Insert(0, c);
                }
            }
        }
    }

    private static void CopyEcbsConflicts(List<Conflict> src, List<Conflict> dst, IList<int> excluded)
    {
        foreach (var c in src)
            if (!excluded.Any(a => c.A1 == a || c.A2 == a) &&
                c.Constraint1.Count > 0 && c.Constraint2.Count > 0)
                dst.Add(c);
    }

    // -------------------------------------------------------------------------
    // 冲突分类（简化版）
    // -------------------------------------------------------------------------

    private void ClassifyEcbsConflicts(ECBSNode node)
    {
        while (node.UnknownConflicts.Count > 0)
        {
            var con = node.UnknownConflicts[0];
            node.UnknownConflicts.RemoveAt(0);
            con.Priority          = ConflictPriority.Non;
            con.SecondaryPriority = con.Constraint1.Count > 0 ? con.Constraint1[0].Timestep : 0;
            node.Conflicts.Add(con);
        }
        RemoveEcbsLowPriorityConflicts(node.Conflicts);
    }

    private void RemoveEcbsLowPriorityConflicts(List<Conflict> conflicts)
    {
        if (conflicts.Count == 0) return;
        var keep     = new Dictionary<int, Conflict>();
        var toDelete = new List<Conflict>();
        foreach (var c in conflicts)
        {
            int key = Math.Min(c.A1, c.A2) * _numAgents + Math.Max(c.A1, c.A2);
            if (!keep.TryGetValue(key, out var ex))
                keep[key] = c;
            else if (ex.Priority > c.Priority ||
                     (ex.Priority == c.Priority && ex.SecondaryPriority < c.SecondaryPriority))
            { toDelete.Add(ex); keep[key] = c; }
            else
                toDelete.Add(c);
        }
        foreach (var c in toDelete) conflicts.Remove(c);
    }

    // -------------------------------------------------------------------------
    // 冲突选择 & 约束分裂
    // -------------------------------------------------------------------------

    private Conflict? ChooseEcbsConflict(HLNode node)
    {
        var pool = node.Conflicts.Count > 0 ? node.Conflicts : node.UnknownConflicts;
        if (pool.Count == 0) return null;
        Conflict? best = null;
        foreach (var c in pool)
            if (best == null ||
                c.Priority < best.Priority ||
                (c.Priority == best.Priority && c.SecondaryPriority > best.SecondaryPriority))
                best = c;
        return best;
    }

    private static void AddEcbsConstraints(HLNode curr, ECBSNode child0, ECBSNode child1)
    {
        var conflict = curr.ChosenConflict!;
        child0.Constraints = new List<Constraint>(conflict.Constraint1);
        child1.Constraints = new List<Constraint>(conflict.Constraint2);
    }

    // -------------------------------------------------------------------------
    // 终止检测
    // -------------------------------------------------------------------------

    private bool EcbsTerminate(ECBSNode curr)
    {
        Runtime = _sw.Elapsed.TotalSeconds;

        if (curr.Conflicts.Count == 0 && curr.UnknownConflicts.Count == 0)
        {
            SolutionFound = true;
            GoalNode      = curr;
            SolutionCost  = curr.SumOfCosts;
            return true;
        }

        if (Runtime > _timeLimit || NumHLExpanded > (ulong)_nodeLimit)
        {
            SolutionCost  = -1;
            SolutionFound = false;
            return true;
        }

        return false;
    }

    // -------------------------------------------------------------------------
    // 辅助
    // -------------------------------------------------------------------------

    private HashSet<int> GetEcbsInvalidAgents(List<Constraint> constraints)
    {
        var agents = new HashSet<int>();
        if (constraints.Count == 0) return agents;
        var first = constraints[0];
        if (first.Type == ConstraintType.LeqLength)
        {
            for (int ag = 0; ag < _numAgents; ag++)
            {
                if (ag == first.Agent) continue;
                for (int i = first.Timestep; i < _paths[ag]!.Count; i++)
                    if (_paths[ag]![i].location == first.Loc1) { agents.Add(ag); break; }
            }
        }
        else if (first.Type == ConstraintType.PositiveVertex)
        {
            for (int ag = 0; ag < _numAgents; ag++)
            {
                if (ag == first.Agent) continue;
                int t = Math.Min(first.Timestep, _paths[ag]!.Count - 1);
                if (_paths[ag]![t].location == first.Loc1) agents.Add(ag);
            }
        }
        else
            agents.Add(first.Agent);

        return agents;
    }

    /// <summary>验证 ECBS 解：无冲突且代价在次优界内</summary>
    public bool ValidateEcbsSolution()
    {
        if (!SolutionFound) return false;
        int soc = 0;
        for (int a1 = 0; a1 < _numAgents; a1++)
        {
            soc += _paths[a1]!.Count - 1;
            for (int a2 = a1 + 1; a2 < _numAgents; a2++)
            {
                var p1 = _paths[a1]!; var p2 = _paths[a2]!;
                int minLen = Math.Min(p1.Count, p2.Count);
                for (int t = 0; t < minLen; t++)
                {
                    if (p1[t].location == p2[t].location) return false;
                    if (t < minLen - 1 &&
                        p1[t].location == p2[t + 1].location &&
                        p2[t].location == p1[t + 1].location) return false;
                }
                if (p1.Count != p2.Count)
                {
                    var pShort = p1.Count < p2.Count ? p1 : p2;
                    var pLong  = p1.Count < p2.Count ? p2 : p1;
                    int goal   = pShort[^1].location;
                    for (int t = minLen; t < pLong.Count; t++)
                        if (pLong[t].location == goal) return false;
                }
            }
        }
        return soc == SolutionCost;
    }
}
