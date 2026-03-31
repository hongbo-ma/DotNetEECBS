namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// CBS（Conflict-Based Search）高层搜索。
/// 维护约束树，通过分裂冲突生成子节点，直到找到无冲突解。
/// </summary>
public class CBS
{
    // -------------------------------------------------------------------------
    // 统计
    // -------------------------------------------------------------------------
    public double Runtime, RuntimeGenerateChild, RuntimeBuildCT, RuntimeBuildCAT;
    public double RuntimePathFinding, RuntimeDetectConflicts, RuntimePreprocessing;
    public ulong  NumCardinalConflicts, NumCorridorConflicts, NumRectangleConflicts;
    public ulong  NumTargetConflicts, NumMutexConflicts, NumStandardConflicts;
    public ulong  NumAdoptBypass;
    public ulong  NumHLExpanded, NumHLGenerated, NumLLExpanded, NumLLGenerated;
    public ulong  NumCleanup, NumOpen, NumFocal;

    // -------------------------------------------------------------------------
    // 配置
    // -------------------------------------------------------------------------
    public bool RandomRoot;
    public bool RectangleReasoning, CorridorReasoning, TargetReasoning;
    public bool DisjointSplitting, MutexReasoning, Bypass, PC, SaveStats;
    public HighLevelSolverType SolverType            = HighLevelSolverType.AStar;
    public ConflictSelection   ConflictSelectionRule = ConflictSelection.Earliest;
    public NodeSelection       NodeSelectionRule     = NodeSelection.NodeH;

    // -------------------------------------------------------------------------
    // 状态
    // -------------------------------------------------------------------------
    protected HLNode? DummyStart;
    protected HLNode? GoalNode;
    public    bool    SolutionFound;
    public    int     SolutionCost = -2;

    // -------------------------------------------------------------------------
    // 核心数据
    // -------------------------------------------------------------------------
    protected readonly Instance           _instance;
    protected readonly int                _numAgents;
    protected List<Path?>                 _paths               = new();
    protected List<Path>                  _pathsFoundInitially = new();
    protected List<SingleAgentSolver>     _searchEngines       = new();
    protected List<ConstraintTable>       _initialConstraints  = new();
    protected MDDTable           _mddTable            = null!;
    protected CorridorReasoning  _corridorReasoning   = null!;
    protected RectangleReasoning _rectangleReasoning  = null!;
    protected CBSHeuristic       _cbsHeuristic        = null!;
    public CBSHeuristic Heuristic => _cbsHeuristic;

    private readonly UpdatablePriorityQueue<CBSNode> _cleanupList
        = new(new CBSNode.CompareByF());
    private readonly UpdatablePriorityQueue<CBSNode> _openList
        = new(new CBSNode.CompareByInadmissibleF());
    private readonly UpdatablePriorityQueue<CBSNode> _focalList
        = new(new CBSNode.CompareByD());
    private readonly List<CBSNode> _allNodes = new();

    protected double _timeLimit;
    protected double _suboptimality = 1.0;
    protected int    _costLowerBound;
    protected int    _inadmissibleCostLowerBound;
    protected int    _costUpperBound = Common.MaxCost;
    protected int    _nodeLimit      = Common.MaxNodes;
    protected System.Diagnostics.Stopwatch _sw = new();

    // -------------------------------------------------------------------------
    // 构造函数
    // -------------------------------------------------------------------------

    public CBS(Instance instance, bool sipp, int screen = 0)
    {
        _instance  = instance;
        _numAgents = instance.GetDefaultNumberOfAgents();

        var t = System.Diagnostics.Stopwatch.StartNew();
        _initialConstraints = Enumerable.Range(0, _numAgents)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize))
            .ToList();

        _searchEngines = Enumerable.Range(0, _numAgents)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i))
            .ToList();

        _mddTable            = new MDDTable(_initialConstraints, _searchEngines);
        _mddTable.Init(_numAgents);
        _corridorReasoning   = new CorridorReasoning(_searchEngines, _initialConstraints);
        _rectangleReasoning  = new RectangleReasoning(_instance);
        _cbsHeuristic        = new CBSHeuristic(_numAgents, _paths, _searchEngines,
                                                 _initialConstraints, _mddTable);

        RuntimePreprocessing = t.Elapsed.TotalSeconds;
    }

    // -------------------------------------------------------------------------
    // 公开接口
    // -------------------------------------------------------------------------

    public virtual bool Solve(double timeLimit, int costLowerBound = 0, int costUpperBound = int.MaxValue)
    {
        _costLowerBound             = costLowerBound;
        _inadmissibleCostLowerBound = 0;
        _costUpperBound             = costUpperBound == int.MaxValue ? Common.MaxCost : costUpperBound;
        _timeLimit                  = timeLimit;
        _sw                         = System.Diagnostics.Stopwatch.StartNew();

        if (!GenerateRoot()) return false;

        while (!_cleanupList.IsEmpty && !SolutionFound)
        {
            var curr = SelectNode();
            if (Terminate(curr)) return SolutionFound;

            if (PC) ClassifyConflicts(curr);

            NumHLExpanded++;
            curr.TimeExpanded = NumHLExpanded;

            bool foundBypass = true;
            while (foundBypass)
            {
                if (Terminate(curr)) return SolutionFound;
                foundBypass = false;

                var child0 = new CBSNode();
                var child1 = new CBSNode();

                curr.ChosenConflict = ChooseConflict(curr);
                AddConstraints(curr, child0, child1);

                bool[] solved    = { false, false };
                var    pathsCopy = new List<Path?>(_paths);

                for (int i = 0; i < 2; i++)
                {
                    if (i > 0) _paths = pathsCopy;
                    var child = i == 0 ? child0 : child1;
                    solved[i] = GenerateChild(child, curr);
                    if (!solved[i]) continue;

                    if (Bypass && child.GVal == curr.GVal && child.DistanceToGo < curr.DistanceToGo)
                    {
                        if (i == 1 && !solved[0]) continue;
                        foundBypass = true;
                        NumAdoptBypass++;
                        curr.Conflicts        = child.Conflicts;
                        curr.UnknownConflicts = child.UnknownConflicts;
                        curr.DistanceToGo     = child.DistanceToGo;
                        curr.ChosenConflict   = null;
                        foreach (var (agId, path) in child.Paths)
                        {
                            bool updated = false;
                            for (int k = 0; k < curr.Paths.Count; k++)
                            {
                                if (curr.Paths[k].AgentId == agId)
                                {
                                    curr.Paths[k] = (agId, path);
                                    _paths[agId]  = curr.Paths[k].Path;
                                    updated = true;
                                    break;
                                }
                            }
                            if (!updated)
                            {
                                curr.Paths.Add((agId, path));
                                _paths[agId] = curr.Paths[^1].Path;
                            }
                        }
                        break;
                    }
                }

                if (foundBypass)
                {
                    if (PC) ClassifyConflicts(curr);
                }
                else
                {
                    for (int i = 0; i < 2; i++)
                    {
                        var child = i == 0 ? child0 : child1;
                        if (solved[i]) { PushNode(child); curr.Children.Add(child); }
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
            }
        }
        return SolutionFound;
    }

    public int  GetLowerBound() => _costLowerBound;

    public virtual void Clear()
    {
        _cleanupList.Clear();
        _openList.Clear();
        _focalList.Clear();
        _allNodes.Clear();
        _paths.Clear();
        _pathsFoundInitially.Clear();
        DummyStart    = null;
        GoalNode      = null;
        SolutionFound = false;
        SolutionCost  = -2;
    }

    public Path? GetPath(int agent) => _paths[agent];

    // -------------------------------------------------------------------------
    // 私有：根节点生成
    // -------------------------------------------------------------------------

    private bool GenerateRoot()
    {
        var root = new CBSNode { GVal = 0, Depth = 0 };
        _paths               = new List<Path?>(new Path?[_numAgents]);
        _pathsFoundInitially = new List<Path>(new Path[_numAgents]);

        var agents = Enumerable.Range(0, _numAgents).ToList();
        if (RandomRoot) agents = agents.OrderBy(_ => Guid.NewGuid()).ToList();

        foreach (int i in agents)
        {
            var path = _searchEngines[i].FindOptimalPath(root, _initialConstraints[i], _paths, i, 0);
            if (path.Count == 0) return false;
            _pathsFoundInitially[i] = path;
            _paths[i]               = path;
            root.Makespan           = Math.Max(root.Makespan, path.Count - 1);
            root.GVal              += path.Count - 1;
            NumLLExpanded          += _searchEngines[i].NumExpanded;
            NumLLGenerated         += _searchEngines[i].NumGenerated;
        }

        root.HVal = 0;
        FindConflicts(root);
        _cbsHeuristic.ComputeQuickHeuristics(root);
        PushNode(root);
        DummyStart = root;
        return true;
    }

    // -------------------------------------------------------------------------
    // 私有：节点操作
    // -------------------------------------------------------------------------

    private void PushNode(CBSNode node)
    {
        NumHLGenerated++;
        node.TimeGenerated = NumHLGenerated;
        _cleanupList.Enqueue(node);

        if (SolverType == HighLevelSolverType.AStarEps &&
            node.FVal <= _suboptimality * _costLowerBound)
            _focalList.Enqueue(node);
        else if (SolverType == HighLevelSolverType.EES)
        {
            _openList.Enqueue(node);
            if (node.GetFHatVal() <= _suboptimality * _inadmissibleCostLowerBound)
                _focalList.Enqueue(node);
        }
        _allNodes.Add(node);
    }

    private CBSNode SelectNode()
    {
        CBSNode curr;
        if (SolverType == HighLevelSolverType.AStarEps)
        {
            if (_cleanupList.Peek().FVal > _costLowerBound)
            {
                double oldThr = _suboptimality * _costLowerBound;
                _costLowerBound = Math.Max(_costLowerBound, _cleanupList.Peek().FVal);
                double newThr = _suboptimality * _costLowerBound;
                foreach (var n in _allNodes)
                    if (n.InOpenList && n.FVal > oldThr && n.FVal <= newThr)
                        _focalList.Enqueue(n);
            }
            curr = _focalList.Dequeue();
            _cleanupList.Remove(curr);
            NumFocal++;
        }
        else
        {
            _costLowerBound = Math.Max(_costLowerBound, _cleanupList.Peek().FVal);
            curr = _cleanupList.Dequeue();
            NumCleanup++;
        }
        UpdatePaths(curr);
        return curr;
    }

    // -------------------------------------------------------------------------
    // 私有：路径更新
    // -------------------------------------------------------------------------

    private void UpdatePaths(CBSNode curr)
    {
        for (int i = 0; i < _numAgents; i++)
            _paths[i] = _pathsFoundInitially[i];

        var updated = new bool[_numAgents];
        var node    = curr;
        while (node != null)
        {
            foreach (var (agId, path) in node.Paths)
                if (!updated[agId]) { _paths[agId] = path; updated[agId] = true; }
            node = node.Parent as CBSNode;
        }
    }

    // -------------------------------------------------------------------------
    // 私有：子节点生成
    // -------------------------------------------------------------------------

    private bool GenerateChild(CBSNode child, CBSNode parent)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        child.Parent   = parent;
        child.GVal     = parent.GVal;
        child.Makespan = parent.Makespan;
        child.Depth    = parent.Depth + 1;

        foreach (int agent in GetInvalidAgents(child.Constraints))
        {
            int lb = _paths[agent]!.Count - 1;
            if (!FindPathForSingleAgent(child, agent, lb))
            {
                RuntimeGenerateChild += sw.Elapsed.TotalSeconds;
                return false;
            }
        }

        FindConflicts(child);
        _cbsHeuristic.ComputeQuickHeuristics(child);
        RuntimeGenerateChild += sw.Elapsed.TotalSeconds;
        return true;
    }

    private bool FindPathForSingleAgent(CBSNode node, int ag, int lowerBound)
    {
        var sw   = System.Diagnostics.Stopwatch.StartNew();
        var path = _searchEngines[ag].FindOptimalPath(node, _initialConstraints[ag], _paths, ag, lowerBound);
        NumLLExpanded      += _searchEngines[ag].NumExpanded;
        NumLLGenerated     += _searchEngines[ag].NumGenerated;
        RuntimeBuildCT     += _searchEngines[ag].RuntimeBuildCT;
        RuntimeBuildCAT    += _searchEngines[ag].RuntimeBuildCAT;
        RuntimePathFinding += sw.Elapsed.TotalSeconds;

        if (path.Count == 0) return false;

        node.Paths.Add((ag, path));
        node.GVal     = node.GVal - _paths[ag]!.Count + path.Count;
        _paths[ag]    = path;
        node.Makespan = Math.Max(node.Makespan, path.Count - 1);
        return true;
    }

    // -------------------------------------------------------------------------
    // 私有：冲突检测
    // -------------------------------------------------------------------------

    private void FindConflicts(HLNode curr)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();

        if (curr.Parent != null)
        {
            var newAgents = curr.GetReplannedAgents();
            CopyConflicts(curr.Parent.Conflicts,        curr.Conflicts,        newAgents);
            CopyConflicts(curr.Parent.UnknownConflicts, curr.UnknownConflicts, newAgents);

            for (int i = 0; i < newAgents.Count; i++)
            {
                int a1 = newAgents[i];
                for (int a2 = 0; a2 < _numAgents; a2++)
                {
                    if (a1 == a2) continue;
                    bool skip = false;
                    for (int j = 0; j < i; j++)
                        if (newAgents[j] == a2) { skip = true; break; }
                    if (!skip) FindConflictsBetween(curr, a1, a2);
                }
            }
        }
        else
        {
            for (int a1 = 0; a1 < _numAgents; a1++)
                for (int a2 = a1 + 1; a2 < _numAgents; a2++)
                    FindConflictsBetween(curr, a1, a2);
        }

        RuntimeDetectConflicts += sw.Elapsed.TotalSeconds;
    }

    private void FindConflictsBetween(HLNode curr, int a1, int a2)
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

    private static void CopyConflicts(List<Conflict> src, List<Conflict> dst, IList<int> excluded)
    {
        foreach (var c in src)
        {
            if (!excluded.Any(a => c.A1 == a || c.A2 == a) &&
                c.Constraint1.Count > 0 && c.Constraint2.Count > 0)
                dst.Add(c);
        }
    }

    // -------------------------------------------------------------------------
    // 私有：冲突分类（使用 MDD 判断 Cardinal/Semi/Non）
    // -------------------------------------------------------------------------

    private void ClassifyConflicts(CBSNode node)
    {
        while (node.UnknownConflicts.Count > 0)
        {
            var con = node.UnknownConflicts[0];
            node.UnknownConflicts.RemoveAt(0);

            // 用 MDD 判断冲突优先级
            ComputeConflictPriority(con, node);

            // 目标冲突直接加入
            if (con.Type == ConflictType.Target)
            {
                ComputeSecondPriority(con, node);
                node.Conflicts.Add(con);
                continue;
            }

            // 走廊推理
            if (CorridorReasoning)
            {
                var corridor = _corridorReasoning.Run(con, _paths, node);
                if (corridor != null)
                {
                    corridor.Priority = con.Priority;
                    ComputeSecondPriority(corridor, node);
                    node.Conflicts.Add(corridor);
                    continue;
                }
            }

            // 矩形推理（仅对顶点冲突，且两个智能体都未到达终点）
            if (RectangleReasoning &&
                con.Constraint1.Count > 0 &&
                con.Constraint1[0].Type == ConstraintType.Vertex)
            {
                int a1 = con.A1, a2 = con.A2;
                int t  = con.Constraint1[0].Timestep;
                if (t < _paths[a1]!.Count && t < _paths[a2]!.Count)
                {
                    var mdd1 = _mddTable.GetMDD(node, a1, _paths[a1]!.Count);
                    var mdd2 = _mddTable.GetMDD(node, a2, _paths[a2]!.Count);
                    var rect = _rectangleReasoning.Run(_paths, t, a1, a2, mdd1, mdd2);
                    if (rect != null)
                    {
                        ComputeSecondPriority(rect, node);
                        node.Conflicts.Add(rect);
                        continue;
                    }
                }
            }

            ComputeSecondPriority(con, node);
            node.Conflicts.Add(con);
        }
        RemoveLowPriorityConflicts(node.Conflicts);
    }

    /// <summary>
    /// 使用 MDD 判断冲突是 Cardinal / Semi / Non。
    /// Cardinal：两个智能体在冲突时间步的 MDD 层都只有一个节点（无其他选择）。
    /// Semi：只有一方是 Cardinal。
    /// Non：两方都有其他选择。
    /// </summary>
    private void ComputeConflictPriority(Conflict con, CBSNode node)
    {
        int a1 = con.A1, a2 = con.A2;
        if (con.Constraint1.Count == 0) { con.Priority = ConflictPriority.Non; return; }

        int timestep = con.Constraint1[^1].Timestep;
        var type     = con.Constraint1[^1].Type;

        bool cardinal1 = false, cardinal2 = false;

        // 超出路径长度的时间步 → 该智能体已停在终点，必然是 Cardinal
        if (timestep >= _paths[a1]!.Count) cardinal1 = true;
        if (timestep >= _paths[a2]!.Count) cardinal2 = true;

        MDD? mdd1 = cardinal1 ? null : _mddTable.GetMDD(node, a1, _paths[a1]!.Count);
        MDD? mdd2 = cardinal2 ? null : _mddTable.GetMDD(node, a2, _paths[a2]!.Count);

        if (type == ConstraintType.Edge) // 边冲突
        {
            if (!cardinal1 && mdd1 != null && timestep < mdd1.Levels.Count)
                cardinal1 = mdd1.Levels[timestep].Count == 1 &&
                            mdd1.Levels[timestep - 1].Count == 1;
            if (!cardinal2 && mdd2 != null && timestep < mdd2.Levels.Count)
                cardinal2 = mdd2.Levels[timestep].Count == 1 &&
                            mdd2.Levels[timestep - 1].Count == 1;
        }
        else // 顶点冲突或目标冲突
        {
            if (!cardinal1 && mdd1 != null && timestep < mdd1.Levels.Count)
                cardinal1 = mdd1.Levels[timestep].Count == 1;
            if (!cardinal2 && mdd2 != null && timestep < mdd2.Levels.Count)
                cardinal2 = mdd2.Levels[timestep].Count == 1;
        }

        con.Priority = (cardinal1, cardinal2) switch
        {
            (true,  true)  => ConflictPriority.Cardinal,
            (true,  false) => ConflictPriority.Semi,
            (false, true)  => ConflictPriority.Semi,
            _              => ConflictPriority.Non
        };
    }

    private void ComputeSecondPriority(Conflict c, HLNode node)
    {
        c.SecondaryPriority = ConflictSelectionRule switch
        {
            ConflictSelection.Earliest  => c.Constraint1.Count > 0 ? c.Constraint1[0].Timestep : 0,
            ConflictSelection.Conflicts => node.Conflicts.Count(
                x => x.A1 == c.A1 || x.A2 == c.A1 || x.A1 == c.A2 || x.A2 == c.A2),
            _                           => 0
        };
    }

    private void RemoveLowPriorityConflicts(List<Conflict> conflicts)
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
    // 私有：冲突选择
    // -------------------------------------------------------------------------

    private Conflict? ChooseConflict(HLNode node)
    {
        var pool = node.Conflicts.Count > 0 ? node.Conflicts : node.UnknownConflicts;
        if (pool.Count == 0) return null;
        Conflict? best = null;
        foreach (var c in pool)
        {
            if (best == null ||
                c.Priority < best.Priority ||
                (c.Priority == best.Priority && c.SecondaryPriority > best.SecondaryPriority))
                best = c;
        }
        return best;
    }

    // -------------------------------------------------------------------------
    // 私有：约束分裂
    // -------------------------------------------------------------------------

    private void AddConstraints(HLNode curr, CBSNode child0, CBSNode child1)
    {
        var conflict = curr.ChosenConflict!;
        if (DisjointSplitting && conflict.Type == ConflictType.Standard)
        {
            bool first = new Random().Next(2) == 0;
            if (first)
            {
                child0.Constraints = new List<Constraint>(conflict.Constraint1);
                var last = conflict.Constraint1[^1];
                child1.Constraints.Add(new Constraint(last.Agent, last.Loc1, last.Loc2, last.Timestep,
                    last.Type == ConstraintType.Vertex ? ConstraintType.PositiveVertex : ConstraintType.PositiveEdge));
            }
            else
            {
                child1.Constraints = new List<Constraint>(conflict.Constraint2);
                var last = conflict.Constraint2[^1];
                child0.Constraints.Add(new Constraint(last.Agent, last.Loc1, last.Loc2, last.Timestep,
                    last.Type == ConstraintType.Vertex ? ConstraintType.PositiveVertex : ConstraintType.PositiveEdge));
            }
        }
        else
        {
            child0.Constraints = new List<Constraint>(conflict.Constraint1);
            child1.Constraints = new List<Constraint>(conflict.Constraint2);
        }
    }

    // -------------------------------------------------------------------------
    // 私有：终止检测
    // -------------------------------------------------------------------------

    private bool Terminate(HLNode curr)
    {
        if (_costLowerBound >= _costUpperBound)
        { SolutionCost = _costLowerBound; SolutionFound = false; return true; }

        Runtime = _sw.Elapsed.TotalSeconds;

        if (curr.Conflicts.Count == 0 && curr.UnknownConflicts.Count == 0)
        {
            SolutionFound = true;
            GoalNode      = curr;
            SolutionCost  = curr.GetFHatVal() - curr.CostToGo;
            return true;
        }

        if (Runtime > _timeLimit || NumHLExpanded > (ulong)_nodeLimit)
        { SolutionCost = -1; SolutionFound = false; return true; }

        return false;
    }

    // -------------------------------------------------------------------------
    // 私有：辅助
    // -------------------------------------------------------------------------

    private HashSet<int> GetInvalidAgents(List<Constraint> constraints)
    {
        var agents = new HashSet<int>();
        if (constraints.Count == 0) return agents;
        var first = constraints[0];
        int agent = first.Agent, x = first.Loc1, t = first.Timestep;

        if (first.Type == ConstraintType.LeqLength)
        {
            for (int ag = 0; ag < _numAgents; ag++)
            {
                if (ag == agent) continue;
                for (int i = t; i < _paths[ag]!.Count; i++)
                    if (_paths[ag]![i].location == x) { agents.Add(ag); break; }
            }
        }
        else if (first.Type == ConstraintType.PositiveVertex)
        {
            for (int ag = 0; ag < _numAgents; ag++)
                if (ag != agent && GetAgentLocation(ag, t) == x) agents.Add(ag);
        }
        else if (first.Type == ConstraintType.PositiveEdge)
        {
            int y = first.Loc2;
            for (int ag = 0; ag < _numAgents; ag++)
            {
                if (ag == agent) continue;
                int cur = GetAgentLocation(ag, t), prev = GetAgentLocation(ag, t - 1);
                if (prev == x || cur == y || (prev == y && cur == x)) agents.Add(ag);
            }
        }
        else
            agents.Add(agent);

        return agents;
    }

    private int GetAgentLocation(int agent, int timestep)
    {
        int t = Math.Max(0, Math.Min(timestep, _paths[agent]!.Count - 1));
        return _paths[agent]![t].location;
    }

    /// <summary>验证解的合法性（无冲突且代价正确）</summary>
    public bool ValidateSolution()
    {
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
