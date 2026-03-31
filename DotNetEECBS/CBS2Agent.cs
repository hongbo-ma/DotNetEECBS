namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 双智能体子 CBS，专供 CBSHeuristic 的 WDG 计算使用。
/// 只支持两个智能体，使用标准 A* 策略（最优解）。
/// </summary>
public class CBS2Agent
{
    public double Runtime;
    public ulong  NumHLExpanded;
    public int    SolutionCost = -2;
    public bool   PC;
    public bool   RectangleReasoning;
    public bool   CorridorReasoning;
    public bool   TargetReasoning;

    public int DummyStartGVal => _dummyStartGVal;

    private readonly List<SingleAgentSolver>  _engines;
    private readonly List<ConstraintTable>    _initialConstraints;
    private readonly List<Path>               _initPaths;

    private List<Path?>  _paths = new();
    private int          _dummyStartGVal;
    private int          _costLowerBound;
    private int          _nodeLimit = 4;

    private readonly UpdatablePriorityQueue<CBSNode> _open
        = new(new CBSNode.CompareByF());
    private readonly List<CBSNode> _allNodes = new();

    private MDDTable?          _mddTable;
    private RectangleReasoning? _rectHelper;
    private CorridorReasoning?  _corrHelper;

    public CBS2Agent(List<SingleAgentSolver> engines,
                     List<ConstraintTable>   initialConstraints,
                     List<Path>              initPaths)
    {
        _engines            = engines;
        _initialConstraints = initialConstraints;
        _initPaths          = initPaths;
    }

    public int GetLowerBound() => _costLowerBound;

    public int GetInitialPathLength(int agent)
    {
        if (_initPaths.Count > agent) return _initPaths[agent].Count - 1;
        if (_paths.Count > agent && _paths[agent] != null) return _paths[agent]!.Count - 1;
        return 0;
    }

    public bool Solve(double timeLimit, int lowerBound = 0, int upperBound = int.MaxValue)
    {
        _costLowerBound = lowerBound;
        var sw = System.Diagnostics.Stopwatch.StartNew();

        // 初始化辅助对象
        if (RectangleReasoning || CorridorReasoning)
        {
            _mddTable = new MDDTable(_initialConstraints, _engines);
            _mddTable.Init(2);
        }
        if (RectangleReasoning && _engines.Count > 0)
        {
            // 从 solver 获取 instance（通过反射或直接访问）
            // 简化：暂不启用矩形推理（需要 Instance 引用）
        }
        if (CorridorReasoning)
            _corrHelper = new CorridorReasoning(_engines, _initialConstraints);

        // 生成根节点
        var root = new CBSNode { GVal = 0, Depth = 0 };
        _paths = new List<Path?>(new Path?[2]);

        if (_initPaths.Count == 2)
        {
            for (int i = 0; i < 2; i++)
            {
                _paths[i]  = _initPaths[i];
                root.GVal += _initPaths[i].Count - 1;
            }
        }
        else
        {
            for (int i = 0; i < 2; i++)
            {
                var path = _engines[i].FindOptimalPath(root, _initialConstraints[i], _paths, i, 0);
                if (path.Count == 0) return false;
                _paths[i]  = path;
                root.GVal += path.Count - 1;
            }
        }

        _dummyStartGVal = root.GVal;
        FindConflicts(root);
        root.UpdateDistanceToGo();
        PushNode(root);

        while (!_open.IsEmpty)
        {
            Runtime = sw.Elapsed.TotalSeconds;
            if (Runtime > timeLimit || NumHLExpanded > (ulong)_nodeLimit) break;

            var curr = _open.Dequeue();
            curr.InOpenList = false;
            UpdatePaths(curr);

            if (curr.Conflicts.Count == 0 && curr.UnknownConflicts.Count == 0)
            {
                SolutionCost = curr.GVal;
                return true;
            }

            NumHLExpanded++;
            curr.TimeExpanded = NumHLExpanded;

            ClassifyConflicts(curr);
            var conflict = ChooseConflict(curr);
            if (conflict == null) break;
            curr.ChosenConflict = conflict;

            for (int i = 0; i < 2; i++)
            {
                var child = new CBSNode();
                child.Constraints = i == 0
                    ? new List<Constraint>(conflict.Constraint1)
                    : new List<Constraint>(conflict.Constraint2);

                var pathsCopy = new List<Path?>(_paths);
                if (!GenerateChild(child, curr)) continue;
                _paths = pathsCopy; // 恢复（下一个子节点用）
                PushNode(child);
            }
            curr.Clear();
        }

        Runtime = sw.Elapsed.TotalSeconds;
        return false;
    }

    private void PushNode(CBSNode node)
    {
        node.InOpenList = true;
        _open.Enqueue(node);
        _allNodes.Add(node);
    }

    private void UpdatePaths(CBSNode curr)
    {
        if (_initPaths.Count == 2)
        {
            _paths[0] = _initPaths[0];
            _paths[1] = _initPaths[1];
        }
        var updated = new bool[2];
        var node    = curr;
        while (node != null)
        {
            foreach (var (agId, path) in node.Paths)
                if (!updated[agId]) { _paths[agId] = path; updated[agId] = true; }
            node = node.Parent as CBSNode;
        }
    }

    private bool GenerateChild(CBSNode child, CBSNode parent)
    {
        child.Parent   = parent;
        child.GVal     = parent.GVal;
        child.Makespan = parent.Makespan;
        child.Depth    = parent.Depth + 1;

        int agent = child.Constraints[0].Agent;
        int lb    = _paths[agent]!.Count - 1;
        var path  = _engines[agent].FindOptimalPath(child, _initialConstraints[agent], _paths, agent, lb);
        if (path.Count == 0) return false;

        child.Paths.Add((agent, path));
        child.GVal     = child.GVal - _paths[agent]!.Count + path.Count;
        _paths[agent]  = path;
        child.Makespan = Math.Max(child.Makespan, path.Count - 1);

        FindConflicts(child);
        child.UpdateDistanceToGo();
        return true;
    }

    private void FindConflicts(HLNode curr)
    {
        if (curr.Parent != null)
        {
            var newAgents = curr.GetReplannedAgents();
            CopyConflicts(curr.Parent.Conflicts,        curr.Conflicts,        newAgents);
            CopyConflicts(curr.Parent.UnknownConflicts, curr.UnknownConflicts, newAgents);
            foreach (int a1 in newAgents)
                for (int a2 = 0; a2 < 2; a2++)
                    if (a1 != a2) FindConflictsBetween(curr, a1, a2);
        }
        else
        {
            FindConflictsBetween(curr, 0, 1);
        }
    }

    private void FindConflictsBetween(HLNode curr, int a1, int a2)
    {
        var p1 = _paths[a1]!; var p2 = _paths[a2]!;
        int minLen = Math.Min(p1.Count, p2.Count);
        for (int t = 0; t < minLen; t++)
        {
            int l1 = p1[t].location, l2 = p2[t].location;
            if (l1 == l2)
            {
                var c = new Conflict();
                if (TargetReasoning && p1.Count == t + 1)      c.SetTargetConflict(a1, a2, l1, t);
                else if (TargetReasoning && p2.Count == t + 1) c.SetTargetConflict(a2, a1, l1, t);
                else                                            c.SetVertexConflict(a1, a2, l1, t);
                curr.UnknownConflicts.Add(c);
            }
            else if (t < minLen - 1 && l1 == p2[t+1].location && l2 == p1[t+1].location)
            {
                var c = new Conflict();
                c.SetEdgeConflict(a1, a2, l1, l2, t + 1);
                curr.UnknownConflicts.Add(c);
            }
        }
        if (p1.Count != p2.Count)
        {
            var pShort = p1.Count < p2.Count ? p1 : p2;
            var pLong  = p1.Count < p2.Count ? p2 : p1;
            int sA = p1.Count < p2.Count ? a1 : a2;
            int lA = p1.Count < p2.Count ? a2 : a1;
            int goal = pShort[^1].location;
            for (int t = minLen; t < pLong.Count; t++)
                if (pLong[t].location == goal)
                {
                    var c = new Conflict();
                    if (TargetReasoning) c.SetTargetConflict(sA, lA, goal, t);
                    else                 c.SetVertexConflict(sA, lA, goal, t);
                    curr.UnknownConflicts.Insert(0, c);
                }
        }
    }

    private static void CopyConflicts(List<Conflict> src, List<Conflict> dst, IList<int> excluded)
    {
        foreach (var c in src)
            if (!excluded.Any(a => c.A1 == a || c.A2 == a) &&
                c.Constraint1.Count > 0 && c.Constraint2.Count > 0)
                dst.Add(c);
    }

    private void ClassifyConflicts(CBSNode node)
    {
        while (node.UnknownConflicts.Count > 0)
        {
            var con = node.UnknownConflicts[0];
            node.UnknownConflicts.RemoveAt(0);
            con.Priority = ConflictPriority.Non;
            node.Conflicts.Add(con);
        }
    }

    private static Conflict? ChooseConflict(HLNode node)
    {
        var pool = node.Conflicts.Count > 0 ? node.Conflicts : node.UnknownConflicts;
        return pool.Count > 0 ? pool[0] : null;
    }
}
