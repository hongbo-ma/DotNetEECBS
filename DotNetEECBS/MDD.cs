namespace DotNetEECBS;

using Path = List<PathEntry>;

// ============================================================================
// MDDNode
// ============================================================================

/// <summary>
/// MDD 中的一个节点，表示智能体在某时间步的位置。
/// </summary>
public class MDDNode
{
    public int Location;
    public int Level;
    /// <summary>经过此节点的最短路径代价（到达终点的时间步）</summary>
    public int Cost;
    public List<MDDNode> Children = new();
    public List<MDDNode> Parents  = new();

    public MDDNode(int location, int level)
    {
        Location = location;
        Level    = level;
    }

    /// <summary>从父节点构造，自动设置 Level = parent.Level + 1</summary>
    public MDDNode(int location, MDDNode parent)
    {
        Location = location;
        Level    = parent.Level + 1;
        Parents.Add(parent);
    }

    public bool Equals(MDDNode other) =>
        Location == other.Location && Level == other.Level;
}

// ============================================================================
// MDD
// ============================================================================

/// <summary>
/// 多值决策图（MDD）：表示单个智能体在给定约束下所有最优路径的集合。
/// Levels[t] 包含时间步 t 时所有可能的位置节点。
/// </summary>
public class MDD
{
    public List<List<MDDNode>> Levels = new();
    public SingleAgentSolver?  Solver;

    public MDD() { }

    /// <summary>深拷贝构造</summary>
    public MDD(MDD other)
    {
        Solver = other.Solver;
        Levels = new List<List<MDDNode>>(other.Levels.Count);
        for (int i = 0; i < other.Levels.Count; i++)
            Levels.Add(new List<MDDNode>());

        var nodeMap = new Dictionary<(int, int), MDDNode>();
        foreach (var level in other.Levels)
            foreach (var n in level)
            {
                var copy = new MDDNode(n.Location, n.Level) { Cost = n.Cost };
                Levels[n.Level].Add(copy);
                nodeMap[(n.Location, n.Level)] = copy;
            }

        foreach (var level in other.Levels)
            foreach (var n in level)
            {
                var copy = nodeMap[(n.Location, n.Level)];
                foreach (var child in n.Children)
                    if (nodeMap.TryGetValue((child.Location, child.Level), out var cc))
                    {
                        copy.Children.Add(cc);
                        cc.Parents.Add(copy);
                    }
            }
    }

    // -------------------------------------------------------------------------
    // BuildMDD：固定层数版本（用于 CBS 启发式）
    // -------------------------------------------------------------------------

    /// <summary>
    /// 构建指定层数的 MDD（BFS 正向 + 反向剪枝）。
    /// numLevels = 路径长度 + 1（包含起点层）。
    /// </summary>
    public bool BuildMDD(ConstraintTable ct, int numLevels, SingleAgentSolver solver)
    {
        Solver = solver;
        var root = new MDDNode(solver.StartLocation, 0);
        var open   = new Queue<MDDNode>();
        var closed = new List<MDDNode>();
        open.Enqueue(root);
        closed.Add(root);

        Levels = new List<List<MDDNode>>(numLevels);
        for (int i = 0; i < numLevels; i++) Levels.Add(new List<MDDNode>());

        while (open.Count > 0)
        {
            var curr = open.Dequeue();
            if (curr.Level == numLevels - 1)
            {
                Levels[numLevels - 1].Add(curr);
                break;
            }

            int hBound = numLevels - curr.Level - 2;
            foreach (int nextLoc in solver.GetNextLocations(curr.Location))
            {
                if (solver.MyHeuristic[nextLoc] > hBound) continue;
                if (ct.Constrained(nextLoc, curr.Level + 1)) continue;
                if (ct.Constrained(curr.Location, nextLoc, curr.Level + 1)) continue;

                MDDNode? existing = null;
                for (int k = closed.Count - 1; k >= 0 && closed[k].Level == curr.Level + 1; k--)
                    if (closed[k].Location == nextLoc) { existing = closed[k]; break; }

                if (existing == null)
                {
                    var child = new MDDNode(nextLoc, curr) { Cost = numLevels - 1 };
                    open.Enqueue(child);
                    closed.Add(child);
                }
                else
                {
                    existing.Parents.Add(curr);
                }
            }
        }

        if (Levels[numLevels - 1].Count == 0) return false;

        // 反向传播：建立 Children 链接
        for (int t = numLevels - 2; t >= 0; t--)
        {
            foreach (var node in Levels[t + 1])
                foreach (var parent in node.Parents)
                {
                    if (parent.Children.Count == 0)
                        Levels[t].Add(parent);
                    parent.Children.Add(node);
                }
        }

        return Levels[0].Count > 0;
    }

    // -------------------------------------------------------------------------
    // BuildMDD：最小 MDD 版本（用于冲突分类）
    // -------------------------------------------------------------------------

    /// <summary>
    /// 构建最小 MDD（自动确定最短路径长度）。
    /// 使用 A* 搜索找到最短路径，然后反向构建 MDD。
    /// </summary>
    public bool BuildMDD(ConstraintTable ct, SingleAgentSolver solver)
    {
        Solver = solver;
        int holdingTime = ct.GetHoldingTime(solver.GoalLocation, ct.LengthMin);
        int upperBound  = ct.LengthMax;

        var allNodes = new Dictionary<(int, int), AStarMDDNode>();
        var open     = new PriorityQueue<AStarMDDNode, int>();

        var startNode = new AStarMDDNode(solver.StartLocation, 0, solver.MyHeuristic[solver.StartLocation]);
        allNodes[(solver.StartLocation, 0)] = startNode;
        open.Enqueue(startNode, startNode.F);

        AStarMDDNode? goalNode = null;

        while (open.Count > 0)
        {
            open.TryDequeue(out var curr, out _);
            if (curr == null) break;

            if (goalNode == null &&
                curr.Location == solver.GoalLocation &&
                curr.Timestep >= holdingTime &&
                (curr.Parents.Count == 0 || curr.Parents[0].Location != solver.GoalLocation))
            {
                goalNode   = curr;
                upperBound = curr.Timestep;
                continue;
            }

            if (curr.Timestep + solver.MyHeuristic[curr.Location] > upperBound) continue;

            foreach (int nextLoc in solver.GetNextLocations(curr.Location))
            {
                int nextT = curr.Timestep + 1;
                int nextH = solver.MyHeuristic[nextLoc];
                if (nextT + nextH > upperBound) continue;
                if (ct.Constrained(nextLoc, nextT)) continue;
                if (ct.Constrained(curr.Location, nextLoc, nextT)) continue;

                var key = (nextLoc, nextT);
                if (!allNodes.TryGetValue(key, out var existing))
                {
                    var next = new AStarMDDNode(nextLoc, nextT, nextH);
                    next.Parents.Add(curr);
                    allNodes[key] = next;
                    open.Enqueue(next, next.F);

                    if (goalNode == null &&
                        nextLoc == solver.GoalLocation &&
                        nextT >= holdingTime &&
                        curr.Location != solver.GoalLocation)
                    {
                        goalNode   = next;
                        upperBound = nextT;
                    }
                }
                else
                {
                    existing.Parents.Add(curr);
                    if (goalNode == null &&
                        existing.Location == solver.GoalLocation &&
                        existing.Timestep >= holdingTime &&
                        curr.Location != solver.GoalLocation)
                    {
                        goalNode   = existing;
                        upperBound = existing.Timestep;
                    }
                }
            }
        }

        if (goalNode == null) return false;

        // 反向构建 MDD
        int depth    = goalNode.Timestep + 1;
        Levels       = new List<List<MDDNode>>(depth);
        for (int i = 0; i < depth; i++) Levels.Add(new List<MDDNode>());

        var mddNodes = new Dictionary<(int, int), MDDNode>();
        var queue    = new Queue<AStarMDDNode>();
        var visited  = new HashSet<AStarMDDNode>(ReferenceEqualityComparer.Instance);

        var goalMDD = new MDDNode(goalNode.Location, goalNode.Timestep);
        Levels[goalNode.Timestep].Add(goalMDD);
        mddNodes[(goalNode.Location, goalNode.Timestep)] = goalMDD;
        queue.Enqueue(goalNode);
        visited.Add(goalNode);

        while (queue.Count > 0)
        {
            var curr    = queue.Dequeue();
            var currMDD = mddNodes[(curr.Location, curr.Timestep)];

            foreach (var parent in curr.Parents)
            {
                if (curr == goalNode && parent.Location == solver.GoalLocation) continue;

                var key = (parent.Location, parent.Timestep);
                if (!mddNodes.TryGetValue(key, out var parentMDD))
                {
                    parentMDD = new MDDNode(parent.Location, parent.Timestep);
                    Levels[parent.Timestep].Add(parentMDD);
                    mddNodes[key] = parentMDD;
                }
                parentMDD.Children.Add(currMDD);
                currMDD.Parents.Add(parentMDD);

                if (!visited.Contains(parent))
                {
                    visited.Add(parent);
                    queue.Enqueue(parent);
                }
            }
        }

        return Levels.Count > 0 && Levels[0].Count > 0;
    }

    // -------------------------------------------------------------------------
    // 辅助方法
    // -------------------------------------------------------------------------

    /// <summary>在指定层查找指定位置的节点</summary>
    public MDDNode? Find(int location, int level)
    {
        if (level >= Levels.Count) return null;
        foreach (var n in Levels[level])
            if (n.Location == location) return n;
        return null;
    }

    /// <summary>递归删除节点及其孤立的父子节点</summary>
    public void DeleteNode(MDDNode node)
    {
        Levels[node.Level].Remove(node);
        foreach (var child in new List<MDDNode>(node.Children))
        {
            child.Parents.Remove(node);
            if (child.Parents.Count == 0) DeleteNode(child);
        }
        foreach (var parent in new List<MDDNode>(node.Parents))
        {
            parent.Children.Remove(node);
            if (parent.Children.Count == 0) DeleteNode(parent);
        }
    }

    public void Clear() => Levels.Clear();

    /// <summary>在指定层查找目标位置节点</summary>
    public MDDNode? GoalAt(int level)
    {
        if (Solver == null || level >= Levels.Count) return null;
        foreach (var n in Levels[level])
            if (n.Location == Solver.GoalLocation && n.Cost == level) return n;
        return null;
    }

    /// <summary>A* 搜索中的临时节点（仅用于 BuildMDD 最小版本）</summary>
    private class AStarMDDNode
    {
        public int Location;
        public int Timestep;
        public int H;
        public int F => Timestep + H;
        public List<AStarMDDNode> Parents = new();

        public AStarMDDNode(int loc, int t, int h)
        {
            Location = loc; Timestep = t; H = h;
        }
    }
}

// ============================================================================
// SyncMDDNode / SyncMDD
// ============================================================================

/// <summary>同步 MDD 节点，用于两个智能体 MDD 的联合分析</summary>
public class SyncMDDNode
{
    public int Location;
    public List<SyncMDDNode> Children = new();
    public List<SyncMDDNode> Parents  = new();
    public List<MDDNode>     CoexistingNodesFromOtherMdds = new();

    public SyncMDDNode(int location) { Location = location; }
}

/// <summary>
/// 同步 MDD：将两个智能体的 MDD 合并，用于互斥推理和冲突分类。
/// </summary>
public class SyncMDD
{
    public List<List<SyncMDDNode>> Levels = new();

    public SyncMDD(MDD mdd)
    {
        Levels = new List<List<SyncMDDNode>>(mdd.Levels.Count);
        for (int i = 0; i < mdd.Levels.Count; i++)
            Levels.Add(new List<SyncMDDNode>());

        if (mdd.Levels.Count == 0 || mdd.Levels[0].Count == 0) return;

        var root = new SyncMDDNode(mdd.Levels[0][0].Location);
        Levels[0].Add(root);

        for (int t = 0; t < Levels.Count - 1; t++)
        {
            foreach (var node in Levels[t])
            {
                var mddNode = mdd.Find(node.Location, t);
                if (mddNode == null) continue;
                foreach (var mddChild in mddNode.Children)
                {
                    var child = Find(mddChild.Location, t + 1);
                    if (child == null)
                    {
                        child = new SyncMDDNode(mddChild.Location);
                        Levels[t + 1].Add(child);
                    }
                    node.Children.Add(child);
                    child.Parents.Add(node);
                }
            }
        }
    }

    public SyncMDDNode? Find(int location, int level)
    {
        if (level >= Levels.Count) return null;
        foreach (var n in Levels[level])
            if (n.Location == location) return n;
        return null;
    }

    public void DeleteNode(SyncMDDNode node, int level)
    {
        Levels[level].Remove(node);
        foreach (var child in new List<SyncMDDNode>(node.Children))
        {
            child.Parents.Remove(node);
            if (child.Parents.Count == 0) DeleteNode(child, level + 1);
        }
        foreach (var parent in new List<SyncMDDNode>(node.Parents))
        {
            parent.Children.Remove(node);
            if (parent.Children.Count == 0) DeleteNode(parent, level - 1);
        }
    }

    public void Clear() => Levels.Clear();
}

// ============================================================================
// MDDTable
// ============================================================================

/// <summary>
/// MDD 缓存表：按（智能体ID + 约束集合）缓存已构建的 MDD，避免重复计算。
/// </summary>
public class MDDTable
{
    public double AccumulatedRuntime;
    public ulong  NumReleasedMdds;

    private const int MaxNumOfMdds = 10000;

    private readonly List<Dictionary<ConstraintsHasher, MDD>> _lookupTable = new();
    private readonly List<ConstraintTable>                     _initialConstraints;
    private readonly List<SingleAgentSolver>                   _searchEngines;

    public MDDTable(List<ConstraintTable> initialConstraints, List<SingleAgentSolver> searchEngines)
    {
        _initialConstraints = initialConstraints;
        _searchEngines      = searchEngines;
    }

    public void Init(int numAgents)
    {
        _lookupTable.Clear();
        for (int i = 0; i < numAgents; i++)
            _lookupTable.Add(new Dictionary<ConstraintsHasher, MDD>(
                new ConstraintsHasher.EqNode()));
    }

    /// <summary>在缓存中查找 MDD，未命中返回 null</summary>
    public MDD? FindMDD(HLNode node, int agent)
    {
        if (agent >= _lookupTable.Count) return null;
        var key = new ConstraintsHasher(agent, node);
        return _lookupTable[agent].TryGetValue(key, out var mdd) ? mdd : null;
    }

    /// <summary>
    /// 获取 MDD：先查缓存，未命中则构建并缓存。
    /// mddLevels = -1 表示构建最小 MDD。
    /// </summary>
    public MDD GetMDD(HLNode node, int agent, int mddLevels = -1)
    {
        if (agent < _lookupTable.Count)
        {
            var key = new ConstraintsHasher(agent, node);
            if (_lookupTable[agent].TryGetValue(key, out var cached))
                return cached;
        }

        ReleaseMDDMemory(agent);

        var sw  = System.Diagnostics.Stopwatch.StartNew();
        var mdd = new MDD();
        var ct  = new ConstraintTable(_initialConstraints[agent]);
        ct.Insert2CT(node, agent);

        bool ok = mddLevels >= 0
            ? mdd.BuildMDD(ct, mddLevels, _searchEngines[agent])
            : mdd.BuildMDD(ct, _searchEngines[agent]);

        AccumulatedRuntime += sw.Elapsed.TotalSeconds;

        if (ok && agent < _lookupTable.Count)
            _lookupTable[agent][new ConstraintsHasher(agent, node)] = mdd;

        return mdd;
    }

    public void Clear()
    {
        foreach (var table in _lookupTable)
            table.Clear();
    }

    private void ReleaseMDDMemory(int id)
    {
        if (id < 0 || id >= _lookupTable.Count) return;
        if (_lookupTable[id].Count < MaxNumOfMdds) return;

        int minLen = int.MaxValue;
        foreach (var kv in _lookupTable[id])
            if (kv.Value.Levels.Count < minLen)
                minLen = kv.Value.Levels.Count;

        var toRemove = new List<ConstraintsHasher>();
        foreach (var kv in _lookupTable[id])
            if (kv.Value.Levels.Count == minLen)
                toRemove.Add(kv.Key);

        foreach (var k in toRemove)
        {
            _lookupTable[id].Remove(k);
            NumReleasedMdds++;
        }
    }
}
