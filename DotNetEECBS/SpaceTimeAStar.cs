namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 时空 A* 搜索节点，继承自 LLNode，额外持有在 OPEN/FOCAL 堆中的句柄信息。
/// </summary>
public class AStarNode : LLNode
{
    public AStarNode(int location, int g, int h, LLNode? parent, int timestep,
                     int numConflicts = 0, bool inOpenList = false)
        : base(location, g, h, parent, timestep, numConflicts, inOpenList)
    {
    }
}

/// <summary>
/// 时空 A* 搜索（Space-Time A*）。
/// 在时空图上搜索满足约束的最短路径，支持次优搜索（FOCAL 列表）。
/// </summary>
public class SpaceTimeAStar : SingleAgentSolver
{
    // OPEN 列表：按 f 值排序的最小堆
    private readonly UpdatablePriorityQueue<AStarNode> _openList
        = new(new LLNode.CompareByF());

    // FOCAL 列表：在 OPEN 中 f ≤ w*f_min 的节点，按冲突数排序
    private readonly UpdatablePriorityQueue<AStarNode> _focalList
        = new(new LLNode.CompareByConflicts());

    // 哈希表：(location, timestep) → 节点，用于判断节点是否已生成
    private readonly Dictionary<(int loc, int t), AStarNode> _allNodes = new();

    public SpaceTimeAStar(Instance instance, int agent)
        : base(instance, agent)
    {
    }

    public override string GetName() => "SpaceTimeAStar";

    // -------------------------------------------------------------------------
    // 公开搜索接口
    // -------------------------------------------------------------------------

    /// <summary>寻找最优路径（w=1，等价于标准 A*）</summary>
    public override Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound)
        => FindSuboptimalPath(node, initialConstraints, paths, agent, lowerBound, 1.0).Path;

    /// <summary>
    /// 寻找次优路径（bounded suboptimal，w ≥ 1）。
    /// 使用 OPEN + FOCAL 双列表：OPEN 保证最优性下界，FOCAL 在界内优先选冲突少的节点。
    /// </summary>
    public override (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w)
    {
        W = w;
        NumExpanded  = 0;
        NumGenerated = 0;

        // 构建约束表
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var ct = new ConstraintTable(initialConstraints);
        ct.Insert2CT(node, agent);
        RuntimeBuildCT = sw.Elapsed.TotalSeconds;

        // 起点被约束则无解
        if (ct.Constrained(StartLocation, 0))
            return (new Path(), 0);

        sw.Restart();
        ct.Insert2CAT(agent, paths);
        RuntimeBuildCAT = sw.Elapsed.TotalSeconds;

        int holdingTime   = ct.GetHoldingTime(GoalLocation, ct.LengthMin);
        int staticTs      = ct.GetMaxTimestep() + 1;
        lowerBound        = Math.Max(holdingTime, lowerBound);

        // 生成起点节点
        var start = new AStarNode(
            StartLocation, 0,
            Math.Max(lowerBound, MyHeuristic[StartLocation]),
            null, 0, 0);

        NumGenerated++;
        start.InOpenList = true;
        _openList.Enqueue(start);
        _focalList.Enqueue(start);
        _allNodes[(StartLocation, 0)] = start;
        MinFVal = start.FVal;

        Path path = new();

        while (!_openList.IsEmpty)
        {
            UpdateFocalList();
            var curr = PopNode();

            // 到达目标：位置正确、不是在目标等待、且可以持续占据
            if (curr.Location == GoalLocation &&
                !curr.WaitAtGoal &&
                curr.Timestep >= holdingTime)
            {
                path = BuildPath(curr);
                break;
            }

            if (curr.Timestep >= ct.LengthMax)
                continue;

            // 扩展邻居（含原地等待）
            var nextLocs = Instance.GetNeighbors(curr.Location);
            nextLocs.Add(curr.Location);

            foreach (int nextLoc in nextLocs)
            {
                int nextTs = curr.Timestep + 1;

                // 超过静态时间步后退化为空间 A*（不再增加时间步，跳过原地等待）
                if (nextTs > staticTs)
                {
                    if (nextLoc == curr.Location) continue;
                    nextTs--;
                }

                if (ct.Constrained(nextLoc, nextTs) ||
                    ct.Constrained(curr.Location, nextLoc, nextTs))
                    continue;

                int nextG = curr.GVal + 1;
                int nextH = Math.Max(lowerBound - nextG, MyHeuristic[nextLoc]);
                if (nextG + nextH > ct.LengthMax) continue;

                int nextConflicts = curr.NumOfConflicts +
                    ct.GetNumOfConflictsForStep(curr.Location, nextLoc, nextTs);

                var next = new AStarNode(nextLoc, nextG, nextH, curr, nextTs, nextConflicts);
                if (nextLoc == GoalLocation && curr.Location == GoalLocation)
                    next.WaitAtGoal = true;

                var key = (nextLoc, nextTs);
                if (!_allNodes.TryGetValue(key, out var existing))
                {
                    // 新节点
                    PushNode(next);
                    _allNodes[key] = next;
                }
                else
                {
                    // 已存在：判断是否需要更新
                    bool betterF         = existing.FVal > next.FVal;
                    bool sameFFewConflicts = existing.FVal == next.FVal &&
                                            existing.NumOfConflicts > next.NumOfConflicts;

                    if (betterF || sameFFewConflicts)
                    {
                        bool wasInFocal  = existing.FVal <= w * MinFVal;
                        bool willInFocal = next.FVal     <= w * MinFVal;
                        bool fDecreased  = existing.FVal > next.FVal;

                        existing.Copy(next);

                        if (!existing.InOpenList)
                        {
                            // 节点在 CLOSED，重新打开
                            PushNode(existing);
                        }
                        else
                        {
                            if (fDecreased)
                                _openList.Update(existing, existing);

                            if (!wasInFocal && willInFocal)
                                _focalList.Enqueue(existing);
                            else if (wasInFocal)
                                _focalList.Update(existing, existing);
                        }
                    }
                }
            }
        }

        ReleaseNodes();
        return (path, MinFVal);
    }

    /// <summary>
    /// 计算从 start 到 end 的最短行驶时间（忽略 CAT，仅考虑 CT 约束）。
    /// 用于走廊推理等需要快速估算旅行时间的场景。
    /// </summary>
    public override int GetTravelTime(int start, int end, ConstraintTable ct, int upperBound)
    {
        int length    = Common.MaxTimestep;
        int staticTs  = ct.GetMaxTimestep() + 1;

        var root = new AStarNode(start, 0, ComputeHeuristic(start, end), null, 0);
        root.InOpenList = true;
        _openList.Enqueue(root);
        _allNodes[(start, 0)] = root;

        while (!_openList.IsEmpty)
        {
            var curr = _openList.Dequeue();
            curr.InOpenList = false;

            if (curr.Location == end)
            {
                length = curr.GVal;
                break;
            }

            var nextLocs = Instance.GetNeighbors(curr.Location);
            nextLocs.Add(curr.Location);

            foreach (int nextLoc in nextLocs)
            {
                int nextTs = curr.Timestep + 1;
                int nextG  = curr.GVal + 1;

                if (nextTs > staticTs)
                {
                    if (nextLoc == curr.Location) continue;
                    nextTs--;
                }

                if (ct.Constrained(nextLoc, nextTs) ||
                    ct.Constrained(curr.Location, nextLoc, nextTs))
                    continue;

                int nextH = ComputeHeuristic(nextLoc, end);
                if (nextG + nextH >= upperBound) continue;

                var key = (nextLoc, nextTs);
                if (!_allNodes.TryGetValue(key, out var existing))
                {
                    var next = new AStarNode(nextLoc, nextG, nextH, null, nextTs);
                    next.InOpenList = true;
                    _openList.Enqueue(next);
                    _allNodes[key] = next;
                }
                else if (existing.GVal > nextG)
                {
                    existing.GVal     = nextG;
                    existing.Timestep = nextTs;
                    _openList.Update(existing, existing);
                }
            }
        }

        ReleaseNodes();
        return length;
    }

    // -------------------------------------------------------------------------
    // 私有辅助方法
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从 FOCAL 列表弹出节点，同时从 OPEN 列表逻辑删除。
    /// </summary>
    private AStarNode PopNode()
    {
        var node = _focalList.Dequeue();
        _openList.Remove(node);
        node.InOpenList = false;
        NumExpanded++;
        return node;
    }

    /// <summary>
    /// 将节点推入 OPEN，若 f ≤ w*MinFVal 同时推入 FOCAL。
    /// </summary>
    private void PushNode(AStarNode node)
    {
        node.InOpenList = true;
        _openList.Enqueue(node);
        NumGenerated++;
        if (node.FVal <= W * MinFVal)
            _focalList.Enqueue(node);
    }

    /// <summary>
    /// 当 OPEN 堆顶 f 值增大时，将新进入 FOCAL 范围的节点补充进 FOCAL 列表。
    /// </summary>
    private void UpdateFocalList()
    {
        if (_openList.IsEmpty) return;
        var openHead = _openList.Peek();
        if (openHead.FVal <= MinFVal) return;

        int newMinF = openHead.FVal;
        // 遍历 OPEN 中所有节点，把新进入 [w*oldMin, w*newMin] 范围的节点加入 FOCAL
        // 由于 UpdatablePriorityQueue 不支持遍历，改用已记录的 _allNodes 遍历
        foreach (var node in _allNodes.Values)
        {
            if (!node.InOpenList) continue;
            if (node.FVal > W * MinFVal && node.FVal <= W * newMinF)
                _focalList.Enqueue(node);
        }
        MinFVal = newMinF;
    }

    /// <summary>从目标节点回溯构建路径（正序）</summary>
    private static Path BuildPath(AStarNode goal)
    {
        var curr = goal.IsGoal ? goal.Parent! : goal;
        var path = new Path(curr.GVal + 1);
        while (curr != null)
        {
            path.Add(new PathEntry(curr.Location));
            curr = (AStarNode?)curr.Parent;
        }
        path.Reverse();
        return path;
    }

    /// <summary>释放所有节点，清空搜索状态</summary>
    private void ReleaseNodes()
    {
        _openList.Clear();
        _focalList.Clear();
        _allNodes.Clear();
    }
}
