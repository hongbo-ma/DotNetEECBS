namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 走廊推理（Corridor Reasoning）：
/// 检测两个智能体是否在狭窄走廊（度为2的节点序列）中相向而行，
/// 若是则生成更精确的 RANGE 约束，替代普通顶点约束，减少搜索树分支。
/// </summary>
public class CorridorReasoning
{
    public double AccumulatedRuntime;

    private readonly IList<SingleAgentSolver> _searchEngines;
    private readonly IList<ConstraintTable>   _initialConstraints;

    public CorridorReasoning(IList<SingleAgentSolver> searchEngines,
                              IList<ConstraintTable>   initialConstraints)
    {
        _searchEngines      = searchEngines;
        _initialConstraints = initialConstraints;
    }

    // -------------------------------------------------------------------------
    // 公开接口
    // -------------------------------------------------------------------------

    /// <summary>
    /// 尝试将普通冲突升级为走廊冲突。
    /// 若成功返回新的走廊冲突，否则返回 null。
    /// </summary>
    public Conflict? Run(Conflict conflict, IList<Path?> paths, HLNode node)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = FindCorridorConflict(conflict, paths, node);
        AccumulatedRuntime += sw.Elapsed.TotalSeconds;
        return result;
    }

    // -------------------------------------------------------------------------
    // 私有：走廊冲突检测
    // -------------------------------------------------------------------------

    /// <summary>
    /// 核心逻辑：
    /// 1. 确认冲突发生在走廊节点（度为2）
    /// 2. 找到两个智能体进入走廊的时间步和位置
    /// 3. 计算走廊长度和关键边
    /// 4. 用 GetTravelTime 验证绕行代价，确认是真正的走廊冲突
    /// 5. 生成 RANGE 约束
    /// </summary>
    private Conflict? FindCorridorConflict(Conflict conflict, IList<Path?> paths, HLNode node)
    {
        int a1 = conflict.A1, a2 = conflict.A2;
        if (paths[a1]!.Count <= 1 || paths[a2]!.Count <= 1) return null;
        if (conflict.Constraint1.Count == 0) return null;

        var c1   = conflict.Constraint1[^1];
        int loc1 = c1.Loc1;   // 顶点冲突时为冲突位置；边冲突时为 from
        int loc2 = c1.Loc2;   // 顶点冲突时为 -1；边冲突时为 to
        int t    = c1.Timestep;

        if (t < 1) return null;

        // 确定走廊中的一个节点 curr
        int curr;
        if (loc2 < 0) // 顶点冲突
        {
            if (_searchEngines[0].GetNeighbors(loc1).Count != 2) return null;
            curr = loc1;
        }
        else // 边冲突
        {
            bool deg1 = _searchEngines[0].GetNeighbors(loc1).Count == 2;
            bool deg2 = _searchEngines[0].GetNeighbors(loc2).Count == 2;
            if (!deg1 && !deg2) return null;
            curr = deg1 ? loc1 : loc2;
            t--;  // 边冲突时间步退一步，对应进入走廊的时刻
        }

        if (curr <= 0) return null;

        int[] a = { a1, a2 };

        // 找两个智能体进入走廊的时间步
        int[] enterT = new int[2];
        for (int i = 0; i < 2; i++)
            enterT[i] = GetEnteringTime(paths[a[i]]!, paths[a[1 - i]]!, t);

        // 按进入时间排序（先进入的为 a[0]）
        if (enterT[0] > enterT[1])
        {
            (enterT[0], enterT[1]) = (enterT[1], enterT[0]);
            (a[0], a[1])           = (a[1], a[0]);
        }

        // 进入走廊的位置
        int[] u = new int[2];
        for (int i = 0; i < 2; i++)
            u[i] = paths[a[i]]![enterT[i]].location;

        if (u[0] == u[1]) return null;

        // 验证两个智能体都会经过对方的入口位置（确认相向而行）
        for (int i = 0; i < 2; i++)
        {
            bool found = false;
            for (int time = enterT[i]; time < paths[a[i]]!.Count && !found; time++)
                if (paths[a[i]]![time].location == u[1 - i])
                    found = true;
            if (!found) return null;
        }

        // 计算走廊长度和关键边
        var edge = (from: -1, to: -1);
        int corridorLength = GetCorridorLength(paths[a[0]]!, enterT[0], u[1], ref edge);

        // 用 GetTravelTime 验证绕行代价
        var ct1 = new ConstraintTable(_initialConstraints[conflict.A1]);
        ct1.Insert2CT(node, conflict.A1);
        int t3 = _searchEngines[conflict.A1].GetTravelTime(
            paths[conflict.A1]![0].location, u[1], ct1, Common.MaxTimestep);

        var ct1b = new ConstraintTable(_initialConstraints[conflict.A1]);
        ct1b.Insert2CT(node, conflict.A1);
        BlockEdge(ct1b, edge.from, edge.to);
        int t3b = _searchEngines[conflict.A1].GetTravelTime(
            paths[conflict.A1]![0].location, u[1], ct1b, t3 + 2 * corridorLength + 1);

        var ct2 = new ConstraintTable(_initialConstraints[conflict.A2]);
        ct2.Insert2CT(node, conflict.A2);
        int t4 = _searchEngines[conflict.A2].GetTravelTime(
            paths[conflict.A2]![0].location, u[0], ct2, Common.MaxTimestep);

        var ct2b = new ConstraintTable(_initialConstraints[conflict.A2]);
        ct2b.Insert2CT(node, conflict.A2);
        BlockEdge(ct2b, edge.from, edge.to);
        int t4b = _searchEngines[conflict.A2].GetTravelTime(
            paths[conflict.A2]![0].location, u[0], ct2b, t3 + corridorLength + 1);

        // 条件：两智能体到达对方入口的时间差 ≤ 走廊长度，且封锁后绕行代价增加
        if (Math.Abs(t3 - t4) <= corridorLength && t3b > t3 && t4b > t4)
        {
            int newT1 = Math.Min(t3b - 1, t4  + corridorLength);
            int newT2 = Math.Min(t4b - 1, t3  + corridorLength);

            var corridorConflict = new Conflict();
            corridorConflict.SetCorridorConflict(conflict.A1, conflict.A2, u[1], u[0], newT1, newT2);

            // 验证约束确实阻塞了当前路径
            if (Blocked(paths[corridorConflict.A1]!, corridorConflict.Constraint1[0]) &&
                Blocked(paths[corridorConflict.A2]!, corridorConflict.Constraint2[0]))
                return corridorConflict;
        }

        return null;
    }

    // -------------------------------------------------------------------------
    // 私有辅助方法
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从时间步 t 向前回溯，找到智能体进入走廊的时间步。
    /// 停止条件：到达路径起点、对方终点、或走廊出口（度 != 2 的节点）。
    /// </summary>
    private int GetEnteringTime(Path path, Path path2, int t)
    {
        if (t >= path.Count) t = path.Count - 1;
        int loc = path[t].location;
        while (loc != path[0].location &&
               loc != path2[^1].location &&
               _searchEngines[0].GetNeighbors(loc).Count == 2)
        {
            t--;
            loc = path[t].location;
        }
        return t;
    }

    /// <summary>
    /// 从时间步 t 向后推进，找到智能体离开走廊的时间步。
    /// 停止条件：到达路径终点、或走廊出口（度 != 2 的节点）。
    /// </summary>
    private int GetExitingTime(Path path, int t)
    {
        if (t >= path.Count) t = path.Count - 1;
        int loc = path[t].location;
        while (loc != path[^1].location &&
               _searchEngines[0].GetNeighbors(loc).Count == 2)
        {
            t++;
            loc = path[t].location;
        }
        return t;
    }

    /// <summary>
    /// 沿路径从 tStart 走到 locEnd，计算走廊长度（净前进步数）。
    /// 同时记录走廊中第一条前进边 edge，用于后续封锁。
    /// </summary>
    private int GetCorridorLength(Path path, int tStart, int locEnd, ref (int from, int to) edge)
    {
        int curr      = path[tStart].location;
        int prev      = -1;
        int length    = 0;
        int t         = tStart;
        bool forward  = true;
        bool edgeSet  = false;

        while (curr != locEnd)
        {
            t++;
            int next = path[t].location;
            if (next == curr) continue;          // 原地等待，跳过
            if (next == prev) forward = !forward; // 掉头

            if (forward)
            {
                if (!edgeSet) { edge = (curr, next); edgeSet = true; }
                length++;
            }
            else
            {
                length--;
            }
            prev = curr;
            curr = next;
        }
        return length;
    }

    /// <summary>
    /// 判断路径是否被 RANGE 约束阻塞：
    /// 路径在 [t1, t2) 时间段内是否经过 loc。
    /// </summary>
    private static bool Blocked(Path path, Constraint c)
    {
        // c.Type == Range: (agent, loc, t1, t2, Range)
        int loc = c.Loc1;
        int t1  = c.Loc2;      // RANGE 约束中 Loc2 存 t_min
        int t2  = c.Timestep;  // Timestep 存 t_max

        for (int t = t1; t < t2; t++)
        {
            if (t >= path.Count)
            {
                if (loc == path[^1].location) return true;
            }
            else if (t >= 0 && path[t].location == loc)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>向约束表中插入双向边封锁（用于计算绕行代价）</summary>
    private static void BlockEdge(ConstraintTable ct, int from, int to)
    {
        ct.InsertEdge(from, to, 0, Common.MaxTimestep);
        ct.InsertEdge(to, from, 0, Common.MaxTimestep);
    }
}
