namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 约束表（Constraint Table）：存储 CBS 节点施加给某个智能体的所有约束，
/// 以及其他智能体路径构成的冲突规避表（CAT）。
/// A* 搜索在扩展节点时查询此表判断移动是否合法。
/// </summary>
public class ConstraintTable
{
    // -------------------------------------------------------------------------
    // 公开字段
    // -------------------------------------------------------------------------

    /// <summary>路径长度下限（来自 GLength 约束）</summary>
    public int LengthMin = 0;

    /// <summary>路径长度上限（来自 LeqLength 约束）</summary>
    public int LengthMax = Common.MaxTimestep;

    /// <summary>CT 中最大时间步，用于确定搜索截止时间</summary>
    public int CtMaxTimestep { get; private set; } = 0;

    /// <summary>CAT 中最大时间步</summary>
    public int CatMaxTimestep { get; private set; } = 0;

    // -------------------------------------------------------------------------
    // 私有字段
    // -------------------------------------------------------------------------

    private readonly int _numCol;
    private readonly int _mapSize;

    /// <summary>
    /// 约束表 CT：位置/边索引 → 禁止时间段列表 [t_min, t_max)
    /// </summary>
    private readonly Dictionary<int, List<(int TMin, int TMax)>> _ct = new();

    /// <summary>
    /// 冲突规避表 CAT：cat[loc][t] = true 表示某其他智能体在时刻 t 占据 loc
    /// </summary>
    private List<List<bool>> _cat = new();

    /// <summary>cat_goals[loc] = 某其他智能体到达 loc 作为终点的时间步，用于检测目标占用冲突</summary>
    private int[] _catGoals = Array.Empty<int>();

    /// <summary>地标约束：timestep → location，智能体在该时刻必须在该位置</summary>
    private readonly SortedDictionary<int, int> _landmarks = new();

    // -------------------------------------------------------------------------
    // 构造函数
    // -------------------------------------------------------------------------

    public ConstraintTable(int numCol, int mapSize)
    {
        _numCol  = numCol;
        _mapSize = mapSize;
    }

    /// <summary>拷贝构造</summary>
    public ConstraintTable(ConstraintTable other)
    {
        _numCol          = other._numCol;
        _mapSize         = other._mapSize;
        LengthMin        = other.LengthMin;
        LengthMax        = other.LengthMax;
        CtMaxTimestep    = other.CtMaxTimestep;
        CatMaxTimestep   = other.CatMaxTimestep;

        foreach (var kv in other._ct)
            _ct[kv.Key] = new List<(int, int)>(kv.Value);

        _cat = other._cat.Select(row => new List<bool>(row)).ToList();
        _catGoals = (int[])other._catGoals.Clone();

        foreach (var kv in other._landmarks)
            _landmarks[kv.Key] = kv.Value;
    }

    // -------------------------------------------------------------------------
    // 查询方法
    // -------------------------------------------------------------------------

    /// <summary>
    /// 返回智能体能持续占据 location 的最早时间步（>= earliestTimestep）。
    /// 即从 earliestTimestep 开始，跳过所有约束区间后的第一个空闲时刻。
    /// </summary>
    public int GetHoldingTime(int location, int earliestTimestep)
    {
        int rst = earliestTimestep;

        // 检查 CT 中对该位置的约束
        if (_ct.TryGetValue(location, out var ranges))
            foreach (var (tMin, tMax) in ranges)
                rst = Math.Max(rst, tMax);

        // 检查地标：若某时刻智能体必须在其他位置，则该时刻之后才能持续占据
        foreach (var (t, loc) in _landmarks)
            if (loc != location)
                rst = Math.Max(rst, t + 1);

        return rst;
    }

    /// <summary>
    /// 返回搜索需要考虑的最大时间步。
    /// 超过此时间步后状态不再变化，可安全截止搜索。
    /// </summary>
    public int GetMaxTimestep()
    {
        int rst = Math.Max(Math.Max(CtMaxTimestep, CatMaxTimestep), LengthMin);
        if (LengthMax < Common.MaxTimestep)
            rst = Math.Max(rst, LengthMax);
        if (_landmarks.Count > 0)
            rst = Math.Max(rst, _landmarks.Keys.Max());
        return rst;
    }

    /// <summary>返回 CAT 中 location 最后一次发生冲突的时间步，无冲突返回 -1</summary>
    public int GetLastCollisionTimestep(int location)
    {
        if (_cat.Count == 0) return -1;
        var row = _cat[location];
        for (int t = row.Count - 1; t >= 0; t--)
            if (row[t]) return t;
        return -1;
    }

    /// <summary>
    /// 判断智能体在时刻 t 是否被约束不能出现在 loc。
    /// 同时检查地标约束（正约束）：若该时刻有地标且不在 loc，则被约束。
    /// </summary>
    public bool Constrained(int loc, int t)
    {
        // 检查地标（正顶点约束）：该时刻必须在某个特定位置
        if (loc < _mapSize && _landmarks.TryGetValue(t, out int landmarkLoc) && landmarkLoc != loc)
            return true;

        if (!_ct.TryGetValue(loc, out var ranges))
            return false;

        foreach (var (tMin, tMax) in ranges)
            if (tMin <= t && t < tMax)
                return true;

        return false;
    }

    /// <summary>判断从 currLoc 到 nextLoc 在时刻 nextT 的边移动是否被约束</summary>
    public bool Constrained(int currLoc, int nextLoc, int nextT)
        => Constrained(GetEdgeIndex(currLoc, nextLoc), nextT);

    /// <summary>
    /// 计算从 currId 移动到 nextId（时刻 nextTimestep）与其他智能体的冲突数。
    /// 包含：顶点冲突、边冲突、目标占用冲突。
    /// </summary>
    public int GetNumOfConflictsForStep(int currId, int nextId, int nextTimestep)
    {
        int rst = 0;
        if (_cat.Count == 0) return rst;

        var catNext = _cat[nextId];

        // 顶点冲突：nextId 在 nextTimestep 被占用
        if (catNext.Count > nextTimestep && catNext[nextTimestep])
            rst++;

        // 边冲突：nextId 在 nextTimestep-1 被占用 且 currId 在 nextTimestep 被占用（交换位置）
        if (currId != nextId &&
            catNext.Count >= nextTimestep &&
            _cat[currId].Count > nextTimestep &&
            catNext[nextTimestep - 1] &&
            _cat[currId][nextTimestep])
            rst++;

        // 目标占用冲突：某智能体已在 nextId 停止，而我们在其之后到达
        if (_catGoals[nextId] < nextTimestep)
            rst++;

        return rst;
    }

    /// <summary>判断从 currId 移动到 nextId（时刻 nextTimestep）是否存在任意冲突</summary>
    public bool HasConflictForStep(int currId, int nextId, int nextTimestep)
    {
        if (_cat.Count == 0) return false;

        var catNext = _cat[nextId];

        if (catNext.Count > nextTimestep && catNext[nextTimestep])
            return true;

        if (currId != nextId &&
            catNext.Count >= nextTimestep &&
            _cat[currId].Count > nextTimestep &&
            catNext[nextTimestep - 1] &&
            _cat[currId][nextTimestep])
            return true;

        if (_catGoals[nextId] < nextTimestep)
            return true;

        return false;
    }

    /// <summary>仅检查边冲突（交换位置）</summary>
    public bool HasEdgeConflict(int currId, int nextId, int nextTimestep)
    {
        if (_cat.Count == 0 || currId == nextId) return false;
        var catNext = _cat[nextId];
        return catNext.Count >= nextTimestep &&
               _cat[currId].Count > nextTimestep &&
               catNext[nextTimestep - 1] &&
               _cat[currId][nextTimestep];
    }

    /// <summary>返回 loc 在时刻 t 之后（不含 t）在 CAT 中的未来冲突总数</summary>
    public int GetFutureNumOfCollisions(int loc, int t)
    {
        if (_cat.Count == 0) return 0;
        int rst = 0;
        var row = _cat[loc];
        for (int ts = t + 1; ts < row.Count; ts++)
            if (row[ts]) rst++;
        return rst;
    }

    // -------------------------------------------------------------------------
    // 插入约束（CT）
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从 CBS 节点向上遍历到根，将路径上所有属于 agent 的约束插入 CT。
    /// </summary>
    public void Insert2CT(HLNode node, int agent)
    {
        var curr = node;
        while (curr.Parent != null)
        {
            Insert2CT(curr.Constraints, agent);
            curr = curr.Parent;
        }
    }

    /// <summary>
    /// 将一组约束中属于 agent 的部分插入 CT。
    /// 根据约束类型分别处理：正约束转为地标，负约束转为禁止时间段。
    /// </summary>
    public void Insert2CT(IList<Constraint> constraints, int agent)
    {
        if (constraints.Count == 0) return;

        var first = constraints[0];
        int a = first.Agent, x = first.Loc1, y = first.Loc2, t = first.Timestep;
        var type = first.Type;

        switch (type)
        {
            case ConstraintType.LeqLength:
                // 该智能体路径长度 ≤ t；其他智能体不能在 t 之后占用 x
                if (agent == a)
                    LengthMax = Math.Min(LengthMax, t);
                else
                    InsertRangeIntoCT(x, t, Common.MaxTimestep);
                break;

            case ConstraintType.GLength:
                // 该智能体路径长度 ≥ t+1
                if (agent == a)
                    LengthMin = Math.Max(LengthMin, t + 1);
                break;

            case ConstraintType.PositiveVertex:
                if (agent == a)
                    InsertLandmark(x, t);
                else
                    InsertRangeIntoCT(x, t, t + 1);
                break;

            case ConstraintType.PositiveEdge:
                if (agent == a)
                {
                    InsertLandmark(x, t - 1);
                    InsertLandmark(y, t);
                }
                else
                {
                    InsertRangeIntoCT(x, t - 1, t);
                    InsertRangeIntoCT(y, t, t + 1);
                    InsertEdgeRangeIntoCT(y, x, t, t + 1);
                }
                break;

            case ConstraintType.Vertex:
                if (agent == a)
                    foreach (var c in constraints)
                        InsertRangeIntoCT(c.Loc1, c.Timestep, c.Timestep + 1);
                break;

            case ConstraintType.Edge:
                if (agent == a)
                    InsertEdgeRangeIntoCT(x, y, t, t + 1);
                break;

            case ConstraintType.Barrier:
                if (agent == a)
                    foreach (var c in constraints)
                        foreach (var (loc, ts) in DecodeBarrier(c.Loc1, c.Loc2, c.Timestep))
                            InsertRangeIntoCT(loc, ts, ts + 1);
                break;

            case ConstraintType.Range:
                // 智能体不能在时间段 [y, t] 内经过 x
                if (agent == a)
                    InsertRangeIntoCT(x, y, t + 1);
                break;
        }
    }

    /// <summary>
    /// 将一条路径插入 CT，用于 SIPP 中把已知路径作为约束。
    /// 对路径上每次位置变化记录顶点约束和边约束，终点记录到 MaxTimestep。
    /// </summary>
    public void Insert2CT(Path path)
    {
        int prevLoc = path[0].location;
        int prevTs  = 0;

        for (int ts = 0; ts < path.Count; ts++)
        {
            int currLoc = path[ts].location;
            if (prevLoc != currLoc)
            {
                InsertRangeIntoCT(prevLoc, prevTs, ts);
                InsertEdgeRangeIntoCT(currLoc, prevLoc, ts, ts + 1);
                prevLoc = currLoc;
                prevTs  = ts;
            }
        }
        // 终点：从最后一次到达时刻起永久占用
        InsertRangeIntoCT(path[^1].location, path.Count - 1, Common.MaxTimestep);
    }

    // -------------------------------------------------------------------------
    // 插入冲突规避表（CAT）
    // -------------------------------------------------------------------------

    /// <summary>将除 agent 以外所有智能体的路径插入 CAT</summary>
    public void Insert2CAT(int agent, IList<Path?> paths)
    {
        for (int ag = 0; ag < paths.Count; ag++)
        {
            if (ag == agent || paths[ag] == null) continue;
            Insert2CAT(paths[ag]!);
        }
    }

    // -------------------------------------------------------------------------
    // 私有辅助方法
    // -------------------------------------------------------------------------

    /// <summary>将顶点约束 [tMin, tMax) 插入 CT</summary>
    private void InsertRangeIntoCT(int loc, int tMin, int tMax)
    {
        if (!_ct.TryGetValue(loc, out var list))
        {
            list = new List<(int, int)>();
            _ct[loc] = list;
        }
        list.Add((tMin, tMax));

        // 更新 CtMaxTimestep
        if (tMax < Common.MaxTimestep)
            CtMaxTimestep = Math.Max(CtMaxTimestep, tMax);
        else
            CtMaxTimestep = Math.Max(CtMaxTimestep, tMin);
    }

    /// <summary>将边约束 (from→to) 在 [tMin, tMax) 插入 CT（公开版，供走廊推理等模块使用）</summary>
    public void InsertEdge(int from, int to, int tMin, int tMax)
        => InsertEdgeRangeIntoCT(from, to, tMin, tMax);

    /// <summary>将边约束 (from→to) 在 [tMin, tMax) 插入 CT</summary>
    private void InsertEdgeRangeIntoCT(int from, int to, int tMin, int tMax)
        => InsertRangeIntoCT(GetEdgeIndex(from, to), tMin, tMax);

    /// <summary>插入地标约束：智能体在时刻 t 必须在 loc</summary>
    private void InsertLandmark(int loc, int t)
    {
        if (_landmarks.TryGetValue(t, out int existing))
        {
            // 同一时刻只能有一个地标，且必须一致
            if (existing != loc)
                throw new InvalidOperationException($"地标冲突：时刻 {t} 已有地标 {existing}，无法再设为 {loc}");
        }
        else
        {
            _landmarks[t] = loc;
        }
    }

    /// <summary>将一条路径插入 CAT</summary>
    private void Insert2CAT(Path path)
    {
        if (_cat.Count == 0)
        {
            _cat = Enumerable.Range(0, _mapSize).Select(_ => new List<bool>()).ToList();
            _catGoals = new int[_mapSize];
            Array.Fill(_catGoals, Common.MaxTimestep);
        }

        int goalLoc = path[^1].location;
        _catGoals[goalLoc] = path.Count - 1;

        for (int ts = path.Count - 1; ts >= 0; ts--)
        {
            int loc = path[ts].location;
            var row = _cat[loc];
            if (row.Count <= ts)
            {
                // 扩展到 ts+1，填 false
                while (row.Count <= ts) row.Add(false);
            }
            row[ts] = true;
        }

        CatMaxTimestep = Math.Max(CatMaxTimestep, path.Count - 1);
    }

    /// <summary>
    /// 解码屏障约束，返回屏障上每个 (位置, 时间步) 对，按时间步升序。
    /// 屏障是矩形边界上的一段，B1/B2 是两个端点的线性化坐标，t 是到达时刻。
    /// </summary>
    private List<(int Loc, int Timestep)> DecodeBarrier(int b1, int b2, int t)
    {
        var rst = new List<(int, int)>();
        int x1 = b1 / _numCol, y1 = b1 % _numCol;
        int x2 = b2 / _numCol, y2 = b2 % _numCol;

        if (x1 == x2) // 同行，沿列方向
        {
            int steps = Math.Min(Math.Abs(y2 - y1), t);
            int dir   = y1 < y2 ? -1 : 1; // 从 y2 向 y1 方向回溯
            for (int i = steps; i >= 0; i--)
                rst.Add((x1 * _numCol + y2 + dir * i, t - i));
        }
        else // 同列，沿行方向
        {
            int steps = Math.Min(Math.Abs(x2 - x1), t);
            int dir   = x1 < x2 ? -1 : 1;
            for (int i = steps; i >= 0; i--)
                rst.Add(((x2 + dir * i) * _numCol + y1, t - i));
        }

        return rst;
    }

    /// <summary>
    /// 计算边 (from→to) 的唯一索引，用于在 CT 中存储边约束。
    /// 公式与 C++ 原版一致：(1 + from) * mapSize + to
    /// </summary>
    private int GetEdgeIndex(int from, int to) => (1 + from) * _mapSize + to;
}
