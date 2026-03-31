namespace DotNetEECBS;

using Path = List<PathEntry>;

/// <summary>
/// 矩形推理（Rectangle Reasoning）：
/// 检测两个智能体是否在矩形区域内发生冲突，
/// 用屏障约束（BARRIER）替代普通顶点约束，大幅减少搜索树分支。
/// 使用 RM（Rectangle with MDD）算法。
/// </summary>
public class RectangleReasoning
{
    public double AccumulatedRuntime;

    private readonly Instance _instance;

    public RectangleReasoning(Instance instance)
    {
        _instance = instance;
    }

    // -------------------------------------------------------------------------
    // 公开接口
    // -------------------------------------------------------------------------

    public Conflict? Run(IList<Path?> paths, int timestep, int a1, int a2, MDD mdd1, MDD mdd2)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = FindRectangleConflictByRM(paths, timestep, a1, a2, mdd1, mdd2);
        AccumulatedRuntime += sw.Elapsed.TotalSeconds;
        return result;
    }

    // -------------------------------------------------------------------------
    // RM 算法：枚举 MDD 中的单节点层作为起终点候选
    // -------------------------------------------------------------------------

    public Conflict? FindRectangleConflictByRM(IList<Path?> paths, int timestep,
        int a1, int a2, MDD mdd1, MDD mdd2)
    {
        var p1 = paths[a1]!;
        var p2 = paths[a2]!;

        var s1s = GetStartCandidates(p1, mdd1, timestep);
        var g1s = GetGoalCandidates(p1, mdd1, timestep);
        var s2s = GetStartCandidates(p2, mdd2, timestep);
        var g2s = GetGoalCandidates(p2, mdd2, timestep);

        var location = _instance.GetCoordinate(p1[timestep].location);

        Conflict? best = null;
        int bestType = -1, bestArea = 0;

        foreach (int t1s in s1s)
        foreach (int t1e in g1s)
        {
            var s1 = _instance.GetCoordinate(p1[t1s].location);
            var g1 = _instance.GetCoordinate(p1[t1e].location);
            if (_instance.GetManhattanDistance(
                    _instance.LinearizeCoordinate(s1.Row, s1.Col),
                    _instance.LinearizeCoordinate(g1.Row, g1.Col)) != t1e - t1s)
                continue;

            foreach (int t2s in s2s)
            foreach (int t2e in g2s)
            {
                var s2 = _instance.GetCoordinate(p2[t2s].location);
                var g2 = _instance.GetCoordinate(p2[t2e].location);
                if (_instance.GetManhattanDistance(
                        _instance.LinearizeCoordinate(s2.Row, s2.Col),
                        _instance.LinearizeCoordinate(g2.Row, g2.Col)) != t2e - t2s)
                    continue;

                if (!IsRectangleConflictRM(s1, s2, g1, g2)) continue;

                var rg = GetRg(s1, g1, g2);
                var rs = GetRs(s1, s2, g1);
                int newArea = (Math.Abs(rs.Row - rg.Row) + 1) * (Math.Abs(rs.Col - rg.Col) + 1);
                int newType = ClassifyRectangleConflictRM(s1, s2, g1, g2, rg);

                if (newType > bestType || (newType == bestType && newArea > bestArea))
                {
                    int rgT = timestep +
                              Math.Abs(rg.Row - location.Row) +
                              Math.Abs(rg.Col - location.Col);

                    var c1 = new List<Constraint>();
                    var c2 = new List<Constraint>();
                    bool ok = AddModifiedBarrierConstraints(
                        a1, a2, rs, rg, s1, s2, rgT, mdd1, mdd2, c1, c2);

                    if (ok && Blocked(p1, c1) && Blocked(p2, c2))
                    {
                        bestType = newType;
                        bestArea = newArea;
                        best     = new Conflict();
                        best.SetRectangleConflict(a1, a2,
                            (_instance.LinearizeCoordinate(rs.Row, rs.Col),
                             _instance.LinearizeCoordinate(rs.Row, rs.Col)),
                            (_instance.LinearizeCoordinate(rg.Row, rg.Col),
                             _instance.LinearizeCoordinate(rg.Row, rg.Col)),
                            rgT, c1, c2);
                        best.Priority = newType switch
                        {
                            2 => ConflictPriority.Cardinal,
                            1 => ConflictPriority.Semi,
                            _ => ConflictPriority.Non
                        };
                        if (bestType == 2) return best;
                    }
                }
            }
        }
        return best;
    }

    public Conflict? FindRectangleConflictByGR(IList<Path?> paths, int timestep,
        int a1, int a2, MDD mdd1, MDD mdd2)
    {
        // GR 算法（Generalized Rectangle）暂不实现，RM 已覆盖主要场景
        return null;
    }

    // -------------------------------------------------------------------------
    // 矩形冲突判断（RM 版本）
    // -------------------------------------------------------------------------

    private bool IsRectangleConflictRM(
        (int Row, int Col) s1, (int Row, int Col) s2,
        (int Row, int Col) g1, (int Row, int Col) g2)
    {
        if (s1 == s2) return false;
        if (s1 == g1 || s2 == g2) return false;
        if ((s1.Row - g1.Row) * (s2.Row - g2.Row) < 0) return false;
        if ((s1.Col - g1.Col) * (s2.Col - g2.Col) < 0) return false;
        // 排除纯顶点冲突
        return !((s1.Row == g1.Row && s2.Col == g2.Col) ||
                 (s1.Col == g1.Col && s2.Row == g2.Row));
    }

    // -------------------------------------------------------------------------
    // 矩形冲突分类（RM 版本）
    // -------------------------------------------------------------------------

    private int ClassifyRectangleConflictRM(
        (int Row, int Col) s1, (int Row, int Col) s2,
        (int Row, int Col) g1, (int Row, int Col) g2,
        (int Row, int Col) rg)
    {
        // s1 在中间
        if ((s2.Row - s1.Row) * (s1.Row - g1.Row) < 0 &&
            (s2.Col - s1.Col) * (s1.Col - g1.Col) < 0) return 0;
        // s2 在中间
        if ((s1.Row - s2.Row) * (s2.Row - g2.Row) < 0 &&
            (s1.Col - s2.Col) * (s2.Col - g2.Col) < 0) return 0;

        int c1 = 0, c2 = 0;
        if ((s1.Row == s2.Row && (s1.Col - s2.Col) * (s2.Col - rg.Col) >= 0) ||
            (s1.Row != s2.Row && (s1.Row - s2.Row) * (s2.Row - rg.Row) < 0))
        {
            if (rg.Row == g1.Row) c1 = 1;
            if (rg.Col == g2.Col) c2 = 1;
        }
        else
        {
            if (rg.Col == g1.Col) c1 = 1;
            if (rg.Row == g2.Row) c2 = 1;
        }
        return c1 + c2;
    }

    // -------------------------------------------------------------------------
    // 矩形角点计算
    // -------------------------------------------------------------------------

    private (int Row, int Col) GetRs(
        (int Row, int Col) s1, (int Row, int Col) s2, (int Row, int Col) g1)
    {
        int r = s1.Row == g1.Row ? s1.Row :
                s1.Row < g1.Row  ? Math.Max(s1.Row, s2.Row) :
                                   Math.Min(s1.Row, s2.Row);
        int c = s1.Col == g1.Col ? s1.Col :
                s1.Col < g1.Col  ? Math.Max(s1.Col, s2.Col) :
                                   Math.Min(s1.Col, s2.Col);
        return (r, c);
    }

    private (int Row, int Col) GetRg(
        (int Row, int Col) s1, (int Row, int Col) g1, (int Row, int Col) g2)
    {
        int r = s1.Row == g1.Row ? g1.Row :
                s1.Row < g1.Row  ? Math.Min(g1.Row, g2.Row) :
                                   Math.Max(g1.Row, g2.Row);
        int c = s1.Col == g1.Col ? g1.Col :
                s1.Col < g1.Col  ? Math.Min(g1.Col, g2.Col) :
                                   Math.Max(g1.Col, g2.Col);
        return (r, c);
    }

    // -------------------------------------------------------------------------
    // 起终点候选（MDD 中单节点层）
    // -------------------------------------------------------------------------

    private List<int> GetStartCandidates(Path path, MDD mdd, int timestep)
    {
        var result = new List<int>();
        for (int t = 0; t <= timestep && t < mdd.Levels.Count; t++)
        {
            if (mdd.Levels[t].Count == 1 &&
                mdd.Levels[t][0].Location == path[t].location &&
                _instance.GetManhattanDistance(path[t].location, path[timestep].location) == timestep - t)
                result.Add(t);
        }
        return result;
    }

    private List<int> GetGoalCandidates(Path path, MDD mdd, int timestep)
    {
        var result = new List<int>();
        for (int t = path.Count - 1; t >= timestep; t--)
        {
            if (t >= mdd.Levels.Count) continue;
            if (mdd.Levels[t].Count == 1 &&
                mdd.Levels[t][0].Location == path[t].location &&
                _instance.GetManhattanDistance(path[t].location, path[timestep].location) == t - timestep)
                result.Add(t);
        }
        return result;
    }

    // -------------------------------------------------------------------------
    // 屏障约束生成
    // -------------------------------------------------------------------------

    private bool AddModifiedBarrierConstraints(
        int a1, int a2,
        (int Row, int Col) rs, (int Row, int Col) rg,
        (int Row, int Col) s1, (int Row, int Col) s2,
        int rgT, MDD mdd1, MDD mdd2,
        List<Constraint> c1, List<Constraint> c2)
    {
        int rsLoc = _instance.LinearizeCoordinate(rs.Row, rs.Col);
        int rgLoc = _instance.LinearizeCoordinate(rg.Row, rg.Col);

        // s1 在中间
        if ((s2.Row - s1.Row) * (s1.Row - rg.Row) > 0 &&
            (s2.Col - s1.Col) * (s1.Col - rg.Col) > 0)
        {
            int rsT = rgT - _instance.GetManhattanDistance(rsLoc, rgLoc);
            int offset = rs.Row > rg.Row ? 1 : -1;
            if (!HasNodeOnBarrier(mdd2, rs.Col, rg.Col, rs.Row + offset, rsT - 1, horizontal: true))
            {
                return AddModifiedHorizontalBarrierConstraint(a1, mdd1, rg.Row, rs.Col, rg.Col, rgT, c1) &&
                       AddModifiedVerticalBarrierConstraint(a2, mdd2, rg.Col, rs.Row, rg.Row, rgT, c2);
            }
            offset = rs.Col > rg.Col ? 1 : -1;
            if (!HasNodeOnBarrier(mdd2, rs.Row, rg.Row, rs.Col + offset, rsT - 1, horizontal: false))
            {
                return AddModifiedVerticalBarrierConstraint(a1, mdd1, rg.Col, rs.Row, rg.Row, rgT, c1) &&
                       AddModifiedHorizontalBarrierConstraint(a2, mdd2, rg.Row, rs.Col, rg.Col, rgT, c2);
            }
            return false;
        }
        // s2 在中间
        if ((s1.Row - s2.Row) * (s2.Row - rg.Row) > 0 &&
            (s1.Col - s2.Col) * (s2.Col - rg.Col) > 0)
        {
            int rsT = rgT - _instance.GetManhattanDistance(rsLoc, rgLoc);
            int offset = rs.Row > rg.Row ? 1 : -1;
            if (!HasNodeOnBarrier(mdd1, rs.Col, rg.Col, rs.Row + offset, rsT - 1, horizontal: true))
            {
                return AddModifiedHorizontalBarrierConstraint(a1, mdd1, rg.Row, rs.Col, rg.Col, rgT, c1) &&
                       AddModifiedVerticalBarrierConstraint(a2, mdd2, rg.Col, rs.Row, rg.Row, rgT, c2);
            }
            offset = rs.Col > rg.Col ? 1 : -1;
            if (!HasNodeOnBarrier(mdd1, rs.Row, rg.Row, rs.Col + offset, rsT - 1, horizontal: false))
            {
                return AddModifiedVerticalBarrierConstraint(a1, mdd1, rg.Col, rs.Row, rg.Row, rgT, c1) &&
                       AddModifiedHorizontalBarrierConstraint(a2, mdd2, rg.Row, rs.Col, rg.Col, rgT, c2);
            }
            return false;
        }
        // 同行
        if (s1.Row == s2.Row)
        {
            if ((s1.Col - s2.Col) * (s2.Col - rg.Col) >= 0)
                return AddModifiedVerticalBarrierConstraint(a1, mdd1, rg.Col, rs.Row, rg.Row, rgT, c1) &&
                       AddModifiedHorizontalBarrierConstraint(a2, mdd2, rg.Row, rs.Col, rg.Col, rgT, c2);
            else
                return AddModifiedHorizontalBarrierConstraint(a1, mdd1, rg.Row, rs.Col, rg.Col, rgT, c1) &&
                       AddModifiedVerticalBarrierConstraint(a2, mdd2, rg.Col, rs.Row, rg.Row, rgT, c2);
        }
        // 一般情况
        if ((s1.Row - s2.Row) * (s2.Row - rg.Row) >= 0)
            return AddModifiedHorizontalBarrierConstraint(a1, mdd1, rg.Row, rs.Col, rg.Col, rgT, c1) &&
                   AddModifiedVerticalBarrierConstraint(a2, mdd2, rg.Col, rs.Row, rg.Row, rgT, c2);
        else
            return AddModifiedVerticalBarrierConstraint(a1, mdd1, rg.Col, rs.Row, rg.Row, rgT, c1) &&
                   AddModifiedHorizontalBarrierConstraint(a2, mdd2, rg.Row, rs.Col, rg.Col, rgT, c2);
    }

    /// <summary>添加水平屏障约束：行 x，列从 riY 到 rgY，终止时刻 rgT</summary>
    private bool AddModifiedHorizontalBarrierConstraint(
        int agent, MDD mdd, int x, int riY, int rgY, int rgT, List<Constraint> constraints)
    {
        int sign = riY < rgY ? 1 : -1;
        int riT  = rgT - Math.Abs(riY - rgY);
        int tMin = Math.Max(riT, 0);
        int tMax = Math.Min(rgT, mdd.Levels.Count - 1);
        int t1   = -1;

        for (int t2 = tMin; t2 <= tMax; t2++)
        {
            int loc = _instance.LinearizeCoordinate(x, riY + (t2 - riT) * sign);
            bool found = mdd.Levels[t2].Any(n => n.Location == loc);

            if (!found && t1 >= 0)
            {
                int l1 = _instance.LinearizeCoordinate(x, riY + (t1 - riT) * sign);
                int l2 = _instance.LinearizeCoordinate(x, riY + (t2 - 1 - riT) * sign);
                constraints.Add(new Constraint(agent, l1, l2, t2 - 1, ConstraintType.Barrier));
                t1 = -1;
            }
            else if (found && t1 < 0)
            {
                t1 = t2;
            }

            if (found && t2 == tMax)
            {
                int l1 = _instance.LinearizeCoordinate(x, riY + (t1 - riT) * sign);
                constraints.Add(new Constraint(agent, l1, loc, t2, ConstraintType.Barrier));
            }
        }
        return constraints.Count > 0;
    }

    /// <summary>添加垂直屏障约束：列 y，行从 riX 到 rgX，终止时刻 rgT</summary>
    private bool AddModifiedVerticalBarrierConstraint(
        int agent, MDD mdd, int y, int riX, int rgX, int rgT, List<Constraint> constraints)
    {
        int sign = riX < rgX ? 1 : -1;
        int riT  = rgT - Math.Abs(riX - rgX);
        int tMin = Math.Max(riT, 0);
        int tMax = Math.Min(rgT, mdd.Levels.Count - 1);
        int t1   = -1;

        for (int t2 = tMin; t2 <= tMax; t2++)
        {
            int loc = _instance.LinearizeCoordinate(riX + (t2 - riT) * sign, y);
            bool found = mdd.Levels[t2].Any(n => n.Location == loc);

            if (!found && t1 >= 0)
            {
                int l1 = _instance.LinearizeCoordinate(riX + (t1 - riT) * sign, y);
                int l2 = _instance.LinearizeCoordinate(riX + (t2 - 1 - riT) * sign, y);
                constraints.Add(new Constraint(agent, l1, l2, t2 - 1, ConstraintType.Barrier));
                t1 = -1;
            }
            else if (found && t1 < 0)
            {
                t1 = t2;
            }

            if (found && t2 == tMax)
            {
                int l1 = _instance.LinearizeCoordinate(riX + (t1 - riT) * sign, y);
                constraints.Add(new Constraint(agent, l1, loc, t2, ConstraintType.Barrier));
            }
        }
        return constraints.Count > 0;
    }

    // -------------------------------------------------------------------------
    // 辅助方法
    // -------------------------------------------------------------------------

    /// <summary>检查 MDD 在屏障上是否有节点（用于判断 s1/s2 在中间的情况）</summary>
    private bool HasNodeOnBarrier(MDD mdd, int yStart, int yEnd, int x, int tMin, bool horizontal)
    {
        int sign = yStart < yEnd ? 1 : -1;
        int tMax = tMin + Math.Abs(yStart - yEnd);
        for (int t = tMin + 1; t <= Math.Min(tMax, mdd.Levels.Count - 1); t++)
        {
            int loc = horizontal
                ? _instance.LinearizeCoordinate(x, yStart + (t - tMin) * sign)
                : _instance.LinearizeCoordinate(yStart + (t - tMin) * sign, x);
            if (mdd.Levels[t].Any(n => n.Location == loc)) return true;
        }
        return false;
    }

    /// <summary>判断路径是否被屏障约束列表阻塞</summary>
    private bool Blocked(Path path, List<Constraint> constraints)
    {
        foreach (var c in constraints)
        {
            int x1 = _instance.GetRowCoordinate(c.Loc1), y1 = _instance.GetColCoordinate(c.Loc1);
            int x2 = _instance.GetRowCoordinate(c.Loc2), y2 = _instance.GetColCoordinate(c.Loc2);
            int t  = c.Timestep;

            if (x1 == x2) // 水平屏障
            {
                int sign = y1 < y2 ? 1 : -1;
                for (int i = 0; i <= Math.Min(Math.Abs(y2 - y1), t); i++)
                    if (Traverse(path, _instance.LinearizeCoordinate(x1, y2 - sign * i), t - i))
                        return true;
            }
            else // 垂直屏障
            {
                int sign = x1 < x2 ? 1 : -1;
                for (int i = 0; i <= Math.Min(Math.Abs(x2 - x1), t); i++)
                    if (Traverse(path, _instance.LinearizeCoordinate(x2 - sign * i, y1), t - i))
                        return true;
            }
        }
        return false;
    }

    /// <summary>判断路径在时刻 t 是否经过 loc</summary>
    private static bool Traverse(Path path, int loc, int t)
    {
        if (t >= path.Count) return loc == path[^1].location;
        return t >= 0 && path[t].location == loc;
    }
}
