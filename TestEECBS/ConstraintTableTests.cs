namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// ConstraintTable 单元测试。
/// 使用 32×32 地图（numCol=32, mapSize=1024）作为测试环境。
/// </summary>
[TestFixture]
public class ConstraintTableTests
{
    private const int NumCol  = 32;
    private const int MapSize = 32 * 32;

    private ConstraintTable _ct = null!;

    [SetUp]
    public void SetUp()
    {
        _ct = new ConstraintTable(NumCol, MapSize);
    }

    // 辅助：线性化坐标
    private static int Loc(int row, int col) => row * NumCol + col;

    // -------------------------------------------------------------------------
    // 初始状态
    // -------------------------------------------------------------------------

    [Test]
    public void 初始LengthMin应为0()
        => Assert.That(_ct.LengthMin, Is.EqualTo(0));

    [Test]
    public void 初始LengthMax应为MaxTimestep()
        => Assert.That(_ct.LengthMax, Is.EqualTo(Common.MaxTimestep));

    [Test]
    public void 初始状态任意位置不应被约束()
    {
        Assert.That(_ct.Constrained(Loc(5, 5), 3), Is.False);
        Assert.That(_ct.Constrained(Loc(0, 0), Loc(0, 1), 1), Is.False);
    }

    // -------------------------------------------------------------------------
    // InsertRangeIntoCT（通过 Insert2CT(constraints, agent) 间接测试）
    // -------------------------------------------------------------------------

    [Test]
    public void 顶点约束_在约束时间步内应被约束()
    {
        // 约束 agent=0 在 t=3 不能在 loc(2,2)
        var c = new List<Constraint> { new(0, Loc(2, 2), -1, 3, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(Loc(2, 2), 3), Is.True);
    }

    [Test]
    public void 顶点约束_约束时间步之外不应被约束()
    {
        var c = new List<Constraint> { new(0, Loc(2, 2), -1, 3, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(Loc(2, 2), 2), Is.False);
        Assert.That(_ct.Constrained(Loc(2, 2), 4), Is.False);
    }

    [Test]
    public void 顶点约束_其他智能体的约束不应影响当前智能体()
    {
        // agent=1 的约束，对 agent=0 无效
        var c = new List<Constraint> { new(1, Loc(2, 2), -1, 3, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(Loc(2, 2), 3), Is.False);
    }

    [Test]
    public void 边约束_在约束时间步内应被约束()
    {
        int from = Loc(1, 1), to = Loc(1, 2);
        var c = new List<Constraint> { new(0, from, to, 2, ConstraintType.Edge) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(from, to, 2), Is.True);
    }

    [Test]
    public void 边约束_反向边不应被约束()
    {
        int from = Loc(1, 1), to = Loc(1, 2);
        var c = new List<Constraint> { new(0, from, to, 2, ConstraintType.Edge) };
        _ct.Insert2CT(c, 0);

        // 反向 to→from 不受约束
        Assert.That(_ct.Constrained(to, from, 2), Is.False);
    }

    [Test]
    public void LeqLength约束_当前智能体LengthMax应被更新()
    {
        var c = new List<Constraint> { new(0, Loc(5, 5), -1, 8, ConstraintType.LeqLength) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.LengthMax, Is.EqualTo(8));
    }

    [Test]
    public void LeqLength约束_其他智能体应在该位置t之后被约束()
    {
        // agent=0 的 LeqLength 约束，对 agent=1 意味着 loc 在 t 之后被占用
        var c = new List<Constraint> { new(0, Loc(5, 5), -1, 8, ConstraintType.LeqLength) };
        _ct.Insert2CT(c, 1); // 以 agent=1 的视角插入

        Assert.That(_ct.Constrained(Loc(5, 5), 8),  Is.True);
        Assert.That(_ct.Constrained(Loc(5, 5), 100), Is.True);
        Assert.That(_ct.Constrained(Loc(5, 5), 7),  Is.False);
    }

    [Test]
    public void GLength约束_当前智能体LengthMin应被更新()
    {
        var c = new List<Constraint> { new(0, Loc(5, 5), -1, 5, ConstraintType.GLength) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.LengthMin, Is.EqualTo(6)); // t+1
    }

    [Test]
    public void Range约束_在时间段内应被约束()
    {
        // agent=0 不能在 [y=2, t=5] 内经过 loc(3,3)
        // 即 InsertRangeIntoCT(x, y, t+1) = InsertRangeIntoCT(loc, 2, 6)
        var c = new List<Constraint> { new(0, Loc(3, 3), 2, 5, ConstraintType.Range) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(Loc(3, 3), 2), Is.True);
        Assert.That(_ct.Constrained(Loc(3, 3), 5), Is.True);
        Assert.That(_ct.Constrained(Loc(3, 3), 6), Is.False);
        Assert.That(_ct.Constrained(Loc(3, 3), 1), Is.False);
    }

    [Test]
    public void PositiveVertex约束_当前智能体应插入地标()
    {
        var c = new List<Constraint> { new(0, Loc(4, 4), -1, 3, ConstraintType.PositiveVertex) };
        _ct.Insert2CT(c, 0);

        // 地标：t=3 必须在 loc(4,4)，所以其他位置在 t=3 被约束
        Assert.That(_ct.Constrained(Loc(4, 5), 3), Is.True);
        // 自身位置不被约束
        Assert.That(_ct.Constrained(Loc(4, 4), 3), Is.False);
    }

    [Test]
    public void PositiveVertex约束_其他智能体在该位置该时刻应被约束()
    {
        var c = new List<Constraint> { new(0, Loc(4, 4), -1, 3, ConstraintType.PositiveVertex) };
        _ct.Insert2CT(c, 1); // 以 agent=1 视角

        Assert.That(_ct.Constrained(Loc(4, 4), 3), Is.True);
    }

    // -------------------------------------------------------------------------
    // GetMaxTimestep
    // -------------------------------------------------------------------------

    [Test]
    public void GetMaxTimestep_无约束时应为0()
        => Assert.That(_ct.GetMaxTimestep(), Is.EqualTo(0));

    [Test]
    public void GetMaxTimestep_应反映CT中最大时间步()
    {
        var c = new List<Constraint> { new(0, Loc(1, 1), -1, 7, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0);

        Assert.That(_ct.GetMaxTimestep(), Is.GreaterThanOrEqualTo(8)); // t+1
    }

    // -------------------------------------------------------------------------
    // Insert2CT(Path)
    // -------------------------------------------------------------------------

    [Test]
    public void 路径插入CT_终点应被永久约束()
    {
        var path = new Path
        {
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 1)),
            new PathEntry(Loc(0, 2)),
        };
        _ct.Insert2CT(path);

        // 终点 loc(0,2) 从 t=2 起永久占用
        Assert.That(_ct.Constrained(Loc(0, 2), 2),   Is.True);
        Assert.That(_ct.Constrained(Loc(0, 2), 1000), Is.True);
    }

    [Test]
    public void 路径插入CT_中间位置应在对应时间段被约束()
    {
        var path = new Path
        {
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 1)),
            new PathEntry(Loc(0, 2)),
        };
        _ct.Insert2CT(path);

        // loc(0,0) 在 t=0 被占用（t=0 到 t=1 之前）
        Assert.That(_ct.Constrained(Loc(0, 0), 0), Is.True);
        Assert.That(_ct.Constrained(Loc(0, 0), 1), Is.False);
    }

    // -------------------------------------------------------------------------
    // Insert2CAT + GetNumOfConflictsForStep / HasConflictForStep
    // -------------------------------------------------------------------------

    [Test]
    public void CAT为空时冲突数应为0()
    {
        Assert.That(_ct.GetNumOfConflictsForStep(Loc(0,0), Loc(0,1), 1), Is.EqualTo(0));
        Assert.That(_ct.HasConflictForStep(Loc(0,0), Loc(0,1), 1), Is.False);
    }

    [Test]
    public void 插入CAT后顶点冲突应被检测到()
    {
        // agent=1 的路径经过 loc(1,1) 在 t=2
        var path = new Path
        {
            new PathEntry(Loc(1, 0)),
            new PathEntry(Loc(1, 1)),
            new PathEntry(Loc(1, 2)),
        };
        var paths = new List<Path?> { null, path };
        _ct.Insert2CAT(0, paths);

        // agent=0 想在 t=1 移动到 loc(1,1)，与 agent=1 冲突
        int conflicts = _ct.GetNumOfConflictsForStep(Loc(1, 0), Loc(1, 1), 1);
        Assert.That(conflicts, Is.GreaterThan(0));
        Assert.That(_ct.HasConflictForStep(Loc(1, 0), Loc(1, 1), 1), Is.True);
    }

    [Test]
    public void 插入CAT后目标占用冲突应被检测到()
    {
        // agent=1 的终点是 loc(2,2)，在 t=1 到达
        var path = new Path
        {
            new PathEntry(Loc(2, 1)),
            new PathEntry(Loc(2, 2)),
        };
        var paths = new List<Path?> { null, path };
        _ct.Insert2CAT(0, paths);

        // agent=0 在 t=5 到达 loc(2,2)，晚于 agent=1 的终点时刻 t=1，产生目标占用冲突
        Assert.That(_ct.HasConflictForStep(Loc(2, 1), Loc(2, 2), 5), Is.True);
    }

    [Test]
    public void Insert2CAT_跳过当前智能体自身路径()
    {
        var path = new Path
        {
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 1)),
        };
        // agent=0 的路径不应插入 CAT（自身）
        var paths = new List<Path?> { path };
        _ct.Insert2CAT(0, paths);

        Assert.That(_ct.HasConflictForStep(Loc(0, 0), Loc(0, 1), 1), Is.False);
    }

    // -------------------------------------------------------------------------
    // GetHoldingTime
    // -------------------------------------------------------------------------

    [Test]
    public void GetHoldingTime_无约束时应返回earliestTimestep()
    {
        Assert.That(_ct.GetHoldingTime(Loc(3, 3), 5), Is.EqualTo(5));
    }

    [Test]
    public void GetHoldingTime_有约束时应返回约束结束后的时间步()
    {
        // 约束 loc(3,3) 在 [2, 8) 不可用
        var c = new List<Constraint> { new(0, Loc(3, 3), -1, 7, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0); // 插入 [7, 8)

        // 从 t=0 开始，最早可持续占据的时刻是 8
        Assert.That(_ct.GetHoldingTime(Loc(3, 3), 0), Is.EqualTo(8));
    }

    // -------------------------------------------------------------------------
    // GetLastCollisionTimestep
    // -------------------------------------------------------------------------

    [Test]
    public void GetLastCollisionTimestep_CAT为空时应返回负1()
        => Assert.That(_ct.GetLastCollisionTimestep(Loc(0, 0)), Is.EqualTo(-1));

    [Test]
    public void GetLastCollisionTimestep_应返回最后一次冲突时间步()
    {
        var path = new Path
        {
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 1)),
            new PathEntry(Loc(0, 2)),
        };
        var paths = new List<Path?> { null, path };
        _ct.Insert2CAT(0, paths);

        // loc(0,1) 在 t=1 被占用，是最后一次
        Assert.That(_ct.GetLastCollisionTimestep(Loc(0, 1)), Is.EqualTo(1));
    }

    // -------------------------------------------------------------------------
    // 拷贝构造
    // -------------------------------------------------------------------------

    [Test]
    public void 拷贝构造_应复制所有约束()
    {
        var c = new List<Constraint> { new(0, Loc(1, 1), -1, 3, ConstraintType.Vertex) };
        _ct.Insert2CT(c, 0);

        var copy = new ConstraintTable(_ct);

        Assert.That(copy.Constrained(Loc(1, 1), 3), Is.True);
        Assert.That(copy.LengthMin, Is.EqualTo(_ct.LengthMin));
        Assert.That(copy.LengthMax, Is.EqualTo(_ct.LengthMax));
    }

    [Test]
    public void 拷贝构造_修改副本不影响原表()
    {
        var copy = new ConstraintTable(_ct);
        var c = new List<Constraint> { new(0, Loc(2, 2), -1, 5, ConstraintType.Vertex) };
        copy.Insert2CT(c, 0);

        Assert.That(_ct.Constrained(Loc(2, 2), 5), Is.False);
    }

    // -------------------------------------------------------------------------
    // GetFutureNumOfCollisions
    // -------------------------------------------------------------------------

    [Test]
    public void GetFutureNumOfCollisions_CAT为空时应为0()
        => Assert.That(_ct.GetFutureNumOfCollisions(Loc(0, 0), 0), Is.EqualTo(0));

    [Test]
    public void GetFutureNumOfCollisions_应统计t之后的冲突数()
    {
        var path = new Path
        {
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 0)),
            new PathEntry(Loc(0, 0)),
        };
        var paths = new List<Path?> { null, path };
        _ct.Insert2CAT(0, paths);

        // loc(0,0) 在 t=0,1,2 均被占用，t=0 之后有 2 次冲突
        Assert.That(_ct.GetFutureNumOfCollisions(Loc(0, 0), 0), Is.EqualTo(2));
    }
}
