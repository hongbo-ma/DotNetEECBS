namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// MDD 单元测试：验证 MDD 构建的正确性。
/// </summary>
[TestFixture]
public class MDDTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    private Instance         _instance = null!;
    private SpaceTimeAStar   _solver   = null!;
    private ConstraintTable  _emptyCT  = null!;

    [SetUp]
    public void SetUp()
    {
        _instance = new Instance(MapFile, ScenFile, numAgents: 3);
        _solver   = new SpaceTimeAStar(_instance, 0);
        _emptyCT  = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
    }

    // -------------------------------------------------------------------------
    // MDDNode
    // -------------------------------------------------------------------------

    [Test]
    public void MDDNode_构造函数应正确设置字段()
    {
        var n = new MDDNode(42, 3);
        Assert.That(n.Location, Is.EqualTo(42));
        Assert.That(n.Level,    Is.EqualTo(3));
        Assert.That(n.Children, Is.Empty);
        Assert.That(n.Parents,  Is.Empty);
    }

    [Test]
    public void MDDNode_从父节点构造Level应加1()
    {
        var parent = new MDDNode(10, 2);
        var child  = new MDDNode(11, parent);
        Assert.That(child.Level,          Is.EqualTo(3));
        Assert.That(child.Parents.Count,  Is.EqualTo(1));
        Assert.That(child.Parents[0],     Is.SameAs(parent));
    }

    // -------------------------------------------------------------------------
    // MDD.BuildMDD（固定层数版本）
    // -------------------------------------------------------------------------

    [Test]
    public void BuildMDD固定层数_应成功构建()
    {
        int optLen = _solver.MyHeuristic[_solver.StartLocation];
        var mdd    = new MDD();
        bool ok    = mdd.BuildMDD(_emptyCT, optLen + 1, _solver);
        Assert.That(ok, Is.True);
    }

    [Test]
    public void BuildMDD固定层数_层数应等于numLevels()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);
        Assert.That(mdd.Levels.Count, Is.EqualTo(numLevels));
    }

    [Test]
    public void BuildMDD固定层数_第0层应只含起点()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);
        Assert.That(mdd.Levels[0].Count,            Is.EqualTo(1));
        Assert.That(mdd.Levels[0][0].Location,      Is.EqualTo(_solver.StartLocation));
    }

    [Test]
    public void BuildMDD固定层数_最后一层应只含终点()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);
        Assert.That(mdd.Levels[^1].Count,       Is.EqualTo(1));
        Assert.That(mdd.Levels[^1][0].Location, Is.EqualTo(_solver.GoalLocation));
    }

    [Test]
    public void BuildMDD固定层数_每个节点的子节点层数应加1()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        for (int t = 0; t < mdd.Levels.Count - 1; t++)
            foreach (var node in mdd.Levels[t])
                foreach (var child in node.Children)
                    Assert.That(child.Level, Is.EqualTo(t + 1),
                        $"节点 ({node.Location},{t}) 的子节点层数应为 {t+1}");
    }

    // -------------------------------------------------------------------------
    // MDD.BuildMDD（最小 MDD 版本）
    // -------------------------------------------------------------------------

    [Test]
    public void BuildMDD最小版本_应成功构建()
    {
        var mdd = new MDD();
        bool ok = mdd.BuildMDD(_emptyCT, _solver);
        Assert.That(ok, Is.True);
    }

    [Test]
    public void BuildMDD最小版本_层数应等于最短路径长度加1()
    {
        var mdd = new MDD();
        mdd.BuildMDD(_emptyCT, _solver);
        int optLen = _solver.MyHeuristic[_solver.StartLocation];
        Assert.That(mdd.Levels.Count, Is.EqualTo(optLen + 1));
    }

    [Test]
    public void BuildMDD最小版本_第0层应只含起点()
    {
        var mdd = new MDD();
        mdd.BuildMDD(_emptyCT, _solver);
        Assert.That(mdd.Levels[0].Count,       Is.EqualTo(1));
        Assert.That(mdd.Levels[0][0].Location, Is.EqualTo(_solver.StartLocation));
    }

    [Test]
    public void BuildMDD最小版本_最后一层应只含终点()
    {
        var mdd = new MDD();
        mdd.BuildMDD(_emptyCT, _solver);
        Assert.That(mdd.Levels[^1].Count,       Is.EqualTo(1));
        Assert.That(mdd.Levels[^1][0].Location, Is.EqualTo(_solver.GoalLocation));
    }

    // -------------------------------------------------------------------------
    // MDD.Find
    // -------------------------------------------------------------------------

    [Test]
    public void Find_应返回正确节点()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        var found = mdd.Find(_solver.StartLocation, 0);
        Assert.That(found, Is.Not.Null);
        Assert.That(found!.Location, Is.EqualTo(_solver.StartLocation));
    }

    [Test]
    public void Find_不存在的节点应返回null()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        // 终点不在第 0 层（除非起终点相同）
        if (_solver.StartLocation != _solver.GoalLocation)
            Assert.That(mdd.Find(_solver.GoalLocation, 0), Is.Null);
    }

    // -------------------------------------------------------------------------
    // MDD 深拷贝
    // -------------------------------------------------------------------------

    [Test]
    public void 深拷贝_层数应相同()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        var copy = new MDD(mdd);
        Assert.That(copy.Levels.Count, Is.EqualTo(mdd.Levels.Count));
    }

    [Test]
    public void 深拷贝_修改副本不影响原MDD()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);
        int origCount = mdd.Levels[0].Count;

        var copy = new MDD(mdd);
        copy.Levels[0].Clear();

        Assert.That(mdd.Levels[0].Count, Is.EqualTo(origCount));
    }

    // -------------------------------------------------------------------------
    // MDD.Clear
    // -------------------------------------------------------------------------

    [Test]
    public void Clear_应清空所有层()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);
        mdd.Clear();
        Assert.That(mdd.Levels, Is.Empty);
    }

    // -------------------------------------------------------------------------
    // MDDTable
    // -------------------------------------------------------------------------

    [Test]
    public void MDDTable_GetMDD_应返回有效MDD()
    {
        var initialCTs = Enumerable.Range(0, _instance.NumOfAgents)
            .Select(_ => new ConstraintTable(_instance.NumOfCols, _instance.MapSize))
            .ToList();
        var solvers = Enumerable.Range(0, _instance.NumOfAgents)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(_instance, i))
            .ToList();

        var table = new MDDTable(initialCTs, solvers);
        table.Init(_instance.NumOfAgents);

        var node = new DummyHLNode();
        var mdd  = table.GetMDD(node, 0);

        Assert.That(mdd.Levels.Count, Is.GreaterThan(0));
        Assert.That(mdd.Levels[0][0].Location, Is.EqualTo(solvers[0].StartLocation));
    }

    [Test]
    public void MDDTable_GetMDD_第二次调用应命中缓存()
    {
        var initialCTs = Enumerable.Range(0, _instance.NumOfAgents)
            .Select(_ => new ConstraintTable(_instance.NumOfCols, _instance.MapSize))
            .ToList();
        var solvers = Enumerable.Range(0, _instance.NumOfAgents)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(_instance, i))
            .ToList();

        var table = new MDDTable(initialCTs, solvers);
        table.Init(_instance.NumOfAgents);

        var node  = new DummyHLNode();
        var mdd1  = table.GetMDD(node, 0);
        double t1 = table.AccumulatedRuntime;

        var mdd2  = table.GetMDD(node, 0);
        double t2 = table.AccumulatedRuntime;

        // 第二次命中缓存，运行时间不应增加
        Assert.That(t2, Is.EqualTo(t1));
        Assert.That(mdd1, Is.SameAs(mdd2));
    }

    // -------------------------------------------------------------------------
    // SyncMDD
    // -------------------------------------------------------------------------

    [Test]
    public void SyncMDD_从MDD构造_层数应相同()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        var sync = new SyncMDD(mdd);
        Assert.That(sync.Levels.Count, Is.EqualTo(mdd.Levels.Count));
    }

    [Test]
    public void SyncMDD_第0层应只含起点()
    {
        int numLevels = _solver.MyHeuristic[_solver.StartLocation] + 1;
        var mdd       = new MDD();
        mdd.BuildMDD(_emptyCT, numLevels, _solver);

        var sync = new SyncMDD(mdd);
        Assert.That(sync.Levels[0].Count,       Is.EqualTo(1));
        Assert.That(sync.Levels[0][0].Location, Is.EqualTo(_solver.StartLocation));
    }
}
