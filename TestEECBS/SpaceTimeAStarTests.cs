namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// SingleAgentSolver 基类方法 + SpaceTimeAStar 的单元测试。
/// 使用 random-32-32-20.map 和 random-32-32-20-random-1.scen。
/// </summary>
[TestFixture]
public class SpaceTimeAStarTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    private Instance _instance = null!;

    [SetUp]
    public void SetUp()
    {
        // 加载 5 个智能体
        _instance = new Instance(MapFile, ScenFile, numAgents: 5);
    }

    // -------------------------------------------------------------------------
    // ComputeHeuristics（反向 Dijkstra）
    // -------------------------------------------------------------------------

    [Test]
    public void 启发式_终点到自身距离应为0()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        Assert.That(solver.MyHeuristic[solver.GoalLocation], Is.EqualTo(0));
    }

    [Test]
    public void 启发式_起点到终点距离应大于等于曼哈顿距离()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        int h      = solver.MyHeuristic[solver.StartLocation];
        int manhattan = _instance.GetManhattanDistance(solver.StartLocation, solver.GoalLocation);
        Assert.That(h, Is.GreaterThanOrEqualTo(manhattan));
    }

    [Test]
    public void 启发式_不可达位置应为MaxTimestep()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        // 找一个障碍物位置
        for (int i = 0; i < _instance.MapSize; i++)
        {
            if (_instance.IsObstacle(i))
            {
                Assert.That(solver.MyHeuristic[i], Is.EqualTo(Common.MaxTimestep));
                return;
            }
        }
        Assert.Ignore("地图中没有障碍物");
    }

    // -------------------------------------------------------------------------
    // GetNextLocations / GetNeighbors
    // -------------------------------------------------------------------------

    [Test]
    public void GetNextLocations_应包含自身()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        int loc    = solver.StartLocation;
        var next   = solver.GetNextLocations(loc);
        Assert.That(next, Contains.Item(loc));
    }

    [Test]
    public void GetNextLocations_数量应比邻居多1()
    {
        var solver    = new SpaceTimeAStar(_instance, 0);
        int loc       = solver.StartLocation;
        var neighbors = solver.GetNeighbors(loc);
        var next      = solver.GetNextLocations(loc);
        Assert.That(next.Count, Is.EqualTo(neighbors.Count + 1));
    }

    // -------------------------------------------------------------------------
    // ComputeHeuristic
    // -------------------------------------------------------------------------

    [Test]
    public void ComputeHeuristic_同一位置应为0()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        Assert.That(solver.ComputeHeuristic(solver.GoalLocation, solver.GoalLocation), Is.EqualTo(0));
    }

    [Test]
    public void ComputeHeuristic_应大于等于曼哈顿距离()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        int h      = solver.ComputeHeuristic(solver.StartLocation, solver.GoalLocation);
        int manhattan = _instance.GetManhattanDistance(solver.StartLocation, solver.GoalLocation);
        Assert.That(h, Is.GreaterThanOrEqualTo(manhattan));
    }

    // -------------------------------------------------------------------------
    // FindOptimalPath（无约束）
    // -------------------------------------------------------------------------

    [Test]
    public void 无约束寻路_应找到路径()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        Assert.That(path.Count, Is.GreaterThan(0));
    }

    [Test]
    public void 无约束寻路_路径起点应为起始位置()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        Assert.That(path[0].location, Is.EqualTo(solver.StartLocation));
    }

    [Test]
    public void 无约束寻路_路径终点应为目标位置()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        Assert.That(path[^1].location, Is.EqualTo(solver.GoalLocation));
    }

    [Test]
    public void 无约束寻路_路径长度应大于等于启发式值()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        int h    = solver.MyHeuristic[solver.StartLocation];
        // 路径步数 = path.Count - 1（节点数比步数多1）
        Assert.That(path.Count - 1, Is.GreaterThanOrEqualTo(h));
    }

    [Test]
    public void 无约束寻路_路径中每步应合法()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        for (int i = 1; i < path.Count; i++)
            Assert.That(_instance.ValidMove(path[i - 1].location, path[i].location), Is.True,
                $"步骤 {i}: {path[i-1].location} → {path[i].location} 不合法");
    }

    [Test]
    public void 多个智能体无约束寻路_均应找到路径()
    {
        var ct    = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node  = new DummyHLNode();

        for (int ag = 0; ag < _instance.NumOfAgents; ag++)
        {
            var solver = new SpaceTimeAStar(_instance, ag);
            var path   = solver.FindOptimalPath(node, ct, paths, ag, 0);
            Assert.That(path.Count, Is.GreaterThan(0), $"agent {ag} 未找到路径");
            Assert.That(path[^1].location, Is.EqualTo(solver.GoalLocation), $"agent {ag} 终点错误");
        }
    }

    // -------------------------------------------------------------------------
    // FindOptimalPath（带顶点约束）
    // -------------------------------------------------------------------------

    [Test]
    public void 带顶点约束_路径不应经过被约束位置()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct0    = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node0  = new DummyHLNode();

        // 先找无约束路径，取 t=1 的位置施加约束
        var freePath = solver.FindOptimalPath(node0, ct0, paths, 0, 0);
        if (freePath.Count < 3) Assert.Ignore("路径太短，跳过");

        int blockedLoc = freePath[1].location;
        int blockedT   = 1;

        // 直接向约束表插入顶点约束（不依赖 HLNode 遍历）
        var ct2 = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        ct2.Insert2CT(new List<Constraint>
        {
            new Constraint(0, blockedLoc, -1, blockedT, ConstraintType.Vertex)
        }, 0);

        var path = solver.FindOptimalPath(node0, ct2, paths, 0, 0);

        if (path.Count > 1)
            Assert.That(path[1].location, Is.Not.EqualTo(blockedLoc));
    }

    // -------------------------------------------------------------------------
    // GetTravelTime
    // -------------------------------------------------------------------------

    [Test]
    public void GetTravelTime_起终点相同应为0()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        int time   = solver.GetTravelTime(solver.StartLocation, solver.StartLocation, ct, Common.MaxTimestep);
        Assert.That(time, Is.EqualTo(0));
    }

    [Test]
    public void GetTravelTime_应大于等于曼哈顿距离()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        int time   = solver.GetTravelTime(solver.StartLocation, solver.GoalLocation, ct, Common.MaxTimestep);
        int manhattan = _instance.GetManhattanDistance(solver.StartLocation, solver.GoalLocation);
        Assert.That(time, Is.GreaterThanOrEqualTo(manhattan));
    }

    [Test]
    public void GetTravelTime_应与FindOptimalPath路径长度一致()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        var path = solver.FindOptimalPath(node, ct, paths, 0, 0);
        int time = solver.GetTravelTime(solver.StartLocation, solver.GoalLocation,
                       new ConstraintTable(_instance.NumOfCols, _instance.MapSize), Common.MaxTimestep);

        Assert.That(time, Is.EqualTo(path.Count - 1));
    }

    // -------------------------------------------------------------------------
    // 次优搜索（w > 1）
    // -------------------------------------------------------------------------

    [Test]
    public void 次优搜索_路径长度不超过最优路径的w倍()
    {
        var solver = new SpaceTimeAStar(_instance, 0);
        var ct     = new ConstraintTable(_instance.NumOfCols, _instance.MapSize);
        var paths  = new List<Path?>(new Path?[_instance.NumOfAgents]);
        var node   = new DummyHLNode();

        double w = 1.5;
        var (optPath, _)  = solver.FindSuboptimalPath(node, ct, paths, 0, 0, 1.0);
        var (subPath, _)  = solver.FindSuboptimalPath(node, ct, paths, 0, 0, w);

        Assert.That(subPath.Count, Is.GreaterThan(0));
        Assert.That(subPath.Count - 1, Is.LessThanOrEqualTo((int)Math.Ceiling((optPath.Count - 1) * w) + 1));
    }
}

/// <summary>
/// 用于测试的最简 HLNode 实现（无约束、无父节点）。
/// </summary>
public class DummyHLNode : HLNode
{
    public DummyHLNode()
    {
        Constraints      = new List<Constraint>();
        Conflicts        = new List<Conflict>();
        UnknownConflicts = new List<Conflict>();
        Children         = new List<HLNode>();
        Parent           = null;
    }

    public override int        GetFHatVal()          => FVal;
    public override int        GetNumNewPaths()       => 0;
    public override IList<int> GetReplannedAgents()   => new List<int>();
    public override string     GetName()              => "DummyHLNode";
}
