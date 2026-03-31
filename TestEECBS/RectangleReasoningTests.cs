namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// RectangleReasoning 单元测试。
/// </summary>
[TestFixture]
public class RectangleReasoningTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    // -------------------------------------------------------------------------
    // CBS 集成测试：开启矩形推理后结果仍然正确
    // -------------------------------------------------------------------------

    [Test]
    public void CBS开启矩形推理_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.RectangleReasoning = true;
        cbs.PC                 = true;

        bool solved = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启矩形推理_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.RectangleReasoning = true;
        cbs.PC                 = true;

        bool solved = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启矩形推理_5智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);
        var cbs      = new CBS(instance, sipp: false);
        cbs.RectangleReasoning = true;
        cbs.PC                 = true;

        bool solved = cbs.Solve(timeLimit: 60.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启矩形推理_解代价应与不开启时一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);

        var cbsBase = new CBS(instance, sipp: false);
        cbsBase.Solve(timeLimit: 30.0);
        int baseCost = cbsBase.SolutionCost;

        var cbsRect = new CBS(instance, sipp: false);
        cbsRect.RectangleReasoning = true;
        cbsRect.PC                 = true;
        cbsRect.Solve(timeLimit: 30.0);

        Assert.That(cbsRect.SolutionCost, Is.EqualTo(baseCost),
            "矩形推理不应改变最优解代价");
    }

    // -------------------------------------------------------------------------
    // 同时开启走廊 + 矩形推理
    // -------------------------------------------------------------------------

    [Test]
    public void CBS同时开启走廊和矩形推理_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.CorridorReasoning  = true;
        cbs.RectangleReasoning = true;
        cbs.PC                 = true;

        bool solved = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS同时开启走廊和矩形推理_5智能体_解代价应与基准一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);

        var cbsBase = new CBS(instance, sipp: false);
        cbsBase.Solve(timeLimit: 60.0);
        int baseCost = cbsBase.SolutionCost;

        var cbsFull = new CBS(instance, sipp: false);
        cbsFull.CorridorReasoning  = true;
        cbsFull.RectangleReasoning = true;
        cbsFull.PC                 = true;
        cbsFull.Solve(timeLimit: 60.0);

        Assert.That(cbsFull.SolutionCost, Is.EqualTo(baseCost));
    }

    // -------------------------------------------------------------------------
    // RectangleReasoning.Run 直接测试
    // -------------------------------------------------------------------------

    [Test]
    public void Run_非矩形冲突应返回null()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var rr       = new RectangleReasoning(instance);

        // 构造两条平行路径（不会形成矩形冲突）
        var p1 = new Path { new PathEntry(0), new PathEntry(1), new PathEntry(2) };
        var p2 = new Path { new PathEntry(32), new PathEntry(33), new PathEntry(34) };

        var initialCTs = Enumerable.Range(0, 2)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize))
            .ToList();
        var solvers = Enumerable.Range(0, 2)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i))
            .ToList();
        var mddTable = new MDDTable(initialCTs, solvers);
        mddTable.Init(2);

        var node = new DummyHLNode();
        var mdd1 = mddTable.GetMDD(node, 0);
        var mdd2 = mddTable.GetMDD(node, 1);

        var paths  = new List<Path?> { p1, p2 };
        var result = rr.Run(paths, 1, 0, 1, mdd1, mdd2);

        // 平行路径不构成矩形冲突
        Assert.That(result, Is.Null);
    }

    [Test]
    public void AccumulatedRuntime_调用Run后应增加()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var rr       = new RectangleReasoning(instance);
        double before = rr.AccumulatedRuntime;

        var initialCTs = Enumerable.Range(0, 2)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize))
            .ToList();
        var solvers = Enumerable.Range(0, 2)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i))
            .ToList();
        var mddTable = new MDDTable(initialCTs, solvers);
        mddTable.Init(2);

        var node = new DummyHLNode();
        var mdd1 = mddTable.GetMDD(node, 0);
        var mdd2 = mddTable.GetMDD(node, 1);

        var p1    = new Path { new PathEntry(0), new PathEntry(1) };
        var p2    = new Path { new PathEntry(0), new PathEntry(32) };
        var paths = new List<Path?> { p1, p2 };

        rr.Run(paths, 1, 0, 1, mdd1, mdd2);

        Assert.That(rr.AccumulatedRuntime, Is.GreaterThanOrEqualTo(before));
    }

    // -------------------------------------------------------------------------
    // ECBS 集成测试
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS开启矩形推理_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        ecbs.RectangleReasoning = true;

        bool solved = ecbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(ecbs.ValidateEcbsSolution(), Is.True);
    }
}
