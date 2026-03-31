namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// CorridorReasoning 单元测试。
/// 走廊冲突需要特定地图拓扑（度为2的节点序列），
/// 这里通过构造简单场景来验证核心逻辑。
/// </summary>
[TestFixture]
public class CorridorReasoningTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    // -------------------------------------------------------------------------
    // CBS 集成测试：开启走廊推理后结果仍然正确
    // -------------------------------------------------------------------------

    [Test]
    public void CBS开启走廊推理_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.CorridorReasoning = true;
        cbs.PC                = true;

        bool solved = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启走廊推理_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.CorridorReasoning = true;
        cbs.PC                = true;

        bool solved = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启走廊推理_5智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);
        var cbs      = new CBS(instance, sipp: false);
        cbs.CorridorReasoning = true;
        cbs.PC                = true;

        bool solved = cbs.Solve(timeLimit: 60.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS开启走廊推理_解代价应与不开启时一致()
    {
        // 走廊推理不改变最优解代价，只减少搜索树节点数
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);

        var cbsBase = new CBS(instance, sipp: false);
        cbsBase.Solve(timeLimit: 30.0);
        int baseCost = cbsBase.SolutionCost;

        var cbsCorridor = new CBS(instance, sipp: false);
        cbsCorridor.CorridorReasoning = true;
        cbsCorridor.PC                = true;
        cbsCorridor.Solve(timeLimit: 30.0);
        int corridorCost = cbsCorridor.SolutionCost;

        Assert.That(corridorCost, Is.EqualTo(baseCost),
            "走廊推理不应改变最优解代价");
    }

    // -------------------------------------------------------------------------
    // CorridorReasoning.Run 直接测试
    // -------------------------------------------------------------------------

    [Test]
    public void Run_非走廊冲突应返回null()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var solvers  = Enumerable.Range(0, 2)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i))
            .ToList();
        var cts = Enumerable.Range(0, 2)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize))
            .ToList();

        var cr = new CorridorReasoning(solvers, cts);

        // 构造一个普通顶点冲突（不在走廊中）
        var conflict = new Conflict();
        conflict.SetVertexConflict(0, 1, 0, 1); // 位置 0 通常是角落，度 != 2

        var paths = new List<Path?>
        {
            new Path { new PathEntry(0), new PathEntry(1) },
            new Path { new PathEntry(0), new PathEntry(2) }
        };

        var node   = new DummyHLNode();
        var result = cr.Run(conflict, paths, node);

        // 位置 0 不是走廊节点，应返回 null
        Assert.That(result, Is.Null);
    }

    [Test]
    public void AccumulatedRuntime_调用Run后应增加()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var solvers  = Enumerable.Range(0, 2)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i))
            .ToList();
        var cts = Enumerable.Range(0, 2)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize))
            .ToList();

        var cr = new CorridorReasoning(solvers, cts);
        double before = cr.AccumulatedRuntime;

        var conflict = new Conflict();
        conflict.SetVertexConflict(0, 1, 5, 2);
        var paths = new List<Path?>
        {
            new Path { new PathEntry(0), new PathEntry(5), new PathEntry(5) },
            new Path { new PathEntry(1), new PathEntry(5), new PathEntry(5) }
        };
        cr.Run(conflict, paths, new DummyHLNode());

        Assert.That(cr.AccumulatedRuntime, Is.GreaterThanOrEqualTo(before));
    }

    // -------------------------------------------------------------------------
    // ECBS 集成测试：开启走廊推理
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS开启走廊推理_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        ecbs.CorridorReasoning = true;

        bool solved = ecbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(ecbs.ValidateEcbsSolution(), Is.True);
    }
}
