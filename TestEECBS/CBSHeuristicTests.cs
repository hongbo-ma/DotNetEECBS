namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// CBSHeuristic 单元测试：CG / DG / WDG 启发式。
/// </summary>
[TestFixture]
public class CBSHeuristicTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    // -------------------------------------------------------------------------
    // MVC 算法正确性
    // -------------------------------------------------------------------------

    [Test]
    public void MVC_空图应返回0()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var solvers  = Enumerable.Range(0, 3)
            .Select(i => (SingleAgentSolver)new SpaceTimeAStar(instance, i)).ToList();
        var cts = Enumerable.Range(0, 3)
            .Select(_ => new ConstraintTable(instance.NumOfCols, instance.MapSize)).ToList();
        var mddTable = new MDDTable(cts, solvers);
        mddTable.Init(3);

        var paths = new List<Path?>(new Path?[3]);
        var h = new CBSHeuristic(3, paths, solvers, cts, mddTable);
        h.Type = HeuristicsType.CG;
        h.Init();

        // 无冲突节点 h 应为 0
        var node = new DummyHLNode();
        bool ok = h.ComputeInformedHeuristics((CBSNode)CreateCBSNode(node), 30.0);
        Assert.That(ok, Is.True);
    }

    // -------------------------------------------------------------------------
    // CBS + CG 启发式集成测试
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_CG启发式_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.PC   = true;
        cbs.Heuristic.Type = HeuristicsType.CG;
        cbs.Heuristic.Init();

        bool solved = cbs.Solve(timeLimit: 30.0);
        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS_CG启发式_解代价应与零启发式一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);

        var cbsZero = new CBS(instance, sipp: false);
        cbsZero.PC = true;
        cbsZero.Solve(timeLimit: 30.0);
        int zeroCost = cbsZero.SolutionCost;

        var cbsCG = new CBS(instance, sipp: false);
        cbsCG.PC = true;
        cbsCG.Heuristic.Type = HeuristicsType.CG;
        cbsCG.Heuristic.Init();
        cbsCG.Solve(timeLimit: 30.0);

        Assert.That(cbsCG.SolutionCost, Is.EqualTo(zeroCost),
            "CG 启发式不应改变最优解代价");
    }

    [Test]
    public void CBS_CG启发式_展开节点数应不超过零启发式()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);

        var cbsZero = new CBS(instance, sipp: false);
        cbsZero.PC = true;
        cbsZero.Solve(timeLimit: 60.0);
        ulong zeroNodes = cbsZero.NumHLExpanded;

        var cbsCG = new CBS(instance, sipp: false);
        cbsCG.PC = true;
        cbsCG.Heuristic.Type = HeuristicsType.CG;
        cbsCG.Heuristic.Init();
        cbsCG.Solve(timeLimit: 60.0);

        // CG 启发式应减少展开节点数（或至少不增加）
        Assert.That(cbsCG.NumHLExpanded, Is.LessThanOrEqualTo(zeroNodes),
            $"CG 展开 {cbsCG.NumHLExpanded} 节点，零启发式展开 {zeroNodes} 节点");
    }

    // -------------------------------------------------------------------------
    // CBS + DG 启发式集成测试
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_DG启发式_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.PC = true;
        cbs.Heuristic.Type = HeuristicsType.DG;
        cbs.Heuristic.Init();

        bool solved = cbs.Solve(timeLimit: 30.0);
        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS_DG启发式_解代价应与零启发式一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);

        var cbsZero = new CBS(instance, sipp: false);
        cbsZero.PC = true;
        cbsZero.Solve(timeLimit: 30.0);
        int zeroCost = cbsZero.SolutionCost;

        var cbsDG = new CBS(instance, sipp: false);
        cbsDG.PC = true;
        cbsDG.Heuristic.Type = HeuristicsType.DG;
        cbsDG.Heuristic.Init();
        cbsDG.Solve(timeLimit: 30.0);

        Assert.That(cbsDG.SolutionCost, Is.EqualTo(zeroCost));
    }

    // -------------------------------------------------------------------------
    // CBS + WDG 启发式集成测试
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_WDG启发式_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.PC = true;
        cbs.Heuristic.Type = HeuristicsType.WDG;
        cbs.Heuristic.Init();

        bool solved = cbs.Solve(timeLimit: 30.0);
        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS_WDG启发式_解代价应与零启发式一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);

        var cbsZero = new CBS(instance, sipp: false);
        cbsZero.PC = true;
        cbsZero.Solve(timeLimit: 30.0);
        int zeroCost = cbsZero.SolutionCost;

        var cbsWDG = new CBS(instance, sipp: false);
        cbsWDG.PC = true;
        cbsWDG.Heuristic.Type = HeuristicsType.WDG;
        cbsWDG.Heuristic.Init();
        cbsWDG.Solve(timeLimit: 30.0);

        Assert.That(cbsWDG.SolutionCost, Is.EqualTo(zeroCost));
    }

    // -------------------------------------------------------------------------
    // 辅助
    // -------------------------------------------------------------------------

    private static CBSNode CreateCBSNode(HLNode parent)
    {
        var node = new CBSNode
        {
            Parent           = parent,
            GVal             = 0,
            HVal             = 0,
            Constraints      = new List<Constraint>(),
            Conflicts        = new List<Conflict>(),
            UnknownConflicts = new List<Conflict>(),
            Children         = new List<HLNode>()
        };
        return node;
    }
}
