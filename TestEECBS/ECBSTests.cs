namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// ECBS 端到端测试：验证次优多智能体路径规划。
/// </summary>
[TestFixture]
public class ECBSTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    private static bool HasConflict(IList<Path?> paths)
    {
        int n = paths.Count;
        for (int a1 = 0; a1 < n; a1++)
        {
            if (paths[a1] == null) continue;
            for (int a2 = a1 + 1; a2 < n; a2++)
            {
                if (paths[a2] == null) continue;
                var p1 = paths[a1]!;
                var p2 = paths[a2]!;
                int minLen = Math.Min(p1.Count, p2.Count);
                for (int t = 0; t < minLen; t++)
                {
                    if (p1[t].location == p2[t].location) return true;
                    if (t < minLen - 1 &&
                        p1[t].location == p2[t + 1].location &&
                        p2[t].location == p1[t + 1].location) return true;
                }
                if (p1.Count != p2.Count)
                {
                    var pShort = p1.Count < p2.Count ? p1 : p2;
                    var pLong  = p1.Count < p2.Count ? p2 : p1;
                    int goal   = pShort[^1].location;
                    for (int t = minLen; t < pLong.Count; t++)
                        if (pLong[t].location == goal) return true;
                }
            }
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // 基本求解
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        bool solved  = ecbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True, "ECBS 未能在时限内找到解");
        Assert.That(ecbs.SolutionCost, Is.GreaterThan(0));
    }

    [Test]
    public void ECBS_2智能体_解应通过验证()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        ecbs.Solve(timeLimit: 30.0);

        Assert.That(ecbs.ValidateEcbsSolution(), Is.True);
    }

    [Test]
    public void ECBS_2智能体_路径应无冲突()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        ecbs.Solve(timeLimit: 30.0);

        var paths = Enumerable.Range(0, 2).Select(i => ecbs.GetEcbsPath(i)).ToList();
        Assert.That(HasConflict(paths), Is.False);
    }

    [Test]
    public void ECBS_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        bool solved  = ecbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(ecbs.ValidateEcbsSolution(), Is.True);
    }

    [Test]
    public void ECBS_5智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        bool solved  = ecbs.Solve(timeLimit: 60.0);

        Assert.That(solved, Is.True);
        Assert.That(ecbs.ValidateEcbsSolution(), Is.True);
    }

    // -------------------------------------------------------------------------
    // 次优性验证：ECBS 代价 <= w * CBS 最优代价
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS代价不超过CBS最优代价的w倍()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        double w     = 1.5;

        var cbs = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);
        int optCost = cbs.SolutionCost;

        var ecbs = new ECBS(instance, suboptimality: w);
        ecbs.Solve(timeLimit: 30.0);
        int subCost = ecbs.SolutionCost;

        Assert.That(subCost, Is.LessThanOrEqualTo((int)Math.Ceiling(optCost * w) + 1),
            $"ECBS 代价 {subCost} 超过 w*最优代价 {optCost * w}");
    }

    // -------------------------------------------------------------------------
    // w=1 时 ECBS 应等价于 CBS
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS_w等于1时代价应与CBS一致()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);

        var cbs = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        var ecbs = new ECBS(instance, suboptimality: 1.0);
        ecbs.Solve(timeLimit: 30.0);

        Assert.That(ecbs.SolutionCost, Is.EqualTo(cbs.SolutionCost));
    }

    // -------------------------------------------------------------------------
    // 统计信息
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS_展开节点数应大于0()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);
        ecbs.Solve(timeLimit: 30.0);

        Assert.That(ecbs.NumHLGenerated, Is.GreaterThan(0UL));
        Assert.That(ecbs.NumLLExpanded,  Is.GreaterThan(0UL));
    }

    // -------------------------------------------------------------------------
    // Clear 后可重新求解
    // -------------------------------------------------------------------------

    [Test]
    public void ECBS_Clear后可重新求解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var ecbs     = new ECBS(instance, suboptimality: 1.5);

        ecbs.Solve(timeLimit: 30.0);
        int firstCost = ecbs.SolutionCost;

        ecbs.Clear();
        ecbs.Solve(timeLimit: 30.0);

        Assert.That(ecbs.SolutionCost, Is.EqualTo(firstCost));
    }
}
