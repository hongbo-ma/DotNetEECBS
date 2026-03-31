namespace TestEECBS;

using DotNetEECBS;
using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;

/// <summary>
/// CBS 端到端测试：使用真实地图验证多智能体无冲突路径规划。
/// </summary>
[TestFixture]
public class CBSTests
{
    private const string MapFile  = "random-32-32-20.map";
    private const string ScenFile = "random-32-32-20-random-1.scen";

    // -------------------------------------------------------------------------
    // 辅助：验证路径集合无冲突
    // -------------------------------------------------------------------------

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
                    // 顶点冲突
                    if (p1[t].location == p2[t].location) return true;
                    // 边冲突
                    if (t < minLen - 1 &&
                        p1[t].location == p2[t + 1].location &&
                        p2[t].location == p1[t + 1].location) return true;
                }

                // 目标占用冲突
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
    // 2 个智能体
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_2智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        bool solved  = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True, "CBS 未能在时限内找到解");
        Assert.That(cbs.SolutionCost, Is.GreaterThan(0));
    }

    [Test]
    public void CBS_2智能体_解应通过验证()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        Assert.That(cbs.ValidateSolution(), Is.True, "解验证失败");
    }

    [Test]
    public void CBS_2智能体_路径应无冲突()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        var paths = Enumerable.Range(0, 2).Select(i => cbs.GetPath(i)).ToList();
        Assert.That(HasConflict(paths), Is.False, "路径存在冲突");
    }

    [Test]
    public void CBS_2智能体_每条路径起终点应正确()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        for (int i = 0; i < 2; i++)
        {
            var path = cbs.GetPath(i)!;
            Assert.That(path[0].location,  Is.EqualTo(instance.StartLocations[i]), $"agent {i} 起点错误");
            Assert.That(path[^1].location, Is.EqualTo(instance.GoalLocations[i]),  $"agent {i} 终点错误");
        }
    }

    // -------------------------------------------------------------------------
    // 3 个智能体
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_3智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        bool solved  = cbs.Solve(timeLimit: 30.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS_3智能体_路径应无冲突()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        var paths = Enumerable.Range(0, 3).Select(i => cbs.GetPath(i)).ToList();
        Assert.That(HasConflict(paths), Is.False);
    }

    // -------------------------------------------------------------------------
    // 5 个智能体
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_5智能体_应找到无冲突解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);
        var cbs      = new CBS(instance, sipp: false);
        bool solved  = cbs.Solve(timeLimit: 60.0);

        Assert.That(solved, Is.True);
        Assert.That(cbs.ValidateSolution(), Is.True);
    }

    [Test]
    public void CBS_5智能体_路径应无冲突()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 5);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 60.0);

        var paths = Enumerable.Range(0, 5).Select(i => cbs.GetPath(i)).ToList();
        Assert.That(HasConflict(paths), Is.False);
    }

    // -------------------------------------------------------------------------
    // 统计信息合理性
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_展开节点数应大于0()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        Assert.That(cbs.NumHLGenerated, Is.GreaterThan(0UL));
        Assert.That(cbs.NumLLExpanded,  Is.GreaterThan(0UL));
    }

    [Test]
    public void CBS_代价下界应不超过解代价()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 3);
        var cbs      = new CBS(instance, sipp: false);
        cbs.Solve(timeLimit: 30.0);

        Assert.That(cbs.GetLowerBound(), Is.LessThanOrEqualTo(cbs.SolutionCost));
    }

    // -------------------------------------------------------------------------
    // Clear 后可重新求解
    // -------------------------------------------------------------------------

    [Test]
    public void CBS_Clear后可重新求解()
    {
        var instance = new Instance(MapFile, ScenFile, numAgents: 2);
        var cbs      = new CBS(instance, sipp: false);

        cbs.Solve(timeLimit: 30.0);
        int firstCost = cbs.SolutionCost;

        cbs.Clear();
        cbs.Solve(timeLimit: 30.0);

        Assert.That(cbs.SolutionCost, Is.EqualTo(firstCost), "两次求解结果应一致");
    }
}
