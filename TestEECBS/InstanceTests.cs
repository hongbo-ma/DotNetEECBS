namespace TestEECBS;

using DotNetEECBS;

/// <summary>
/// Instance 类的单元测试。
/// 使用项目中附带的 random-32-32-20.map 和 random-32-32-20-random-1.scen 文件。
/// </summary>
[TestFixture]
public class InstanceTests
{
    // 测试文件路径（相对于测试项目输出目录）
    private const string MapFile   = "random-32-32-20.map";
    private const string ScenFile  = "random-32-32-20-random-1.scen";

    private Instance _instance = null!;

    [SetUp]
    public void SetUp()
    {
        // 加载 5 个智能体，足够覆盖各类测试场景
        _instance = new Instance(MapFile, ScenFile, numAgents: 5);
    }

    // -------------------------------------------------------------------------
    // 地图基本属性
    // -------------------------------------------------------------------------

    [Test]
    public void 地图行列数应正确读取()
    {
        Assert.That(_instance.NumOfRows, Is.EqualTo(32));
        Assert.That(_instance.NumOfCols, Is.EqualTo(32));
    }

    [Test]
    public void 地图总格子数应等于行乘列()
    {
        Assert.That(_instance.MapSize, Is.EqualTo(_instance.NumOfRows * _instance.NumOfCols));
    }

    // -------------------------------------------------------------------------
    // 智能体信息
    // -------------------------------------------------------------------------

    [Test]
    public void 智能体数量应与请求数量一致()
    {
        Assert.That(_instance.NumOfAgents, Is.EqualTo(5));
    }

    [Test]
    public void 起点列表长度应等于智能体数量()
    {
        Assert.That(_instance.StartLocations.Count, Is.EqualTo(_instance.NumOfAgents));
    }

    [Test]
    public void 终点列表长度应等于智能体数量()
    {
        Assert.That(_instance.GoalLocations.Count, Is.EqualTo(_instance.NumOfAgents));
    }

    [Test]
    public void 所有起点应在地图范围内且不是障碍物()
    {
        foreach (int loc in _instance.StartLocations)
        {
            Assert.That(loc, Is.GreaterThanOrEqualTo(0).And.LessThan(_instance.MapSize),
                $"起点 {loc} 超出地图范围");
            Assert.That(_instance.IsObstacle(loc), Is.False,
                $"起点 {loc} 是障碍物");
        }
    }

    [Test]
    public void 所有终点应在地图范围内且不是障碍物()
    {
        foreach (int loc in _instance.GoalLocations)
        {
            Assert.That(loc, Is.GreaterThanOrEqualTo(0).And.LessThan(_instance.MapSize),
                $"终点 {loc} 超出地图范围");
            Assert.That(_instance.IsObstacle(loc), Is.False,
                $"终点 {loc} 是障碍物");
        }
    }

    // -------------------------------------------------------------------------
    // 坐标转换
    // -------------------------------------------------------------------------

    [Test]
    public void 线性化坐标与行列坐标互相转换应一致()
    {
        // 测试几个典型位置
        int[] rows = { 0, 0, 15, 31, 31 };
        int[] cols = { 0, 31, 16, 0,  31 };

        for (int i = 0; i < rows.Length; i++)
        {
            int loc = _instance.LinearizeCoordinate(rows[i], cols[i]);
            Assert.That(_instance.GetRowCoordinate(loc), Is.EqualTo(rows[i]),
                $"行坐标不匹配：({rows[i]},{cols[i]})");
            Assert.That(_instance.GetColCoordinate(loc), Is.EqualTo(cols[i]),
                $"列坐标不匹配：({rows[i]},{cols[i]})");
        }
    }

    [Test]
    public void GetCoordinate应与分别获取行列一致()
    {
        for (int loc = 0; loc < _instance.MapSize; loc += 50)
        {
            var (row, col) = _instance.GetCoordinate(loc);
            Assert.That(row, Is.EqualTo(_instance.GetRowCoordinate(loc)));
            Assert.That(col, Is.EqualTo(_instance.GetColCoordinate(loc)));
        }
    }

    // -------------------------------------------------------------------------
    // 曼哈顿距离
    // -------------------------------------------------------------------------

    [Test]
    public void 同一位置曼哈顿距离应为零()
    {
        int loc = _instance.LinearizeCoordinate(5, 5);
        Assert.That(_instance.GetManhattanDistance(loc, loc), Is.EqualTo(0));
    }

    [Test]
    public void 曼哈顿距离应满足对称性()
    {
        int a = _instance.LinearizeCoordinate(3, 4);
        int b = _instance.LinearizeCoordinate(10, 1);
        Assert.That(_instance.GetManhattanDistance(a, b),
                    Is.EqualTo(_instance.GetManhattanDistance(b, a)));
    }

    [Test]
    public void 曼哈顿距离计算应正确()
    {
        // (0,0) -> (3,4) 距离 = 3+4 = 7
        int a = _instance.LinearizeCoordinate(0, 0);
        int b = _instance.LinearizeCoordinate(3, 4);
        Assert.That(_instance.GetManhattanDistance(a, b), Is.EqualTo(7));
    }

    // -------------------------------------------------------------------------
    // 合法移动与邻居
    // -------------------------------------------------------------------------

    [Test]
    public void 原地等待应是合法移动()
    {
        // 找一个非障碍物格子
        int loc = FindPassable();
        Assert.That(_instance.ValidMove(loc, loc), Is.True, "原地等待应合法");
    }

    [Test]
    public void 移动到障碍物应不合法()
    {
        // 找一个有障碍物邻居的格子
        for (int loc = 0; loc < _instance.MapSize; loc++)
        {
            if (_instance.IsObstacle(loc)) continue;
            int right = loc + 1;
            if (right < _instance.MapSize && _instance.IsObstacle(right))
            {
                Assert.That(_instance.ValidMove(loc, right), Is.False);
                return;
            }
        }
        Assert.Ignore("未找到合适的测试格子");
    }

    [Test]
    public void 越界移动应不合法()
    {
        Assert.That(_instance.ValidMove(0, -1), Is.False);
        Assert.That(_instance.ValidMove(_instance.MapSize - 1, _instance.MapSize), Is.False);
    }

    [Test]
    public void 邻居列表中所有位置应合法可达()
    {
        int loc = FindPassable();
        var neighbors = _instance.GetNeighbors(loc);
        foreach (int next in neighbors)
        {
            Assert.That(_instance.ValidMove(loc, next), Is.True,
                $"邻居 {next} 从 {loc} 不可达");
            Assert.That(_instance.IsObstacle(next), Is.False,
                $"邻居 {next} 是障碍物");
        }
    }

    [Test]
    public void 邻居数量应在0到4之间()
    {
        int loc = FindPassable();
        var neighbors = _instance.GetNeighbors(loc);
        Assert.That(neighbors.Count, Is.InRange(0, 4));
    }

    // -------------------------------------------------------------------------
    // 度数
    // -------------------------------------------------------------------------

    [Test]
    public void 度数应与邻居数量一致()
    {
        int loc = FindPassable();
        int degree    = _instance.GetDegree(loc);
        int neighborCount = _instance.GetNeighbors(loc).Count;
        Assert.That(degree, Is.EqualTo(neighborCount));
    }

    // -------------------------------------------------------------------------
    // GetDefaultNumberOfAgents
    // -------------------------------------------------------------------------

    [Test]
    public void GetDefaultNumberOfAgents应返回加载的智能体数量()
    {
        Assert.That(_instance.GetDefaultNumberOfAgents(), Is.EqualTo(5));
    }

    // -------------------------------------------------------------------------
    // 文件不存在时应抛出异常
    // -------------------------------------------------------------------------

    [Test]
    public void 地图文件不存在且无随机参数时应抛出异常()
    {
        Assert.Throws<FileNotFoundException>(() =>
            new Instance("不存在.map", ScenFile));
    }

    [Test]
    public void 智能体文件不存在且无智能体数量时应抛出异常()
    {
        Assert.Throws<FileNotFoundException>(() =>
            new Instance(MapFile, "不存在.scen"));
    }

    // -------------------------------------------------------------------------
    // 辅助方法
    // -------------------------------------------------------------------------

    /// <summary>在地图中找到第一个非障碍物格子</summary>
    private int FindPassable()
    {
        for (int i = 0; i < _instance.MapSize; i++)
            if (!_instance.IsObstacle(i)) return i;
        throw new InvalidOperationException("地图中没有可通行格子");
    }
}
