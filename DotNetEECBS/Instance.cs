namespace DotNetEECBS;

/// <summary>
/// 问题实例：封装地图和智能体的起点/终点信息。
/// 目前仅支持无向无权重的四邻域网格地图。
/// </summary>
public class Instance
{
    // -------------------------------------------------------------------------
    // 公开属性
    // -------------------------------------------------------------------------

    /// <summary>地图列数</summary>
    public int NumOfCols { get; private set; }

    /// <summary>地图行数</summary>
    public int NumOfRows { get; private set; }

    /// <summary>地图总格子数 = 行数 × 列数</summary>
    public int MapSize { get; private set; }

    /// <summary>智能体数量</summary>
    public int NumOfAgents { get; private set; }

    /// <summary>每个智能体的起点位置（线性化坐标）</summary>
    public IReadOnlyList<int> StartLocations => _startLocations;

    /// <summary>每个智能体的终点位置（线性化坐标）</summary>
    public IReadOnlyList<int> GoalLocations => _goalLocations;

    // -------------------------------------------------------------------------
    // 私有字段
    // -------------------------------------------------------------------------

    /// <summary>地图障碍物数组，true 表示该格子是障碍物</summary>
    private bool[] _myMap = Array.Empty<bool>();

    private string _mapFname;
    private string _agentFname;

    private List<int> _startLocations = new();
    private List<int> _goalLocations  = new();

    // -------------------------------------------------------------------------
    // 构造函数
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从文件加载地图和智能体信息。
    /// 若地图文件不存在且提供了行列数和障碍物数，则随机生成地图并保存。
    /// 若智能体文件不存在且提供了智能体数量，则随机生成起终点并保存。
    /// </summary>
    /// <param name="mapFile">地图文件路径</param>
    /// <param name="agentFile">智能体文件路径</param>
    /// <param name="numAgents">智能体数量（随机生成时使用）</param>
    /// <param name="numRows">行数（随机生成地图时使用）</param>
    /// <param name="numCols">列数（随机生成地图时使用）</param>
    /// <param name="numObstacles">障碍物数量（随机生成地图时使用）</param>
    /// <param name="warehouseWidth">仓库场景的通道宽度，0 表示普通随机场景</param>
    public Instance(string mapFile, string agentFile,
        int numAgents = 0, int numRows = 0, int numCols = 0,
        int numObstacles = 0, int warehouseWidth = 0)
    {
        _mapFname   = mapFile;
        _agentFname = agentFile;
        NumOfAgents = numAgents;

        // 加载地图，失败则尝试随机生成
        if (!LoadMap())
        {
            if (numRows > 0 && numCols > 0 && numObstacles >= 0 &&
                numObstacles < numRows * numCols)
            {
                GenerateConnectedRandomGrid(numRows, numCols, numObstacles);
                SaveMap();
            }
            else
            {
                throw new FileNotFoundException($"地图文件 {mapFile} 未找到，且未提供有效的随机生成参数。");
            }
        }

        // 加载智能体，失败则尝试随机生成
        if (!LoadAgents())
        {
            if (numAgents > 0)
            {
                GenerateRandomAgents(warehouseWidth);
                SaveAgents();
            }
            else
            {
                throw new FileNotFoundException($"智能体文件 {agentFile} 未找到，且未提供有效的智能体数量。");
            }
        }
    }

    // -------------------------------------------------------------------------
    // 公开方法
    // -------------------------------------------------------------------------

    /// <summary>判断指定位置是否为障碍物</summary>
    public bool IsObstacle(int loc) => _myMap[loc];

    /// <summary>
    /// 判断从 curr 移动到 next 是否合法：
    /// next 必须在地图范围内、不是障碍物、且曼哈顿距离小于 2（即相邻或原地等待）
    /// </summary>
    public bool ValidMove(int curr, int next)
    {
        if (next < 0 || next >= MapSize) return false;
        if (_myMap[next]) return false;
        return GetManhattanDistance(curr, next) < 2;
    }

    /// <summary>返回 curr 位置的所有合法邻居（上下左右四个方向中可通行的格子）</summary>
    public IList<int> GetNeighbors(int curr)
    {
        var neighbors = new List<int>(4);
        int[] candidates = { curr + 1, curr - 1, curr + NumOfCols, curr - NumOfCols };
        foreach (int next in candidates)
        {
            if (ValidMove(curr, next))
                neighbors.Add(next);
        }
        return neighbors;
    }

    /// <summary>将二维坐标 (row, col) 转换为线性化的一维索引</summary>
    public int LinearizeCoordinate(int row, int col) => NumOfCols * row + col;

    /// <summary>从线性化索引获取行坐标</summary>
    public int GetRowCoordinate(int id) => id / NumOfCols;

    /// <summary>从线性化索引获取列坐标</summary>
    public int GetColCoordinate(int id) => id % NumOfCols;

    /// <summary>从线性化索引获取 (行, 列) 二维坐标</summary>
    public (int Row, int Col) GetCoordinate(int id) => (id / NumOfCols, id % NumOfCols);

    /// <summary>计算两个线性化位置之间的曼哈顿距离</summary>
    public int GetManhattanDistance(int loc1, int loc2)
    {
        int r1 = GetRowCoordinate(loc1), c1 = GetColCoordinate(loc1);
        int r2 = GetRowCoordinate(loc2), c2 = GetColCoordinate(loc2);
        return Math.Abs(r1 - r2) + Math.Abs(c1 - c2);
    }

    /// <summary>
    /// 返回指定位置的度数（四邻域中可通行的邻居数量）。
    /// 度数为 2 的位置通常是走廊节点，用于走廊冲突推理。
    /// </summary>
    public int GetDegree(int loc)
    {
        int degree = 0;
        if (loc - NumOfCols >= 0        && !_myMap[loc - NumOfCols]) degree++;
        if (loc + NumOfCols < MapSize   && !_myMap[loc + NumOfCols]) degree++;
        if (loc % NumOfCols > 0         && !_myMap[loc - 1])         degree++;
        if (loc % NumOfCols < NumOfCols - 1 && !_myMap[loc + 1])     degree++;
        return degree;
    }

    /// <summary>返回文件中记录的默认智能体数量</summary>
    public int GetDefaultNumberOfAgents() => NumOfAgents;

    /// <summary>将所有智能体的起终点信息打印到控制台</summary>
    public void PrintAgents()
    {
        for (int i = 0; i < NumOfAgents; i++)
        {
            Console.WriteLine(
                $"Agent{i} : S=({GetRowCoordinate(_startLocations[i])},{GetColCoordinate(_startLocations[i])})" +
                $" ; G=({GetRowCoordinate(_goalLocations[i])},{GetColCoordinate(_goalLocations[i])})");
        }
    }

    // -------------------------------------------------------------------------
    // 私有方法：地图加载与保存
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从文件加载地图。
    /// 支持两种格式：
    ///   - Nathan's benchmark：首行为 "type octile"，后续行包含 height/width 关键字
    ///   - 自定义格式：首行为 "rows,cols"
    /// 地图字符中 '.' 表示可通行，其他字符（如 '@'、'T'）表示障碍物。
    /// </summary>
    private bool LoadMap()
    {
        if (!File.Exists(_mapFname)) return false;

        using var reader = new StreamReader(_mapFname);
        string? firstLine = reader.ReadLine();
        if (firstLine == null) return false;

        if (firstLine.StartsWith("type", StringComparison.OrdinalIgnoreCase)) // Nathan's benchmark 格式
        {
            // 读取 "height N"
            var heightLine = reader.ReadLine()?.Split(' ');
            NumOfRows = int.Parse(heightLine![1]);
            // 读取 "width N"
            var widthLine = reader.ReadLine()?.Split(' ');
            NumOfCols = int.Parse(widthLine![1]);
            reader.ReadLine(); // 跳过 "map" 行
        }
        else // 自定义格式：首行 "rows,cols"
        {
            var parts = firstLine.Split(',');
            NumOfRows = int.Parse(parts[0]);
            NumOfCols = int.Parse(parts[1]);
        }

        MapSize = NumOfRows * NumOfCols;
        _myMap  = new bool[MapSize];

        for (int i = 0; i < NumOfRows; i++)
        {
            string? line = reader.ReadLine() ?? string.Empty;
            for (int j = 0; j < NumOfCols; j++)
            {
                // '.' 表示可通行，其余均视为障碍物
                _myMap[LinearizeCoordinate(i, j)] = (j < line.Length && line[j] != '.');
            }
        }

        return true;
    }

    /// <summary>将当前地图保存到文件（自定义格式）</summary>
    private void SaveMap()
    {
        using var writer = new StreamWriter(_mapFname);
        writer.WriteLine($"{NumOfRows},{NumOfCols}");
        for (int i = 0; i < NumOfRows; i++)
        {
            for (int j = 0; j < NumOfCols; j++)
                writer.Write(_myMap[LinearizeCoordinate(i, j)] ? '@' : '.');
            writer.WriteLine();
        }
    }

    /// <summary>将地图打印到控制台（调试用）</summary>
    private void PrintMap()
    {
        for (int i = 0; i < NumOfRows; i++)
        {
            for (int j = 0; j < NumOfCols; j++)
                Console.Write(_myMap[LinearizeCoordinate(i, j)] ? '@' : '.');
            Console.WriteLine();
        }
    }

    // -------------------------------------------------------------------------
    // 私有方法：智能体加载与保存
    // -------------------------------------------------------------------------

    /// <summary>
    /// 从文件加载智能体起终点。
    /// 支持两种格式：
    ///   - Nathan's benchmark：首行为 "version 1"，每行 tab 分隔，列在行之前
    ///   - 自定义格式：首行为智能体数量，后续每行 "startRow,startCol,goalRow,goalCol"
    /// </summary>
    private bool LoadAgents()
    {
        if (!File.Exists(_agentFname)) return false;

        using var reader = new StreamReader(_agentFname);
        string? firstLine = reader.ReadLine();
        if (firstLine == null) return false;

        if (firstLine.StartsWith("version", StringComparison.OrdinalIgnoreCase)) // Nathan's benchmark 格式
        {
            if (NumOfAgents == 0)
                throw new InvalidOperationException("使用 Nathan's benchmark 格式时，智能体数量必须大于 0。");

            _startLocations = new List<int>(NumOfAgents);
            _goalLocations  = new List<int>(NumOfAgents);

            for (int i = 0; i < NumOfAgents; i++)
            {
                string? line = reader.ReadLine() ?? throw new InvalidDataException($"智能体文件行数不足，期望 {NumOfAgents} 个智能体。");
                var tokens = line.Split('\t');
                // 格式：index  mapName  cols  rows  startCol  startRow  goalCol  goalRow  optimal
                int startCol = int.Parse(tokens[4]);
                int startRow = int.Parse(tokens[5]);
                int goalCol  = int.Parse(tokens[6]);
                int goalRow  = int.Parse(tokens[7]);
                _startLocations.Add(LinearizeCoordinate(startRow, startCol));
                _goalLocations.Add(LinearizeCoordinate(goalRow, goalCol));
            }
        }
        else // 自定义格式
        {
            NumOfAgents = int.Parse(firstLine.Split(',')[0]);
            _startLocations = new List<int>(NumOfAgents);
            _goalLocations  = new List<int>(NumOfAgents);

            for (int i = 0; i < NumOfAgents; i++)
            {
                string? line = reader.ReadLine() ?? throw new InvalidDataException($"智能体文件行数不足，期望 {NumOfAgents} 个智能体。");
                var parts = line.Split(',');
                int startRow = int.Parse(parts[0]);
                int startCol = int.Parse(parts[1]);
                int goalRow  = int.Parse(parts[2]);
                int goalCol  = int.Parse(parts[3]);
                _startLocations.Add(LinearizeCoordinate(startRow, startCol));
                _goalLocations.Add(LinearizeCoordinate(goalRow, goalCol));
            }
        }

        return true;
    }

    /// <summary>将智能体起终点保存到文件（自定义格式）</summary>
    private void SaveAgents()
    {
        using var writer = new StreamWriter(_agentFname);
        writer.WriteLine(NumOfAgents);
        for (int i = 0; i < NumOfAgents; i++)
        {
            writer.WriteLine(
                $"{GetRowCoordinate(_startLocations[i])},{GetColCoordinate(_startLocations[i])}," +
                $"{GetRowCoordinate(_goalLocations[i])},{GetColCoordinate(_goalLocations[i])},");
        }
    }

    // -------------------------------------------------------------------------
    // 私有方法：随机地图与智能体生成
    // -------------------------------------------------------------------------

    /// <summary>
    /// 生成一个 rows×cols 的随机连通网格，并在外围添加一圈障碍物边框，
    /// 然后随机放置 obstacles 个障碍物（保证地图始终连通）。
    /// </summary>
    private void GenerateConnectedRandomGrid(int rows, int cols, int obstacles)
    {
        Console.WriteLine($"生成 {rows} x {cols} 的网格，包含 {obstacles} 个障碍物。");

        // 外围加一圈边框，实际尺寸 +2
        NumOfRows = rows + 2;
        NumOfCols = cols + 2;
        MapSize   = NumOfRows * NumOfCols;
        _myMap    = new bool[MapSize];

        // 设置上下边框
        for (int j = 0; j < NumOfCols; j++)
        {
            _myMap[LinearizeCoordinate(0, j)]             = true;
            _myMap[LinearizeCoordinate(NumOfRows - 1, j)] = true;
        }
        // 设置左右边框
        for (int i = 0; i < NumOfRows; i++)
        {
            _myMap[LinearizeCoordinate(i, 0)]             = true;
            _myMap[LinearizeCoordinate(i, NumOfCols - 1)] = true;
        }

        // 随机添加障碍物，保证连通性
        var rng = new Random();
        int added = 0;
        while (added < obstacles)
        {
            int loc = rng.Next(MapSize);
            if (AddObstacle(loc))
            {
                PrintMap();
                added++;
            }
        }
    }

    /// <summary>
    /// 随机生成智能体的起终点。
    /// warehouseWidth == 0：完全随机；
    /// warehouseWidth > 0：仓库场景，起终点分布在地图两侧的通道区域。
    /// </summary>
    private void GenerateRandomAgents(int warehouseWidth)
    {
        Console.WriteLine($"随机生成 {NumOfAgents} 个智能体的起终点。");

        var starts = new bool[MapSize];
        var goals  = new bool[MapSize];
        _startLocations = new List<int>(new int[NumOfAgents]);
        _goalLocations  = new List<int>(new int[NumOfAgents]);

        var rng = new Random();

        if (warehouseWidth == 0) // 普通随机场景
        {
            int k = 0;
            while (k < NumOfAgents)
            {
                // 随机选起点
                int x     = rng.Next(NumOfRows);
                int y     = rng.Next(NumOfCols);
                int start = LinearizeCoordinate(x, y);
                if (_myMap[start] || starts[start]) continue;

                _startLocations[k] = start;
                starts[start]      = true;

                // 随机选终点
                int goal = rng.Next(MapSize);
                while (_myMap[goal] || goals[goal])
                    goal = rng.Next(MapSize);

                _goalLocations[k] = goal;
                goals[goal]       = true;
                k++;
            }
        }
        else // 仓库场景：起点和终点分布在左右两侧通道
        {
            // 生成起点
            int k = 0;
            while (k < NumOfAgents)
            {
                int x = rng.Next(NumOfRows);
                int y = rng.Next(warehouseWidth);
                if (k % 2 == 0) y = NumOfCols - y - 1; // 偶数智能体从右侧出发
                int start = LinearizeCoordinate(x, y);
                if (starts[start]) continue;
                _startLocations[k] = start;
                starts[start]      = true;
                k++;
            }
            // 生成终点
            k = 0;
            while (k < NumOfAgents)
            {
                int x = rng.Next(NumOfRows);
                int y = rng.Next(warehouseWidth);
                if (k % 2 == 1) y = NumOfCols - y - 1; // 奇数智能体终点在右侧
                int goal = LinearizeCoordinate(x, y);
                if (goals[goal]) continue;
                _goalLocations[k] = goal;
                goals[goal]       = true;
                k++;
            }
        }
    }

    /// <summary>
    /// 尝试在指定位置添加障碍物。
    /// 添加后若地图不再连通则撤销，返回 false；否则返回 true。
    /// </summary>
    private bool AddObstacle(int obstacle)
    {
        if (_myMap[obstacle]) return false;
        _myMap[obstacle] = true;

        // 检查该障碍物四个方向的邻居，确保任意两个可通行邻居之间仍然连通
        int ox = GetRowCoordinate(obstacle);
        int oy = GetColCoordinate(obstacle);
        int[] nx = { ox,     ox + 1, ox,     ox - 1 };
        int[] ny = { oy - 1, oy,     oy + 1, oy     };

        int start = 0, goal = 1;
        while (start < 3 && goal < 4)
        {
            if (nx[start] < 0 || nx[start] >= NumOfRows ||
                ny[start] < 0 || ny[start] >= NumOfCols ||
                _myMap[LinearizeCoordinate(nx[start], ny[start])])
            {
                start++;
            }
            else if (goal <= start)
            {
                goal = start + 1;
            }
            else if (nx[goal] < 0 || nx[goal] >= NumOfRows ||
                     ny[goal] < 0 || ny[goal] >= NumOfCols ||
                     _myMap[LinearizeCoordinate(nx[goal], ny[goal])])
            {
                goal++;
            }
            else if (IsConnected(
                         LinearizeCoordinate(nx[start], ny[start]),
                         LinearizeCoordinate(nx[goal],  ny[goal])))
            {
                start = goal;
                goal++;
            }
            else
            {
                // 添加该障碍物会导致地图不连通，撤销
                _myMap[obstacle] = false;
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// 使用 BFS 判断 start 和 goal 之间是否存在可通行路径。
    /// </summary>
    private bool IsConnected(int start, int goal)
    {
        var open   = new Queue<int>();
        var closed = new bool[MapSize];
        open.Enqueue(start);
        closed[start] = true;

        while (open.Count > 0)
        {
            int curr = open.Dequeue();
            if (curr == goal) return true;
            foreach (int next in GetNeighbors(curr))
            {
                if (!closed[next])
                {
                    open.Enqueue(next);
                    closed[next] = true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// 随机游走：从 curr 出发，随机移动 steps 步，返回最终位置。
    /// （用于生成分散的随机起终点，当前版本未使用）
    /// </summary>
    private int RandomWalk(int curr, int steps)
    {
        var rng = new Random();
        for (int w = 0; w < steps; w++)
        {
            var neighbors = GetNeighbors(curr);
            if (neighbors.Count == 0) break;
            curr = neighbors[rng.Next(neighbors.Count)];
        }
        return curr;
    }
}
