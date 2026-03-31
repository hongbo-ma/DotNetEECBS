using DotNetEECBS;

// 对应 README 中的命令：
// ./eecbs -m random-32-32-20.map -a random-32-32-20-random-1.scen
//         -o test.csv --outputPaths=paths.txt -k 50 -t 60 --suboptimality=1.2

// 解析命令行参数
string mapFile        = "random-32-32-20.map";
string scenFile       = "random-32-32-20-random-1.scen";
string outputCsv      = "test.csv";
string outputPaths    = "paths.txt";
int    numAgents      = 50;
double timeLimit      = 60.0;
double suboptimality  = 1.2;

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "-m": mapFile       = args[++i]; break;
        case "-a": scenFile      = args[++i]; break;
        case "-o": outputCsv     = args[++i]; break;
        case "-k": numAgents     = int.Parse(args[++i]); break;
        case "-t": timeLimit     = double.Parse(args[++i]); break;
        case "--suboptimality":  suboptimality = double.Parse(args[++i]); break;
        case "--outputPaths":    outputPaths   = args[++i]; break;
    }
}

Console.WriteLine($"地图:       {mapFile}");
Console.WriteLine($"场景:       {scenFile}");
Console.WriteLine($"智能体数:   {numAgents}");
Console.WriteLine($"时间限制:   {timeLimit}s");
Console.WriteLine($"次优界 w:   {suboptimality}");
Console.WriteLine();

// 加载实例
var instance = new Instance(mapFile, scenFile, numAgents);

// 构建 ECBS 求解器（对应 README 的默认配置）
var ecbs = new ECBS(instance, suboptimality: suboptimality);
ecbs.PC                = true;   // 优先处理 Cardinal 冲突
ecbs.CorridorReasoning = true;   // 走廊推理
ecbs.TargetReasoning   = true;   // 目标冲突推理
ecbs.Heuristic.Type = HeuristicsType.WDG;
ecbs.Heuristic.Init();
var sw = System.Diagnostics.Stopwatch.StartNew();
bool solved = ecbs.Solve(timeLimit);
sw.Stop();

// 输出结果
Console.WriteLine(solved ? "求解成功！" : "超时或无解。");
Console.WriteLine($"解代价 (sum-of-costs): {ecbs.SolutionCost}");
Console.WriteLine($"运行时间:              {sw.Elapsed.TotalSeconds:F3}s");
Console.WriteLine($"高层展开节点:          {ecbs.NumHLExpanded}");
Console.WriteLine($"低层展开节点:          {ecbs.NumLLExpanded}");

if (!solved) return;

// 保存路径文件
using (var pw = new StreamWriter(outputPaths))
{
    for (int i = 0; i < numAgents; i++)
    {
        var path = ecbs.GetEcbsPath(i);
        if (path == null) continue;
        pw.Write($"Agent {i}: ");
        foreach (var entry in path)
        {
            int row = instance.GetRowCoordinate(entry.location);
            int col = instance.GetColCoordinate(entry.location);
            pw.Write($"({row},{col})->");
        }
        pw.WriteLine();
    }
}
Console.WriteLine($"路径已保存到: {outputPaths}");

// 保存统计 CSV
bool csvExists = File.Exists(outputCsv);
using (var cw = new StreamWriter(outputCsv, append: true))
{
    if (!csvExists)
        cw.WriteLine("runtime,#HL_expanded,#LL_expanded,solution_cost,suboptimality,#agents,map,scen");
    cw.WriteLine($"{sw.Elapsed.TotalSeconds:F3},{ecbs.NumHLExpanded},{ecbs.NumLLExpanded}," +
                 $"{ecbs.SolutionCost},{suboptimality},{numAgents},{mapFile},{scenFile}");
}
Console.WriteLine($"统计已保存到: {outputCsv}");
