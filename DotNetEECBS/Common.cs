global using Path = System.Collections.Generic.List<DotNetEECBS.PathEntry>;
namespace DotNetEECBS;

public class Common
{
    /// <summary>
    /// 最大时间步长
    /// </summary>
    public static readonly int  MaxTimestep = Int32.MaxValue / 2;
    
    /// <summary>
    /// 最大成本
    /// </summary>
    public static readonly int  MaxCost = Int32.MaxValue / 2;
    
    /// <summary>
    /// 最大的展开的节点数量
    /// </summary>
    public static readonly int  MaxNodes = Int32.MaxValue / 2;
    
    /// <summary>
    /// 将 Path 输出到指定的 TextWriter（例如 Console.Out）
    /// </summary>
    public static void PrintPath(Path path, System.IO.TextWriter writer)
    {
        foreach (var state in path)
        {
            writer.Write(state.location);  // 与 C++ 相同，无分隔符
        }
    }

    /// <summary>
    /// 判断两个 Path 是否相等（比较大小和每个位置的 location）
    /// </summary>
    public static bool IsSamePath(Path p1, Path p2)
    {
        if (p1.Count != p2.Count) return false;
        for (int i = 0; i < p1.Count; i++)
        {
            if (p1[i].location != p2[i].location) return false;
        }
        return true;
    }
    
}
/// <summary>
/// 路径中的一个节点 此处使用class 而非 struct 是因为 cpp和c#的差异
/// </summary>
public class PathEntry
{
    public int location = -1;

    public PathEntry(int loc = -1)
    {
        location = loc;
    }
}