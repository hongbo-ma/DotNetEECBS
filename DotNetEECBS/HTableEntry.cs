namespace DotNetEECBS;

/// <summary>
/// CBSHeuristic 查找表的键：(a1, a2, CBS节点)。
/// 两个键相等当且仅当智能体相同，且两个智能体各自的约束集合完全一致。
/// </summary>
public class HTableEntry
{
    public int    A1;
    public int    A2;
    public HLNode Node;

    public HTableEntry(int a1, int a2, HLNode node)
    {
        A1 = a1; A2 = a2; Node = node;
    }

    private (SortedSet<(int,int,int,int,int)>, SortedSet<(int,int,int,int,int)>) CollectConstraints()
    {
        var c1 = new SortedSet<(int,int,int,int,int)>();
        var c2 = new SortedSet<(int,int,int,int,int)>();
        var curr = Node;
        while (curr.Parent != null)
        {
            if (curr.Constraints.Count > 0)
            {
                var first = curr.Constraints[0];
                bool isGlobal = first.Type == ConstraintType.LeqLength ||
                                first.Type == ConstraintType.PositiveVertex ||
                                first.Type == ConstraintType.PositiveEdge;
                foreach (var c in curr.Constraints)
                {
                    var t = ((int)c.Type, c.Agent, c.Loc1, c.Loc2, c.Timestep);
                    if (isGlobal) { c1.Add(t); c2.Add(t); }
                    else if (c.Agent == A1) c1.Add(t);
                    else if (c.Agent == A2) c2.Add(t);
                }
            }
            curr = curr.Parent;
        }
        return (c1, c2);
    }

    public class EqNode : IEqualityComparer<HTableEntry>
    {
        public bool Equals(HTableEntry? x, HTableEntry? y)
        {
            if (x is null || y is null) return false;
            if (x.A1 != y.A1 || x.A2 != y.A2) return false;
            var (xc1, xc2) = x.CollectConstraints();
            var (yc1, yc2) = y.CollectConstraints();
            return xc1.SetEquals(yc1) && xc2.SetEquals(yc2);
        }

        public int GetHashCode(HTableEntry obj)
        {
            var curr = obj.Node;
            int h1 = 0, h2 = 0;
            while (curr.Parent != null)
            {
                if (curr.Constraints.Count > 0)
                {
                    var first = curr.Constraints[0];
                    bool isGlobal = first.Type == ConstraintType.LeqLength ||
                                    first.Type == ConstraintType.PositiveVertex ||
                                    first.Type == ConstraintType.PositiveEdge;
                    foreach (var c in curr.Constraints)
                    {
                        int hash = 3 * c.Agent + 5 * c.Loc1 + 7 * c.Loc2 + 11 * c.Timestep;
                        if (isGlobal) { h1 += hash; h2 += hash; }
                        else if (c.Agent == obj.A1) h1 += hash;
                        else if (c.Agent == obj.A2) h2 += hash;
                    }
                }
                curr = curr.Parent;
            }
            return h1 ^ (h2 << 1);
        }
    }
}
