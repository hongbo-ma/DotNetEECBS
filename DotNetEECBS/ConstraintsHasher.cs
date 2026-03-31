namespace DotNetEECBS;

/// <summary>
/// 约束哈希器：以（智能体ID + CBS节点约束链）为键，用于 MDDTable 缓存查找。
/// 两个 ConstraintsHasher 相等当且仅当智能体相同，且约束集合完全一致。
/// </summary>
public class ConstraintsHasher
{
    public int    Agent;
    public HLNode Node;

    public ConstraintsHasher(int agent, HLNode node)
    {
        Agent = agent;
        Node  = node;
    }

    private SortedSet<(int, int, int, int, int)> CollectConstraints()
    {
        var cons = new SortedSet<(int, int, int, int, int)>();
        var curr = Node;
        while (curr.Parent != null)
        {
            if (curr.Constraints.Count > 0)
            {
                var first = curr.Constraints[0];
                bool isGlobal = first.Type == ConstraintType.LeqLength ||
                                first.Type == ConstraintType.PositiveVertex ||
                                first.Type == ConstraintType.PositiveEdge;
                if (isGlobal || first.Agent == Agent)
                    foreach (var c in curr.Constraints)
                        cons.Add(((int)c.Type, c.Agent, c.Loc1, c.Loc2, c.Timestep));
            }
            curr = curr.Parent;
        }
        return cons;
    }

    public class EqNode : IEqualityComparer<ConstraintsHasher>
    {
        public bool Equals(ConstraintsHasher? x, ConstraintsHasher? y)
        {
            if (x is null || y is null) return false;
            if (x.Agent != y.Agent) return false;
            return x.CollectConstraints().SetEquals(y.CollectConstraints());
        }

        public int GetHashCode(ConstraintsHasher obj) => obj.ComputeHash();
    }

    public int ComputeHash()
    {
        var curr = Node;
        int hash = 0;
        while (curr.Parent != null)
        {
            if (curr.Constraints.Count > 0)
            {
                var first = curr.Constraints[0];
                bool isGlobal = first.Type == ConstraintType.LeqLength ||
                                first.Type == ConstraintType.PositiveVertex ||
                                first.Type == ConstraintType.PositiveEdge;
                if (isGlobal || first.Agent == Agent)
                    foreach (var c in curr.Constraints)
                        hash += 3 * c.Agent + 5 * c.Loc1 + 7 * c.Loc2 + 11 * c.Timestep;
            }
            curr = curr.Parent;
        }
        return hash;
    }
}
