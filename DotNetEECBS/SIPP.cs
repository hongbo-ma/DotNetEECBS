namespace DotNetEECBS;

public class SIPPNode : LLNode
{
    public int  HighGeneration;
    public int  HighExpansion;
    public bool CollisionV;

    public SIPPNode(int location, int g, int h, LLNode? parent, int timestep,
        int highGen, int highExp, bool collisionV, int numConflicts = 0)
    {
        
    }
}

public class SIPP : SingleAgentSolver
{
    public SIPP(Instance instance, int agent) : base(instance, agent)
    {
    }

    public override Path FindOptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound)
    {
        throw new NotImplementedException();
    }

    public override (Path Path, int MinF) FindSuboptimalPath(
        HLNode node, ConstraintTable initialConstraints,
        IList<Path?> paths, int agent, int lowerBound, double w)
    {
        throw new NotImplementedException();
    }

    public override int GetTravelTime(int start, int end, ConstraintTable ct, int upperBound)
    {
        throw new NotImplementedException();
    }

    public override string GetName()
    { 
        return "SIPP";
    }
}