namespace DotNetEECBS;

public class ConstraintPropagation
{
    public ConstraintPropagation(MDD mdd0, MDD mdd1)
    {
        
    }

    public void InitMutex()
    {
        
    }

    public void FwdMutexProp()
    {
        
    }

    public void BwdMutexProp()
    {
        
    }

    public bool HasMutex(MDDNode a, MDDNode b)
    {
        throw new NotImplementedException();
    }
    public bool HasFwdMutex(MDDNode a, MDDNode b)
    {
        throw new NotImplementedException();
    }
    public bool Mutexed(int level0, int level1)
    {
        throw new NotImplementedException();
    }
    public bool Feasible(int level0, int level1)
    {
        throw new NotImplementedException();
    }
    public bool SemiCardinal(int level, int loc)
    {
        throw new NotImplementedException();
    }
    public (List<Constraint> C1, List<Constraint> C2) GenerateConstraints(int a1, int a2)
    {
        throw new NotImplementedException();
    }
}