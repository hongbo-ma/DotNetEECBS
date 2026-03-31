namespace DotNetEECBS;

public class ReservationTable
{
    public ReservationTable(ConstraintTable ct, int goalLocation)
    {
        
    }

    public IList<(int TMin, int TMax, int NumCollisions, bool IsGoal, bool IsFirst)>
        GetSafeIntervals(int from, int to, int lowerBound, int upperBound)
    {
        throw new NotImplementedException();
    }

    public Interval GetFirstSafeInterval(int location)
    {
        throw new NotImplementedException();
    }

    public bool FindSafeInterval(ref Interval interval, int location, int tMin)
    {
        throw new NotImplementedException();
    }

    public void UpdateSIT(int location)
    {
        throw new NotImplementedException();
    }
}