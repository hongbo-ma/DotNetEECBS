namespace TestEECBS;

using DotNetEECBS;

/// <summary>
/// Conflict 类的单元测试。
/// </summary>
[TestFixture]
public class ConflictTests
{
    private Conflict _conflict = null!;

    [SetUp]
    public void SetUp()
    {
        _conflict = new Conflict();
    }

    // -------------------------------------------------------------------------
    // 初始状态
    // -------------------------------------------------------------------------

    [Test]
    public void 初始优先级应为Unknown()
    {
        Assert.That(_conflict.Priority, Is.EqualTo(ConflictPriority.Unknown));
    }

    [Test]
    public void 初始约束列表应为空()
    {
        Assert.That(_conflict.Constraint1, Is.Empty);
        Assert.That(_conflict.Constraint2, Is.Empty);
    }

    // -------------------------------------------------------------------------
    // SetVertexConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 顶点冲突_类型应为Standard()
    {
        _conflict.SetVertexConflict(0, 1, 10, 3);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Standard));
    }

    [Test]
    public void 顶点冲突_智能体ID应正确设置()
    {
        _conflict.SetVertexConflict(2, 5, 10, 3);
        Assert.That(_conflict.A1, Is.EqualTo(2));
        Assert.That(_conflict.A2, Is.EqualTo(5));
    }

    [Test]
    public void 顶点冲突_每方各生成一条约束()
    {
        _conflict.SetVertexConflict(0, 1, 10, 3);
        Assert.That(_conflict.Constraint1.Count, Is.EqualTo(1));
        Assert.That(_conflict.Constraint2.Count, Is.EqualTo(1));
    }

    [Test]
    public void 顶点冲突_约束内容应正确()
    {
        _conflict.SetVertexConflict(0, 1, 10, 3);

        var c1 = _conflict.Constraint1[0];
        Assert.That(c1.Agent,     Is.EqualTo(0));
        Assert.That(c1.Loc1,      Is.EqualTo(10));
        Assert.That(c1.Loc2,      Is.EqualTo(-1));
        Assert.That(c1.Timestep,  Is.EqualTo(3));
        Assert.That(c1.Type,      Is.EqualTo(ConstraintType.Vertex));

        var c2 = _conflict.Constraint2[0];
        Assert.That(c2.Agent,     Is.EqualTo(1));
        Assert.That(c2.Loc1,      Is.EqualTo(10));
        Assert.That(c2.Loc2,      Is.EqualTo(-1));
        Assert.That(c2.Timestep,  Is.EqualTo(3));
        Assert.That(c2.Type,      Is.EqualTo(ConstraintType.Vertex));
    }

    [Test]
    public void 顶点冲突_重复调用应清空旧约束()
    {
        _conflict.SetVertexConflict(0, 1, 10, 3);
        _conflict.SetVertexConflict(0, 1, 20, 5);
        Assert.That(_conflict.Constraint1.Count, Is.EqualTo(1));
        Assert.That(_conflict.Constraint1[0].Loc1, Is.EqualTo(20));
    }

    // -------------------------------------------------------------------------
    // SetEdgeConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 边冲突_类型应为Standard()
    {
        _conflict.SetEdgeConflict(0, 1, 5, 6, 2);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Standard));
    }

    [Test]
    public void 边冲突_约束方向应相反()
    {
        _conflict.SetEdgeConflict(0, 1, 5, 6, 2);

        var c1 = _conflict.Constraint1[0];
        Assert.That(c1.Agent,    Is.EqualTo(0));
        Assert.That(c1.Loc1,     Is.EqualTo(5));  // a1: 5→6
        Assert.That(c1.Loc2,     Is.EqualTo(6));
        Assert.That(c1.Timestep, Is.EqualTo(2));
        Assert.That(c1.Type,     Is.EqualTo(ConstraintType.Edge));

        var c2 = _conflict.Constraint2[0];
        Assert.That(c2.Agent,    Is.EqualTo(1));
        Assert.That(c2.Loc1,     Is.EqualTo(6));  // a2: 6→5（方向相反）
        Assert.That(c2.Loc2,     Is.EqualTo(5));
        Assert.That(c2.Timestep, Is.EqualTo(2));
        Assert.That(c2.Type,     Is.EqualTo(ConstraintType.Edge));
    }

    // -------------------------------------------------------------------------
    // SetCorridorConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 走廊冲突_类型应为Corridor()
    {
        _conflict.SetCorridorConflict(0, 1, 3, 7, 2, 4);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Corridor));
    }

    [Test]
    public void 走廊冲突_约束类型应为Range()
    {
        _conflict.SetCorridorConflict(0, 1, 3, 7, 2, 4);
        Assert.That(_conflict.Constraint1[0].Type, Is.EqualTo(ConstraintType.Range));
        Assert.That(_conflict.Constraint2[0].Type, Is.EqualTo(ConstraintType.Range));
    }

    [Test]
    public void 走廊冲突_约束内容应正确()
    {
        _conflict.SetCorridorConflict(0, 1, 3, 7, 2, 4);

        var c1 = _conflict.Constraint1[0];
        Assert.That(c1.Agent,    Is.EqualTo(0));
        Assert.That(c1.Loc1,     Is.EqualTo(3));
        Assert.That(c1.Timestep, Is.EqualTo(2));

        var c2 = _conflict.Constraint2[0];
        Assert.That(c2.Agent,    Is.EqualTo(1));
        Assert.That(c2.Loc1,     Is.EqualTo(7));
        Assert.That(c2.Timestep, Is.EqualTo(4));
    }

    // -------------------------------------------------------------------------
    // SetRectangleConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 矩形冲突_类型应为Rectangle()
    {
        var c1 = new List<Constraint> { new(0, 1, 2, 3, ConstraintType.Barrier) };
        var c2 = new List<Constraint> { new(1, 4, 5, 3, ConstraintType.Barrier) };
        _conflict.SetRectangleConflict(0, 1, (0, 0), (2, 2), 3, c1, c2);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Rectangle));
    }

    [Test]
    public void 矩形冲突_应返回true()
    {
        var c1 = new List<Constraint> { new(0, 1, 2, 3, ConstraintType.Barrier) };
        var c2 = new List<Constraint> { new(1, 4, 5, 3, ConstraintType.Barrier) };
        bool result = _conflict.SetRectangleConflict(0, 1, (0, 0), (2, 2), 3, c1, c2);
        Assert.That(result, Is.True);
    }

    [Test]
    public void 矩形冲突_约束列表应被复制()
    {
        var c1 = new List<Constraint> { new(0, 1, 2, 3, ConstraintType.Barrier) };
        var c2 = new List<Constraint> { new(1, 4, 5, 3, ConstraintType.Barrier) };
        _conflict.SetRectangleConflict(0, 1, (0, 0), (2, 2), 3, c1, c2);

        Assert.That(_conflict.Constraint1.Count, Is.EqualTo(1));
        Assert.That(_conflict.Constraint2.Count, Is.EqualTo(1));
        Assert.That(_conflict.Constraint1[0], Is.EqualTo(c1[0]));
        Assert.That(_conflict.Constraint2[0], Is.EqualTo(c2[0]));
    }

    // -------------------------------------------------------------------------
    // SetTargetConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 目标冲突_类型应为Target()
    {
        _conflict.SetTargetConflict(0, 1, 15, 5);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Target));
    }

    [Test]
    public void 目标冲突_两条约束的agent均为a1()
    {
        // 与 C++ 原版一致：两条约束的 agent 字段均为 a1
        _conflict.SetTargetConflict(0, 1, 15, 5);
        Assert.That(_conflict.Constraint1[0].Agent, Is.EqualTo(0));
        Assert.That(_conflict.Constraint2[0].Agent, Is.EqualTo(0));
    }

    [Test]
    public void 目标冲突_约束类型应为LeqLength和GLength()
    {
        _conflict.SetTargetConflict(0, 1, 15, 5);
        Assert.That(_conflict.Constraint1[0].Type, Is.EqualTo(ConstraintType.LeqLength));
        Assert.That(_conflict.Constraint2[0].Type, Is.EqualTo(ConstraintType.GLength));
    }

    [Test]
    public void 目标冲突_约束位置和时间步应正确()
    {
        _conflict.SetTargetConflict(0, 1, 15, 5);
        Assert.That(_conflict.Constraint1[0].Loc1,     Is.EqualTo(15));
        Assert.That(_conflict.Constraint1[0].Timestep, Is.EqualTo(5));
        Assert.That(_conflict.Constraint2[0].Loc1,     Is.EqualTo(15));
        Assert.That(_conflict.Constraint2[0].Timestep, Is.EqualTo(5));
    }

    // -------------------------------------------------------------------------
    // SetMutexConflict
    // -------------------------------------------------------------------------

    [Test]
    public void 互斥冲突_类型应为Mutex()
    {
        _conflict.SetMutexConflict(0, 1);
        Assert.That(_conflict.Type, Is.EqualTo(ConflictType.Mutex));
    }

    [Test]
    public void 互斥冲突_优先级应自动设为Cardinal()
    {
        _conflict.SetMutexConflict(0, 1);
        Assert.That(_conflict.Priority, Is.EqualTo(ConflictPriority.Cardinal));
    }

    [Test]
    public void 互斥冲突_约束列表应为空()
    {
        _conflict.SetMutexConflict(0, 1);
        Assert.That(_conflict.Constraint1, Is.Empty);
        Assert.That(_conflict.Constraint2, Is.Empty);
    }

    // -------------------------------------------------------------------------
    // GetConflictId
    // -------------------------------------------------------------------------

    [Test]
    public void GetConflictId_应返回冲突类型的整数值()
    {
        _conflict.SetVertexConflict(0, 1, 5, 2);
        Assert.That(_conflict.GetConflictId(), Is.EqualTo((int)ConflictType.Standard));

        _conflict.SetMutexConflict(0, 1);
        Assert.That(_conflict.GetConflictId(), Is.EqualTo((int)ConflictType.Mutex));
    }
}
