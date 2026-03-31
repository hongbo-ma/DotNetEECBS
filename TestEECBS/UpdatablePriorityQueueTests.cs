namespace TestEECBS;

using DotNetEECBS;

/// <summary>
/// UpdatablePriorityQueue 的单元测试。
/// 使用简单的整数包装类模拟节点，验证入队、出队、更新、删除等行为。
/// </summary>
[TestFixture]
public class UpdatablePriorityQueueTests
{
    // 用于测试的简单节点类
    private class Node
    {
        public int Priority;
        public string Name;
        public Node(int priority, string name) { Priority = priority; Name = name; }
        public override string ToString() => $"{Name}(p={Priority})";
    }

    // 按 Priority 升序的比较器（最小堆）
    private static readonly IComparer<Node> AscComparer =
        Comparer<Node>.Create((a, b) => a.Priority.CompareTo(b.Priority));

    private UpdatablePriorityQueue<Node> _queue = null!;

    [SetUp]
    public void SetUp()
    {
        _queue = new UpdatablePriorityQueue<Node>(AscComparer);
    }

    // -------------------------------------------------------------------------
    // 基本入队 / 出队
    // -------------------------------------------------------------------------

    [Test]
    public void 初始队列Count应为零()
    {
        Assert.That(_queue.Count, Is.EqualTo(0));
        Assert.That(_queue.IsEmpty, Is.True);
    }

    [Test]
    public void 入队后Count应增加()
    {
        _queue.Enqueue(new Node(5, "A"));
        Assert.That(_queue.Count, Is.EqualTo(1));
        Assert.That(_queue.IsEmpty, Is.False);
    }

    [Test]
    public void 出队应按优先级升序返回()
    {
        _queue.Enqueue(new Node(3, "C"));
        _queue.Enqueue(new Node(1, "A"));
        _queue.Enqueue(new Node(2, "B"));

        Assert.That(_queue.Dequeue().Priority, Is.EqualTo(1));
        Assert.That(_queue.Dequeue().Priority, Is.EqualTo(2));
        Assert.That(_queue.Dequeue().Priority, Is.EqualTo(3));
    }

    [Test]
    public void 出队后Count应减少()
    {
        _queue.Enqueue(new Node(1, "A"));
        _queue.Enqueue(new Node(2, "B"));
        _queue.Dequeue();
        Assert.That(_queue.Count, Is.EqualTo(1));
    }

    [Test]
    public void Peek不应改变Count()
    {
        _queue.Enqueue(new Node(1, "A"));
        var top = _queue.Peek();
        Assert.That(top.Priority, Is.EqualTo(1));
        Assert.That(_queue.Count, Is.EqualTo(1));
    }

    [Test]
    public void Peek应返回优先级最高的节点()
    {
        _queue.Enqueue(new Node(5, "B"));
        _queue.Enqueue(new Node(2, "A"));
        Assert.That(_queue.Peek().Priority, Is.EqualTo(2));
    }

    // -------------------------------------------------------------------------
    // 空队列异常
    // -------------------------------------------------------------------------

    [Test]
    public void 空队列Dequeue应抛出异常()
    {
        Assert.Throws<InvalidOperationException>(() => _queue.Dequeue());
    }

    [Test]
    public void 空队列Peek应抛出异常()
    {
        Assert.Throws<InvalidOperationException>(() => _queue.Peek());
    }

    // -------------------------------------------------------------------------
    // TryDequeue / TryPeek
    // -------------------------------------------------------------------------

    [Test]
    public void TryDequeue空队列应返回false()
    {
        bool result = _queue.TryDequeue(out var item);
        Assert.That(result, Is.False);
        Assert.That(item, Is.Null);
    }

    [Test]
    public void TryDequeue有元素时应返回true并取出节点()
    {
        _queue.Enqueue(new Node(1, "A"));
        bool result = _queue.TryDequeue(out var item);
        Assert.That(result, Is.True);
        Assert.That(item!.Priority, Is.EqualTo(1));
        Assert.That(_queue.Count, Is.EqualTo(0));
    }

    [Test]
    public void TryPeek空队列应返回false()
    {
        bool result = _queue.TryPeek(out var item);
        Assert.That(result, Is.False);
        Assert.That(item, Is.Null);
    }

    // -------------------------------------------------------------------------
    // Update（核心功能：等价于 boost decrease-key）
    // -------------------------------------------------------------------------

    [Test]
    public void Update后应按新优先级出队()
    {
        var a = new Node(5, "A");
        var b = new Node(3, "B");
        _queue.Enqueue(a);
        _queue.Enqueue(b);

        // 将 a 的优先级从 5 降低到 1
        var aUpdated = new Node(1, "A_updated");
        _queue.Update(a, aUpdated);

        // 现在 aUpdated(1) 应该最先出队
        Assert.That(_queue.Dequeue().Name, Is.EqualTo("A_updated"));
        Assert.That(_queue.Dequeue().Priority, Is.EqualTo(3));
    }

    [Test]
    public void Update后Count不变()
    {
        var a = new Node(5, "A");
        _queue.Enqueue(a);
        _queue.Enqueue(new Node(3, "B"));

        _queue.Update(a, new Node(1, "A_updated"));

        Assert.That(_queue.Count, Is.EqualTo(2));
    }

    [Test]
    public void Update旧节点不再出队()
    {
        var a = new Node(5, "A");
        _queue.Enqueue(a);
        var aNew = new Node(1, "A_new");
        _queue.Update(a, aNew);

        var dequeued = _queue.Dequeue();
        // 出队的应该是新节点，不是旧节点
        Assert.That(dequeued, Is.SameAs(aNew));
        Assert.That(_queue.Count, Is.EqualTo(0));
    }

    [Test]
    public void 多次Update只保留最新版本()
    {
        var a = new Node(10, "A");
        _queue.Enqueue(a);

        var a2 = new Node(7, "A2");
        _queue.Update(a, a2);

        var a3 = new Node(2, "A3");
        _queue.Update(a2, a3);

        Assert.That(_queue.Count, Is.EqualTo(1));
        Assert.That(_queue.Dequeue(), Is.SameAs(a3));
    }

    // -------------------------------------------------------------------------
    // Remove（逻辑删除）
    // -------------------------------------------------------------------------

    [Test]
    public void Remove后Count减少()
    {
        var a = new Node(1, "A");
        _queue.Enqueue(a);
        _queue.Enqueue(new Node(2, "B"));

        _queue.Remove(a);
        Assert.That(_queue.Count, Is.EqualTo(1));
    }

    [Test]
    public void Remove后该节点不再出队()
    {
        var a = new Node(1, "A");
        var b = new Node(2, "B");
        _queue.Enqueue(a);
        _queue.Enqueue(b);

        _queue.Remove(a);

        var top = _queue.Dequeue();
        Assert.That(top, Is.SameAs(b));
        Assert.That(_queue.Count, Is.EqualTo(0));
    }

    // -------------------------------------------------------------------------
    // Clear
    // -------------------------------------------------------------------------

    [Test]
    public void Clear后Count为零且IsEmpty为true()
    {
        _queue.Enqueue(new Node(1, "A"));
        _queue.Enqueue(new Node(2, "B"));
        _queue.Clear();

        Assert.That(_queue.Count, Is.EqualTo(0));
        Assert.That(_queue.IsEmpty, Is.True);
    }

    [Test]
    public void Clear后可以重新入队()
    {
        _queue.Enqueue(new Node(1, "A"));
        _queue.Clear();
        _queue.Enqueue(new Node(5, "B"));

        Assert.That(_queue.Count, Is.EqualTo(1));
        Assert.That(_queue.Dequeue().Priority, Is.EqualTo(5));
    }

    // -------------------------------------------------------------------------
    // 与 LLNode 比较器集成测试
    // -------------------------------------------------------------------------

    [Test]
    public void 与CompareByF集成_应按f值升序出队()
    {
        var queue = new UpdatablePriorityQueue<LLNode>(new LLNode.CompareByF());

        var n1 = new LLNode(1, 3, 5, null, 0); // f=8
        var n2 = new LLNode(2, 1, 2, null, 0); // f=3
        var n3 = new LLNode(3, 2, 3, null, 0); // f=5

        queue.Enqueue(n1);
        queue.Enqueue(n2);
        queue.Enqueue(n3);

        Assert.That(queue.Dequeue().FVal, Is.EqualTo(3));
        Assert.That(queue.Dequeue().FVal, Is.EqualTo(5));
        Assert.That(queue.Dequeue().FVal, Is.EqualTo(8));
    }

    [Test]
    public void 与CompareByConflicts集成_应按冲突数升序出队()
    {
        var queue = new UpdatablePriorityQueue<LLNode>(new LLNode.CompareByConflicts());

        var n1 = new LLNode(1, 1, 1, null, 0, numConflicts: 3);
        var n2 = new LLNode(2, 1, 1, null, 0, numConflicts: 1);
        var n3 = new LLNode(3, 1, 1, null, 0, numConflicts: 2);

        queue.Enqueue(n1);
        queue.Enqueue(n2);
        queue.Enqueue(n3);

        Assert.That(queue.Dequeue().NumOfConflicts, Is.EqualTo(1));
        Assert.That(queue.Dequeue().NumOfConflicts, Is.EqualTo(2));
        Assert.That(queue.Dequeue().NumOfConflicts, Is.EqualTo(3));
    }
}
