namespace DotNetEECBS;

/// <summary>
/// 支持原地更新优先级的最小堆，等价于 boost::heap::pairing_heap 的 decrease-key 操作。
///
/// 实现原理：惰性删除（Lazy Deletion）。
/// 更新节点时不从堆中物理移除旧版本，而是将其加入失效集合，
/// 并将新版本重新入队。出队时跳过所有失效节点。
///
/// 使用 ReferenceEqualityComparer 按对象引用识别同一节点，
/// 与 C++ 中通过 handle 定位节点的语义等价。
/// </summary>
public class UpdatablePriorityQueue<T> where T : class
{
    private readonly PriorityQueue<T, T> _heap;

    /// <summary>已被逻辑删除（被更新替代）的节点集合，按引用比较</summary>
    private readonly HashSet<T> _invalidated =
        new(ReferenceEqualityComparer.Instance);

    /// <summary>当前有效节点数量</summary>
    public int Count { get; private set; }

    public bool IsEmpty => Count == 0;

    /// <param name="comparer">节点优先级比较器，返回负数表示第一个参数优先级更高</param>
    public UpdatablePriorityQueue(IComparer<T> comparer)
    {
        // PriorityQueue 内部是最小堆，priority 越小越先出队
        // 我们把节点本身作为 priority，用传入的 comparer 比较
        _heap = new PriorityQueue<T, T>(Comparer<T>.Create(comparer.Compare));
    }

    /// <summary>将节点入队</summary>
    public void Enqueue(T item)
    {
        _heap.Enqueue(item, item);
        Count++;
    }

    /// <summary>
    /// 更新节点优先级。
    /// 将旧节点标记为失效，把修改后的新节点重新入队。
    /// 调用方应先在节点对象上修改相关字段，再调用此方法。
    /// 若 oldItem 与 newItem 是同一对象（原地修改），也可正常工作。
    /// </summary>
    public void Update(T oldItem, T newItem)
    {
        _invalidated.Add(oldItem);
        _heap.Enqueue(newItem, newItem);
        // Count 不变：一个失效，一个新增，净变化为 0
    }

    /// <summary>取出优先级最高的有效节点，跳过所有失效节点</summary>
    public T Dequeue()
    {
        SkipInvalidated();
        if (_heap.Count == 0)
            throw new InvalidOperationException("队列为空。");
        Count--;
        return _heap.Dequeue();
    }

    /// <summary>查看堆顶有效节点但不移除</summary>
    public T Peek()
    {
        SkipInvalidated();
        if (_heap.Count == 0)
            throw new InvalidOperationException("队列为空。");
        return _heap.Peek();
    }

    /// <summary>尝试取出堆顶有效节点，队列为空时返回 false</summary>
    public bool TryDequeue(out T? item)
    {
        SkipInvalidated();
        if (_heap.Count == 0)
        {
            item = null;
            return false;
        }
        Count--;
        item = _heap.Dequeue();
        return true;
    }

    /// <summary>尝试查看堆顶有效节点，队列为空时返回 false</summary>
    public bool TryPeek(out T? item)
    {
        SkipInvalidated();
        if (_heap.Count == 0)
        {
            item = null;
            return false;
        }
        item = _heap.Peek();
        return true;
    }

    /// <summary>将指定节点标记为失效（逻辑删除），不影响 Count</summary>
    public void Remove(T item)
    {
        if (_invalidated.Add(item))
            Count--;
    }

    /// <summary>清空队列</summary>
    public void Clear()
    {
        _heap.Clear();
        _invalidated.Clear();
        Count = 0;
    }

    /// <summary>将堆顶的所有失效节点弹出，使堆顶始终是有效节点</summary>
    private void SkipInvalidated()
    {
        while (_heap.Count > 0 && _invalidated.Contains(_heap.Peek()))
        {
            _invalidated.Remove(_heap.Dequeue());
        }
    }
}
