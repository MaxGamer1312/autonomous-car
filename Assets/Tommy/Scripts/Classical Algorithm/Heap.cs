using System;
using System.Collections.Generic;
using UnityEngine;


namespace Tommy.Scripts.Classical_Algorithm
{
    /// <summary>
    /// This is a min heap:
    ///     Enqueue(T item) // insert object into heap
    ///     Peek() // returns top element
    ///     Dequeue() // returns and remove top element
    ///     IsEmpty() // returns bool
    /// </summary>
    /// <typeparam name="T">A comparable type to sort by.</typeparam>
    public class Heap<T> where T : IHeapElement<T> 
    {
        public T[] heap;
        private int size;
        private Dictionary<T, T> hashTable;

        public Heap() : this(8) { }

        public Heap(int capacity)
        {
            heap = new T[capacity];
            hashTable = new();
            size = 0;
        }

        public void Enqueue(T item)
        {
            if(size >= heap.Length - 1)
                Array.Resize<T>(ref heap, heap.Length * 2);

            if (hashTable.TryGetValue(item, out T oldItem))
            {
                if (item.CompareTo(oldItem) < 0)
                {
                    // new item has a lower score
                    // must remove old item
                    RemoveElementAtIndex(oldItem.HeapIndex);
                    hashTable.Remove(item);
                }
                else
                {
                    return;
                }
            }
            
            int indexToPlace = size;
            while (indexToPlace > 0 && item.CompareTo(heap[GetParentIndex(indexToPlace)]) < 0)
            {
                heap[indexToPlace] = heap[GetParentIndex(indexToPlace)];
                heap[indexToPlace].HeapIndex = indexToPlace;
                indexToPlace = GetParentIndex(indexToPlace);
            }
            heap[indexToPlace] = item;
            heap[indexToPlace].HeapIndex = indexToPlace;
            int GetParentIndex(int i) => (i - 1) / 2;
            size++;
            hashTable.Add(item, item);
        }
        
        public T Peek()
        {
            if (size == 0)
            {
                Debug.LogError("Heap Empty");
                return default;
            }
                
            return heap[0];
        }

        public T Dequeue() => RemoveElementAtIndex(0);
        
        
        private T RemoveElementAtIndex(int index)
        {
            if (size == 0)
            {
                Debug.LogError("Heap Empty");
                return default;
            }
        
            // Save the top element, which is the smallest element in a min heap
            T top = heap[index];
            top.HeapIndex = -1;

            // Move the last element to the root
            heap[index] = heap[size - 1];
            heap[index].HeapIndex = index;
            heap[size - 1] = default;  // Clear the last position (optional)
            size--;  // Decrease size of heap

            // Variables to manage the hole (current index to fix)
            int hole = index;

            // Keep processing the heap until the property is restored
            while (GetLeftChildIndex(hole) < size)
            {
                // Get left and right child indices
                int leftChildIndex = GetLeftChildIndex(hole);
                int rightChildIndex = GetRightChildIndex(hole);

                // Determine which child is smaller (handle cases where there's no right child)
                int smallerChild = leftChildIndex;

                if (rightChildIndex < size && heap[rightChildIndex].CompareTo(heap[leftChildIndex]) < 0)
                {
                    smallerChild = rightChildIndex; // Right child is smaller
                }

                // If the current element is smaller than both children, stop
                if (heap[hole].CompareTo(heap[smallerChild]) <= 0)
                {
                    break;
                }

                // Swap the current element with the smallest child
                Swap(hole, smallerChild);
                hole = smallerChild;  // Move the hole to the child's position
            }

            return top;
            int GetLeftChildIndex(int i) => 2 * i + 1;
            int GetRightChildIndex(int i) => 2 * i + 2;

            void Swap(int i, int j)
            {
                (heap[i], heap[j]) = (heap[j], heap[i]);
                heap[i].HeapIndex = i;
                heap[j].HeapIndex = j;
            }
        }

        public bool IsEmpty() => size == 0;
    }

    public interface IHeapElement<T> : IComparable<T>
    {
        int HeapIndex { get; set; }
    }
}