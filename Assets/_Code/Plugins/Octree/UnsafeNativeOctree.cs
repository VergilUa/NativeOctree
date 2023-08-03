using System;
using System.Collections;
using System.Collections.Generic;
using Octree;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// An Octree aimed to be used with Burst, supports fast bulk insertion and querying.
/// </summary>
// TODO:
// - Better test coverage
// - Automated depth / max leaf elements calculation
public unsafe partial struct UnsafeNativeOctree<T> : INativeDisposable where T : unmanaged {
   #region [Properties]

   public bool IsCreated => _elements != null;

   public AABB Bounds => *_bounds;

   #endregion
   
   #region [Fields]

   // Data
   [NativeDisableUnsafePtrRestriction]
   private UnsafeList<OctElement<T>>* _elements;

   [NativeDisableUnsafePtrRestriction]
   private UnsafeList<int>* _lookup;

   [NativeDisableUnsafePtrRestriction]
   private UnsafeList<OctNode>* _nodes;

   /// <summary>
   /// Maximum radius that used by the elements.
   /// Used to ensure range queries hit large elements
   /// </summary>
   [NativeDisableUnsafePtrRestriction]
   private float* _maxRadius;

   private int _elementsCount;

   private readonly int _maxDepth;
   private readonly short _maxLeafElements;

   [NativeDisableUnsafePtrRestriction]
   private AABB* _bounds; // NOTE: Currently assuming uniform

   private readonly Allocator _allocator;

   private const float ExtraQueryBoundsExpansion = 0.001f; 
   
   #endregion

   /// <summary>
   /// Creates a new Octree.
   /// - Ensure the bounds are not way bigger than needed, otherwise the buckets are very off. Probably best to calculate bounds
   /// - The higher the depth, the larger the overhead, it especially goes up at a depth of 7/8
   /// </summary>
   public UnsafeNativeOctree(Allocator allocator = Allocator.Temp,
                             int maxDepth = 6,
                             short maxLeafElements = 16,
                             int initialElementsCapacity = 256,
                             NativeArrayOptions options = NativeArrayOptions.ClearMemory) : this() {
      _allocator = allocator;
			
      _maxDepth = maxDepth;
      _maxLeafElements = maxLeafElements;
      _elementsCount = 0;
			
      // Allocate memory for every depth, the nodes on all depths are stored in a single continuous array
      int totalSize = LookupTables.DepthSizeLookup[maxDepth+1];

      _lookup = UnsafeList<int>.Create(totalSize, allocator, options);
      _nodes = UnsafeList<OctNode>.Create(totalSize, allocator, options);

      _elements = UnsafeList<OctElement<T>>.Create(initialElementsCapacity, allocator);

      _bounds = (AABB*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<AABB>(), UnsafeUtility.AlignOf<AABB>(), allocator, 1);

      _maxRadius = (float*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<float>(),
                                                        UnsafeUtility.AlignOf<float>(),
                                                        allocator,
                                                        1);
   }

   /// <summary>
   /// Creates a new instance of UnsafeNativeOctree with specified parameters as a memory block / pointer / reference
   /// </summary>
   public static UnsafeNativeOctree<T>* Create(Allocator allocator,
                                               int maxDepth,
                                               short maxLeafElements,
                                               int initialElementCapacity) {
	   var octreeDataPtr = (UnsafeNativeOctree<T>*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<UnsafeNativeOctree<T>>(),
	                                                                            UnsafeUtility.AlignOf<UnsafeNativeOctree<T>>(),
	                                                                            allocator,
	                                                                            1);

	   *octreeDataPtr = new UnsafeNativeOctree<T>(allocator, maxDepth, maxLeafElements, initialElementCapacity);
	   return octreeDataPtr;
   }

   public void ClearAndBulkInsert(NativeArray<OctElement<T>> incomingElements) {
	   // Always have to clear before bulk insert as otherwise the lookup and node allocations need to account
	   // for existing data.
	   Clear();

	   // Resize if needed
	   if (_elements -> Capacity < _elementsCount + incomingElements.Length) {
		   _elements -> Resize(math.max(incomingElements.Length, _elements -> Capacity * 2));
	   }

	   // Automatically grow bounds
	   if (incomingElements.Length > 0) {
		   OctElement<T> element = incomingElements[0];
		   _bounds -> Center = element.Pos;

		   EncapsulateElement(element);

		   for (int i = 1; i < incomingElements.Length; i++) {
			   EncapsulateElement(incomingElements[i]);
		   }

		   // Ensure extents are uniform
		   float3 extents = _bounds -> Extents;
		   float max = float.MinValue;
		   max = extents.x > max ? extents.x : max;
		   max = extents.y > max ? extents.y : max;
		   max = extents.z > max ? extents.z : max;

		   // Ensure bounds never zero, even with 1 element, otherwise this will cause depthExtentsScaling -> NaN
		   max += 1;

		   _bounds -> Extents = new float3(max, max, max);
	   }

	   float3 boundsCenter = _bounds -> Center;
	   float3 boundsExtents = _bounds -> Extents;

	   // Prepare morton codes
	   NativeArray<int> mortonCodes = new NativeArray<int>(incomingElements.Length, Allocator.Temp);

	   // ReSharper disable Unity.BurstLoadingManagedType -> static readonly
	   float3 depthExtentsScaling = LookupTables.DepthLookup[_maxDepth] / boundsExtents;

	   uint[] mortonLookup = LookupTables.MortonLookup;

	   for (int i = 0; i < incomingElements.Length; i++) {
		   float3 incPos = incomingElements[i].Pos;
		   incPos -= boundsCenter; // Offset by center
		   incPos.y = -incPos.y; // World -> array

		   float3 pos = (incPos + boundsExtents) * .5f; // Make positive

		   // Now scale into available space that belongs to the depth
		   pos *= depthExtentsScaling;

		   // And interleave the bits for the morton code
		   mortonCodes[i] = (int) (mortonLookup[(int) pos.x]
		                           | (mortonLookup[(int) pos.y] << 1)
		                           | (mortonLookup[(int) pos.z] << 2));
	   }
	   // ReSharper restore Unity.BurstLoadingManagedType


	   // Index total child element count per node (total, so parent's counts include those of child nodes)
	   for (int i = 0; i < mortonCodes.Length; i++) {
		   int atIndex = 0;
		   for (int depth = 0; depth <= _maxDepth; depth++) {
			   // Increment the node on this depth that this element is contained in
			   (*(int*) ((IntPtr) _lookup -> Ptr + atIndex * sizeof(int)))++;
			   atIndex = IncrementIndex(depth, mortonCodes, i, atIndex);
		   }
	   }

	   // Prepare the tree leaf nodes
	   RecursivePrepareLeaves(1, 1);

	   int addr = 0;
	   // Add elements to leaf nodes
	   for (int i = 0; i < incomingElements.Length; i++) {
		   addr = 0;

		   for (int depth = 0; depth <= _maxDepth; depth++) {
			   OctNode node = UnsafeUtility.ReadArrayElement<OctNode>(_nodes -> Ptr, addr);
			   if (node.IsLeaf) {
				   InsertElementToLeaf(ref node, addr, incomingElements[i]);
				   break;
			   }

			   // No leaf found, we keep going deeper until we find one
			   addr = IncrementIndex(depth, mortonCodes, i, addr);
		   }
	   }

	   mortonCodes.Dispose();
   }

   public void Clear() {
	   UnsafeUtility.MemClear(_lookup->Ptr, _lookup->Capacity * UnsafeUtility.SizeOf<int>());
	   UnsafeUtility.MemClear(_nodes->Ptr, _nodes->Capacity * UnsafeUtility.SizeOf<OctNode>());
	   UnsafeUtility.MemClear(_elements->Ptr, _elements->Capacity * UnsafeUtility.SizeOf<OctElement<T>>());
	   UnsafeUtility.MemClear(_bounds, UnsafeUtility.SizeOf<AABB>());
	   UnsafeUtility.MemClear(_maxRadius, UnsafeUtility.SizeOf<float>());
			
	   _elementsCount = 0;
   }

   private void EncapsulateElement(OctElement<T> octElement) {
			float3 pos = octElement.Pos;
			float radius = octElement.Radius;

			float maxSize = *_maxRadius;
			*_maxRadius = math.max(maxSize, radius);

			float3 minPos = pos - radius;
			float3 maxPos = pos + radius;

			_bounds -> Encapsulate(minPos);
			_bounds -> Encapsulate(maxPos);
		}

		private void InsertElementToLeaf(ref OctNode node, int atIndex, OctElement<T> element) {
			// We found a leaf, add this element to it and move to the next element
			UnsafeUtility.WriteArrayElement(_elements -> Ptr, node.FirstChildIndex + node.Count, element);
			node.Count++;
			UnsafeUtility.WriteArrayElement(_nodes -> Ptr, atIndex, node);
		}

		private int IncrementIndex(int depth, NativeArray<int> mortonCodes, int i, int atIndex)
		{
			int atDepth = math.max(0, _maxDepth - depth);
			// Shift to the right and only get the first three bits
			int shiftedMortonCode = (mortonCodes[i] >> ((atDepth - 1) * 3)) & 0b111;
			// so the index becomes that... (0,1,2,3)
			atIndex += LookupTables.DepthSizeLookup[atDepth] * shiftedMortonCode;
			atIndex++; // offset for self
			return atIndex;
		}

		private void RecursivePrepareLeaves(int prevOffset, int depth) {
			for (int l = 0; l < 8; l++) {
				int at = prevOffset + l * LookupTables.DepthSizeLookup[_maxDepth - depth + 1];

				int elementCount = UnsafeUtility.ReadArrayElement<int>(_lookup -> Ptr, at);

				if (elementCount > _maxLeafElements && depth < _maxDepth) {
					// There's more elements than allowed on this node so keep going deeper
					RecursivePrepareLeaves(at + 1, depth + 1);
				} else if (elementCount != 0) {
					// We either hit max depth or there's less than the max elements on this node, make it a leaf
					OctNode node = new OctNode {FirstChildIndex = _elementsCount, Count = 0, IsLeaf = true};
					UnsafeUtility.WriteArrayElement(_nodes -> Ptr, at, node);
					_elementsCount += elementCount;
				}
			}
		}

   public void Dispose() {
	   UnsafeList<OctElement<T>>.Destroy(_elements);
	   _elements = null;
	   
	   UnsafeList<int>.Destroy(_lookup);
	   _lookup = null;
	   
	   UnsafeList<OctNode>.Destroy(_nodes);
	   _nodes = null;
			
	   UnsafeUtility.FreeTracked(_bounds, _allocator);
	   _bounds = null;
			
	   UnsafeUtility.FreeTracked(_maxRadius, _allocator);
	   _maxRadius = null;
   }

   public JobHandle Dispose(JobHandle inputDeps) {
	   if (!IsCreated)
	   {
		   return inputDeps;
	   }

	   // Identical to CollectionHelper.ShouldDeallocate
	   if (_allocator > Allocator.None)
	   {
		   var jobHandle = new UnsafeOctreeDisposeJob<T>
		                   {
		                   }.Schedule(inputDeps);
		   Allocator = AllocatorManager.Invalid;

		   return jobHandle;
	   }

	   Ptr = null;

	   return inputDeps;
   }
   
   [BurstCompile]
   internal unsafe struct OctreeDisposeJob<T> : IJob where T : unmanaged {
	   internal OctreeDispose<T> Data;

	   public void Execute()
	   {
		   Data.Dispose();
	   }
   }
		
   [NativeContainer]
   [GenerateTestsForBurstCompatibility]
   internal unsafe struct OctreeDispose<T> where T : unmanaged {
	   [NativeDisableUnsafePtrRestriction]
	   public UnsafeNativeOctree<T>* _octreeData;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
	   internal AtomicSafetyHandle m_Safety;
#endif
	   public void Dispose() {
		   _octreeData -> Dispose();
	   }
   }
   
   [BurstCompile]
   internal unsafe struct UnsafeOctreeDisposeJob : IJob
   {
	   [NativeDisableUnsafePtrRestriction]
	   public void* Ptr;
	   public AllocatorManager.AllocatorHandle Allocator;

	   public void Execute()
	   {
		   AllocatorManager.Free(Allocator, Ptr);
	   }
   }
}
