using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Octree {
   /// <summary>
   /// An Octree aimed to be used with Burst, supports fast bulk insertion and querying.
   /// </summary>
   // TODO:
   // - Better test coverage
   // - Automated depth / max leaf elements calculation
   [BurstCompile]
   [StructLayout(LayoutKind.Sequential)]
   public unsafe partial struct UnsafeOctree<T> : IDisposable where T : unmanaged {
      #region [Properties]

      public bool IsCreated => _elements != null;

      public AABB Bounds => *_bounds;

      public float MaxRadius => *_maxRadius;

      public short MaxLeafElements => _maxLeafElements;
      private readonly short _maxLeafElements;

      public int MaxDepth => _maxDepth;
      private readonly int _maxDepth;

      #endregion

      #region [Fields]

      // Data
      [NativeDisableUnsafePtrRestriction]
      internal UnsafeList<OctElement<T>>* _elements;

      [NativeDisableUnsafePtrRestriction]
      internal UnsafeList<int>* _lookup;

      [NativeDisableUnsafePtrRestriction]
      internal UnsafeList<OctNode>* _nodes;

      /// <summary>
      /// Maximum radius that used by the elements.
      /// Used to ensure range queries hit large elements
      /// </summary>
      [NativeDisableUnsafePtrRestriction]
      private float* _maxRadius;

      private int _elementsCount;

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
      public UnsafeOctree(Allocator allocator = Allocator.Temp,
                          int maxDepth = 6,
                          short maxLeafElements = 16,
                          int initialElementsCapacity = 256,
                          NativeArrayOptions options = NativeArrayOptions.ClearMemory) : this() {
         _allocator = allocator;

         _maxDepth = maxDepth;
         _maxLeafElements = maxLeafElements;
         _elementsCount = 0;

         // Allocate memory for every depth, the nodes on all depths are stored in a single continuous array
         int totalSize = LookupTables.DepthSizeLookup[maxDepth + 1];

         _lookup = UnsafeList<int>.Create(totalSize, allocator, options);
         _nodes = UnsafeList<OctNode>.Create(totalSize, allocator, options);

         _elements = UnsafeList<OctElement<T>>.Create(initialElementsCapacity, allocator);

         _bounds = (AABB*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<AABB>(),
                                                       UnsafeUtility.AlignOf<AABB>(),
                                                       allocator,
                                                       1);

         _maxRadius = (float*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<float>(),
                                                           UnsafeUtility.AlignOf<float>(),
                                                           allocator,
                                                           1);
      }

      /// <summary>
      /// Creates a new instance of UnsafeOctree with specified parameters as a memory block / pointer / reference
      /// </summary>
      public static UnsafeOctree<T>* Create(Allocator allocator,
                                            int maxDepth,
                                            short maxLeafElements,
                                            int initialElementCapacity) {
         var dataPtr = (UnsafeOctree<T>*) UnsafeUtility.MallocTracked(UnsafeUtility.SizeOf<UnsafeOctree<T>>(),
                                                                      UnsafeUtility.AlignOf<UnsafeOctree<T>>(),
                                                                      allocator,
                                                                      1);

         *dataPtr = new UnsafeOctree<T>(allocator, maxDepth, maxLeafElements, initialElementCapacity);
         return dataPtr;
      }

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
         IntPtr lookupStartPtr = (IntPtr) _lookup -> Ptr;
         for (int i = 0; i < mortonCodes.Length; i++) {
            int atIndex = 0;
            for (int depth = 0; depth <= _maxDepth; depth++) {
               // Increment the node on this depth that this element is contained in
               (*(int*) (lookupStartPtr + atIndex * sizeof(int)))++;
               atIndex = IncrementIndex(depth, mortonCodes, i, atIndex);
            }
         }

         // Prepare the tree leaf nodes
         RecursivePrepareLeaves(1, 1);

         int addr;
         // Add elements to leaf nodes
         int totalElementCount = incomingElements.Length;
         OctNode* nodesPtr = _nodes -> Ptr;

         for (int i = 0; i < totalElementCount; i++) {
            addr = 0;

            for (int depth = 0; depth <= _maxDepth; depth++) {
               OctNode node = UnsafeUtility.ReadArrayElement<OctNode>(nodesPtr, addr);
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

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public void Clear() {
         UnsafeUtility.MemClear(_lookup -> Ptr, _lookup -> Capacity * UnsafeUtility.SizeOf<int>());
         UnsafeUtility.MemClear(_nodes -> Ptr, _nodes -> Capacity * UnsafeUtility.SizeOf<OctNode>());
         UnsafeUtility.MemClear(_elements -> Ptr, _elements -> Capacity * UnsafeUtility.SizeOf<OctElement<T>>());
         UnsafeUtility.MemClear(_bounds, UnsafeUtility.SizeOf<AABB>());
         UnsafeUtility.MemClear(_maxRadius, UnsafeUtility.SizeOf<float>());

         _elementsCount = 0;
      }

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      private void InsertElementToLeaf(ref OctNode node, int atIndex, OctElement<T> element) {
         // We found a leaf, add this element to it and move to the next element
         UnsafeUtility.WriteArrayElement(_elements -> Ptr, node.FirstChildIndex + node.Count, element);
         node.Count++;
         UnsafeUtility.WriteArrayElement(_nodes -> Ptr, atIndex, node);
      }

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      private int IncrementIndex(int depth, NativeArray<int> mortonCodes, int i, int atIndex) {
         int atDepth = math.max(0, _maxDepth - depth);
         // Shift to the right and only get the first three bits
         int shiftedMortonCode = (mortonCodes[i] >> ((atDepth - 1) * 3)) & 0b111;
         // so the index becomes that... (0,1,2,3)
         atIndex += LookupTables.DepthSizeLookup[atDepth] * shiftedMortonCode;
         atIndex++; // offset for self
         return atIndex;
      }

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

      #region [Queries]

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public void RangeQuery(AABB bounds, NativeList<OctElement<T>> results) =>
         new OctreeRangeQuery<T>().Query(this, bounds, results);

      /// <summary>
      /// Performs a raycast into the tree by building an AABB and raycasting at each hit by bounds
      /// </summary>
      /// <remarks>
      /// Returned results are not sorted by distance
      /// </remarks>
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public void RaycastNonSorted(float3 origin,
                                   float3 direction,
                                   float distance,
                                   NativeList<QuadTreeHit<T>> results) {
         direction = math.normalizesafe(direction);
         float3 endPoint = origin + direction * distance;

         // TODO should the AABB optimized & split into segments?
         AABB aabb = new AABB
                     {
                        Center = origin,
                        Extents = new float3(ExtraQueryBoundsExpansion,
                                             ExtraQueryBoundsExpansion,
                                             ExtraQueryBoundsExpansion)
                     };

         aabb.Encapsulate(endPoint);

         NativeList<OctElement<T>> aabbResults = new NativeList<OctElement<T>>(results.Length, Allocator.Temp);
         RangeQuery(aabb, aabbResults);

         float3 hitNormal = -direction;

         for (int i = aabbResults.Length - 1; i >= 0; i--) {
            OctElement<T> hit = aabbResults[i];

            float3 toCenter = hit.Pos - origin;
            float distSqr = math.lengthsq(toCenter);

            float radiusSqr = hit.Radius;
            radiusSqr *= radiusSqr;

            // Ray inside sphere -> Valid hit
            if (distSqr < radiusSqr) {
               results.Add(new QuadTreeHit<T>
                           {
                              Value = hit.Value,
                              HitPoint = origin,
                              HitNormal = hitNormal
                              //Distance = 0
                           });
               continue;
            }

            // Project vector pointing from origin of the ray to the sphere onto the direction of the ray
            float projectedDist = math.dot(toCenter, direction);

            // Construct the sides of a triangle using the radius of the circle at
            // the projected point from the last step. The sides of this
            // triangle are radius, b and f, in squared units
            float bSqr = distSqr - projectedDist * projectedDist;

            // No collision
            if (radiusSqr - bSqr < 0f) continue;

            float f = math.sqrt(radiusSqr - bSqr);
            float hitDistance = projectedDist - f;

            // No collision or too far
            if (hitDistance < 0 || hitDistance > distance) continue;

            results.Add(new QuadTreeHit<T>
                        {
                           Value = hit.Value,
                           HitPoint = origin + hitDistance * direction,
                           HitNormal = hitNormal,
                           Distance = hitDistance
                        });
         }
      }

      /// <summary>
      /// Converts position and radius to AABB, then performs RangeQuery on it.
      /// Afterwards filters obtained hits by radius.
      /// </summary>
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public void SphereQuery(float3 pos, float radius, NativeList<OctElement<T>> results) {
         pos.SphereToAABB(radius, out AABB bounds);
         RangeQuery(bounds, results);

         for (int i = results.Length - 1; i >= 0; i--) {
            OctElement<T> hit = results[i];

            float sqrRadius = hit.Radius + radius;
            sqrRadius *= sqrRadius;

            // Not hit actually (corner case)
            // Remove from obtained hits
            float sqrDist = math.distancesq(pos, hit.Pos);
            if (sqrDist > sqrRadius) {
               results.RemoveAt(i);
            }
         }
      }

      #endregion

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
   }
}