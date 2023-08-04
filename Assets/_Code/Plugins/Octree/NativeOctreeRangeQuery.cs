using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Octree {
   public unsafe partial struct OctreeRangeQuery<T> where T : unmanaged {
      private UnsafeNativeOctree<T> _tree;

      private UnsafeList<OctElement<T>>* _fastResults;
      private int _count;

      private AABB _bounds;

      public void Query(UnsafeNativeOctree<T> tree, AABB bounds, NativeList<OctElement<T>> results) {
         _tree = tree;
         _bounds = bounds;

         // TODO find a better way to hit single large elements?
         // Increase extents by maximum element radius, to ensure hits with large entities.
         // This can be costly, but doesn't seems to be a way to do otherwise
         _bounds.Extents += tree.MaxRadius;

         _count = 0;

         // Get pointer to inner list data for faster writing
         _fastResults = (UnsafeList<OctElement<T>>*) NativeListUnsafeUtility.GetInternalListDataPtrUnchecked(ref results);

         RecursiveRangeQuery(tree.Bounds, false, 1, 1);

         _fastResults -> Length = _count;
      }

      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public void RecursiveRangeQuery(AABB parentBounds, bool parentContained, int prevOffset, int depth) {
         short maxLeafElements = _tree.MaxLeafElements;
         int maxDepth = _tree.MaxDepth;

         if (_count + 8 * maxLeafElements > _fastResults -> Capacity) {
            _fastResults -> Resize(max(_fastResults -> Capacity * 2, _count + 8 * maxLeafElements));
         }

         // ReSharper disable once Unity.BurstLoadingManagedType -> static readonly
         int depthSize = LookupTables.DepthSizeLookup[maxDepth - depth + 1];
         for (int l = 0; l < 8; l++) {
            AABB childBounds = GetChildBounds(parentBounds, l);

            bool contained = parentContained;
            if (!contained) {
               if (_bounds.Contains(childBounds)) {
                  contained = true;
               } else if (!Intersects(_bounds, childBounds)) {
                  continue;
               }
            }

            int at = prevOffset + l * depthSize;

            int elementCount = UnsafeUtility.ReadArrayElement<int>(_tree._lookup -> Ptr, at);

            if (elementCount == 0) continue;

            if (elementCount > maxLeafElements && depth < maxDepth) {
               RecursiveRangeQuery(childBounds, contained, at + 1, depth + 1);
               continue;
            }

            OctNode node = UnsafeUtility.ReadArrayElement<OctNode>(_tree._nodes -> Ptr, at);

            if (contained) {
               void* index = (void*) ((IntPtr) _tree._elements -> Ptr
                                      + node.FirstChildIndex * UnsafeUtility.SizeOf<OctElement<T>>());

               UnsafeUtility.MemCpy((void*) ((IntPtr) _fastResults -> Ptr
                                             + _count * UnsafeUtility.SizeOf<OctElement<T>>()),
                                    index,
                                    node.Count * UnsafeUtility.SizeOf<OctElement<T>>());

               _count += node.Count;
            } else {
               for (int k = 0; k < node.Count; k++) {
                  OctElement<T> element =
                     UnsafeUtility.ReadArrayElement<OctElement<T>>(_tree._elements -> Ptr, node.FirstChildIndex + k);
                  if (_bounds.Contains(element.Pos)) {
                     UnsafeUtility.WriteArrayElement(_fastResults -> Ptr, _count++, element);
                  }
               }
            }
         }
      }

      public bool Intersects(AABB a, AABB b) {
         return abs(a.Center[0] - b.Center[0]) < a.Extents[0] + b.Extents[0]
                && abs(a.Center[1] - b.Center[1]) < a.Extents[1] + b.Extents[1]
                && abs(a.Center[2] - b.Center[2]) < a.Extents[2] + b.Extents[2];
      }

      private static AABB GetChildBounds(AABB parentBounds, int childZIndex) {
         float half = parentBounds.Extents.x * .5f;
         switch (childZIndex) {
            case 0:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x - half,
                                             parentBounds.Center.y + half,
                                             parentBounds.Center.z - half),
                         Extents = half
                      };
            case 1:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x + half,
                                             parentBounds.Center.y + half,
                                             parentBounds.Center.z - half),
                         Extents = half
                      };
            case 2:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x - half,
                                             parentBounds.Center.y - half,
                                             parentBounds.Center.z - half),
                         Extents = half
                      };
            case 3:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x + half,
                                             parentBounds.Center.y - half,
                                             parentBounds.Center.z - half),
                         Extents = half
                      };
            case 4:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x - half,
                                             parentBounds.Center.y + half,
                                             parentBounds.Center.z + half),
                         Extents = half
                      };
            case 5:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x + half,
                                             parentBounds.Center.y + half,
                                             parentBounds.Center.z + half),
                         Extents = half
                      };
            case 6:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x - half,
                                             parentBounds.Center.y - half,
                                             parentBounds.Center.z + half),
                         Extents = half
                      };
            case 7:
               return new AABB
                      {
                         Center = new float3(parentBounds.Center.x + half,
                                             parentBounds.Center.y - half,
                                             parentBounds.Center.z + half),
                         Extents = half
                      };
            default: throw new Exception();
         }
      }
   }
}