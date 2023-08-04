using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Octree {
   /// <summary>
   /// <inheritdoc cref="UnsafeOctree{T}"/>
   /// </summary>
   /// <remarks>
   /// Identical behaviour to <see cref="UnsafeOctree{T}"/>,
   /// but includes job / thread safety checks for usage in jobs outside of Entities scope.
   /// </remarks>
   [NativeContainer]
   public unsafe partial struct NativeOctree<T> : IDisposable where T : unmanaged {
      #region [Properties]

      public bool IsCreated => _dataPtr != null && _dataPtr -> IsCreated;

      public AABB Bounds => _dataPtr -> Bounds;

      public UnsafeOctree<T> Data => *_dataPtr;

      public UnsafeOctree<T>* DataPtr => _dataPtr;

      #endregion

      #region [Fields]

#if ENABLE_UNITY_COLLECTIONS_CHECKS
      //
      // Safety system variables
      //
      // The AtomicSafetyHandle field must be named exactly 'm_Safety'.
      // ReSharper disable once InconsistentNaming
      // ReSharper disable once FieldCanBeMadeReadOnly.Local
      private AtomicSafetyHandle m_Safety;

      // Statically register this type with the safety system, using a name derived from the type itself
      // ReSharper disable once InconsistentNaming
      internal static readonly int s_staticSafetyId = AtomicSafetyHandle.NewStaticSafetyId<NativeOctree<T>>();
#endif

      [NativeDisableUnsafePtrRestriction]
      internal UnsafeOctree<T>* _dataPtr;

      private readonly Allocator _allocator;

      #endregion

      /// <summary>
      /// Creates a new Octree.
      /// - Ensure the bounds are not way bigger than needed, otherwise the buckets are very off. Probably best to calculate bounds
      /// - The higher the depth, the larger the overhead, it especially goes up at a depth of 7/8
      /// </summary>
      public NativeOctree(Allocator allocator = Allocator.Temp,
                          int maxDepth = 6,
                          short maxLeafElements = 16,
                          int initialElementsCapacity = 256) : this() {
         if (maxDepth > 8) {
            // Currently no support for higher depths, the morton code lookup tables would have to support it
            throw new InvalidOperationException();
         }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
         // Create the AtomicSafetyHandle and DisposeSentinel (unmanaged)
         m_Safety = AtomicSafetyHandle.Create();

         // Set the safety ID on the AtomicSafetyHandle so that error messages describe this container type properly.
         AtomicSafetyHandle.SetStaticSafetyId(ref m_Safety, s_staticSafetyId);

         // Automatically bump the secondary version any time this container is scheduled for writing in a job			
         AtomicSafetyHandle.SetBumpSecondaryVersionOnScheduleWrite(m_Safety, true);

         // Check if this is a nested container, and if so, set the nested container flag
         if (UnsafeUtility.IsNativeContainerType<T>())
            AtomicSafetyHandle.SetNestedContainer(m_Safety, true);
#endif
         _allocator = allocator;
         _dataPtr = UnsafeOctree<T>.Create(allocator, maxDepth, maxLeafElements, initialElementsCapacity);
      }

      public void ClearAndBulkInsert(NativeArray<OctElement<T>> incomingElements) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
         _dataPtr -> ClearAndBulkInsert(incomingElements);
      }

      /// <summary>
      /// <inheritdoc cref="UnsafeOctree{T}"/>
      /// </summary>
      public void RaycastNonSorted(float3 origin,
                                   float3 direction,
                                   float distance,
                                   NativeList<QuadTreeHit<T>> results) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
         _dataPtr -> RaycastNonSorted(origin, direction, distance, results);
      }

      /// <summary>
      /// <inheritdoc cref="SphereQuery(Unity.Mathematics.float3,float,Unity.Collections.NativeList{Octree.OctElement{T}})"/>
      /// </summary>
      /// <remarks>
      /// This overload creates buffer as well. Make sure to dispose it when you're done
      /// </remarks>
      public void SphereQuery(float3 pos, float radius, out NativeList<OctElement<T>> results) {
         results = new NativeList<OctElement<T>>(Allocator.Temp);
         SphereQuery(pos, radius, results);
      }

      /// <summary>
      /// <inheritdoc cref="UnsafeOctree{T}"/>>
      /// </summary>
      public void SphereQuery(float3 pos, float radius, NativeList<OctElement<T>> results) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
         _dataPtr -> SphereQuery(pos, radius, results);
      }

      public void RangeQuery(AABB bounds, NativeList<OctElement<T>> results) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
         _dataPtr -> RangeQuery(bounds, results);
      }

      public void Clear() {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
         _dataPtr -> Clear();
      }

      public void Dispose() {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
         if (!AtomicSafetyHandle.IsDefaultValue(m_Safety)) {
            AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
         }
#endif
         if (!IsCreated)
            return;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
         CollectionHelper.DisposeSafetyHandle(ref m_Safety);
#endif
         _dataPtr -> Dispose();
         UnsafeUtility.FreeTracked(_dataPtr, _allocator);

         _dataPtr = null;
      }
   }
}