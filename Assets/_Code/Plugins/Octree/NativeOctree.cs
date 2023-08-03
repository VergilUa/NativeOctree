using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace Octree
{
	/// <summary>
	/// <inheritdoc cref="UnsafeNativeOctree{T}"/>
	/// </summary>
	/// <remarks>
	/// Identical behaviour to <see cref="UnsafeNativeOctree{T}"/>,
	/// but includes job / thread safety checks for usage in jobs outside of Entities scope.
	/// </remarks>
	[NativeContainer]
	public unsafe partial struct NativeOctree<T> : INativeDisposable where T : unmanaged {
		#region [Properties]

		public bool IsCreated => _octreeData != null && _octreeData -> IsCreated;

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

		[NativeDisableParallelForRestriction]
		internal UnsafeNativeOctree<T>* _octreeData;

		private Allocator _allocator;
		
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
			if(maxDepth > 8)
			{
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
			_octreeData = UnsafeNativeOctree<T>.Create(allocator, maxDepth, maxLeafElements, initialElementsCapacity);
		}

		public void ClearAndBulkInsert(NativeArray<OctElement<T>> incomingElements) { 
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
			_octreeData -> ClearAndBulkInsert(incomingElements);
		}

		/// <summary>
		/// Performs a raycast into the tree by building an AABB and raycasting at each hit by bounds
		/// </summary>
		/// <remarks>
		/// Returned results are not sorted by distance
		/// </remarks>
		public void RaycastNonSorted(float3 origin,
		                             float3 direction,
		                             float distance,
		                             NativeList<QuadTreeHit<T>> results) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
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
				
				// No collision
				if (hitDistance < 0) continue;
				
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
		/// Converts position and radius to AABB, then performs RangeQuery on it.
		/// Afterwards filters obtained hits by radius.
		/// </summary>
		public void SphereQuery(float3 pos, float radius, NativeList<OctElement<T>> results) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
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

		public void RangeQuery(AABB bounds, NativeList<OctElement<T>> results)
		{
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
			new OctreeRangeQuery().Query(this, bounds, results);
		}

		public void Clear()
		{
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckWriteAndBumpSecondaryVersion(m_Safety);
#endif
			_octreeData -> Clear();
		}

		public void Dispose() {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			if (!AtomicSafetyHandle.IsDefaultValue(m_Safety))
			{
				AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
			}
#endif
			if (!IsCreated)
			{
				return;
			}
			
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			CollectionHelper.DisposeSafetyHandle(ref m_Safety);
#endif
			_octreeData -> Dispose();
			UnsafeUtility.FreeTracked(_octreeData, _allocator);

			_octreeData = null;
		}

		// TODO 
		public JobHandle Dispose(JobHandle inputDeps) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			if (!AtomicSafetyHandle.IsDefaultValue(m_Safety)) {
				AtomicSafetyHandle.CheckExistsAndThrow(m_Safety);
			}
#endif
			if (!IsCreated) 
				return inputDeps;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
			var jobHandle = new OctreeDisposeJob<T>
			                {
				                Data = new OctreeDispose<T>
				                       {_octreeData = _octreeData, m_Safety = m_Safety}
			                }.Schedule(inputDeps);
			AtomicSafetyHandle.Release(m_Safety);
#else
			var jobHandle = new OctreeDisposeJob<T>
			                {
				                Data = new OctreeDispose<T> {_octreeData = _octreeData}
			                }.Schedule(inputDeps);
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (UntypedUnsafeList*)m_ListData } }.Schedule(inputDeps);
#endif
			_octreeData = null;
			return jobHandle;
		}
	}
}
