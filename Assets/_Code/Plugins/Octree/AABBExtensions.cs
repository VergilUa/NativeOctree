using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

namespace Octree {
   /// <summary>
   /// Extension methods for AABB from Unity.Mathematics.Extensions lib
   /// </summary>
   public static class AABBExtensions {
      public static readonly float3 one = new float3(1, 1, 1);
      
      /// <summary>
      /// Grows the Bounds to include the point.
      /// </summary>
      /// <param name="bounds"></param>
      /// <param name="point"></param>
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public static void Encapsulate(ref this AABB bounds, float3 point) =>
         bounds.SetMinMax(math.min(bounds.Min, point), math.max(bounds.Max, point));

      /// <summary>
      ///   <para>Sets the bounds to the min and max value of the box.</para>
      /// </summary>
      /// <param name="bounds"></param>
      /// <param name="min"></param>
      /// <param name="max"></param>
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      public static void SetMinMax(ref this AABB bounds, float3 min, float3 max) {
         bounds.Extents = (max - min) * 0.5f;
         bounds.Center = min + bounds.Extents;
      }
      
      /// <summary>
      /// Temporary sphere to AABB conversion
      /// </summary>
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      [BurstCompile]
      public static void SphereToAABB(this in float3 pos, in float radius, out AABB result) {
         result = new AABB
                  {
                     Center = pos,
                     Extents = one * radius
                  }; 
      }

#if DEBUG
      /// <summary>
      /// Draws debug outline for the AABB
      /// </summary>
      /// <param name="bounds"></param>
      /// <param name="color"></param>
      public static void DrawBounds(ref this AABB bounds, Color color) {
         Vector3 center = bounds.Center;
         Vector3 extents = bounds.Extents;

         Vector3 frontTopLeft = new Vector3(center.x - extents.x, center.y + extents.y, center.z - extents.z);
         Vector3 frontTopRight = new Vector3(center.x + extents.x, center.y + extents.y, center.z - extents.z);
         Vector3 frontBottomLeft = new Vector3(center.x - extents.x, center.y - extents.y, center.z - extents.z);
         Vector3 frontBottomRight = new Vector3(center.x + extents.x, center.y - extents.y, center.z - extents.z);

         Vector3 backTopLeft = new Vector3(center.x - extents.x, center.y + extents.y, center.z + extents.z);
         Vector3 backTopRight = new Vector3(center.x + extents.x, center.y + extents.y, center.z + extents.z);

         Vector3 backBottomLeft = new Vector3(center.x - extents.x, center.y - extents.y, center.z + extents.z);
         Vector3 backBottomRight = new Vector3(center.x + extents.x, center.y - extents.y, center.z + extents.z);

         Debug.DrawLine(frontTopLeft, frontTopRight, color);
         Debug.DrawLine(frontTopRight, frontBottomRight, color);
         Debug.DrawLine(frontBottomRight, frontBottomLeft, color);
         Debug.DrawLine(frontBottomLeft, frontTopLeft, color);

         Debug.DrawLine(backTopLeft, backTopRight, color);
         Debug.DrawLine(backTopRight, backBottomRight, color);
         Debug.DrawLine(backBottomRight, backBottomLeft, color);
         Debug.DrawLine(backBottomLeft, backTopLeft, color);

         Debug.DrawLine(frontTopLeft, backTopLeft, color);
         Debug.DrawLine(frontTopRight, backTopRight, color);
         Debug.DrawLine(frontBottomRight, backBottomRight, color);
         Debug.DrawLine(frontBottomLeft, backBottomLeft, color);
      }
#endif
   }
}