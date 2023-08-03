using System.Collections.Generic;
using Unity.Mathematics;

namespace Octree {
   public struct QuadTreeHit<T> where T : unmanaged {
      public T Value;
      
      public float3 HitPoint;
      public float3 HitNormal;
      public float Distance;
   }

   public struct SortByDistance<T> : IComparer<QuadTreeHit<T>> where T : unmanaged {
      public int Compare(QuadTreeHit<T> x, QuadTreeHit<T> y) => x.Distance.CompareTo(y.Distance);

      public static SortByDistance<T> Default => new SortByDistance<T>();
   }
}