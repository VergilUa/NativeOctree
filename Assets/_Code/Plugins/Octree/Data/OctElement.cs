using Unity.Mathematics;

namespace Octree {
   public struct OctElement<T> where T : unmanaged {
      public float3 Pos;

      /// <summary>
      /// Radius determines extra expansion,
      /// and shifts Pos point while testing children leaves towards testing bounds   
      /// </summary>
      public float Radius;

      public T Value;
   }
}