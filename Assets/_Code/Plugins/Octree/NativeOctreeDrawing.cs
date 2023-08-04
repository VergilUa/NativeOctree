using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace Octree {
   /// <summary>
   /// Editor drawing of the NativeOctree. Currently only drawing x,y (z should also be supported but then a 3d view is needed)
   /// </summary>
   public unsafe partial struct NativeOctree<T> where T : unmanaged {
      public static void Draw(NativeOctree<T> tree, NativeList<OctElement<T>> results, AABB range, Color[][] texture) {
         AABB bounds = tree.Bounds;
         var widthMult = texture.Length / bounds.Extents.x * 2 / 2 / 2;
         var heightMult = texture[0].Length / bounds.Extents.y * 2 / 2 / 2;

         var widthAdd = bounds.Center.x + bounds.Extents.x;
         var heightAdd = bounds.Center.y + bounds.Extents.y;

         UnsafeNativeOctree<T> data = tree.Data;
         
         for (int i = 0; i < data._nodes -> Capacity; i++) {
            var node = UnsafeUtility.ReadArrayElement<OctNode>(data._nodes -> Ptr, i);

            if (node.Count > 0) {
               for (int k = 0; k < node.Count; k++) {
                  var element =
                     UnsafeUtility.ReadArrayElement<OctElement<T>>(data._elements -> Ptr, node.FirstChildIndex + k);

                  DrawPoint(element, Color.red);

                  // Draw 2d to texture
                  texture[(int) ((element.Pos.x + widthAdd) * widthMult)][(int) ((element.Pos.y + heightAdd)
                                                                                   * heightMult)] = Color.red;
               }
            }
         }

         if (results.IsCreated) {
            foreach (var element in results) {
               DrawPoint(element, Color.green);
               texture[(int) ((element.Pos.x + widthAdd) * widthMult)]
                  [(int) ((element.Pos.y + heightAdd) * heightMult)] = Color.green;
            }
         }

         //DrawBounds(texture, range, tree);
      }

      private static void DrawPoint(OctElement<T> element, Color color) {
         Debug.DrawLine(element.Pos + (float3) Vector3.left, element.Pos + (float3) Vector3.right, color, 15f);
         Debug.DrawLine(element.Pos + (float3) Vector3.up, element.Pos + (float3) Vector3.down, color, 15f);
         Debug.DrawLine(element.Pos + (float3) Vector3.back, element.Pos + (float3) Vector3.forward, color, 15f);
      }

      private static void DrawBounds(Color[][] texture, AABB bounds, NativeOctree<T> tree) {
         AABB treeBounds = tree.Bounds;

         var widthMult = texture.Length / treeBounds.Extents.x * 2 / 2 / 2;
         var heightMult = texture[0].Length / treeBounds.Extents.y * 2 / 2 / 2;

         var widthAdd = treeBounds.Center.x + treeBounds.Extents.x;
         var heightAdd = treeBounds.Center.y + treeBounds.Extents.y;

         var top = new float2(bounds.Center.x, bounds.Center.y - bounds.Extents.y);
         var left = new float2(bounds.Center.x - bounds.Extents.x, bounds.Center.y);

         for (int leftToRight = 0; leftToRight < bounds.Extents.x * 2; leftToRight++) {
            var poxX = left.x + leftToRight;
            texture[(int) ((poxX + widthAdd) * widthMult)][(int) ((bounds.Center.y + heightAdd + bounds.Extents.y)
                                                                  * heightMult)] = Color.blue;
            texture[(int) ((poxX + widthAdd) * widthMult)][(int) ((bounds.Center.y + heightAdd - bounds.Extents.y)
                                                                  * heightMult)] = Color.blue;
         }

         for (int topToBottom = 0; topToBottom < bounds.Extents.y * 2; topToBottom++) {
            var posY = top.y + topToBottom;
            texture[(int) ((bounds.Center.x + widthAdd + bounds.Extents.x) * widthMult)]
               [(int) ((posY + heightAdd) * heightMult)] = Color.blue;
            texture[(int) ((bounds.Center.x + widthAdd - bounds.Extents.x) * widthMult)]
               [(int) ((posY + heightAdd) * heightMult)] = Color.blue;
         }
      }
   }
}