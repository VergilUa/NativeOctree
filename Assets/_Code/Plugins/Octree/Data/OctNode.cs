namespace Octree {
   internal struct OctNode {
      // Points to this node's first child index in elements
      public int FirstChildIndex;

      // Number of elements in the leaf
      public short Count;
      public bool IsLeaf;
   }
}