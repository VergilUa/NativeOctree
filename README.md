# Native Octree
An Octree Native Collection for Unity DOTS. 
Changes & improvements:
- Added automatic bounds resizing;
- Added more queries into the tree -> RaycastNonSorted & SphereQuery (approximate) SphereCast as AABB;
- Updated to Unity v2022.3 LTS & Entities 1.0.11;
- Incompatible with original repo due to current & potential future changes. 
</br> Remove before replacing old one and vice versa.

## Implementation
- It's a DOTS native container, meaning it's handling its own unmanaged memory and can be passed into jobs!
- Supports the storing of point + respective element radius
- The bulk insertion is using morton codes. This allows very fast bulk insertion but causes an increasing (minor) overhead with an increased depth

## Stability
- Basic performance tests included. 
- Tested on multiple projects over 3+ years, no major issues so far. If in doubt - write more tests.
- Performance tests are available via Test Runner.

## Potential future work / missing features
- More unit tests
- Support for other basic shapes & queries
- Support individual adding and removing of elements

Forked from https://github.com/marijnz/NativeQuadtree.
