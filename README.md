# Native Octree
An Octree Native Collection for Unity DOTS. 
Changes & improvements:
- Added automatic bounds resizing;
- Added more queries into the tree -> RaycastNonSorted & SphereQuery (approximate) SphereCast as AABB;
- Updated to Unity v2022.3 LTS & Entities 1.0.11;
- Incompatible with original repo due to changes [& potential future changes]; 
(remove before replacing old one and vice versa)

## Implementation
- It's a DOTS native container, meaning it's handling its own unmanaged memory and can be passed into jobs!
- Supports the storing of point + respective element radius
- The bulk insertion is using morton codes. This allows very fast bulk insertion but causes an increasing (minor) overhead with an increased depth

## Performance
There's some very rudimentary performance tests included. 
With 20k elements on a 2000x2000x2000m map, a max depth of 6 and 16 max elements per leaf. 
Burst enabled, ran on main thread on i7-7700K CPU @ 4.20GHz:</br>

- Job: Bulk insertion of all elements - Takes ~1ms
- Job: 1k queries on a 200x200x200m range - Takes ~2.7ms

With Burst disabled the tests are about 10x slower.

## Stability
Mostly stable - tested on multiple projects.
If in doubt - write more tests.

## Potential future work / missing features
- More unit tests
- Support for other basic shapes & queries
- Support individual adding and removing of elements

Forked from https://github.com/marijnz/NativeQuadtree.