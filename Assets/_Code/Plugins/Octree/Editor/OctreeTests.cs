using System.Diagnostics;
using NUnit.Framework;
using Octree;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

public partial class OctreeTests
{
    float3[] GetPositions(int elementCount = 20000)
    {
        Random.InitState(0);
        float3[] values = new float3[elementCount];

        for (int x = 0; x < values.Length; x++) {
            values[x] = new float3(Random.Range(-1000, 1000), 
                                   Random.Range(-1000, 1000), 
                                   Random.Range(-1000, 1000));
        }
        
        return values;
    }

    [Test]
    public void InsertTriggerDivideBulk()
    {
        var values = GetPositions();

        var elements = new NativeArray<OctElement<IntTestData>>(values.Length, Allocator.TempJob);

        for (int i = 0; i < values.Length; i++)
        {
            elements[i] = new OctElement<IntTestData>
            {
                Pos = values[i],
                Value = new IntTestData
                          {
                              Value = i
                          }
            };
        }

        var job = new OctreeJobs.AddBulkJob<IntTestData>
        {
            Elements = elements,
            Octree = new NativeOctree<IntTestData>(Allocator.TempJob)
        };

        var s = Stopwatch.StartNew();

        job.Run();

        s.Stop();
        Debug.Log(s.Elapsed.TotalMilliseconds);

        job.Octree.Dispose();
        elements.Dispose();
    }
    
    [Test]
    public void InsertBulkMemCpy([Values(1, 50, 500, 1000)] int elementCount)
    {
        float3[] positions = GetPositions(elementCount);

        NativeArray<OctElement<MemCpyData>> elements = new NativeArray<OctElement<MemCpyData>>(elementCount, Allocator.TempJob);

        for (int i = 0; i < elementCount; i++)
        {
            elements[i] = new OctElement<MemCpyData>
                          {
                              Pos = positions[i],
                              Value = new MemCpyData()
                          };
        }

        OctreeJobs.AddBulkJob<MemCpyData> job = new OctreeJobs.AddBulkJob<MemCpyData>
                                                {
                                                    Elements = elements,
                                                    Octree = new NativeOctree<MemCpyData>(Allocator.TempJob)
                                                };

        Stopwatch s = Stopwatch.StartNew();

        job.Run();

        s.Stop();
        Debug.Log(s.Elapsed.TotalMilliseconds);

        job.Octree.Dispose();
        elements.Dispose();
    }

    [Test]
    public void RangeQueryAfterBulk()
    {
        var values = GetPositions();

        NativeArray<OctElement<IntTestData>> elements = new NativeArray<OctElement<IntTestData>>(values.Length, Allocator.TempJob);

        for (int i = 0; i < values.Length; i++)
        {
            elements[i] = new OctElement<IntTestData>
            {
                Pos = values[i],
                Value = new IntTestData
                          {
                              Value = i
                          }
            };
        }

        var octree = new NativeOctree<IntTestData>(Allocator.TempJob);
        octree.ClearAndBulkInsert(elements);

        var queryJob = new OctreeJobs.RangeQueryJob<IntTestData>
        {
            Octree = octree,
            Bounds = new AABB {Center = 500, Extents = new float3(100, 100, 100)},
            Results = new NativeList<OctElement<IntTestData>>(1000, Allocator.TempJob)
        };

        var s = Stopwatch.StartNew();
        queryJob.Run();
        s.Stop();
        Debug.Log(s.Elapsed.TotalMilliseconds + " result: " + queryJob.Results.Length);

        octree.Dispose();
        elements.Dispose();
        queryJob.Results.Dispose();
    }
    
    [Test]
    public void TestQueryOne() {
        // Tree bounds:
        // float3(-53.25f, -44.4f, -19.9f)
        // float3(14.01591f, 14.01591f, 14.01591f)
        NativeArray<OctElement<int>> elements = new NativeArray<OctElement<int>>(1, Allocator.TempJob);
        elements[0] = new OctElement<int>
                      {
                          Pos = new float3(-53.25f, -44.4f, -19.9f),
                          Radius = 13.01591f,
                      };

        var octree = new NativeOctree<int>(Allocator.TempJob);
        octree.ClearAndBulkInsert(elements);

        var queryJob = new OctreeJobs.RangeQueryJob<int>
                       {
                           Octree = octree,
                           Bounds = new AABB {Center = new float3(-60.28234f, -43.33581f, -24.49575f), Extents = new float3(7f, 7f, 7f)},
                           Results = new NativeList<OctElement<int>>(1000, Allocator.TempJob)
                       };

        queryJob.Run();

        NativeList<OctElement<int>> results = queryJob.Results;
        int result = results.Length;
        
        octree.Dispose();
        elements.Dispose();
        queryJob.Results.Dispose();
        
        Assert.Greater(result, 0);
    }
    
    public struct IntTestData {
        public int Value;
    }

    public struct MemCpyData : IComponentData {
        public Entity Entity;
        public float3 MoveDelta;
        public float MovementSpeed;
        public float RotationSpeed;
        public float VelocityLimit;
        public float3 AccumulatedVelocity;
        public float Radius;
    }
}
