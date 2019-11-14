using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MIConvexHull;
using QuickGraph;
using QuickGraph.Algorithms;
using QuickGraph.Algorithms.Search;
using Random = UnityEngine.Random;

public class GraphController : MonoBehaviour
{
    [SerializeField] Texture2D _texture = null;
    [SerializeField] Mesh _mesh = null;
    [SerializeField] Material _material = null;

    Mesh _renderMesh;
    Color32[] _colors;

    void Start()
    {
        Random.InitState(42);

        // DiscreteBunny();
        // DelaunayBunny();
        // MinimumSpanningTree();
        // StartCoroutine(DepthFirstSearchAnimated());
        StartCoroutine(ShortestPathBunnyAnimated());
    }

    void Update()
    {
        Graphics.DrawMesh(_renderMesh, Matrix4x4.identity, _material, 0);
    }

    void DiscreteBunny()
    {
        var points = GetPoints(1_000_000);
        MeshFromPoints(points);
    }

    IEnumerable<Edge> DelaunayEdges(float maxLength)
    {
        var points = GetPoints(1_000);
        var vertices = points.Select(p => new Vertex() { Location = p }).ToList();
        var delaunay = Triangulation.CreateDelaunay<Vertex, Face>(vertices);

        var edges = delaunay.Cells
            .SelectMany(c => c.GetEdges())
            .Distinct()
            .Where(e => e.Length < maxLength);

        return edges;
    }

    void DelaunayBunny()
    {
        float maxLength = 0.1f;
        var edges = DelaunayEdges(maxLength).ToList();

        MeshFromEdges(edges);
        SetMeshColors(edges.Select(e => maxLength - e.Length), maxLength);
    }

    void MinimumSpanningTree()
    {
        float maxLength = 0.1f;
        var edges = DelaunayEdges(maxLength).ToList();
        MeshFromEdges(edges);

        var graph = edges.ToUndirectedGraph<Vertex, Edge>();
        var tree = graph.MinimumSpanningTreePrim(e => e.Length);

        foreach (var edge in tree)
            edge.Value = 4;

        SetMeshColors(edges.Select(e => e.Value), 4);
    }

    IEnumerator DepthFirstSearchAnimated()
    {
        var edges = DelaunayEdges(0.1f).ToList();
        MeshFromEdges(edges);

        var graph = edges.ToUndirectedGraph<Vertex, Edge>();

        var sequence = new List<Edge>();

        var s = new UndirectedDepthFirstSearchAlgorithm<Vertex, Edge>(graph);
        s.ExamineEdge += (o, e) => sequence.Add(e.Edge);
        s.Compute(graph.Vertices.First());

        foreach (var edge in sequence)
        {
            edge.Value = 4;
            SetMeshColors(edges.Select(e => e.Value), 4);
            yield return new WaitForSeconds(0.01f);
        }
    }

    IEnumerator ShortestPathBunnyAnimated()
    {
        var edges = DelaunayEdges(0.1f).ToList();
        MeshFromEdges(edges);

        var graph = edges.ToUndirectedGraph<Vertex, Edge>();
        var start = graph.Vertices.First();
        var shortest = graph.ShortestPathsDijkstra(e => e.Length, start);

        foreach (var vertex in graph.Vertices)
        {
            if (shortest(vertex, out var path))
            {
                foreach (var edge in path)
                {
                    edge.Value += 1;

                    SetMeshColors(edges.Select(e => e.Value), 6);
                    yield return new WaitForSeconds(0.001f);
                }
            }
        }
    }

    void MeshFromEdges(IEnumerable<Edge> edges)
    {
        SetMesh(edges, e =>
        {
            var rotation = Quaternion.LookRotation(Vector3.forward, e.Vector);
            var scale = new Vector3(0.002f, e.Length, 1);
            return Matrix4x4.TRS(e.Center, rotation, scale);
        });
    }

    void MeshFromPoints(IEnumerable<Vector3> points)
    {
        SetMesh(points, p => Matrix4x4.TRS(p, Quaternion.identity, Vector3.one * 0.0005f));
    }

    void SetMeshColors(IEnumerable<float> values, float maxValue)
    {
        int i = 0;

        foreach (var value in values)
        {
            var t = value / maxValue;
            t = Mathf.Pow(t, 2.2f);
            t = Mathf.Clamp01(t);

            byte a = (byte)(t * 255);

            for (int j = 0; j < 4; j++)
                _colors[i * 4 + j].a = a;

            i++;
        }

        _renderMesh.SetColors(_colors);
    }

    void SetMesh<T>(IEnumerable<T> elements, Func<T, Matrix4x4> projection)
    {
        var matrices = elements.Select(projection);
        _renderMesh = CombineMesh(_mesh, matrices);

        _colors = new Color32[_renderMesh.vertexCount];

        for (int i = 0; i < _renderMesh.vertexCount; i++)
            _colors[i] = Color.white;
    }

    Mesh CombineMesh(Mesh instance, IEnumerable<Matrix4x4> matrices)
    {
        var instances = matrices
            .Select(m => new CombineInstance() { mesh = instance, transform = m });

        var mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.CombineMeshes(instances.ToArray());
        return mesh;
    }

    IEnumerable<Vector3> GetPoints(int count)
    {
        while (count > 0)
        {
            float x = Random.value;
            float y = Random.value;

            var color = _texture.GetPixelBilinear(x, y);
            var v = Mathf.Pow(color.r, 2.2f);

            if (v > Random.value)
            {
                count--;
                yield return new Vector2(x, y);
            }
        }
    }
}
