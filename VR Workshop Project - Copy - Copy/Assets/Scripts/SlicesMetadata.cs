using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using UnityEngine.UIElements;

namespace Assets.Scripts 
{ 

    //The side of the mesh
    public enum MeshSide
    {
        Positive = 0,
        Negative = 1
    }

    //object used to manage the positive and negative side mesh data for a sliced object
    class SlicesMetadata
    {
        private Mesh posSideMesh;
        private List<Vector3> PSVerticies;
        private List<int> PSTriangles;
        private List<Vector2> PSUVs;
        private List<Vector3> PSNormals;

        private Mesh negSideMesh;
        private List<Vector3> NSVertices;
        private List<int> NSTriangles;
        private List<Vector2> NSUVs;
        private List<Vector3> NSNormals;

        private readonly List<Vector3> PointsOnPlane;
        private Plane SlicePlane;
        private Mesh WoodMesh;


        public Mesh PosSideMesh
        {
            get
            {
                if (posSideMesh == null)
                {
                    posSideMesh = new Mesh();
                }

                SetMeshData(MeshSide.Positive);
                return posSideMesh;
            }
        }

        public Mesh NegSideMesh
        {
            get
            {
                if (negSideMesh == null)
                {
                    negSideMesh = new Mesh();
                }

                SetMeshData(MeshSide.Negative);
                return negSideMesh;
            }
        }


        public SlicesMetadata(Plane plane, Mesh mesh)
        {
            PSTriangles = new List<int>();
            PSVerticies = new List<Vector3>();
            PSUVs = new List<Vector2>();
            PSNormals = new List<Vector3>();

            NSTriangles = new List<int>();
            NSVertices = new List<Vector3>();
            NSUVs = new List<Vector2>();
            NSNormals = new List<Vector3>();

            PointsOnPlane = new List<Vector3>();
            SlicePlane = plane;
            WoodMesh = mesh;

            ComputeNewMeshes();
        }

        // Compute the positive and negative meshes based on the plane intersection and mesh
        private void ComputeNewMeshes()
        {
            //Temp MeshData
            int[] TempWoodTriangles = WoodMesh.triangles;
            Vector3[] TempWoodVerts = WoodMesh.vertices;
            Vector3[] TempWoodNormals = WoodMesh.normals;
            Vector2[] TempWoodUVs = WoodMesh.uv;

            //Vertex 1 Data
            Vector3 TriangleVertex1;
            int TriVertex1Index;
            Vector2 TriangleVertexUV1;
            Vector3 TriangleVertexNormal1;
            bool TriangleVertex1Side;

            //Vertex 2 Data
            Vector3 TriangleVertex2;
            int TriVertex2Index;
            Vector2 TriangleVertexUV2;
            Vector3 TriangleVertexNormal2;
            bool TriangleVertex2Side;

            //Vertex 3 Data
            Vector3 TriangleVertex3;
            int TriangleVertex3Index;
            Vector2 TriangleVertexUV3;
            Vector3 TriangleVertexNormal3;
            bool TriangleVertex3Side;

            //For each vertex in WoodMesh
            for (int i = 0; i < TempWoodTriangles.Length; i += 3)
            {
                //Assign Correct Vertex 1 Data
                TriangleVertex1 = TempWoodVerts[TempWoodTriangles[i]];
                TriVertex1Index = Array.IndexOf(TempWoodVerts, TriangleVertex1);
                TriangleVertexUV1 = TempWoodUVs[TriVertex1Index];
                TriangleVertexNormal1 = TempWoodNormals[TriVertex1Index];
                TriangleVertex1Side = SlicePlane.GetSide(TriangleVertex1);

                //Assign Correct Vertex 2 Data
                TriangleVertex2 = TempWoodVerts[TempWoodTriangles[i + 1]];
                TriVertex2Index = Array.IndexOf(TempWoodVerts, TriangleVertex2);
                TriangleVertexUV2 = TempWoodUVs[TriVertex2Index];
                TriangleVertexNormal2 = TempWoodNormals[TriVertex2Index];
                TriangleVertex2Side = SlicePlane.GetSide(TriangleVertex2);

                //Assign Correct Vertex 3 Data
                TriangleVertex3 = TempWoodVerts[TempWoodTriangles[i + 2]];
                TriangleVertex3Index = Array.IndexOf(TempWoodVerts, TriangleVertex3);
                TriangleVertexUV3 = TempWoodUVs[TriangleVertex3Index];
                TriangleVertexNormal3 = TempWoodNormals[TriangleVertex3Index];
                TriangleVertex3Side = SlicePlane.GetSide(TriangleVertex3);

                //All vertex are on the same side
                if (TriangleVertex1Side == TriangleVertex2Side && TriangleVertex2Side == TriangleVertex3Side)
                {
                    //Add the relevant triangle
                    MeshSide side = (TriangleVertex1Side) ? MeshSide.Positive : MeshSide.Negative;
                    AddSideNormalAndUvs(side, TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                }
                else
                {
                    //prepare to store the intersection points of the mesh data
                    Vector3 intersection1;
                    Vector3 intersection2;

                    Vector2 intersection1Uv;
                    Vector2 intersection2Uv;

                    MeshSide side1 = (TriangleVertex1Side) ? MeshSide.Positive : MeshSide.Negative;
                    MeshSide side2 = (TriangleVertex1Side) ? MeshSide.Negative : MeshSide.Positive;

                    //Vertex 1 and 2 on the same side
                    if (TriangleVertex1Side == TriangleVertex2Side)
                    {
                        //Cast a ray from Vertex 2 to Vertex 3 and from Vertex 3 to Vertex 1 to get the intersection points                       
                        intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection1Uv);
                        intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection2Uv);

                        //Attempt to calculate Normals and UVs with interesection
                        AddSideNormalAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, TriangleVertex2, null, TriangleVertexUV2, intersection1, null, intersection1Uv);
                        AddSideNormalAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv);

                        AddSideNormalAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv);

                    }
                    //Vertex 1 and 3 are on the same side
                    else if (TriangleVertex1Side == TriangleVertex3Side)
                    {
                        //Cast a ray from v1 to v2 and from v2 to v3 to get the intersections                       
                        intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                        intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                        //Add the positive triangles
                        AddSideNormalAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3);
                        AddSideNormalAndUvs(side1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv, TriangleVertex3, null, TriangleVertexUV3);

                        AddSideNormalAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex2, null, TriangleVertexUV2, intersection2, null, intersection2Uv);
                    }
                    //Vertex 1 is alone
                    else
                    {
                        //Cast a ray from v1 to v2 and from v1 to v3 to get the intersections                       
                        intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                        intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                        AddSideNormalAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv);

                        AddSideNormalAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex2, null, TriangleVertexUV2, TriangleVertex3, null, TriangleVertexUV3);
                        AddSideNormalAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv);
                    }

                    //Add the newly created points on the plane.
                    PointsOnPlane.Add(intersection1);
                    PointsOnPlane.Add(intersection2);
                }
            }

            JoinPointsAlongPlane();

        }

        // Find new Veretx Point by casting a ray to find the planes intersection between 2 vertices && calculate Uv
        private Vector3 GetRayPlaneIntersectionPointAndUv(Vector3 vertex1, Vector2 vertex1Uv, Vector3 vertex2, Vector2 vertex2Uv, out Vector2 uv)
        {
            float distance = GetDistanceRelativeToPlane(vertex1, vertex2, out Vector3 pointOfIntersection);
            uv = InterpolateUvs(vertex1Uv, vertex2Uv, distance);
            return pointOfIntersection;
        }

        /// Computes the distance from the slice plane
        private float GetDistanceRelativeToPlane(Vector3 vertex1, Vector3 vertex2, out Vector3 pointOfintersection)
        {
            Vector3 direction = vertex2 - vertex1;
            Ray ray = new Ray(vertex1, direction);
            SlicePlane.Raycast(ray, out float distance);
            pointOfintersection = ray.GetPoint(distance);
            return distance;
        }

        /// Attempt to find UV between 2 known vericy Uvs
        private Vector2 InterpolateUvs(Vector2 uv1, Vector2 uv2, float distance)
        {
            Vector2 uv = Vector2.Lerp(uv1, uv2, distance);
            return uv;
        }

        /// Divide and add the triangles, normals, and uvs to array of data
        private void AddSideNormalAndUvs(MeshSide side, Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            if (side == MeshSide.Positive)
            {
                AddTrianglesNormalsAndUvs(ref PSVerticies, ref PSTriangles, ref PSNormals, ref PSUVs, vertex1, normal1, uv1, vertex2, normal2, uv2, vertex3, normal3, uv3);
            }
            else
            {
                AddTrianglesNormalsAndUvs(ref NSVertices, ref NSTriangles, ref NSNormals, ref NSUVs, vertex1, normal1, uv1, vertex2, normal2, uv2, vertex3, normal3, uv3);
            }
        }


        /// Adds the vertices to the mesh  
        private void AddTrianglesNormalsAndUvs(ref List<Vector3> vertices, ref List<int> triangles, ref List<Vector3> normals, ref List<Vector2> uvs, Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            ShiftTriangleIndices(ref triangles);

          //Compute normal if needed
            if(normal1 == null) normal1 = ComputeNormal(vertex1, vertex2, vertex3);

            AddVertNormalUv(ref vertices, ref normals, ref uvs, ref triangles, vertex1, (Vector3)normal1, uv1, 0);

            if (normal2 == null) normal2 = ComputeNormal(vertex2, vertex3, vertex1);

            AddVertNormalUv(ref vertices, ref normals, ref uvs, ref triangles, vertex2, (Vector3)normal2, uv2, 1);

            if (normal3 == null) normal3 = ComputeNormal(vertex3, vertex1, vertex2);

            AddVertNormalUv(ref vertices, ref normals, ref uvs, ref triangles, vertex3, (Vector3)normal3, uv3, 2);
        }

        //Shift Triangle Indices
        private void ShiftTriangleIndices(ref List<int> triangles)
        {
            for (int j = 0; j < triangles.Count; j += 3)
            {
                triangles[j] += 3;
                triangles[j + 1] += 3;
                triangles[j + 2] += 3;
            }
        }

        //Inserts values into specified List parameters
        private void AddVertNormalUv(ref List<Vector3> vertices, ref List<Vector3> normals, ref List<Vector2> uvs, ref List<int> triangles, Vector3 vertex, Vector3 normal, Vector2 uv, int num)
        {
            vertices.Insert(num, vertex);
            uvs.Insert(num, uv);
            normals.Insert(num, normal);
            triangles.Insert(num, num);
        }

        /// Join the points along the plane to the Midpoint
        private void JoinPointsAlongPlane()
        {
            Vector3 Mid = GetMidPoint(out float distance);

            //For each 2 point on the slice plane
            for (int i = 0; i < PointsOnPlane.Count; i += 2)
            {
                Vector3 firstVertex;
                Vector3 secondVertex;

                firstVertex = PointsOnPlane[i];
                secondVertex = PointsOnPlane[i + 1];

                //Compute normal
                Vector3 normal3 = ComputeNormal(Mid, secondVertex, firstVertex);
                normal3.Normalize();

                var direction = Vector3.Dot(normal3, SlicePlane.normal);

                if (direction > 0)
                {
                    AddSideNormalAndUvs(MeshSide.Positive, Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);
                    AddSideNormalAndUvs(MeshSide.Negative, Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                }
                else
                {
                    AddSideNormalAndUvs(MeshSide.Positive, Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                    AddSideNormalAndUvs(MeshSide.Negative, Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);
                }
            }
        }

        // get the MidPoint between the first and furthest point
        private Vector3 GetMidPoint(out float distance)
        {
            if (PointsOnPlane.Count > 0)
            {
                Vector3 firstPoint = PointsOnPlane[0];
                Vector3 furthestPoint = Vector3.zero;
                distance = 0f;

                foreach (Vector3 point in PointsOnPlane)
                {
                    //Initialize furthest point with first point
                    float currentDistance = 0f;
                    currentDistance = Vector3.Distance(firstPoint, point);

                    //if new point is further set it to the new furthest
                    if (currentDistance > distance)
                    {
                        distance = currentDistance;
                        furthestPoint = point;
                    }
                }

                return Vector3.Lerp(firstPoint, furthestPoint, 0.5f);
            }
            else
            {
                distance = 0;
                return Vector3.zero;
            }
        }

        // Gets the point perpendicular to the face defined by the provided vertices        
        private Vector3 ComputeNormal(Vector3 vertex1, Vector3 vertex2, Vector3 vertex3)
        {
            Vector3 side1 = vertex2 - vertex1;
            Vector3 side2 = vertex3 - vertex1;

            Vector3 normal = Vector3.Cross(side1, side2);

            return normal;
        }

        // Setup the mesh object for the specified side
        private void SetMeshData(MeshSide side)
        {
            if (MeshSide.Positive == side)
            {
                posSideMesh = new Mesh();
                posSideMesh.vertices = PSVerticies.ToArray();
                posSideMesh.triangles = PSTriangles.ToArray();
                posSideMesh.normals = PSNormals.ToArray();
                posSideMesh.uv = PSUVs.ToArray();
            }
            else
            {
                negSideMesh = new Mesh();
                negSideMesh.vertices = NSVertices.ToArray();
                negSideMesh.triangles = NSTriangles.ToArray();
                negSideMesh.normals = NSNormals.ToArray();
                negSideMesh.uv = NSUVs.ToArray();
            }
        }
    }
}
