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

        private Ray edgeRay;



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

            Debug.Log("TempWoodTrianglesLength is " + TempWoodTriangles.Length);
            Debug.Log("TempWoodUVsLength is " + TempWoodUVs.Length);

            //For each vertex in WoodMesh
            for (int i = 0; i < TempWoodTriangles.Length; i += 3)
            {
                //Assign Correct Vertex 1 Data
                TriangleVertex1 = TempWoodVerts[TempWoodTriangles[i]];
                TriVertex1Index = Array.IndexOf(TempWoodVerts, TriangleVertex1);
                TriangleVertexUV1 = TempWoodUVs[TempWoodTriangles[i]];
                TriangleVertexNormal1 = TempWoodNormals[TempWoodTriangles[i]];
                TriangleVertex1Side = SlicePlane.GetSide(TriangleVertex1);

                //Assign Correct Vertex 2 Data
                TriangleVertex2 = TempWoodVerts[TempWoodTriangles[i + 1]];
                TriVertex2Index = Array.IndexOf(TempWoodVerts, TriangleVertex2);
                TriangleVertexUV2 = TempWoodUVs[TempWoodTriangles[i + 1]];
                TriangleVertexNormal2 = TempWoodNormals[TempWoodTriangles[i + 1]];
                TriangleVertex2Side = SlicePlane.GetSide(TriangleVertex2);

                //Assign Correct Vertex 3 Data
                TriangleVertex3 = TempWoodVerts[TempWoodTriangles[i + 2]];
                TriangleVertex3Index = Array.IndexOf(TempWoodVerts, TriangleVertex3);
                TriangleVertexUV3 = TempWoodUVs[TempWoodTriangles[i + 2]];
                TriangleVertexNormal3 = TempWoodNormals[TempWoodTriangles[i + 2]];
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
            float distance = GetDistanceRelativeToPlane(vertex1, vertex2, out Vector3 pointOfIntersection, out float MaxDistance);
            uv = InterpolateUvs(vertex1Uv, vertex2Uv, distance, MaxDistance);
            return pointOfIntersection;
        }

        /// Computes the distance from the slice plane
        private float GetDistanceRelativeToPlane(Vector3 vertex1, Vector3 vertex2, out Vector3 pointOfintersection, out float MaxDist)
        {

            edgeRay.origin = vertex1;
            edgeRay.direction = (vertex2 - vertex1).normalized;
            MaxDist = Vector3.Distance(vertex1, vertex2);

            if (!SlicePlane.Raycast(edgeRay, out float dist))
            {
                throw new UnityException("Line-Plane intersect in wrong direction [new UV error code 1]");
            }
            else if (dist > MaxDist)
            {
                throw new UnityException("Intersect outside of line [new UV error code 2]");
            }

            pointOfintersection = edgeRay.GetPoint(dist);
                //Vector3 direction = vertex2 - vertex1;
                //Ray ray = new Ray(vertex1, direction);
                //SlicePlane.Raycast(ray, out float distance);
                //pointOfintersection = ray.GetPoint(distance);
            return dist;
        }

        /// Attempt to find UV between 2 known vericy Uvs
        private Vector2 InterpolateUvs(Vector2 uv1, Vector2 uv2, float distance, float maxDist)
        {
            distance /= maxDist;
            //Vector2 uv = Vector2.Lerp(uv1, uv2, distance);
            Vector2 uv;

            uv.x = Mathf.Lerp(uv1.x, uv2.x, distance);
            uv.y = Mathf.Lerp(uv1.y, uv2.y, distance);
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
            //Reference variables are the global PS or NS attributes

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
            Vector3 side2 = vertex3 - vertex2;

            Vector3 normal = Vector3.Cross(side1, side2).normalized;

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


    //SliceObject modified to edit(cut) part of mesh, rather than slicing it in two
    class ChipMetadata
    {
        private List<Vector3> Verticies;
        private List<int> Triangles;
        private List<Vector2> UVs;
        private List<Vector3> Normals;

        private Mesh remainder;

        private readonly List<Vector3> PointsOnLeftPlane;
        private readonly List<Vector3> PointsOnRightPlane;

        private Plane SlicPlane;
        private Plane LeftPlane;
        private Plane RightPlane;
        private Plane FloorPlane;

        private Mesh WoodMesh;

        private Ray edgeRay;

        //Remainder of mesh after cut
        public Mesh Remainder
        {
            get
            {
                if (remainder == null)
                {
                    remainder = new Mesh();
                }
                SetMeshData();
                return remainder;
            }
        }

        //Constructor
        public ChipMetadata(Plane RightP, Plane LeftP, Plane SlicePlane, Plane floorPlane, Mesh mesh)
        {
            Triangles = new List<int>();
            Verticies = new List<Vector3>();
            UVs = new List<Vector2>();
            Normals = new List<Vector3>();

            PointsOnLeftPlane = new List<Vector3>();
            PointsOnRightPlane = new List<Vector3>();

            SlicPlane = SlicePlane;
            LeftPlane = LeftP;
            RightPlane = RightP;
            FloorPlane = floorPlane;

            WoodMesh = mesh;

            ComputeNewMeshes();
        }

        //Weither it is valid to cut the chip at this location ( do not need to cut when intersecting verticies [on edges])
        public bool ValidChip()
        {
            int[] TempWoodTriangles = WoodMesh.triangles;
            Vector3[] TempWoodVerts = WoodMesh.vertices;

            //Test each verticy and if any inside slice return without cutting
            for (int i = 0; i < TempWoodTriangles.Length; i += 3)
            {
                if (LeftPlane.GetSide(TempWoodVerts[TempWoodTriangles[i]]) && RightPlane.GetSide(TempWoodVerts[TempWoodTriangles[i]]))                  return false;
                if (LeftPlane.GetSide(TempWoodVerts[TempWoodTriangles[i + 1]]) && RightPlane.GetSide(TempWoodVerts[TempWoodTriangles[i + 1]]))          return false;
                if (LeftPlane.GetSide(TempWoodVerts[TempWoodTriangles[i + 2]]) && RightPlane.GetSide(TempWoodVerts[TempWoodTriangles[i + 2]]))            return false;
            }
            return true;
        }

        // Compute the positive and negative meshes based on the plane intersection and mesh
        private void ComputeNewMeshes()
        {
            if (!ValidChip()) return;

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
            bool TriangleVertex1below;

            //Vertex 2 Data
            Vector3 TriangleVertex2;
            int TriVertex2Index;
            Vector2 TriangleVertexUV2;
            Vector3 TriangleVertexNormal2;
            bool TriangleVertex2Side;
            bool TriangleVertex2below;

            //Vertex 3 Data
            Vector3 TriangleVertex3;
            int TriangleVertex3Index;
            Vector2 TriangleVertexUV3;
            Vector3 TriangleVertexNormal3;
            bool TriangleVertex3Side;
            bool TriangleVertex3below;


            //For each vertex in WoodMesh
            for (int i = 0; i < TempWoodTriangles.Length; i += 3)
            {
                Debug.Log("index (i) is " + i);
                //Assign Correct Vertex 1 Data
                TriangleVertex1 = TempWoodVerts[TempWoodTriangles[i]];
                TriVertex1Index = Array.IndexOf(TempWoodVerts, TriangleVertex1);
                TriangleVertexUV1 = TempWoodUVs[TempWoodTriangles[i]];
                TriangleVertexNormal1 = TempWoodNormals[TempWoodTriangles[i]];
                TriangleVertex1Side = SlicPlane.GetSide(TriangleVertex1);
                TriangleVertex1below = FloorPlane.GetSide(TriangleVertex1);

                //Assign Correct Vertex 2 Data
                TriangleVertex2 = TempWoodVerts[TempWoodTriangles[i + 1]];
                TriVertex2Index = Array.IndexOf(TempWoodVerts, TriangleVertex2);
                TriangleVertexUV2 = TempWoodUVs[TempWoodTriangles[i + 1]];
                TriangleVertexNormal2 = TempWoodNormals[TempWoodTriangles[i + 1]];
                TriangleVertex2Side = SlicPlane.GetSide(TriangleVertex2);
                TriangleVertex2below = FloorPlane.GetSide(TriangleVertex2);

                //Assign Correct Vertex 3 Data
                TriangleVertex3 = TempWoodVerts[TempWoodTriangles[i + 2]];
                TriangleVertex3Index = Array.IndexOf(TempWoodVerts, TriangleVertex3);
                TriangleVertexUV3 = TempWoodUVs[TempWoodTriangles[i + 2]];
                TriangleVertexNormal3 = TempWoodNormals[TempWoodTriangles[i + 2]];
                TriangleVertex3Side = SlicPlane.GetSide(TriangleVertex3);
                TriangleVertex3below = FloorPlane.GetSide(TriangleVertex3);

                //All vertex are on the same side or underneith the area being cut (leave these alone and add to mesh)
                if (TriangleVertex1Side == TriangleVertex2Side && TriangleVertex2Side == TriangleVertex3Side || !TriangleVertex1below && !TriangleVertex2below && !TriangleVertex3below)
                {

                    MeshSide side1 = (TriangleVertex1Side) ? MeshSide.Positive : MeshSide.Negative;
                    AddSideNormalAndUvs(TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                    continue;
                }
                else
                {
                    //prepare to store the intersection points of the mesh data
                    Vector3 intersection1 = Vector3.zero;
                    Vector3 intersection2 = Vector3.zero;
                    Vector3 intersection3 = Vector3.zero;
                    Vector3 intersection4 = Vector3.zero;

                    Vector2 intersection1Uv;
                    Vector2 intersection2Uv;
                    Vector2 intersection3Uv;
                    Vector2 intersection4Uv;

                    Vector2 intersectionPointUV;

                    MeshSide side1 = (TriangleVertex1Side) ? MeshSide.Positive : MeshSide.Negative;


                    //Wiether the apex of slice is located within current triangle of mesh (needed for recalculating mesh around center)
                    bool VertexInsideTri = VertexInsideTriangle( false, TriangleVertex1, TriangleVertex2, TriangleVertex3, out Vector3 intersectionPoint);

                    bool SwapPoints = false;

                    //Vertex 1 and 2 on the same side
                    if (TriangleVertex1Side == TriangleVertex2Side)
                    {
                        //if cut Apex is in Triangle
                        if (VertexInsideTri)
                        {

                            //Calculate and assign all possible relivant Vertex intersections with cutting planes
                            //Default points
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection2Uv);
                            //Alternate points
                            intersection3 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            //Gets the UV of the Cut Apex
                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            //if default calculation is above Apex of cut
                            if (FloorPlane.GetSide(intersection1))
                            {
                                SwapPoints = true;
                                AddSideNormalAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                //Attempt to calculate Normals and UVs with interesection
                                AddSideNormalAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                        }
                        //Apex of cut is not inside triangle (so either do nothing if below slice, or calculate what remains if above apex of cut)
                        else
                        {
                                                
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            //If all intersections with the cut planes are beneath apex of cut
                            if (AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {
                                //Add to mesh and continiue;
                                AddSideNormalAndUvs(TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                //Calculate remains after cut
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, TriangleVertex2, null, TriangleVertex2, intersection3, null, intersection3Uv);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }

                        }

                    }
                    //Vertex 1 and 3 are on the same side
                    else if (TriangleVertex1Side == TriangleVertex3Side)
                    {

                        if (VertexInsideTri)
                        {
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                            intersection3 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection4Uv);

                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            if (FloorPlane.GetSide(intersection1))
                            {
                                SwapPoints = true;
                                AddSideNormalAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddSideNormalAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddSideNormalAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                        }
                        else
                        {                       
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection4Uv);

                            if (AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {
                                AddSideNormalAndUvs( TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, TriangleVertex1, null, TriangleVertex1, intersection3, null, intersection3Uv);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }
                            


                        }

                    }
                    //Vertex 1 is alone
                    else if (TriangleVertex3Side == TriangleVertex2Side)
                    {
                        if (VertexInsideTri)
                        {     
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection2Uv);

                            intersection3 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection4Uv);

                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            if (FloorPlane.GetSide(intersection1))
                            {
                                SwapPoints = true;
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddSideNormalAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                           

                        }
                        else
                        {                       
                            intersection1 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv(LeftPlane, TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(RightPlane, TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            if(AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {                                
                                AddSideNormalAndUvs( TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                 AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                 AddSideNormalAndUvs( TriangleVertex3, null, TriangleVertexUV3, TriangleVertex2, null, TriangleVertexUV2, intersection3, null, intersection3Uv);
                                 AddSideNormalAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }
                           
                        }
                        
                    }


                    //If this triangle contains apex of cut for generating mesh of intersection along wood
                    //Warning: this part is not perfect, I am still in the middle of Debugging a better and simpiler mesh for the cross section of the wood
                    //This doesn't look as good as the other implimentation I am debugging, but it is more consistent
                    if (VertexInsideTri)
                    {
                        if (SwapPoints)
                        {
                            //Backup Code notes for later:

                            //if (SlicPlane.GetSide(intersection1)) PointsOnLeftPlane.Add(intersection1);
                            //else PointsOnRightPlane.Add(intersection1);
                            //if (!SlicPlane.GetSide(intersection1)) PointsOnRightPlane.Add(intersection2);
                            //else PointsOnLeftPlane.Add(intersection2);

                            PointsOnLeftPlane.Add(intersection1);
                            PointsOnRightPlane.Add(intersection2);
                        }
                        else
                        {
                            //Backup Code notes for later:

                            //if (SlicPlane.GetSide(intersection3)) PointsOnLeftPlane.Add(intersection3);
                            //else PointsOnRightPlane.Add(intersection3);
                            //if (!SlicPlane.GetSide(intersection4)) PointsOnRightPlane.Add(intersection4);
                            //else PointsOnLeftPlane.Add(intersection4);

                            PointsOnLeftPlane.Add(intersection3);
                            PointsOnRightPlane.Add(intersection4);
                        }
                        PointsOnLeftPlane.Add(intersectionPoint);
                        PointsOnRightPlane.Add(intersectionPoint);
                    }
                    else
                    {
                        PointsOnLeftPlane.Add(intersection1);
                        PointsOnRightPlane.Add(intersection2);
                        PointsOnLeftPlane.Add(intersection3);
                        PointsOnRightPlane.Add(intersection4);
                    }
                    


                }
            }
         
            //Generate points for crossSection of mesh in cut
            JoinPointsAlongPlane();
            //Wind the meshTriangles so they are always facing outwardly
            AddReverseTriangleWinding();
        }


        //Quick method for determining weither points are under the apex of the cut or not
        private bool AllUnderCut(bool switcher, Vector3 Inter1, Vector3 Inter2, Vector3 Inter3, Vector3 Inter4)
        {
            if (switcher)
            {
                if (!FloorPlane.GetSide(Inter1) && !FloorPlane.GetSide(Inter2) && !FloorPlane.GetSide(Inter3) && !FloorPlane.GetSide(Inter4))
                {
                    return true;
                }
                else return false;
            }
            else
            {
                if (FloorPlane.GetSide(Inter1) && FloorPlane.GetSide(Inter2) && FloorPlane.GetSide(Inter3) && FloorPlane.GetSide(Inter4))
                {
                    return true;
                }
                else return false;
            }
        }

        //Calculates if Vertex of the Apex of Cut is inside triangle face
        private bool VertexInsideTriangle(bool flipper, Vector3 vertex1, Vector3 vertex2, Vector3 vertex3, out Vector3 interPoint)
        {

            //Make Plane out of 3 vertices (triangle)
            Plane corePlane = new Plane(vertex1, vertex2, vertex3);

            //Create Planes for each side of triangle
            Plane TempPlane1;
            Plane TempPlane2;
            Plane TempPlane3;

            //Take Triangle sides (2 of 3 vertices) with a third point of depth, and create a plane for each
            TempPlane1 = new Plane(vertex1, vertex2 + 1 * corePlane.normal, vertex2);
            TempPlane2 = new Plane(vertex2, vertex3 + 1 * corePlane.normal, vertex3);
            TempPlane3 = new Plane(vertex3, vertex1 + 1 * corePlane.normal, vertex1);

            //Take Left and Right Slice Plane (cutting into object) and find the point of interseciton with the Triangle's Plane
            if (!planesIntersectAtSinglePoint(LeftPlane, RightPlane, corePlane, out Vector3 intersectionPoint))
            {
                Debug.Log("Could Not Find Point");
            }

            interPoint = intersectionPoint;

            //If the intersection is on the positive side of each plane it is inside the triangle
            if (TempPlane1.GetSide(intersectionPoint) && TempPlane2.GetSide(intersectionPoint) && TempPlane3.GetSide(intersectionPoint))
            {
                Debug.Log("Found Middle");
                return true;
            }

            return false;
        }

        //Find Apex of Cut UV
        private Vector2 getMidUV(Vector3 vertex1, Vector2 VertexUV1, Vector3 vertex2, Vector2 VertexUV2, Vector3 vertex3, Vector2 VertexUV3, Vector3 Mid)
        {
            //Triangle plane
            Plane corePlane = new Plane(vertex1, vertex2, vertex3);

            //Plane of side of triangle
            Plane TempPlane1 = new Plane(vertex1, vertex2 + 1 * corePlane.normal, vertex2);

            //Set up raycast for intersection with tempPlane
            var direction = Mid - vertex3;

            edgeRay.origin = Mid;
            edgeRay.direction = direction;

            //get distance and convert to Vector3 point 
            TempPlane1.Raycast(edgeRay, out float Tempdist);
            var edgepoint = edgeRay.GetPoint(Tempdist);

            //get distance and interpolate known UV's to assist in second calculation
            var Distance = Vector3.Distance(vertex2, edgepoint);

            var uvtemp = InterpolateUvs(VertexUV1, VertexUV2, Distance, 1);

            //repeat process interpolating for Midpoint UV given the caluclated previous UV
            Distance = Vector3.Distance(vertex3, edgepoint);

            return InterpolateUvs(VertexUV3, uvtemp, Distance, 1);

        }


        //Method for calculating a single Vector3 point from 3 planes
        private bool planesIntersectAtSinglePoint(Plane p0, Plane p1, Plane p2, out Vector3 intersectionPoint)
        {

            const float EPSILON = 1e-4f;

            var det = Vector3.Dot(Vector3.Cross(p0.normal, p1.normal), p2.normal);
            if (det < EPSILON)
            {
                det = Vector3.Dot(Vector3.Cross(p1.normal, p0.normal), p2.normal);
                if (det < EPSILON)
                {
                    intersectionPoint = Vector3.zero;
                    return false;
                }

                intersectionPoint =
                    (-(p1.distance * Vector3.Cross(p0.normal, p2.normal)) -
                    (p0.distance * Vector3.Cross(p2.normal, p1.normal)) -
                    (p2.distance * Vector3.Cross(p1.normal, p0.normal))) / det;

                return true;
            }

            intersectionPoint =
                (-(p0.distance * Vector3.Cross(p1.normal, p2.normal)) -
                (p1.distance * Vector3.Cross(p2.normal, p0.normal)) -
                (p2.distance * Vector3.Cross(p0.normal, p1.normal))) / det;

            return true;
        }



        // Find new Veretx Point by casting a ray to find the planes intersection between 2 vertices && calculate Uv
        private Vector3 GetRayPlaneIntersectionPointAndUv(Plane plane, Vector3 vertex1, Vector2 vertex1Uv, Vector3 vertex2, Vector2 vertex2Uv, out Vector2 uv)
        {
            float distance = GetDistanceRelativeToPlane(plane, vertex1, vertex2, out Vector3 pointOfIntersection, out float MaxDistance);
            uv = InterpolateUvs(vertex1Uv, vertex2Uv, distance, MaxDistance);
            return pointOfIntersection;
        }

        /// Computes the distance from the slice plane
        private float GetDistanceRelativeToPlane(Plane plane, Vector3 vertex1, Vector3 vertex2, out Vector3 pointOfintersection, out float MaxDist)
        {

            edgeRay.origin = vertex1;
            edgeRay.direction = (vertex2 - vertex1).normalized;
            MaxDist = Vector3.Distance(vertex1, vertex2);


            //find whichever plane is first intersected and work from there
            //[I am familair this might not be the most efficient way of calculating this, I plane to change this to be more effecient later]
            Plane pln;

            RightPlane.Raycast(edgeRay, out float Rightdist);
            LeftPlane.Raycast(edgeRay, out float Leftdist);

            //whichever plane is closest from orgin in raycast
            if (Rightdist > Leftdist)
            {
                pln = LeftPlane;
            }
            else
            {
                pln = RightPlane;
            }

            if (!pln.Raycast(edgeRay, out float dist))
            {                
                if (dist == 0) throw new UnityException("raycast is parallel");

                throw new UnityException("raycast pointing in wrong direction");

            }
            else if (dist > MaxDist)
            {
                Debug.Log(dist + " > " + MaxDist);
                throw new UnityException("Intersect outside of line [new UV error code 2]");

            }

            pointOfintersection = edgeRay.GetPoint(dist);

            return dist;
        }

        // Attempt to find UV between 2 known vericy Uvs
        private Vector2 InterpolateUvs(Vector2 uv1, Vector2 uv2, float distance, float maxDist)
        {
            distance /= maxDist;
            //Although I know this works, Im sure I didnt have to split it into its vectors, but at the time i didnt want to change it because
            //I was just happy it worked. This will also be looked at later by me when possible
            Vector2 uv;
            uv.x = Mathf.Lerp(uv1.x, uv2.x, distance);
            uv.y = Mathf.Lerp(uv1.y, uv2.y, distance);
            return uv;
        }

        // This was a remnent of old code for I used for slicing object, This function will be deleted and replaced in the future by me
        private void AddSideNormalAndUvs(Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            AddTrianglesNormalsAndUvs(ref Verticies, ref Triangles, ref Normals, ref UVs, vertex1, normal1, uv1, vertex2, normal2, uv2, vertex3, normal3, uv3);
        }


        // Adds the vertices to the mesh  
        private void AddTrianglesNormalsAndUvs(ref List<Vector3> vertices, ref List<int> triangles, ref List<Vector3> normals, ref List<Vector2> uvs, Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            ShiftTriangleIndices(ref triangles);

            //Compute normal if needed
            if (normal1 == null) normal1 = ComputeNormal(vertex1, vertex2, vertex3);

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
            //Reference variables are the global PS or NS attributes

            vertices.Insert(num, vertex);
            uvs.Insert(num, uv);
            normals.Insert(num, normal);
            triangles.Insert(num, num);
        }

        // Join the points along the plane to the Midpoint (used for crossSection of Cut
        // WIll be replaced later by simplier and cleaner method (still in debugging process)
        private void JoinPointsAlongPlane()
        {

           // Get crosssection for Left Side

            Vector3 Mid = GetMidPoint(true, out float Leftdistance);

            //For each 2 point on the slice plane
            for (int i = 0; i < PointsOnLeftPlane.Count; i += 2)
            {
                Vector3 firstVertex;
                Vector3 secondVertex;

                firstVertex = PointsOnLeftPlane[i];
                secondVertex = PointsOnLeftPlane[i + 1];

                //Compute normal
                Vector3 normal3 = ComputeNormal(Mid, secondVertex, firstVertex);
                normal3.Normalize();

                var direction = Vector3.Dot(normal3, LeftPlane.normal);

                if (direction > 0)
                {
                    AddSideNormalAndUvs( Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);

                }
                else
                {
                    AddSideNormalAndUvs( Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                }
            }

            //Get crossSection for RightSide
            Mid = GetMidPoint(false, out float Rightdistance);

            for (int i = 0; i < PointsOnRightPlane.Count; i += 2)
            {
                Vector3 firstVertex;
                Vector3 secondVertex;

                firstVertex = PointsOnRightPlane[i];
                secondVertex = PointsOnRightPlane[i + 1];

                //Compute normal
                Vector3 normal3 = ComputeNormal(Mid, secondVertex, firstVertex);
                normal3.Normalize();

                var direction = Vector3.Dot(normal3, RightPlane.normal);

                if (direction > 0)
                {
                    AddSideNormalAndUvs( Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);

                }
                else
                {
                    AddSideNormalAndUvs( Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);

                }
            }
        }

        // get the MidPoint between the first and furthest point
        private Vector3 GetMidPoint(bool leftplane, out float distance)
        {
            List<Vector3> PointsOnPlane = (leftplane) ? PointsOnLeftPlane : PointsOnRightPlane;
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
            Vector3 side2 = vertex3 - vertex2;

            Vector3 normal = Vector3.Cross(side1, side2).normalized;

            return normal;
        }

        //Solved any Winding Issues by Duplicating and reverse all faces in mesh so they will be visible
        private void AddReverseTriangleWinding()
        {
            int VertsStartIndex = Verticies.Count;
            //Duplicate the original vertices
            Verticies.AddRange(Verticies);
            UVs.AddRange(UVs);
            Normals.AddRange(FlipNormals(Normals));

            int numPositiveTriangles = Triangles.Count;

            //Add reverse windings
            for (int i = 0; i < numPositiveTriangles; i += 3)
            {
                Triangles.Add(VertsStartIndex + Triangles[i]);
                Triangles.Add(VertsStartIndex + Triangles[i + 2]);
                Triangles.Add(VertsStartIndex + Triangles[i + 1]);
            }
        }

        private List<Vector3> FlipNormals(List<Vector3> currentNormals)
        {
            List<Vector3> flippedNormals = new List<Vector3>();

            foreach (Vector3 normal in currentNormals)
            {
                flippedNormals.Add(-normal);
            }

            return flippedNormals;
        }

        // Setup the mesh object for the specified side
        private void SetMeshData()
        {
            remainder = new Mesh();
            remainder.vertices = Verticies.ToArray();
            remainder.triangles = Triangles.ToArray();
            remainder.normals = Normals.ToArray();
            remainder.uv = UVs.ToArray();
        }
    }
}
