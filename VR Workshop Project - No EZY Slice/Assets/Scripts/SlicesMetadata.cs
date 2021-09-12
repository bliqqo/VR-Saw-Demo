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

        //Constructor
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
                    AddTrianglesNormalsAndUvs(side, TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
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

                        //Add new triangles to correct mesh side
                        AddTrianglesNormalsAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, TriangleVertex2, null, TriangleVertexUV2, intersection1, null, intersection1Uv);
                        AddTrianglesNormalsAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv);

                        AddTrianglesNormalsAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv);

                    }
                    //Vertex 1 and 3 are on the same side
                    else if (TriangleVertex1Side == TriangleVertex3Side)
                    {
                        //Cast a ray from v1 to v2 and from v2 to v3 to get the intersections                       
                        intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                        intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                        //Add new triangles to correct mesh side
                        AddTrianglesNormalsAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3);
                        AddTrianglesNormalsAndUvs(side1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv, TriangleVertex3, null, TriangleVertexUV3);

                        AddTrianglesNormalsAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex2, null, TriangleVertexUV2, intersection2, null, intersection2Uv);
                    }
                    //Vertex 1 is alone
                    else
                    {
                        //Cast a ray from v1 to v2 and from v1 to v3 to get the intersection points                       
                        intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                        intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                        //Add new Triangles to correct mesh side
                        AddTrianglesNormalsAndUvs(side1, TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection2, null, intersection2Uv);

                        AddTrianglesNormalsAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex2, null, TriangleVertexUV2, TriangleVertex3, null, TriangleVertexUV3);
                        AddTrianglesNormalsAndUvs(side2, intersection1, null, intersection1Uv, TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv);
                    }

                    //Add the newly created points on the plane.
                    PointsOnPlane.Add(intersection1);
                    PointsOnPlane.Add(intersection2);
                }
            }

            //Create Cross Section for two objects
            JoinPointsAlongPlane();

        }

        // Find new Veretx Point by casting a ray to find the planes intersection between 2 vertices && calculate Uv
        private Vector3 GetRayPlaneIntersectionPointAndUv(Vector3 vertex1, Vector2 vertex1Uv, Vector3 vertex2, Vector2 vertex2Uv, out Vector2 uv)
        {
            //Get Distance for Interpolating the UV
            float distance = GetDistanceRelativeToPlane(vertex1, vertex2, out Vector3 pointOfIntersection, out float MaxDistance);

            //Interpolate UV
            uv = InterpolateUvs(vertex1Uv, vertex2Uv, distance, MaxDistance);

            return pointOfIntersection;
        }

        // Computes the distance from the slice plane
        private float GetDistanceRelativeToPlane(Vector3 vertex1, Vector3 vertex2, out Vector3 pointOfintersection, out float MaxDist)
        {
            //set up raycast 
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

            return dist;
        }

        /// Attempt to find UV between 2 known vericy Uvs
        private Vector2 InterpolateUvs(Vector2 uv1, Vector2 uv2, float distance, float maxDist)
        {
            distance /= maxDist;
            return Vector2.Lerp(uv1, uv2, distance);
        }

        // Divide and add the triangles, normals, and uvs to array of data
        private void AddTrianglesNormalsAndUvs(MeshSide side, Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
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


        // Adds the vertices to the mesh  
        private void AddTrianglesNormalsAndUvs(ref List<Vector3> vertices, ref List<int> triangles, ref List<Vector3> normals, ref List<Vector2> uvs, Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            ShiftTriangleIndices(ref triangles);

            //Compute normal if needed
            if(normal1 == null) normal1 = ComputeNormal(vertex1, vertex2, vertex3);

            //Add them to referenced Mesh
            AddVertNormalUv(ref vertices, ref normals, ref uvs, ref triangles, vertex1, (Vector3)normal1, uv1, 0);

            //Compute normal if needed
            if (normal2 == null) normal2 = ComputeNormal(vertex2, vertex3, vertex1);

            //Add them to referenced Mesh
            AddVertNormalUv(ref vertices, ref normals, ref uvs, ref triangles, vertex2, (Vector3)normal2, uv2, 1);

            //Compute normal if needed
            if (normal3 == null) normal3 = ComputeNormal(vertex3, vertex1, vertex2);

            //Add them to referenced Mesh
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

        // Join the points along the plane to the Midpoint (for creating cross section)
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
                    AddTrianglesNormalsAndUvs(MeshSide.Positive, Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);
                    AddTrianglesNormalsAndUvs(MeshSide.Negative, Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                }
                else
                {
                    AddTrianglesNormalsAndUvs(MeshSide.Positive, Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                    AddTrianglesNormalsAndUvs(MeshSide.Negative, Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);
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

                //For each point on cross section
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
        //List data for Mesh components
        private List<Vector3> Verticies;
        private List<int> Triangles;
        private List<Vector2> UVs;
        private List<Vector3> Normals;

        //Mesh returned after cut
        private Mesh remainder;

        //the original mesh of the object being cut
        private Mesh WoodMesh;

        //points on Left and right cross section of cut, respectivly 
        private readonly List<Vector3> PointsOnLeftPlane;
        private readonly List<Vector3> PointsOnRightPlane;

        //plane for determining which side of cut the vertices are on (left or right)
        private Plane SlicPlane;

        //plane for calculating intersection with left and right side of cut (cross section)
        private Plane LeftPlane;
        private Plane RightPlane;

        //plane for calculating if points are below where the cut takes place
        private Plane FloorPlane;

        
        //storage variable for raycast calculations
        private Ray edgeRay;

        //Storage for the 2 vectors at the apex of the cut
        private List<Vector3> CutCorners;

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
            CutCorners = new List<Vector3>();

            SlicPlane = SlicePlane;
            LeftPlane = LeftP;
            RightPlane = RightP;
            FloorPlane = floorPlane;

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

                //All vertex are on the same side or underneith the area being cut (leave these alone and add to Remainder mesh)
                if (TriangleVertex1Side == TriangleVertex2Side && TriangleVertex2Side == TriangleVertex3Side || !TriangleVertex1below && !TriangleVertex2below && !TriangleVertex3below)
                {

                    MeshSide side1 = (TriangleVertex1Side) ? MeshSide.Positive : MeshSide.Negative;
                    AddTrianglesNormalsAndUvs(TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
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

                    //Wiether the apex of slice is located within current triangle of mesh (needed for recalculating mesh around center, without slice)
                    bool VertexInsideTri = VertexInsideTriangle( false, TriangleVertex1, TriangleVertex2, TriangleVertex3, out Vector3 intersectionPoint);

                    //storage for knowing if vertecies need to be recalculated based on their arrangement in the triangle (if vertex is inside)
                    bool SwapPoints = false;

                    // Why it matters:
                    //
                    //                           /|  (point 2)
                    //         (intersection 2) X |
                    //                         /| |
                    //                        / | |
                    //      (intersection 1) X  | |
                    //                      / \/  |
                    //                     /(apex)|
                    //                    /   /\  |
                    //                   /   | |  |
                    //         (point 1)-----X-X-- (point 3)
                    //                       ^ ^
                    //        (intersection 3) (intersection 4)
                    //
                    //  The normal algorithm will work regardless of which vertex is above another (of the two on the same side) as long as there is not the
                    //  apex cut in the triangle.
                    //
                    //  If the apex cut is in the triangle face, it does matter for identify how to make new trianlges.
                    //  because it calculates with planes (which are infinte) techincally there are also intersection below the apex cut 
                    //  So if the method of finding intersections for this triangle is done with raycasting (like i have it), It is 
                    //  nessassary to record each of the possible points it could be and swap which ones are used after calculations to determine which are below apex
                    //  
                    //  (in the above example you would need to cast from point 2 to point 1, and then back (for intersection 1 and intersection 2 respectivly)).
                    //  But if those were below the apex, you would need to cast from point 3 to point1, and then back (for intersection 3 and intersection 4 respectivly).



                    //Vertex 1 and 2 on the same side
                    if (TriangleVertex1Side == TriangleVertex2Side)
                    {
                        //if cut Apex is in Triangle
                        if (VertexInsideTri)
                        {

                            //Calculate and assign all possible relevant Vertex intersections with cutting planes
                            //Default points
                            intersection1 = GetRayPlaneIntersectionPointAndUv( TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv( TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection2Uv);
                            //Alternate points
                            intersection3 = GetRayPlaneIntersectionPointAndUv( TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv( TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            //Gets the UV of the Cut Corner
                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            //If the first set of intersections is above the Apex of Cut
                            if (FloorPlane.GetSide(intersection1))
                            {
                                //swap the points recorded at the end of loop
                                SwapPoints = true;

                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddTrianglesNormalsAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                        }
                        //Apex of cut is not inside triangle (so either do nothing if below slice, or calculate what remains if above apex of cut)
                        else
                        {
                            //Find the intersections with the left and right slice planes         
                            intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            //If all intersections with the cut planes are beneath apex of cut
                            if (AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {
                                //Add to mesh and continiue;
                                AddTrianglesNormalsAndUvs(TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                //Calculate remains after cut
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, TriangleVertex2, null, TriangleVertexUV2, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }

                        }

                    }
                    //Vertex 1 and 3 are on the same side
                    else if (TriangleVertex1Side == TriangleVertex3Side)
                    {
                        //if cut Apex is in Triangle
                        if (VertexInsideTri)
                        {
                            //Calculate and assign all possible relevant Vertex intersections with cutting planes
                            //Default points
                            intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);

                            //Alternate points
                            intersection3 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection4Uv);

                            //Gets the UV of the Cut Corner
                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            //If the first set of intersections is above the Apex of Cut
                            if (FloorPlane.GetSide(intersection1))
                            {
                                //swap the points recorded at the end of loop
                                SwapPoints = true;

                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddTrianglesNormalsAndUvs(TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddTrianglesNormalsAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                        }
                        //If Apex of cut is not inside triangle
                        else
                        {   
                            //Find the intersections with the left and right slice planes
                            intersection1 = GetRayPlaneIntersectionPointAndUv( TriangleVertex3, TriangleVertexUV3, TriangleVertex2, TriangleVertexUV2, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv( TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv( TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv( TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection4Uv);

                            //if all the intersections are below the height of the cut (they dont need to be alterned)
                            if (AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {
                                //Add them to Remainder Mesh
                                AddTrianglesNormalsAndUvs( TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                //Add new triangles to remainder mesh
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, TriangleVertex1, null, TriangleVertexUV1, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }
                        }

                    }
                    //Vertex 2 and 3 are on the same side 
                    else if (TriangleVertex3Side == TriangleVertex2Side)
                    {
                        //if cut Apex is in Triangle
                        if (VertexInsideTri)
                        {
                            //Calculate and assign all possible relevant Vertex intersections with cutting planes
                            //Default points
                            intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection2Uv);

                            //Alternate points
                            intersection3 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection4Uv);
                            
                            //Gets the UV of the Cut Corner
                            intersectionPointUV = getMidUV(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, TriangleVertex3, TriangleVertexUV3, intersectionPoint);

                            //If the first set of intersections is above the Apex of Cut
                            if (FloorPlane.GetSide(intersection1))
                            {
                                //swap the points recorded at the end of loop
                                SwapPoints = true;

                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersection1, null, intersection1Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex3, null, TriangleVertexUV3);
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddTrianglesNormalsAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, intersection2, null, intersection2Uv);
                            }
                            else
                            {
                                //Create appropriate triangles and add them to remainder mesh
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersection3, null, intersection3Uv, intersectionPoint, null, intersectionPointUV);
                                AddTrianglesNormalsAndUvs( TriangleVertex3, null, TriangleVertexUV3, intersectionPoint, null, intersectionPointUV, TriangleVertex2, null, TriangleVertexUV2);
                                AddTrianglesNormalsAndUvs( TriangleVertex2, null, TriangleVertexUV2, intersectionPoint, null, intersectionPointUV, TriangleVertex1, null, TriangleVertexUV1);
                                AddTrianglesNormalsAndUvs( TriangleVertex1, null, TriangleVertexUV1, intersectionPoint, null, intersectionPointUV, intersection4, null, intersection4Uv);
                            }

                           

                        }
                        //Apex of cut is not inside triangle (so either do nothing if below slice, or calculate what remains if above apex of cut)
                        else
                        {
                            //Find the intersections with the left and right slice planes
                            intersection1 = GetRayPlaneIntersectionPointAndUv(TriangleVertex3, TriangleVertexUV3, TriangleVertex1, TriangleVertexUV1, out intersection1Uv);
                            intersection2 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex3, TriangleVertexUV3, out intersection2Uv);
                            intersection3 = GetRayPlaneIntersectionPointAndUv(TriangleVertex2, TriangleVertexUV2, TriangleVertex1, TriangleVertexUV1, out intersection3Uv);
                            intersection4 = GetRayPlaneIntersectionPointAndUv(TriangleVertex1, TriangleVertexUV1, TriangleVertex2, TriangleVertexUV2, out intersection4Uv);

                            //If all intersections with the cut planes are beneath apex of cut
                            if (AllUnderCut(true, intersection1, intersection2, intersection3, intersection4))
                            {
                                //Add to mesh and continiue;
                                AddTrianglesNormalsAndUvs( TriangleVertex1, TriangleVertexNormal1, TriangleVertexUV1, TriangleVertex2, TriangleVertexNormal2, TriangleVertexUV2, TriangleVertex3, TriangleVertexNormal3, TriangleVertexUV3);
                                continue;
                            }
                            else
                            {
                                //Add new triangles to remainder mesh
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, intersection1, null, intersection1Uv, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs(TriangleVertex3, null, TriangleVertexUV3, TriangleVertex2, null, TriangleVertexUV2, intersection3, null, intersection3Uv);
                                AddTrianglesNormalsAndUvs(TriangleVertex1, null, TriangleVertexUV1, intersection2, null, intersection2Uv, intersection4, null, intersection4Uv);
                            }
                           
                        }
                        
                    }


                    //If triangle contains Apex of cut / Corner of Cut
                    if (VertexInsideTri)
                    {
                        //If the vertexes (on the same side) are  not reversed
                        if (SwapPoints)
                        {
                            //Add the first intersection points

                            //checking  if the first pair of points are correctly rotated in relation to the slice plane (Near and far side points will automatically think
                            //they are on the wrong side) so check and reverse them in nessesary
                            if (SlicPlane.GetSide(intersection1))
                            {
                                PointsOnLeftPlane.Add(intersection1);
                                PointsOnRightPlane.Add(intersection2);
                            }
                            else
                            {
                                PointsOnLeftPlane.Add(intersection2);
                                PointsOnRightPlane.Add(intersection1);
                            }
                            
                        }
                        else
                        {
                            //If the points are flipped

                            //Again check if they are on the correct side of the cut, to place them appropriately 
                            if (SlicPlane.GetSide(intersection3))
                            {
                                PointsOnLeftPlane.Add(intersection3);
                                PointsOnRightPlane.Add(intersection4);
                            }
                            else
                            {
                                PointsOnLeftPlane.Add(intersection4);
                                PointsOnRightPlane.Add(intersection3);
                            }
                        }
                        //Add verticy to the memory of cut corners
                        CutCorners.Add(intersectionPoint);

                        //Add to left and right cross section planes
                        PointsOnLeftPlane.Add(intersectionPoint);
                        PointsOnRightPlane.Add(intersectionPoint);
                    }
                    //If not part of the cut corner, but were cut
                    else
                    {
                        //because all 4 intersection points are used for a full slice of triangle face, all 4 will beed to be assigned

                        //Checks if the first pair are on the correct side, and if not it reverses and adds them
                        if (SlicPlane.GetSide(intersection1))
                        {
                            PointsOnLeftPlane.Add(intersection1);
                            PointsOnRightPlane.Add(intersection2);
                        }
                        else
                        {
                            PointsOnLeftPlane.Add(intersection2);
                            PointsOnRightPlane.Add(intersection1);
                        }

                        //Checks if the secon pair are on the correct side, and if not it reverses and adds them
                        if (SlicPlane.GetSide(intersection3))
                        {
                            PointsOnLeftPlane.Add(intersection3);
                            PointsOnRightPlane.Add(intersection4);
                        }
                        else
                        {
                            PointsOnLeftPlane.Add(intersection4);
                            PointsOnRightPlane.Add(intersection3);
                        }
                        
                    }
                }
            }

            //Adds last points for Cross Section of mesh using memory of cut corners
            PointsOnLeftPlane.Add(CutCorners[0]);
            PointsOnLeftPlane.Add(CutCorners[1]);
            PointsOnRightPlane.Add(CutCorners[0]);
            PointsOnRightPlane.Add(CutCorners[1]);

            //Generates rest of cross section
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

            //get distance from vertex to plane intersection point
            var DistanceFromVecTwo = Vector3.Distance(vertex2, edgepoint);

            //get percentage of distance from the total distance of the side of the triangle
            var DistancePercentage = DistanceFromVecTwo /Vector3.Distance(vertex2, vertex1);

            //interpolate the UV for the distance ( to be used to interpolate once more for Midpoint Uv)
            var uvtemp = InterpolateUvs(VertexUV1, VertexUV2, DistancePercentage, 1);

            //repeat process interpolating for Midpoint UV given the caluclated previous UV
            var DistanceFromVecThree = Vector3.Distance(vertex3, Mid);
            DistancePercentage = DistanceFromVecThree / Vector3.Distance(vertex3, edgepoint);

            //Interpolate final Uv
            return InterpolateUvs(VertexUV3, uvtemp, DistancePercentage, 1);

        }


        //Method for calculating a single Vector3 point from 3 planes
        private bool planesIntersectAtSinglePoint(Plane p0, Plane p1, Plane p2, out Vector3 intersectionPoint)
        {

            const float EPSILON = 1e-4f;

            var det = Vector3.Dot(Vector3.Cross(p0.normal, p1.normal), p2.normal);
            if (det < EPSILON)
            {
                //If Error (no interseciton) try reversing plane 1 and plane 0

                det = Vector3.Dot(Vector3.Cross(p1.normal, p0.normal), p2.normal);
                if (det < EPSILON)
                {
                    //Not calculatable, no true intersection
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
        private Vector3 GetRayPlaneIntersectionPointAndUv(Vector3 vertex1, Vector2 vertex1Uv, Vector3 vertex2, Vector2 vertex2Uv, out Vector2 uv)
        {
            //Get Distance from vertexes
            float distance = GetDistanceRelativeToPlane(vertex1, vertex2, out Vector3 pointOfIntersection, out float MaxDistance);
            
            //Interpolate the Vertex UV's to find the UV at a given point (with distance)
            uv = InterpolateUvs(vertex1Uv, vertex2Uv, distance, MaxDistance);

            return pointOfIntersection;
        }

        /// Computes the distance from the slice plane
        private float GetDistanceRelativeToPlane(Vector3 vertex1, Vector3 vertex2, out Vector3 pointOfintersection, out float MaxDist)
        {
            //Set up Raycast given orgin and direction 
            edgeRay.origin = vertex1;
            edgeRay.direction = (vertex2 - vertex1).normalized;
            
            //Set max distance it should be allowed to go
            MaxDist = Vector3.Distance(vertex1, vertex2);


            //find whichever plane is first intersected and work from there
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

            //if the plane does not interect raycast
            if (!pln.Raycast(edgeRay, out float dist))
            {                
                //If the raycast is parrallel to plane
                if (dist == 0) throw new UnityException("raycast is parallel");

                throw new UnityException("raycast pointing in wrong direction");

            }
            else if (dist > MaxDist)
            {
                //If Distance is longer than raycast
                Debug.Log(dist + " > " + MaxDist);
                throw new UnityException("Intersect outside of line [new UV error code 2]");

            }

            pointOfintersection = edgeRay.GetPoint(dist);

            return dist;
        }

        // Attempt to find UV between 2 known vericy Uvs
        private Vector2 InterpolateUvs(Vector2 uv1, Vector2 uv2, float distance, float maxDist)
        {
            //Get percentage of distance given the actual distance 
            distance /= maxDist;

            //interpolate UV
            return Vector2.Lerp(uv1, uv2, distance);
        }

        // Adds the vertices to the mesh  
        private void AddTrianglesNormalsAndUvs(Vector3 vertex1, Vector3? normal1, Vector2 uv1, Vector3 vertex2, Vector3? normal2, Vector2 uv2, Vector3 vertex3, Vector3? normal3, Vector2 uv3)
        {
            ShiftTriangleIndices();

            //Compute 1st normal if needed
            if (normal1 == null) normal1 = ComputeNormal(vertex1, vertex2, vertex3);
            
            //Add 1st to Mesh
            AddVertNormalUv(vertex1, (Vector3)normal1, uv1, 0);

            //Compute 2nd normal if needed
            if (normal2 == null) normal2 = ComputeNormal(vertex2, vertex3, vertex1);

            //Add 2nd to Mesh
            AddVertNormalUv(vertex2, (Vector3)normal2, uv2, 1);

            //Compute 3rd normal if needed
            if (normal3 == null) normal3 = ComputeNormal(vertex3, vertex1, vertex2);

            //Add 3rd to Mesh
            AddVertNormalUv(vertex3, (Vector3)normal3, uv3, 2);
        }

        //Shift Triangle Indices
        private void ShiftTriangleIndices()
        {
            for (int j = 0; j < Triangles.Count; j += 3)
            {
                Triangles[j] += 3;
                Triangles[j + 1] += 3;
                Triangles[j + 2] += 3;
            }
        }

        //Inserts values into specified List parameters
        private void AddVertNormalUv(Vector3 vertex, Vector3 normal, Vector2 uv, int num)
        {
            Verticies.Insert(num, vertex);
            UVs.Insert(num, uv);
            Normals.Insert(num, normal);
            Triangles.Insert(num, num);
        }

        // Join the points along the plane to the Midpoint (used for crossSection of Cut
        private void JoinPointsAlongPlane()
        {
           // Create Cross Section for Left Side

            Vector3 Mid = GetMidPoint(true, out float Leftdistance);

            //For each 2 point on the slice plane ( 2 Verticies & the midpoint will create CrossSection Triangle Faces)
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
                    AddTrianglesNormalsAndUvs( Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);

                }
                else
                {
                    AddTrianglesNormalsAndUvs( Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);
                }
            }


            //Create crossSection for RightSide ( 2 Verticies & the midpoint will create CrossSection Triangle Faces)

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
                    AddTrianglesNormalsAndUvs( Mid, -normal3, Vector2.zero, firstVertex, -normal3, Vector2.zero, secondVertex, -normal3, Vector2.zero);

                }
                else
                {
                    AddTrianglesNormalsAndUvs( Mid, normal3, Vector2.zero, secondVertex, normal3, Vector2.zero, firstVertex, normal3, Vector2.zero);

                }
            }
        }

        // get the MidPoint between the first and furthest point (for Cross Section)
        private Vector3 GetMidPoint(bool leftplane, out float distance)
        {
            //Weither or not we are getting the midpoint for the Left or the Right side of the Cut
            List<Vector3> PointsOnPlane = (leftplane) ? PointsOnLeftPlane : PointsOnRightPlane;

            if (PointsOnPlane.Count > 0)
            {
                Vector3 firstPoint = PointsOnPlane[0];
                Vector3 furthestPoint = Vector3.zero;
                distance = 0f;

                //Tests Each vertex to see if it is further from the first point than the others
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

                //Get Midpoint between First and Furthest point
                return Vector3.Lerp(firstPoint, furthestPoint, 0.5f);
            }
            else
            {
                //There is no points on plane (cannot compute)
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

        //Reverses Normals
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
