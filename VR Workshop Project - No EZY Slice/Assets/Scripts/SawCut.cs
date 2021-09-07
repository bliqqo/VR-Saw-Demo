using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Assets.Scripts;


namespace Saw
{
    class SawCut
    {
        public static GameObject[] Slice(Plane plane, GameObject objectToCut)
        {
            //Get the current mesh and its verts and tris
            Mesh mesh = objectToCut.GetComponent<MeshFilter>().mesh;
            var a = mesh.GetSubMesh(0);


            //Create left and right slice of hollow object
            SlicesMetadata slicesMeta = new SlicesMetadata(plane, mesh);

            //Create and name Positive slice
            GameObject positiveObject = CreateMeshGameObject(objectToCut);
            positiveObject.name = string.Format("positive: ", objectToCut.name);

            //Create and name Negative Slice
            GameObject negativeObject = CreateMeshGameObject(objectToCut);
            negativeObject.name = string.Format("negative: ", objectToCut.name);

            //Assigning meshes for Sides
            var positiveSideMeshData = slicesMeta.PosSideMesh;
            var negativeSideMeshData = slicesMeta.NegSideMesh;
            positiveObject.GetComponent<MeshFilter>().mesh = positiveSideMeshData;
            negativeObject.GetComponent<MeshFilter>().mesh = negativeSideMeshData;

            positiveObject.GetComponent<MeshFilter>().mesh.RecalculateNormals();
            negativeObject.GetComponent<MeshFilter>().mesh.RecalculateNormals();

            //Side Setup
            SetupCollidersAndRigidBodys(ref positiveObject, positiveSideMeshData);
            SetupCollidersAndRigidBodys(ref negativeObject, negativeSideMeshData);

            return new GameObject[] { positiveObject, negativeObject };
        }

        public static void cut(Plane LeftPlane,Plane RightPlane, Plane SlicePlane , Plane FloorPlane, GameObject objectToCut)
        {
            //Get the current mesh and its verts and tris
            Mesh mesh = objectToCut.GetComponent<MeshFilter>().mesh;
            var a = mesh.GetSubMesh(0);


            //Create left and right slice of hollow object
            ChipMetadata cutMeta = new ChipMetadata(LeftPlane, RightPlane, SlicePlane, FloorPlane,  mesh);

            objectToCut.GetComponent<MeshFilter>().mesh = cutMeta.Remainder;

            objectToCut.GetComponent<MeshFilter>().mesh.RecalculateNormals();

        }

        /// Creates the default mesh game object.
        private static GameObject CreateMeshGameObject(GameObject originalObject)
        {
            var originalMaterial = originalObject.GetComponent<MeshRenderer>().materials;


            //Creates GameObject
            GameObject meshGameObject = new GameObject();

            //Adds Basic Components
            meshGameObject.AddComponent<MeshFilter>();
            meshGameObject.AddComponent<MeshRenderer>();

            meshGameObject.GetComponent<MeshRenderer>().materials = originalMaterial;

            //Applies correct transform
            meshGameObject.transform.localScale = originalObject.transform.localScale;
            meshGameObject.transform.rotation = originalObject.transform.rotation;
            meshGameObject.transform.position = originalObject.transform.position;

            return meshGameObject;
        }

        /// Add mesh collider and rigid body to game object
        private static void SetupCollidersAndRigidBodys(ref GameObject gameObject, Mesh mesh)
        {
            //Applies collider, Rigidbody, and layer to delay making it slicable
            MeshCollider meshCollider = gameObject.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = mesh;
            meshCollider.convex = true;

            gameObject.AddComponent<Rigidbody>();
            gameObject.AddComponent<AddToLayerDelay>();
        }
    }
}

