using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using EzySlice;
using Saw;
using Assets.Scripts;


public class Slicer : MonoBehaviour
{
    //Vital Variables 
    public Material MaterialAfterSlice;
    public LayerMask sliceMask;

    //Other Gameobject Script Variable
    public SawEmu2 saw;

    //if listener triggered
    public bool isTouched;
    public GameObject ToBeCut;

    private float Timer = 0;

    private void Update()
    {
        //If ready to make a slice
        if (isTouched == true)
        {
            isTouched = false;
            Debug.Log("rigby cut Before");
            if (ToBeCut.gameObject.layer != 6) return;
            Debug.Log("rigby cut");

            var tranformNormal = ((Vector3)(ToBeCut.gameObject.transform.localToWorldMatrix.transpose * gameObject.transform.up)).normalized;

            Vector3 transformedStartingPoint = ToBeCut.gameObject.transform.InverseTransformPoint(gameObject.transform.position);

            //Plane plane = new Plane(gameObject.transform.up, gameObject.transform.localPosition);
            Plane plane = new Plane(tranformNormal, transformedStartingPoint);

            var direction = Vector3.Dot(Vector3.up, gameObject.transform.up);

            if (direction < 0)
            {
                plane = plane.flipped;
            }

            //GameObject tester = GameObject.CreatePrimitive(PrimitiveType.Plane);
            //tester.transform.up = plane.normal;
            //tester.transform.position = plane.normal * plane.distance;

            GameObject[] slices = SawCut.Slice(plane, ToBeCut.gameObject);
            Destroy(ToBeCut.gameObject);

            saw.SetSawReset();
            saw.SawLock = false;

            //SlicedHull slicedObject = SliceObject(objectToBeSliced.gameObject, MaterialAfterSlice);

            //Debug.Log("what an accomplishment");

            //GameObject upperHullGameobject = slicedObject.CreateUpperHull(objectToBeSliced.gameObject, MaterialAfterSlice);
            //GameObject lowerHullGameobject = slicedObject.CreateLowerHull(objectToBeSliced.gameObject, MaterialAfterSlice);


            //upperHullGameobject.transform.position = objectToBeSliced.transform.position;
            //lowerHullGameobject.transform.position = objectToBeSliced.transform.position;

            //Debug.Log("wowee");

            //MakePhysical(upperHullGameobject);
            //MakePhysical(lowerHullGameobject);

            ////destroys original gameobject
            //Destroy(objectToBeSliced.gameObject);
            //saw.SetSawReset();
            //saw.SawLock = false;
        }


    }



    //Creates the halfs of original object and the important components
    private void MakePhysical(GameObject obj)
    {
        obj.AddComponent<MeshCollider>().convex = true;
        obj.AddComponent<Rigidbody>();
        obj.AddComponent<AddToLayerDelay>();
    }




}
