using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EzySlice;
public class Slicer : MonoBehaviour
{
    //Vital Variables 
    public Material MaterialAfterSlice;
    public LayerMask sliceMask;

    //Other Gameobject Script Variable
    public sawEmulator saw;

    //if listener triggered
    public bool isTouched;

    private void Update()
    {
        if (isTouched == true)
        {
            isTouched = false;

            //Gather Objects and Split them into 2 halfs from Slice plane
            Collider[] objectsToBeSliced = Physics.OverlapBox(transform.position, new Vector3(1, 0.1f, 0.1f), transform.rotation, sliceMask);
            foreach (Collider objectToBeSliced in objectsToBeSliced)
            {

                SlicedHull slicedObject = SliceObject(objectToBeSliced.gameObject, MaterialAfterSlice);

                GameObject upperHullGameobject = slicedObject.CreateUpperHull(objectToBeSliced.gameObject, MaterialAfterSlice);
                GameObject lowerHullGameobject = slicedObject.CreateLowerHull(objectToBeSliced.gameObject, MaterialAfterSlice);


                upperHullGameobject.transform.position = objectToBeSliced.transform.position;
                lowerHullGameobject.transform.position = objectToBeSliced.transform.position;


                MakePhysical(upperHullGameobject);
                MakePhysical(lowerHullGameobject);

                //destroys original gameobject
                Destroy(objectToBeSliced.gameObject);

                //notifies saw to stop
                saw.JustCut();
            }
        }

    }

    //Creates the halfs of original object and the important components
    private void MakePhysical(GameObject obj)
    {
        obj.AddComponent<MeshCollider>().convex = true;
        obj.AddComponent<Rigidbody>();
        obj.layer = LayerMask.NameToLayer("Slicer");
    }


    //Slice the object given transforms
    private SlicedHull SliceObject(GameObject obj, Material crossSectionMaterial = null)
    {
        return obj.Slice(transform.position, transform.up, crossSectionMaterial);
    }

}
