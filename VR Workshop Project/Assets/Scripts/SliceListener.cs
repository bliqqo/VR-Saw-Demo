using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SliceListener : MonoBehaviour
{
    public Slicer slicer;

    public BoxCollider cutter;
    public sawEmulator saw;

    private void OnTriggerEnter(Collider other)
    {
        if(saw.getCutting())
        {
            slicer.isTouched = true;
        }
    }
}
