using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SliceListener : MonoBehaviour
{
    public Slicer slicer;

    public BoxCollider cutter;
    public SawEmu2 saw;

    private void OnTriggerEnter(Collider other)
    {
        if (saw.ValidCut() && other.gameObject.layer == 6)
        {
            Debug.Log("Heres a cut");
            saw.isTouched = true;
            saw.ToBeCut = other.gameObject;

        }
    }
    private void OnTriggerStay(Collider other)
    {
        if (saw.ValidCut() && other.gameObject.layer == 6)
        {
            Debug.Log("Heres a cut");
            saw.isTouched = true;
            saw.ToBeCut = other.gameObject;

        }
    }
}
