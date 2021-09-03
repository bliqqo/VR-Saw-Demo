using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddToLayerDelay : MonoBehaviour
{
    //The purpose of this script is to prevent the slices from spawning
    //and being sliced automatically by saw

    private float Timer = 0;

    // Update is called once per frame
    void Update()
    {
        Timer += Time.deltaTime;
        if(Timer >= 2.5)
        {
            gameObject.layer = 6;
            Destroy(this);
        }
    }
}
