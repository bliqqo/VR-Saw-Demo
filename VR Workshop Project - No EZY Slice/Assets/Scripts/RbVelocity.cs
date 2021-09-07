using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RbVelocity : MonoBehaviour
{
    private Vector3 oldpos;
    private Vector3 newpos;
    private Vector3 velocity;
    // Start is called before the first frame update
    void Start()
    {

        oldpos = transform.position;
    }

    void Update()
    {
        newpos = transform.position;
        var media = (newpos - oldpos);
        velocity = media / Time.deltaTime;
        oldpos = newpos;
        newpos = transform.position;
    }

    public Vector3 getVelocity()
    {
        return velocity;
    }


}
