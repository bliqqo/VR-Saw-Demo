using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class sawEmulator : MonoBehaviour
{
    //Quick storage for variables on GameObject
    public Rigidbody rb;
    public BoxCollider trigger;
    public BoxCollider physic;

    //upwards velocity  needed to break "saw lock"
    public float breakforce;

    //status storage
    private bool iscutting = false;
    private float triggerCounter = 0;
    public float triggerWait;
    private bool checkTriggerEn = false;

    private bool freezeRbLocally = false;

    //Variables storing other GameObject Scripts.
    public RbVelocity handvel;





    void Update()
    {
        //if trigger is timed out
        if (checkTriggerEn)
        {
            if (triggerCounter >= triggerWait)
            {
                triggerCounter = 0;
                trigger.enabled = true;
                checkTriggerEn = false;
            }
            else triggerCounter += Time.deltaTime;
        }
    }

    private void OnTriggerExit(Collider Other)
    {
        iscutting = false;
        JustCut();
    }

    // saw hovering over board
    private void OnTriggerStay(Collider other)
    {
        if (other.gameObject.layer == 6)
        {
            float yVel;

            Vector3 localVelocity = transform.InverseTransformDirection(rb.velocity);

            //if escaped "saw lock"
            if (GetBreakforce())
            {
                //iscutting = false;
                yVel = 0;
                rb.constraints = RigidbodyConstraints.None;
                physic.enabled = true;
                setCheckTrigger();
            }
            //if not
            else
            {
                iscutting = true;
                yVel = -(Mathf.Abs(localVelocity.x / 2f));

                freezeRbLocally = true;
                rb.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;

                if (yVel > -0.0075) physic.enabled = true;
                else physic.enabled = false;
            }
 
            localVelocity.y = yVel;
            if(freezeRbLocally) localVelocity.z = 0;

            rb.velocity = transform.TransformDirection(localVelocity);
        }
        
    }

    //retreaves data relevant to breaking "saw lock"
    public bool GetBreakforce()
    {
        if (handvel.getVelocity().y >= breakforce)
        {
            return true;
        }
        
        return false;
    }

    //Getters, and closers


    public void setCheckTrigger()
    {
        iscutting = false;
        triggerCounter = 0;
        rb.constraints = RigidbodyConstraints.None;
        freezeRbLocally = false;
        trigger.enabled = false;
        checkTriggerEn = true;
    }

    public bool getCutting()
    {
        return iscutting;
    }

    public void JustCut()
    {
        freezeRbLocally = false;
        iscutting = false;
        rb.constraints = RigidbodyConstraints.None;
        physic.enabled = true;
        setCheckTrigger();
    }
}
