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

    private Collider ColliderMemory;

    private bool checkMakePhysical = false;
    private float MakePhysicalTimer = 0;
    public float MakePhrsicalWait;

    public float TooLowValue;
    public float ReduceYby;

    private float DebugVel;

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

        if (checkMakePhysical)
        {
            if (MakePhysicalTimer >= MakePhrsicalWait)
            {
                MakePhysicalTimer = 0;
                physic.enabled = true;
                checkMakePhysical = false;
            }
            else MakePhysicalTimer += Time.deltaTime;
        }
        
    }

    private void OnTriggerExit(Collider Other)
    {
        Debug.Log("trigger exit");
        iscutting = false;
        if (ColliderMemory = Other) physic.enabled = true;
        JustCut();
    }

    // saw hovering over board
    private void OnCollisionStay(Collision collision)
    {
        Debug.Log("1: CollisionStay");
        if (collision.collider.gameObject.layer == 6)
        {
            Debug.Log("2: Correct Layer");
            ColliderMemory = collision.collider;
            float yVel;

            Vector3 localVelocity = transform.InverseTransformDirection(rb.velocity);

            //if escaped "saw lock"
            if (GetBreakforce())
            {
                Debug.Log("3: They are Pulling up (past breakforce)");
                //iscutting = false;
                yVel = PullBackSaw(collision);
                rb.constraints = RigidbodyConstraints.None;
                setCheckTrigger();
            }
            //if not
            else
            {
                Debug.Log("3: They are sawing");
                iscutting = true;
                yVel = -(Mathf.Abs(localVelocity.x / ReduceYby));

                freezeRbLocally = true;
                rb.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;

                if (yVel > TooLowValue && physic.enabled == false)
                {
                    Debug.Log("3.5: they are arent trying/ speed too low: " + yVel);
                    yVel = PullBackSaw(collision);
                }
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
        Debug.Log("2.5: Velocity: " + handvel.getVelocity().y + " >=  Breakforce: " + breakforce + " so : " + (handvel.getVelocity().y >= breakforce));
        DebugVel = handvel.getVelocity().y;
        if (handvel.getVelocity().y >= breakforce)
        {
            return true;
        }
        
        return false;
    }


    public float PullBackSaw(Collision col)
    {
        Debug.Log("Pulling back saw ");
        var dir = col.contacts[0].point - gameObject.transform.position;
        dir = -dir.normalized;
        checkMakePhysical = true;
        return dir.y;
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


//Ideas:

//change to on collision stay instead of trigger (see if it works)
//    if so use collision and calculate a force away from the wood