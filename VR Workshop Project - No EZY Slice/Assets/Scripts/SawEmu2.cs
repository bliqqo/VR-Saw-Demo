using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Saw;

public class SawEmu2 : MonoBehaviour
{
    public Slicer slicer;
    public SliceListener sliceListener;

    public GameObject LeftPlane;
    public GameObject rightPlane;
    public GameObject SlicePlane;
    public GameObject FloorPlane;


    //Saw Reset Variables
    private bool SawReset = false;
    private float SRTimer = 0;
    public float SRLimit;

    //Saw Lock
    public bool SawLock = false;

    private bool InsideWood = false;

    private float cuttimer = 0;

    //gameObject component variables
    public Rigidbody rb;
    public BoxCollider trigger;
    public BoxCollider physic;

    private bool JDetach;            //Memory of detatching joint
    public ConfigurableJoint Joint;  //Saw Joint

    //Saw ConfigurableJoint storage
    private float JSMax = 10;
    private float JSDamp = 20;
    private float JSSpring = 200;
    private float JSDriveSpring = 450;
    private float JSDriveDamp = 50;
    private float JSDriveMax = 3.402823e+38f;

    //GameObject & Data Storage
    private GameObject Wood;
    public GameObject WoodBackup;
    public GameObject hand;
    private Vector3 HOld; //Hand old Position Vector
    private Vector3 Hnew; //Hand new Position Vector

    //Variables to adjust experience with handsaw 
    public float ForcePush = 0; //force to push out of wood
    public float TooLowValue; //Determines Staying still sawing 
    public float ReduceYby;   //Determines speed of saw
    public float breakforce; //Force determined to break saw lock

    private int TriggeredObjects = 0;
    private float localVelZ;


    void Start()
    {
        //gets hand for judging distance
        HOld = hand.transform.position;
    }

    void Update()
    {
        //Timer for determining saw lock reset
        if (SawReset)
        {
            //if (InsideWood) return;
            if (SRTimer >= SRLimit)
            {
                SRTimer = 0;
                SawReset = false;
                physic.enabled = true;
            }
            else SRTimer += Time.deltaTime;
        }
        else
        {
            //determines how long the user has been cutting
            if (getCutting())
            {
                cuttimer += Time.deltaTime;
            }
            else cuttimer = 0;
        }
        
    }



    void FixedUpdate()
    {
        //if saw needs to be reset
        if (SawReset)
        {
            //and inside wood
            if (InsideWood)
            {
                Pushout();
            }
            else
            {
                physic.enabled = true;
            }
        }

        else
        {
            //if Saw is not reset & is prepared for lock
            if (SawLock)
            {
                //if not pulling away hand or slamming saw into plank
                Debug.Log("2.5: Velocity: " + GetHandDistance() + " >=  Breakforce: " + breakforce + " so : " + (GetHandDistance() >= breakforce));
                if (GetHandDistance() > breakforce || -rb.velocity.y > 1)
                {
                    Debug.Log("3: They are Pulling up (past breakforce)");
                    CallCutWood();
                    //reset sawlock
                    if (JDetach) RetachJoint();
                    SawLock = false;
                    SetSawReset();
                }
                else
                {
                    Debug.Log("3: They are sawing");
                    Vector3 localVelocity = transform.InverseTransformDirection(rb.velocity);

                    var yVel = (Mathf.Abs(localVelocity.z / ReduceYby));

                    //if they are keeping saw still on plank
                    if (Mathf.Abs(localVelocity.z) < TooLowValue)
                    {
                        Debug.Log("3.5: not trying / speed too low: " + localVelocity.z);
                        yVel = -yVel * ForcePush;
                        rb.angularVelocity = rb.angularVelocity / 2;
                    }

                    localVelZ = localVelocity.z;
                    //set sawlocked velocity
                    localVelocity.y = yVel;
                    localVelocity.x = 0;
                    
                    rb.velocity = transform.TransformDirection(localVelocity);
                }
            }
        }
    }

    //Setter and things that need to be changed with lock
    public void SetSawLock()
    {
        SawLock = true;
        rb.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;
        DetachJoint();
        physic.enabled = false;
    }

    //Setter and things that need to be changed with Reset
    public void SetSawReset()
    {
        SawReset = true;
        rb.constraints = RigidbodyConstraints.None;
        RetachJoint();
    }


    //Emulate slicePlane Object and Slicer Method by setting up and calling the Cut method.
    //(I want to make clean up all code for accessing and using the slice and cut methods but I have not gotten there just yet)
    //(I accessed it though the sawEmu2 on a whim for testing and continued to work)
    public void CallCutWood()
    {
        //Get Plane for Determining left or right of cut
        var tranformNormal = ((Vector3)(Wood.transform.localToWorldMatrix.transpose * SlicePlane.transform.up)).normalized;
        Vector3 transformedStartingPoint = Wood.gameObject.transform.InverseTransformPoint(SlicePlane.transform.position);
        Plane tempSlicePlane = new Plane(tranformNormal, transformedStartingPoint);

        float direction = Vector3.Dot(Vector3.up, SlicePlane.transform.up);
        if (direction < 0)
        {
            tempSlicePlane = tempSlicePlane.flipped;
        }

        //Get Plane for determining the Leftwards edge of cut
        tranformNormal = ((Vector3)(Wood.transform.localToWorldMatrix.transpose * LeftPlane.transform.up)).normalized;
        transformedStartingPoint = Wood.gameObject.transform.InverseTransformPoint(LeftPlane.transform.position);
        Plane tempLeftPlane = new Plane(tranformNormal, transformedStartingPoint);

        direction = Vector3.Dot(Vector3.up, LeftPlane.transform.up);
        if (direction < 0)
        {
            tempLeftPlane = tempLeftPlane.flipped;
        }

        //Get Plane for determining the Rightward edge of cut
        tranformNormal = ((Vector3)(Wood.transform.localToWorldMatrix.transpose * rightPlane.transform.up)).normalized;
        transformedStartingPoint = Wood.gameObject.transform.InverseTransformPoint(rightPlane.transform.position);
        Plane tempRightPlane = new Plane(tranformNormal, transformedStartingPoint);

        direction = Vector3.Dot(Vector3.up, rightPlane.transform.up);
        if (direction < 0)
        {
            tempRightPlane = tempRightPlane.flipped;
        }

        //GetPlane for determining the bottom edge of cut
        tranformNormal = ((Vector3)(Wood.transform.localToWorldMatrix.transpose * FloorPlane.transform.up)).normalized;
        transformedStartingPoint = Wood.gameObject.transform.InverseTransformPoint(FloorPlane.transform.position);
        Plane tempFloorPlane = new Plane(tranformNormal, transformedStartingPoint);

        direction = Vector3.Dot(Vector3.up, FloorPlane.transform.up);
        if (direction < 0)
        {
            tempFloorPlane = tempFloorPlane.flipped;
        }


        Debug.Log("trying to cut");
  
        //Initiate cut
        SawCut.cut(tempLeftPlane, tempRightPlane, tempSlicePlane, tempFloorPlane, Wood.gameObject);
        SetSawReset();
        SawLock = false;
    }

    //Detachs Joint on Y axis for grabbing saw (necessary for controlled saw lock simulation)
    public void DetachJoint()
    {
        JointDrive temp = new JointDrive();
        temp.positionSpring = 0;
        temp.maximumForce = 0;


        Joint.yDrive = temp;
        Joint.angularYZDrive = temp;

        temp.positionSpring = 4;
        temp.maximumForce = 3;
        temp.positionDamper = 4;

        Joint.angularXDrive = temp;

        JDetach = true;
    }

    //Reverts the Joint to its previous state
    public void RetachJoint()
    {
        JointDrive temp = new JointDrive();
        temp.positionSpring = JSSpring;
        temp.maximumForce = JSMax;
        temp.positionDamper = JSDamp;

        Joint.yDrive = temp;

        temp.positionSpring = JSDriveSpring;
        temp.maximumForce = JSDriveMax;
        temp.positionDamper = JSDriveDamp;

        Joint.angularYZDrive = temp;
        Joint.angularXDrive = temp;

        JDetach = false;
    }

    //Quick method for calculating distance from hand to saw (On Y axis) 
    public float GetHandDistance()
    {
        if (hand.transform.position.y < gameObject.transform.position.y) return 0;
        return Vector3.Distance(new Vector3(0, hand.transform.position.y, 0) , new Vector3 (0, gameObject.transform.position.y, 0));
    }

    //calculates force to push the saw outside wood (to be followed by turning its collisions back on)
    public void Pushout()
    {
        var force = gameObject.transform.position - ((Wood) ? Wood.transform.position : WoodBackup.transform.position);
        force = new Vector3(0, force.y, 0);
        force.Normalize();
        rb.AddForce(force * ForcePush);
    }

    //Simple Trigger Enter function
    private void OnTriggerEnter(Collider Other)
    {
        if (Other.gameObject.layer == 6)
        {
            //keeps track of weither the saw is inside of the wood
            InsideWood = true;
        }

        TriggeredObjects++;
    }

    private void OnTriggerStay(Collider Other)
    {
        Debug.Log("1: CollisionStay");
        if (Other.gameObject.layer == 6)
        {
            Debug.Log("2: Correct Layer");

            //Saves reference of what the saw thinks it is interacting with for later usage
            Wood = Other.gameObject;


            //If saw is ready to be locked
            if (!SawReset)
            {
                //If a proper degree is chosen for the saw lock
                if (gameObject.transform.rotation.z > -15 && gameObject.transform.rotation.z < 15)
                {
                    //Initiate Saw Lock
                    SetSawLock();
                }
            }
        }

    }

    private void OnTriggerExit(Collider Other)
    {
        Debug.Log("On Trigger Exit Collider: " + Other.gameObject.name + "TriggeredObjects number: " + TriggeredObjects);

        if (Other.gameObject.layer == 6)
        {
            //If it is leaving the save object it was inside
            
            if (Other.gameObject == Wood)
            {
                
                InsideWood = false;
                //SetSawReset();
                //SawLock = false;
            }
        }

        if (--TriggeredObjects >= 0)
        {
            InsideWood = false;
            //SetSawReset();
            //SawLock = false;
        }
    }

    //If the saw fulfills the requirements for cutting something
    public bool getCutting()
    {
        if (SawLock && !SawReset && InsideWood) return true;
        return false;
    }

    //older Method from First Script
    public void JustCut()
    {
        SetSawLock();
    }

    //IF the saw has been correctly cutting the wood for at least 1 second
    public bool ValidCut()
    {
        if (cuttimer >= .5f) return true;
        return false;
    }

    public bool ShouldMakeNoise()
    {
        return (Mathf.Abs(getLocalVelocityZ()) > (TooLowValue/2));
    }

    public float getLocalVelocityZ()
    {
        return localVelZ;
    }
}

