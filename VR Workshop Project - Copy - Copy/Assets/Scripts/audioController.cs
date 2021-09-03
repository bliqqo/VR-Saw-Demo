using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class audioController : MonoBehaviour
{
    public AudioSource SawForward;
    public AudioSource SawBackward;
    private Rigidbody rb;
    private SawEmu2 emu;

    private bool isPlaying;

    private float direction = 1;

    // Start is called before the first frame update
    void Start()
    {
        rb = gameObject.GetComponent<Rigidbody>();
        emu = gameObject.GetComponent<SawEmu2>();
    }

    // Update is called once per frame
    void Update()
    {
        if (emu.getCutting())
        {
            if (emu.ShouldMakeNoise())
            {
                if (DidTheyChangeDirections())
                {
                    direction *= -1;
                    isPlaying = true;

                    SawForward.Stop();
                    SawBackward.Stop();

                    if (emu.getLocalVelocityZ() > 0)
                    {
                        SawForward.Play();
                        Debug.Log("foward Saw being played");
                    }
                    else
                    {
                        SawBackward.Play();
                        Debug.Log("Backward Saw being played");
                    }
                    
                }

                if (SawForward.isPlaying || SawBackward.isPlaying)
                {
                    if (direction > 0)
                    {
                        SawForward.pitch = Mathf.Lerp(.5f, .9f, calLerpPercent());
                        Debug.Log("Forwad Saw being pitched : " + SawForward.pitch);
                    }
                    else
                    {
                        SawBackward.pitch = Mathf.Lerp(.5f, .9f, calLerpPercent());
                        Debug.Log("Backward Saw being pitched : " + SawBackward.pitch);
                    }
                }
                
            }
        }
    }

    private float calLerpPercent()
    {
        float  temp = ((Mathf.Abs(emu.getLocalVelocityZ()) - .6f) / (3f - .6f) * 100);
        Debug.Log("Local X velocity : " + emu.getLocalVelocityZ() + "\n lerpPercent: " + temp);
        return ((Mathf.Abs(emu.getLocalVelocityZ()) - .6f) / (3f - .6f) * 100);
    }


    private void playSound()
    {
        if(direction < 0)
        {
            SawForward.Play();
        }
        else
        {
            SawBackward.Play();
        }
                    isPlaying = true;
    }

    private bool DidTheyChangeDirections()
    {
        if (direction > 0)
        {
            return emu.getLocalVelocityZ() < 0;
        }
        else return emu.getLocalVelocityZ() > 0;
    }

}
