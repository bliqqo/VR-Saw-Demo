using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SawDustManager : MonoBehaviour
{
    public ParticleSystem SawDust;
    private SawEmu2 saw;


    // Start is called before the first frame update
    void Start()
    {
        saw = gameObject.GetComponent<SawEmu2>();
    }

    // Update is called once per frame
    void Update()
    {
        if (saw.getCutting() && saw.ShouldMakeNoise())
        {
            if (!SawDust.isPlaying) SawDust.Play();
            var em = SawDust.emission;
            em.enabled = true;

        }
        else
        {
            var em = SawDust.emission;
            em.enabled = false;
        }
    }
}
