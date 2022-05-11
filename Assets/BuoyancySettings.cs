using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BuoyancySettings : MonoBehaviour
{

    public float density = 100f;
    public float viscosity = 100f;
    public float cutoff_rey = 1000f;

    public float max_coef = 10f;

    public float calm_cutoff = 3f;

    public float corrective;

    public float r_fac = 1f;

    public float c_min = 1f;

}
