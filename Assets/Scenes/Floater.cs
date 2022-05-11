// Crest Ocean System

// This file is subject to the MIT License as seen in the root of this folder structure (LICENSE)

using Crest;
using UnityEngine;
using UnityEngine.InputSystem;

using System.Collections.Generic;
using System;

public struct TriangleData
{
    // sorted in ascending order
    public Vector3[] vs;
    public float[] vs_h;
    public Vector3[] vs_v;

    public Vector3 waterNormal;

    public Vector3 velocity;

    public Vector3 center;
    public Vector3 normal;
    public float area;
    public float depth;


    public bool cw;

    public TriangleData(Vector3[] vs, float[] vs_h, Vector3[] vs_v, Vector3 waterNormal, bool cw) {

        int lowest = Array.IndexOf(vs_h, Mathf.Min(vs_h));
        int highest = Array.IndexOf(vs_h, Mathf.Max(vs_h));

        if (lowest == highest)
        {
            highest = lowest + 1;
        }

        int mid = 3 - lowest - highest;

        // Did we change the vertex direction?
        switch (lowest, mid, highest)
        {
            case (0, 2, 1):
            case (1, 0, 2):
            case (2, 1, 0):
                cw = !cw;
                break;
            default:
                break;
        }


        this.vs = new Vector3[] { vs[lowest], vs[mid], vs[highest] };
        this.vs_h = new float[] { vs_h[lowest], vs_h[mid], vs_h[highest] };
        this.vs_v = new Vector3[] { vs_v[lowest], vs_v[mid], vs_v[highest] };

        this.waterNormal = waterNormal;

        this.center = (vs[0] + vs[1] + vs[2]) / 3;
        this.velocity = (vs_v[0] + vs_v[1] + vs_v[2]) / 3;

        Vector3 c = Vector3.Cross(this.vs[1] - this.vs[0], this.vs[2] - this.vs[0]);
        this.area = c.magnitude / 2;

        this.cw = cw;

        this.normal = c.normalized;

        if (!this.cw)
        {
            this.normal *= -1;
        }


        this.depth = (vs_h[0] + vs_h[1] + vs_h[2]) / 3;

        Debug.DrawLine(this.vs[0], this.vs[1], Color.magenta);
        Debug.DrawLine(this.vs[0], this.vs[2], Color.magenta);
        Debug.DrawLine(this.vs[1], this.vs[2], Color.magenta);

    }

}


public class Floater : MonoBehaviour
{



    float[] last_h = null;
    Vector3[] last_n = null;
    Vector3[] last_v = null;


    public bool inWater;

    Rigidbody _rb;

    MySampler sampler;

    public GameObject child;
    Transform t_child;
    Mesh objMesh;

    int N;

    BuoyancySettings settings;

    void Start()
    {
        _rb = GetComponent<Rigidbody>();


        //}

        t_child = child.transform;
        objMesh = child.GetComponent<MeshFilter>().mesh;

        N = objMesh.vertices.Length;

        sampler = new MySampler(N, 0.1f);

        settings = FindObjectOfType<BuoyancySettings>();
    }

    void FixedUpdate()
    {
        FixedMeshForces();

    }

    float force_int(float h1, float h2)
    {
        return (-1f / 6) * (h1 - h2) * (5 * h1 + 4 * h2);
    }

    float centroid_int(float h1, float h2)
    {
        return (1f / 12) * (h1 - h2) * (h1 - h2) * (3 * h1 + 5 * h2);
    }

    void BuoyancyApplication(TriangleData td, out Vector3[] forces, out Vector3[] centroids)
    {

        // transform this into the same triangle, but with equal height across everywhere




        forces = new Vector3[2] { new Vector3 { x = 0, y = 0, z = 0 }, new Vector3 { x = 0, y = 0, z = 0 } };
        centroids = new Vector3[2] { new Vector3 { x = 0, y = 0, z = 0 }, new Vector3 { x = 0, y = 0, z = 0 } };

        float angoff = Vector3.Angle(Vector3.up, td.normal);

        float ang_from_normal = Vector3.Angle(Vector3.up, td.waterNormal);

        if (Mathf.Abs(angoff - 180) < 1 || Mathf.Abs(angoff) < 1 || (Mathf.Abs(ang_from_normal - 180) > settings.calm_cutoff && Mathf.Abs(ang_from_normal) > settings.calm_cutoff))
        {
            forces[0] = -settings.density * Mathf.Abs(Physics.gravity[1]) * td.normal * td.area * Mathf.Abs(td.depth) * settings.corrective;
            centroids[0] = td.center;
            return;
        }

        Vector3[] vs1 = { td.vs[0], td.vs[1], td.vs[2] };
        float[] v_h1 = { td.vs_h[0], td.vs_h[1], td.vs_h[2] };

        float avgheight = (vs1[0][1] + vs1[1][1] + vs1[2][1] - v_h1[0] - v_h1[1] - v_h1[2]) / 3;

        float[] _v_h1 = { 0, 0, 0 };
        for (int i = 0; i < 3; i++)
        {

            _v_h1[i] = vs1[i][1] + v_h1[i] - avgheight;
            //vs1[i][1] = vs1[i][1] - v_h1[i] + avgheight;
        }

        td = new TriangleData(vs1, _v_h1, td.vs_v, td.waterNormal, td.cw);


        Quaternion rot = Quaternion.FromToRotation(Vector3.up, td.waterNormal);


        Vector3 opoint = (td.vs_h[1] - td.vs_h[2]) / (td.vs_h[0] - td.vs_h[2]) * (td.vs[0] - td.vs[2]) + td.vs[2];
        float o_height = td.vs_h[1];

        Debug.DrawLine(opoint, td.vs[1], Color.blue);



        float Q = (1 / Mathf.Sin(angoff * Mathf.PI / 180f));

        Debug.Assert(Q >= 0);

        Debug.Assert(Q <= 10000000);

        float w1 = Vector3.Distance(td.vs[1], opoint);


        float h1;
        float h2;
        float int1;
        int idx;

        // Q1
        h1 = td.vs_h[1];
        h2 = td.vs_h[2];

        int1 = w1 * Mathf.Abs(force_int(h1, h2));

        idx = 0;


        if (int1 > 0.00001 && int1 < 10000 && td.vs_h[2] - td.vs_h[1] > 0.00001)
        {
            float c = w1 * Mathf.Abs(centroid_int(h1, h2)) / int1;

            forces[idx] = -settings.density * Mathf.Abs(Physics.gravity[1]) * td.normal * int1 * Q;

            Vector3 cen = (td.vs[1] + opoint) / 2;
            cen += (td.vs[2] - cen) * c;

            centroids[idx] = cen; 
            centroids[idx] = td.center;

            //centroids[idx][1] += c / int1;
        }
        else
        {
            forces[idx] = -settings.density * settings.corrective * Mathf.Abs(Physics.gravity[1]) * td.normal * Vector3.Cross(opoint - td.vs[1], opoint - td.vs[2]).magnitude;

            centroids[idx] = td.center;

        }

        // Q3
        h1 = td.vs_h[1];
        h2 = td.vs_h[0];

        int1 = w1 * Mathf.Abs(force_int(h1, h2));

        idx = 1;

        if (int1 > 0.00001 && int1 < 10000 && td.vs_h[1] - td.vs_h[0] > 0.00001)
        {
            float c = w1 * Mathf.Abs(centroid_int(h1, h2)) / int1;

            forces[idx] = -settings.density * Mathf.Abs(Physics.gravity[1]) * td.normal * int1 * Q;

            Vector3 cen = (td.vs[1] + opoint) / 2;
            cen += (td.vs[0] - cen) * c;

            //centroids[idx] = cen;
            centroids[idx] = td.center;

            //centroids[idx][1] -= c / int1;
        }
        else
        {
            forces[idx] = -settings.density * Mathf.Abs(Physics.gravity[1]) * td.normal * Vector3.Cross(opoint - td.vs[0], opoint - td.vs[1]).magnitude;

            centroids[idx] = td.center;

        }


    }

    void FixedMeshForces()
    {
        // Get world vertices
        Vector3[] vertices = new Vector3[N];
        for (int i = 0; i < N; i++)
        {
            Vector3 v = objMesh.vertices[i];
            vertices[i] = t_child.TransformPoint(v);
        }
        sampler.Init(vertices);

        float[] o_height = null;
        Vector3[] o_n = null;
        Vector3[] o_v = null;

        bool succ = sampler.Sample(out o_height, out o_n, out o_v);

        if (succ)
        {
            last_h = o_height;
            last_n = o_n;
            last_v = o_v;
        }
        else
        {
            Debug.Assert(true);
        }

        if (last_h == null)
        {
            return;
        }

        int[] triangle_idcs = objMesh.GetTriangles(0);

        List<TriangleData> td = new List<TriangleData>();

        for (int i = 0; i < triangle_idcs.Length; i += 3)
        {

            Vector3[] vs = { vertices[triangle_idcs[i]], vertices[triangle_idcs[i + 1]], vertices[triangle_idcs[i + 2]], };
            float[] vs_h = { last_h[triangle_idcs[i]], last_h[triangle_idcs[i + 1]], last_h[triangle_idcs[i + 2]], };
            Vector3[] vs_v = { last_v[triangle_idcs[i]], last_v[triangle_idcs[i + 1]], last_v[triangle_idcs[i + 2]] };
            Vector3 v_n = (last_n[triangle_idcs[i]] + last_n[triangle_idcs[i + 1]] + last_n[triangle_idcs[i + 2]] ) / 3;

            TriangleData td1 = new TriangleData(vs, vs_h, vs_v, v_n, true);
            td.Add(td1);
        }

        List<TriangleData> td_underwater = new List<TriangleData>();

        for (int i = 0; i < td.Count; i++)
        {


            if (td[i].vs_h[2] <= 0)
            {
                td_underwater.Add(td[i]);

            }


            else if (td[i].vs_h[1] <= 0)
            {

                float tm = -td[i].vs_h[1] / (td[i].vs_h[2] - td[i].vs_h[1]);
                float tl = -td[i].vs_h[0] / (td[i].vs_h[2] - td[i].vs_h[0]);

                Vector3 im = td[i].vs[1] + tm * (td[i].vs[2] - td[i].vs[1]);
                Vector3 il = td[i].vs[0] + tl * (td[i].vs[2] - td[i].vs[0]);

                Vector3 vm = td[i].vs_v[1] + tm * (td[i].vs_v[2] - td[i].vs_v[1]);
                Vector3 vl = td[i].vs_v[0] + tm * (td[i].vs_v[2] - td[i].vs_v[0]);

                Vector3[] vs1 = { td[i].vs[0], td[i].vs[1], im };
                float[] vs_h1 = { td[i].vs_h[0], td[i].vs_h[1], 0 };
                Vector3[] vs_v1 = { td[i].vs_v[0], td[i].vs_v[1], vm };

                Vector3[] vs2 = { td[i].vs[0], im, il };
                float[] vs_h2 = { td[i].vs_h[0], 0, 0 };
                Vector3[] vs_v2 = { td[i].vs_v[0], vm, vl };


                TriangleData td1 = new TriangleData(vs1, vs_h1, vs_v1, td[i].waterNormal, td[i].cw);
                TriangleData td2 = new TriangleData(vs2, vs_h2, vs_v2, td[i].waterNormal, td[i].cw);

                td_underwater.Add(td1);
                td_underwater.Add(td2);
            }
            else if (td[i].vs_h[0] <= 0)
            {
                float tm = -td[i].vs_h[0] / (td[i].vs_h[1] - td[i].vs_h[0]);
                float th = -td[i].vs_h[0] / (td[i].vs_h[2] - td[i].vs_h[0]);

                Vector3 jm = td[i].vs[0] + tm * (td[i].vs[1] - td[i].vs[0]);
                Vector3 jh = td[i].vs[0] + th * (td[i].vs[2] - td[i].vs[0]);

                Vector3 vm = td[i].vs_v[0] + tm * (td[i].vs_v[1] - td[i].vs_v[0]);
                Vector3 vh = td[i].vs_v[0] + th * (td[i].vs_v[2] - td[i].vs_v[0]);


                Vector3[] vs1 = { td[i].vs[0], jm, jh };
                float[] vs_h1 = { td[i].vs_h[0], 0, 0 };
                Vector3[] vs_v1 = { td[i].vs_v[0], vm, vh };

                TriangleData td1 = new TriangleData(vs1, vs_h1, vs_v1, td[i].waterNormal, td[i].cw);
                td_underwater.Add(td1);
            }
            else if (td[i].vs_h[1] <= 0)
            {

                float tm = -td[i].vs_h[1] / (td[i].vs_h[2] - td[i].vs_h[1]);
                float tl = -td[i].vs_h[0] / (td[i].vs_h[2] - td[i].vs_h[0]);

                Vector3 im = td[i].vs[1] + tm * (td[i].vs[2] - td[i].vs[1]);
                Vector3 il = td[i].vs[0] + tl * (td[i].vs[2] - td[i].vs[0]);

                Vector3 vm = td[i].vs_v[1] + tm * (td[i].vs_v[2] - td[i].vs_v[1]);
                Vector3 vl = td[i].vs_v[0] + tm * (td[i].vs_v[2] - td[i].vs_v[0]);

                Vector3[] vs1 = { td[i].vs[0], td[i].vs[1], im };
                float[] vs_h1 = { td[i].vs_h[0], td[i].vs_h[1], 0 };
                Vector3[] vs_v1 = { td[i].vs_v[0], td[i].vs_v[1], vm };

                Vector3[] vs2 = { td[i].vs[0], im, il };
                float[] vs_h2 = { td[i].vs_h[0], 0, 0 };
                Vector3[] vs_v2 = { td[i].vs_v[0], vm, vl };


                TriangleData td1 = new TriangleData(vs1, vs_h1, vs_v1, td[i].waterNormal, td[i].cw);
                TriangleData td2 = new TriangleData(vs2, vs_h2, vs_v2, td[i].waterNormal, td[i].cw);

                td_underwater.Add(td1);
                td_underwater.Add(td2);
            }
            else if (td[i].vs_h[0] <= 0)
            {
                float tm = -td[i].vs_h[0] / (td[i].vs_h[1] - td[i].vs_h[0]);
                float th = -td[i].vs_h[0] / (td[i].vs_h[2] - td[i].vs_h[0]);

                Vector3 jm = td[i].vs[0] + tm * (td[i].vs[1] - td[i].vs[0]);
                Vector3 jh = td[i].vs[0] + th * (td[i].vs[2] - td[i].vs[0]);

                Vector3 vm = td[i].vs_v[0] + tm * (td[i].vs_v[1] - td[i].vs_v[0]);
                Vector3 vh = td[i].vs_v[0] + th * (td[i].vs_v[2] - td[i].vs_v[0]);


                Vector3[] vs1 = { td[i].vs[0], jm, jh };
                float[] vs_h1 = { td[i].vs_h[0], 0, 0 };
                Vector3[] vs_v1 = { td[i].vs_v[0], vm, vh };

                TriangleData td1 = new TriangleData(vs1, vs_h1, vs_v1, td[i].waterNormal, td[i].cw);
                td_underwater.Add(td1);
            }

        }

        inWater = (td_underwater.Count > 0);


        List<Vector3> forces = new List<Vector3>();

        for (int i = 0; i < td_underwater.Count; i++)
        {
            Vector3 f = new Vector3(0, 0, 0);

            Vector3 effectiveVelocity = _rb.GetPointVelocity(td_underwater[i].center) - td_underwater[i].velocity;


            ////// Calculate reynolds number

            // Travelled length of the fluid, approximately.
            float characteristic_dimension = Mathf.Abs(Vector3.Dot(effectiveVelocity.normalized, td_underwater[i].vs[1] - td_underwater[i].vs[0]))
                                           + Mathf.Abs(Vector3.Dot(effectiveVelocity.normalized, td_underwater[i].vs[2] - td_underwater[i].vs[0]))
                                           + Mathf.Abs(Vector3.Dot(effectiveVelocity.normalized, td_underwater[i].vs[2] - td_underwater[i].vs[1]));

            characteristic_dimension += settings.c_min;

            float reynolds = settings.density * characteristic_dimension * effectiveVelocity.magnitude / settings.viscosity;

            float coef = settings.r_fac / reynolds;

            if (reynolds > settings.cutoff_rey)
            {
                coef = settings.r_fac / settings.cutoff_rey;
            }

            if (coef > settings.max_coef)
            {
                coef = settings.max_coef;
            }

            Vector3 normalV = Vector3.Dot(effectiveVelocity, td_underwater[i].normal) * td_underwater[i].normal;
            Vector3 lateralV = effectiveVelocity - normalV;

            if (Vector3.Dot(effectiveVelocity, td_underwater[i].normal) > 0)
            {

                Vector3 drag = -coef * settings.density * effectiveVelocity.normalized * (effectiveVelocity.magnitude * effectiveVelocity.magnitude);

                f += drag * td_underwater[i].area;

                Debug.DrawRay(td_underwater[i].center, drag * 0.01f);
            }


            //f += density * td_underwater[i].normal * td_underwater[i].area * (td_underwater[i].depth);


            _rb.AddForceAtPosition(Time.fixedDeltaTime * f, td_underwater[i].center);

            BuoyancyApplication(td_underwater[i], out Vector3[] b_forces, out Vector3[] centroids);

            for (int j = 0; j < 2; j++)
            {
                _rb.AddForceAtPosition(Time.fixedDeltaTime * b_forces[j], centroids[j]);
                Debug.DrawRay(centroids[j], b_forces[j] * 0.1f, Color.red);
            }


        }
    }
}


