// Crest Ocean System

// This file is subject to the MIT License as seen in the root of this folder structure (LICENSE)

using UnityEngine;

namespace Crest
{
    /// <summary>
    /// Helper to obtain the ocean surface height at a single location per frame. This is not particularly efficient to sample a single height,
    /// but is a fairly common case.
    /// </summary>
    public class MySampler
    {
        int _N;
        Vector3[] _queryPos;
        Vector3[] _queryResult;
        Vector3[] _queryResultNormal;
        Vector3[] _queryResultVel;

        float _minLength = 0f;

        public MySampler(int N, float minLength)
        {
            this._N = N;
            this._minLength = minLength;

            this._queryResult = new Vector3[_N];
            this._queryResultNormal = new Vector3[_N];
            this._queryResultVel = new Vector3[_N];
        }

        public void Init(Vector3[] i_queryPos)
        {
            _queryPos = i_queryPos;
        }

        public bool Sample(out float[] o_height, out Vector3[] o_normal, out Vector3[] o_surfaceVel)
        {
            var collProvider = OceanRenderer.Instance?.CollisionProvider;
            if (collProvider == null)
            {
                o_height = null;
                o_normal = null;
                o_surfaceVel = null;
                return false;
            }

            var status = collProvider.Query(GetHashCode(), _minLength, _queryPos, _queryResult, _queryResultNormal, _queryResultVel);

            if (!collProvider.RetrieveSucceeded(status))
            {
                o_height = null;
                o_normal = null;
                o_surfaceVel = null;
                return false;
            }

            o_height = new float[_N];
            o_normal = new Vector3[_N];
            o_surfaceVel = new Vector3[_N];

            for (int i = 0; i < _N; i++)
            {
                o_height[i] = _queryPos[i].y -_queryResult[i].y;
                o_normal[i] = _queryResultNormal[i];
                o_surfaceVel[i] = _queryResultVel[i];
            }

            return true;
        }

    }
}