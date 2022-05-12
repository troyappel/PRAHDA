// Crest Ocean System

// This file is subject to the MIT License as seen in the root of this folder structure (LICENSE)

using Crest;
using UnityEngine;
using UnityEngine.InputSystem;

using System.Collections.Generic;
using System;




public class MyBoat : MonoBehaviour
{

    [Header("Engine Power")]
    [Tooltip("Vertical offset for where engine force should be applied.")]
    public float _forceHeightOffset = -0.3f;
    public float _enginePower = 11f;
    public float _turnPower = 1.3f;


    [Header("Controls")]
    public bool _playerControlled = true;
    [Tooltip("Used to automatically add throttle input")]
    public float _throttleBias = 0f;
    [Tooltip("Used to automatically add turning input")]
    public float _steerBias = 0f;



    bool inWater;
    Rigidbody _rb;


    void Start()
    {
        _rb = GetComponent<Rigidbody>();

    }

    void FixedUpdate()
    {

        inWater = GetComponent<Floater>().inWater;

        if (inWater)
        {
            var forcePosition = _rb.position + _forceHeightOffset * Vector3.up;

            float forward = _throttleBias;
            float rawForward = !Application.isFocused ? 0 : ((Keyboard.current.wKey.isPressed ? 1 : 0) + (Keyboard.current.sKey.isPressed ? -1 : 0));
            if (_playerControlled) forward += rawForward;
            _rb.AddForceAtPosition(transform.forward * _enginePower * forward, forcePosition, ForceMode.Acceleration);

            float reverseMultiplier = (rawForward < 0f ? -1f : 1f);
            float sideways = _steerBias;
            if (_playerControlled) sideways +=
                    !Application.isFocused ? 0 :
                    ((Keyboard.current.aKey.isPressed ? reverseMultiplier * -1f : 0f) +
                    (Keyboard.current.dKey.isPressed ? reverseMultiplier * 1f : 0f));

            _rb.AddTorque(transform.up * _turnPower * sideways, ForceMode.Acceleration);

        }

    }
}