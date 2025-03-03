using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ResetObj : MonoBehaviour
{
    private Vector3 _startPos;
    private Quaternion _startRot;
    private Drive _agent;
    private Rigidbody _rb;
    
    void Start()
    {
        _rb = gameObject.GetComponent<Rigidbody>();
        _agent = transform.parent.GetComponent<Drive>();
        _agent.EnvRestart += OnReset;

        _startPos = transform.localPosition;
        _startRot = transform.rotation;
    }

    void OnReset()
    {
        transform.localPosition = _startPos;
        transform.localRotation = _startRot;

        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
    }
}
