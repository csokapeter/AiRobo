using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class EnvController : MonoBehaviour
{
    [SerializeField] private int _mapLength = 20;
    [SerializeField] private int _mapWidth = 20;
    [SerializeField] private int _offset = 2;

    [SerializeField] private Transform _roverTransform;
    [SerializeField] private Transform _targetTransform;

    private DrivingAgent _agent;

    void Start()
    {
        _agent = _roverTransform.GetComponent<DrivingAgent>();
        _agent.EnvRestart += InitializeEnv;

        InitializeEnv();
    }

    private void InitializeEnv()
    {
        var startPosition = new Vector3(Random.Range(_offset, _mapLength - _offset), 1f, Random.Range(_offset, _mapWidth - _offset));
        _roverTransform.localPosition = startPosition;

        var startRotation = Quaternion.Euler(0, Random.value * 60, 0);
        _roverTransform.rotation = startRotation;

        _targetTransform.localPosition = new Vector3(Random.Range(_offset, _mapLength - _offset), 1f, Random.Range(_offset, _mapWidth - _offset));
        _agent.target = _targetTransform.position;
        _agent.SetTargetPosition(new Vector2(_targetTransform.position.x, _targetTransform.position.z));
    }
}
