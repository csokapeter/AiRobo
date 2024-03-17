using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

public class DrivingAgent : Agent
{
    [SerializeField] private List<GameObject> _frontWheels;
    [SerializeField] private Rigidbody _roverRb;
    [SerializeField] private Transform _roverTransform;
    
    [SerializeField] private int _maxSteeringAngle = 30;
    [SerializeField] private int _maxMotorTorque = 700;
    private int _steps = 1;
    private float _roverTopSpeed = 2f;
    private const float _targetThreshold = 0.1f;

    private float _steerInput;
    private float _oldDistance;
    private float _currentDistance;
    private float _maxDistance;
    private float _signedAngle;
    private Vector2 _currentPosition;
    private Vector2 _targetPosition;
    private Vector2 _currentDirection;
    private Vector2 _targetDirection;

    public Vector3 target;
    public delegate void EnvironmentRestart();
    public event EnvironmentRestart EnvRestart;

    public override void OnEpisodeBegin()
    {
        CallEnvRestart();

        foreach (GameObject wheel in _frontWheels)
        {
            var wc = wheel.GetComponent<WheelCollider>();
            wc.motorTorque = 0;
            wc.brakeTorque = 0;
        }
        _roverRb.velocity = Vector3.zero;
        _roverRb.angularVelocity = Vector3.zero;

        _steps = 1;

        _currentDirection = new Vector2(_roverTransform.forward.x, _roverTransform.forward.z);
        _currentPosition = new Vector2(_roverTransform.position.x, _roverTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _maxDistance = Vector2.Distance(_currentPosition, _targetPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        _steps++;
        var speed = _roverRb.velocity.magnitude / _roverTopSpeed;
        speed *= Mathf.Sign(Vector3.Dot(_roverRb.velocity.normalized, _roverTransform.forward));

        _currentPosition = new Vector2(_roverTransform.position.x, _roverTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _currentDirection.x = _roverTransform.forward.x;
        _currentDirection.y = _roverTransform.forward.z;
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _signedAngle = Vector2.SignedAngle(_currentDirection, _targetDirection) / 180f;
        var dist = _maxDistance == 0 ? 0 : _currentDistance / _maxDistance;
        sensor.AddObservation(_signedAngle); //1
        sensor.AddObservation(speed); //1
        sensor.AddObservation(dist); //1
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        foreach(GameObject wheel in _frontWheels)
        {
            var wc = wheel.GetComponent<WheelCollider>();
            var forwardInput = Mathf.Clamp(actions.ContinuousActions[0], -1.0f, 1.0f);
            _steerInput = Mathf.Clamp(actions.ContinuousActions[1], -1.0f, 1.0f);
            if (_roverRb.velocity.magnitude < _roverTopSpeed)
                wc.motorTorque = forwardInput * _maxMotorTorque;
            else
                wc.motorTorque = 0;
            wc.steerAngle = _steerInput * _maxSteeringAngle;
        }
        CheckRewards();
    }

    private void CheckRewards()
    {
        if (_steps > 1500)
        {
            AddReward(-150f);
            EndEpisode();
            return;
        }

        if (_roverTransform.localEulerAngles.x > 35 && _roverTransform.localEulerAngles.x < 325
            || _roverTransform.localEulerAngles.z > 35 && _roverTransform.localEulerAngles.z < 325)
        {
            AddReward(-150f);
            EndEpisode();
            return;
        }

        if (_currentDistance < _targetThreshold)
        {
            AddReward(200f);
            EndEpisode();
            return;
        }

        var deltaTargetDist = _oldDistance - _currentDistance;
        AddReward(deltaTargetDist * 10);
        _oldDistance = _currentDistance;

        var forwardReward = Mathf.Abs(Vector2.Dot(_currentDirection, _targetDirection)) * Mathf.Abs(_steerInput) * -0.15f;
        AddReward(forwardReward);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ContinuousActions = actionsOut.ContinuousActions;
        var forwardInput = Input.GetAxis("Vertical");
        var steerInput = Input.GetAxis("Horizontal");

        ContinuousActions[0] = forwardInput;
        ContinuousActions[1] = steerInput;
    }

    public void SetTargetPosition(Vector2 targetPos)
    {
        _targetPosition = targetPos;
    }

    private void CallEnvRestart()
    {
        EnvRestart?.Invoke();
    }
}
