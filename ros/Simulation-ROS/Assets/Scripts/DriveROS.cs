using UnityEngine;
using UnityEditor;
using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.NavigationInterfaces;
using Unity.Robotics.ROSTCPConnector;
 
public class DriveROS : MonoBehaviour
{
    [SerializeField] private List<GameObject> _wheels;
    [SerializeField] private Rigidbody _scoutRb;
    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private GameObject _lidarSensorFront;
    [SerializeField] private GameObject _lidarSensorBack;
    
    private const float _targetThreshold = 0.1f;

    private float _steerInput;
    private float _forwardInput;
    private float _currentDistance;
    private float _maxDistance;
    private float _signedAngle;
    private Vector2 _currentPosition;
    private Vector2 _targetPosition;
    private Vector2 _currentDirection;
    private Vector2 _targetDirection;

    public float maxWater = 40f;
    public float maxEnergy = 100f;
    private float _waterUsagePerPlant = 10f;
    private float _energyUsagePerSecond = 0.005f;
    private float _criticalEnergyLevel = 0.3f;

    public float currentWater;
    public float currentEnergy;
    public bool inRefillZone = false;
    private bool _hasCalledTargetPoint = false;

    public delegate void EnvironmentRestart();
    public event EnvironmentRestart EnvRestart;
    public delegate void TargetReached(bool isBaseStation);
    public event TargetReached TargetPointReached;
 
    private ROSConnection _ros;
    [SerializeField] private string _serviceName = "navigation";
    private float _awaitingResponseUntilTimestamp = -1f;
 
    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService<NavigationRequest, NavigationResponse>(_serviceName);

        currentWater = maxWater;
        currentEnergy = maxEnergy;
        StartEpisode();
    }

    public void StartEpisode()
    {
        CallEnvRestart();

        _hasCalledTargetPoint = false;
        _scoutRb.velocity = Vector3.zero;
        _scoutRb.angularVelocity = Vector3.zero;

        _currentDirection = new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z);
        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _maxDistance = Vector2.Distance(_currentPosition, _targetPosition);
    }
 
    public void FixedUpdate()
    {
        if ((currentEnergy < (maxEnergy * _criticalEnergyLevel) || currentWater < _waterUsagePerPlant) && !_hasCalledTargetPoint)
        {
            Debug.Log("Low on resources! Returning to refill station.");
            CallTargetPointReached(true);
            _hasCalledTargetPoint = true;
        }

        float[] lidarSensorMeasurementsFront = _lidarSensorFront.GetComponent<LidarSensor>().GetLidarSensorMeasurements();
        float[] lidarSensorMeasurementsBack = _lidarSensorBack.GetComponent<LidarSensor>().GetLidarSensorMeasurements();

        if (lidarSensorMeasurementsFront.Any(value => value < 0.04) || 
            lidarSensorMeasurementsBack.Any(value => value < 0.04))
        {
            Debug.Log("About to hit something! HELP! Exiting.");
            EditorApplication.ExitPlaymode();
        }


        Vector2 velocity = new Vector2(_scoutRb.velocity.x, _scoutRb.velocity.z);
        float speed = velocity.magnitude;
        speed *= Mathf.Sign(Vector2.Dot(velocity.normalized, new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z)));

        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _currentDirection.x = _scoutTransform.up.x * -1f;
        _currentDirection.y = _scoutTransform.up.z * -1f;
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _signedAngle = Vector2.SignedAngle(_currentDirection, _targetDirection) / 180f;
        float dist = _maxDistance == 0 ? 0 : _currentDistance / _maxDistance;
 
        NavigationRequest navigationRequest = new NavigationRequest(_signedAngle, speed, dist, lidarSensorMeasurementsFront, lidarSensorMeasurementsBack);
 
        // Send message to ROS and return the response
        _ros.SendServiceMessage<NavigationResponse>(_serviceName, navigationRequest, Callback_Destination);
        _awaitingResponseUntilTimestamp = Time.time + 1.0f;
    }
 
    void Callback_Destination(NavigationResponse response)
    {
        if (!inRefillZone && currentEnergy > 0f)
        {
            foreach (GameObject wheel in _wheels)
            {
                JointController joint = wheel.GetComponent<JointController>();
                joint.forwardInput = response.torque;
                _forwardInput = response.torque;
                joint.steerInput = response.steering;
                _steerInput = response.steering;
                joint.UpdateWheel();
            }
            CheckDone();
            currentEnergy -= Time.fixedDeltaTime * 0.3f *(_energyUsagePerSecond + (Mathf.Abs(_forwardInput) + Mathf.Abs(_steerInput)));
            currentEnergy = Mathf.Clamp(currentEnergy, 0, maxEnergy);
        }
        else
        {
            Debug.Log($"Charging water: {currentWater}, energy: {currentEnergy}");
        }
    }
 
    private void CheckDone()
    {
        if (_currentDistance < _targetThreshold)
        {
            Debug.Log("TARGET REACHED");
            WaterPlant();
            CallTargetPointReached(false);
            return;
        }
    }
 
    public void CollisionDetected()
    {
        Debug.Log("COLLISION");
        CallEnvRestart();
    }

    public void SetTargetPosition(Vector2 targetPos)
    {
        _targetPosition = targetPos;
    }

    public void WaterPlant()
    {
        if (currentWater >= _waterUsagePerPlant)
        {
            currentWater -= _waterUsagePerPlant;
            Debug.Log($"Plant watered! Remaining water: {currentWater} Remaining energy: {currentEnergy}");
        }
    }

    private void CallEnvRestart()
    {
        EnvRestart?.Invoke();
    }

    private void CallTargetPointReached(bool isBaseStation)
    {
        TargetPointReached?.Invoke(isBaseStation);
    }
}