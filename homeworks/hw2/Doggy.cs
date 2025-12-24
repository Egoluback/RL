using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;
using System;
using Random = UnityEngine.Random;
using UnityEngine.InputSystem;

public class DoggyAgent : Agent
{
    [Header("Сервоприводы")]
    public ArticulationBody[] legs;

    [Header("Скорость работы сервоприводов")]
    public float servoSpeed;

    [Header("Тело")]
    public ArticulationBody body;
    private Vector3 defPos;
    private Quaternion defRot;
    public float strenghtMove;

    [Header("Куб (цель)")]
    public GameObject cube;

    [Header("Сенсоры")]
    public Unity.MLAgentsExamples.GroundContact[] groundContacts;

    [Header("Reward settings")]
    public float progressRewardScale = 3.0f;
    public float velocityRewardScale = 0.5f;
    public float stepPenalty = 0.0005f;
    public float idlePenalty = 0.01f;
    public float successReward = 10.0f;
    public float fallPenalty = -2.0f;
    public float successDistance = 1.5f;
    public float fallYThreshold = -0.2f;

    private float distToTarget = 0f;
    private int stepCount = 0;
    private int maxStepsPerEpisode = 5000;

    public override void Initialize()
    {
        distToTarget = Vector3.Distance(body.transform.position, cube.transform.position);
        defRot = body.transform.rotation;
        defPos = body.transform.position;

        Time.timeScale = 1f;
        Time.fixedDeltaTime = 0.02f;
    }

    public void ResetDog()
    {
        Quaternion newRot = Quaternion.Euler(-90, 0, Random.Range(0f, 360f));

        body.TeleportRoot(defPos, newRot);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        for (int i = 0; i < 12; i++)
        {
            MoveLeg(
                legs[i],
                Random.Range(legs[i].xDrive.lowerLimit * 0.1f, legs[i].xDrive.upperLimit * 0.1f)
            );
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.Log("Heuristic");
    }

    public override void OnEpisodeBegin()
    {
        ResetDog();
        stepCount = 0;

        cube.transform.position = new Vector3(Random.Range(-7.5f, 7.5f), 0.21f, Random.Range(-7.5f, 7.5f));
        distToTarget = Vector3.Distance(body.transform.position, cube.transform.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(body.transform.position);
        sensor.AddObservation(body.velocity);
        sensor.AddObservation(body.angularVelocity);
        sensor.AddObservation(body.transform.right);

        sensor.AddObservation(cube.transform.position);

        Vector3 relativePosition = cube.transform.position - body.transform.position;
        sensor.AddObservation(relativePosition);

        Vector3 toCube = (cube.transform.position - body.transform.position).normalized;
        float angleToCube = Vector3.SignedAngle(body.transform.right, toCube, Vector3.up);
        sensor.AddObservation(angleToCube);

        float distanceToCube = Vector3.Distance(body.transform.position, cube.transform.position);
        sensor.AddObservation(distanceToCube);
        
        foreach (var leg in legs)
        {
            sensor.AddObservation(leg.xDrive.target);
            sensor.AddObservation(leg.velocity);
            sensor.AddObservation(leg.angularVelocity);
        }

        foreach (var groundContact in groundContacts)
        {
            sensor.AddObservation(groundContact.touchingGround);
        }
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        stepCount++;

        var actions = vectorAction.ContinuousActions;
        for (int i = 0; i < 12; i++)
        {
            float clampAction = Mathf.Clamp(actions[i], -1f, 1f);

            float angle = Mathf.Lerp(
                legs[i].xDrive.lowerLimit,
                legs[i].xDrive.upperLimit,
                (clampAction + 1f) * 0.5f
            );
            MoveLeg(legs[i], angle);
        }

        if (stepCount > maxStepsPerEpisode)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        Vector3 currentPos = body.transform.position;
        Vector3 targetPos = cube.transform.position;
        float currentDistanceToTarget = Vector3.Distance(currentPos, targetPos);

        if (currentPos.y < fallYThreshold)
        {
            AddReward(fallPenalty);
            EndEpisode();
            return;
        }

        if (currentDistanceToTarget < successDistance)
        {
            AddReward(successReward);
            EndEpisode();
            return;
        }

        float progress = distToTarget - currentDistanceToTarget;
        AddReward(progress * progressRewardScale);
        distToTarget = currentDistanceToTarget;

        Vector3 dirToTarget = (targetPos - currentPos).normalized;
        float velocityToTarget = Vector3.Dot(body.velocity, dirToTarget);
        AddReward(velocityToTarget * velocityRewardScale * Time.fixedDeltaTime);
        
        float lookAlignment = Vector3.Dot(body.transform.right, dirToTarget);
        if (lookAlignment > 0)
        {
            AddReward(lookAlignment * 0.002f);
        }

        if (body.velocity.magnitude < 0.1f)
        {
            AddReward(-idlePenalty);
        }

        AddReward(-stepPenalty);
    }

    public void FixedUpdate()
    {
        Vector3 directionToTarget = (cube.transform.position - body.transform.position).normalized;
        float alignmentFactor = Mathf.Clamp01(Vector3.Dot(body.transform.right, directionToTarget));

        body.AddForce(directionToTarget * strenghtMove * alignmentFactor);
        
        for (int i = 0; i < 12; i++)
        {
             legs[i].AddForce(directionToTarget * strenghtMove / 20f * alignmentFactor);
        }

        RaycastHit hit;
        if (Physics.Raycast(body.transform.position, body.transform.right, out hit))
        {
            if (hit.collider.gameObject == cube)
            {
                body.AddForce(2f * strenghtMove * directionToTarget);
                for (int i = 0; i < 12; i++)
                {
                    legs[i].AddForce(directionToTarget * strenghtMove / 10f);
                }
            }
        }
        Debug.DrawRay(body.transform.position, body.transform.right, Color.white);
    }

    void MoveLeg(ArticulationBody leg, float targetAngle)
    {
        leg.GetComponent<Leg>().MoveLeg(targetAngle, servoSpeed);
    }
}