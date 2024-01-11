using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class DroneAgent : Agent
{

    public Rigidbody rb;
    public Rigidbody motorfr;
    public Rigidbody motorfl;
    public Rigidbody motorrr;
    public Rigidbody motorrl;
    public Transform Target;

    private Vector3 initPosition;
    private Quaternion initQuaternion;

    // Start is called before the first frame update
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        initPosition = rb.transform.position;
        initQuaternion = rb.transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        rb.transform.position = initPosition;
        rb.transform.rotation = initQuaternion;

        motorfr.velocity = Vector3.zero;
        motorfl.velocity = Vector3.zero;
        motorrr.velocity = Vector3.zero;
        motorrl.velocity = Vector3.zero;

        // Move the target to a new spot
        Target.localPosition = new Vector3(0, 0, 0);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(Target.localPosition);
        sensor.AddObservation(rb.transform.localPosition);
        sensor.AddObservation(rb.transform.localRotation);

        // Agent velocity
        sensor.AddObservation(rb.velocity.x);
        sensor.AddObservation(rb.velocity.y);
        sensor.AddObservation(rb.velocity.z);
    }

    private float power = 50;

    public void OnActionReceived(float[] vectorAction)
    {
        // Actions, size = 4
        motorfr.AddRelativeForce(Vector3.up * (vectorAction[0] + 0.5f) * power);
        motorfl.AddRelativeForce(Vector3.up * (vectorAction[1] + 0.5f) * power);
        motorrr.AddRelativeForce(Vector3.up * (vectorAction[2] + 0.5f) * power);
        motorrl.AddRelativeForce(Vector3.up * (vectorAction[3] + 0.5f) * power);

        // Rewards
        float distanceToTarget = Vector3.Distance(rb.transform.localPosition, Target.localPosition);

        // Reached target
        if (distanceToTarget < 0.5f)
        {
            SetReward(1.0f);
        }

        SetReward(-distanceToTarget / 10 + Target.transform.position.y - Mathf.Abs(Target.transform.position.y - rb.transform.localPosition.y));

        // Fell off platform
        if (distanceToTarget > 20.0f
            || (rb.transform.localRotation.eulerAngles.x < 225 || rb.transform.localRotation.eulerAngles.x > 315)
            )
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }
}
