using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class DroneAgent : Agent
{
    public int step = 0;

    public Rigidbody rb;
    public Rigidbody Rotor_fr;
    public Rigidbody Rotor_fl;
    public Rigidbody Rotor_br;
    public Rigidbody Rotor_bl;
    public Transform Target;

    private Vector3 initPosition;
    private Quaternion initQuaternion;


    // Start is called before the first frame update
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        step = 0;
        // If the Agent fell, zero its momentum
        rb.transform.localPosition = initPosition;
        rb.transform.localRotation = initQuaternion;

        Rotor_fr.velocity = Vector3.zero;
        Rotor_fl.velocity = Vector3.zero;
        Rotor_br.velocity = Vector3.zero;
        Rotor_bl.velocity = Vector3.zero;
        rb.velocity = Vector3.zero;

        // Move the target to a new spot
        rb.transform.localPosition = new Vector3(0, (float)0.4813456, 0);
        Target.transform.localPosition=new Vector3(Random.Range(-5,5), 9, Random.Range(-5,5));
        Debug.Log("Ep begin");
        SetReward(0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log("collect");
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

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var continuousActions = actionBuffers.ContinuousActions;
        AddReward(-0.01f);
        if (continuousActions.Length >= 4)
        {
            for (int i = 0; i < 4; i++)
            {
                float action = continuousActions.Array[continuousActions.Offset + i];

                // Actions, size = numRotors
                switch (i)
                {
                    case 0:
                        Rotor_fr.AddRelativeForce(Vector3.up * (action + 0.5f) * power);
                        break;
                    case 1:
                        Rotor_fl.AddRelativeForce(Vector3.up * (action + 0.5f) * power);
                        break;
                    case 2:
                        Rotor_br.AddRelativeForce(Vector3.up * (action + 0.5f) * power);
                        break;
                    case 3:
                        Rotor_bl.AddRelativeForce(Vector3.up * (action + 0.5f) * power);
                        break;
                        // Add more cases if needed
                }
            }
        }
        // Actions, size = 4
        // Rewards
        SetReward();
    }

    public void SetReward()
    {
        float distanceToTarget = Vector3.Distance(rb.transform.localPosition, Target.localPosition);

        step++;
        Debug.Log("step : " + step);
        Debug.Log("fr : " + Rotor_fr.velocity + "fl : " + Rotor_fl + "br : " + Rotor_br + "bl : " + Rotor_bl);

        // Reached target
        if (distanceToTarget < 0.5f)
        {
            AddReward(10f);
            EndEpisode();
            return;
        }

        SetReward(-distanceToTarget / 10 + Target.transform.position.y - Mathf.Abs(Target.transform.position.y - rb.transform.localPosition.y));

        // Fell off platform

        if (distanceToTarget > 20.0f)
        {
            AddReward(-5f);
            EndEpisode();
            return;
        }

        if (step > 1000)
        {
            AddReward(-10);
            EndEpisode();
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[0] = 1;
            Rotor_fr.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
        }
        else if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
            Rotor_fl.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
        }
        else if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[0] = 1;
            Rotor_br.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
        }
        else if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = 1;
            Rotor_bl.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
        }
        else if(Input.GetKey(KeyCode.Z))
        {
            discreteActionsOut[0] = 1;
            Rotor_fr.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
            Rotor_fl.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
            Rotor_br.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
            Rotor_bl.AddRelativeForce(Vector3.up * (discreteActionsOut[0] + 0.5f) * power);
        }
    }

}
