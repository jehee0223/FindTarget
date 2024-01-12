using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class DroneAgent : Agent
{
    public int step = 0;
    public double reward = 0;

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

        reward = 0;
        SetReward(0f);
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

    private float power = 20;

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var continuousActions = actionBuffers.ContinuousActions;
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
        step++;
        SetReward();
    }

    
    public void SetReward()
    {
        /*-----------------------------------Target 찾아가기-------------------------------*/
        //float distanceToTarget = Vector3.Distance(rb.transform.localPosition, Target.localPosition);

        //Debug.Log("distanceToTarget : "+distanceToTarget);

        //// 거리가 먼 만큼 음수보상
        //AddReward(-(distanceToTarget / 10));
        //reward -= distanceToTarget / 10;

        //// 몸체의 기울기 x,z축 15도 안넘기
        //if (Mathf.Abs(rb.transform.localRotation.x) < 15 && Mathf.Abs(rb.transform.localRotation.z) < 15)
        //{
        //    AddReward(1f);
        //    reward += 1;
        //}
        //else
        //{
        //    AddReward(-1f);
        //    reward -= 1;
        //}

        //if (rb.velocity.x < 7 || rb.velocity.y < 7 || rb.velocity.z < 7)
        //{
        //    AddReward(-1f);
        //}

        //// 너무 멀어지면 음수보상 & 종료
        ////if (distanceToTarget > 20.0f)
        ////{
        ////    AddReward(-10f);
        ////    reward -= 10;
        ////    Debug.Log("End reward : " + reward);
        ////    EndEpisode();
        ////}

        //// 너무 오래걸리면 종료
        //if (step > 1000)
        //{
        //    AddReward(-10);
        //    Debug.Log("End reward : " + reward);
        //    EndEpisode();
        //}

        /*-------------------------------------------------------------------------------------------------*/
        // 일단 타겟보다 위에 떠있는것 (호버링) 을 먼저 할 수 있도록 해보기(240112)
        float distance = Mathf.Abs(7 - rb.transform.localPosition.y);

        AddReward(7-distance);


        //if (Mathf.Abs(rb.transform.localRotation.x) > 15 || Mathf.Abs(rb.transform.localRotation.z) > 15)
        //{
        //    AddReward(-1f);
        //}
        //else
        //{
        //    AddReward(1f);
        //}


        if (step > 5000)
        {
            AddReward(-10f);
            Debug.Log("End reward : " + reward);
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

    public void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("target"))
        {
            SetReward(10f);
            Debug.Log("success");
            EndEpisode();
        }
    }
}
