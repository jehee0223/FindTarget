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
        //Debug.Log("CollectObservations");
        // Target and Agent positions
        sensor.AddObservation(Target.localPosition);
        sensor.AddObservation(rb.transform.localPosition);
        sensor.AddObservation(rb.transform.localRotation);

        // Agent velocity
        sensor.AddObservation(rb.velocity.x);
        sensor.AddObservation(rb.velocity.y);
        sensor.AddObservation(rb.velocity.z);
    }

    private float power = 10;

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Vector3 currentRotation = rb.rotation.eulerAngles;
        
        // 각 축의 회전 각도 출력
        float pitch = currentRotation.x;
        float yaw = currentRotation.y;
        float roll = currentRotation.z;
        // 회전 각도 출력 예시
        Debug.Log("Pitch: " + pitch + ", Yaw: " + yaw + ", Roll: " + roll);

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
                        Rotor_fr.AddRelativeForce(Vector3.up * (action + 1f) * power);
                        //Debug.Log("fr action: " + action + " fr force: " + Vector3.up * (action + 1f) * power);
                        break;
                    case 1:
                        Rotor_fl.AddRelativeForce(Vector3.up * (action + 1f) * power);
                        //Debug.Log("fl action: " + action + " fl force: " + Vector3.up * (action + 1f) * power);
                        break;
                    case 2:
                        Rotor_br.AddRelativeForce(Vector3.up * (action + 1f) * power);
                        //Debug.Log("br action: " + action + " br force: " + Vector3.up * (action + 1f) * power);
                        break;
                    case 3:
                        Rotor_bl.AddRelativeForce(Vector3.up * (action + 1f) * power);
                        //Debug.Log("bl action: " + action + " bl force: " + Vector3.up * (action + 1f) * power);
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

        Vector3 currentRotation = rb.rotation.eulerAngles;

        // 각 축의 회전 각도 출력
        float pitch = currentRotation.x;
        float yaw = currentRotation.y;
        float roll = currentRotation.z;

        //기본적인 거리에 비례한 보상(목표 y:7)
        AddReward(7-distance);

        // 절대값 10도 이내면 +10
        if ((pitch < 10 || pitch > 350) && (roll < 10 || roll > 350))
        {
            AddReward(10f);
        }

        // 90~270도만큼 회전하면 종료
        if ((pitch > 90 && pitch < 270) || (roll > 90 && roll < 270))
        {
            SetReward(-100f);
            EndEpisode();
        }
        else if (step > 5000)
        {
            EndEpisode();
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        if (Input.GetKey(KeyCode.D))
        {
            Rotor_fr.AddRelativeForce(Vector3.up * 1 * 50);
            
        }
        else if (Input.GetKey(KeyCode.W))
        {
            Rotor_fl.AddRelativeForce(Vector3.up * 1 * 50);
        }
        else if (Input.GetKey(KeyCode.A))
        {
            Rotor_br.AddRelativeForce(Vector3.up * 1 * 50);
        }
        else if (Input.GetKey(KeyCode.S))
        {
            Rotor_bl.AddRelativeForce(Vector3.up * 1 * 50);
        }
        else if(Input.GetKey(KeyCode.Z))
        {
            Rotor_fr.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_fl.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_br.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_bl.AddRelativeForce(Vector3.up * 1 * power);
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
