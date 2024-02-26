using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using System.IO;
using static UnityEngine.UIElements.UxmlAttributeDescription;
//using UnityEditor.PackageManager;
//using UnityEditor.SearchService;

public class DroneAgent11 : Agent
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

    //public double dis;
    //public double pitch;
    //public double roll;
    //public double yaw;
    //public double New_pitch;
    //public double New_roll;
    public int episode=0;

    public Logger logger;

    // Start is called before the first frame update
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        logger = new Logger("training_data.csv");
    }

    public override void OnEpisodeBegin()
    {
        step = 0;

        rb.transform.localPosition = initPosition;
        rb.transform.localRotation = initQuaternion;

        Rotor_fr.velocity = Vector3.zero;
        Rotor_fl.velocity = Vector3.zero;
        Rotor_br.velocity = Vector3.zero;
        Rotor_bl.velocity = Vector3.zero;
        rb.velocity = Vector3.zero;

        // Move the target to a new spot
        rb.transform.localPosition = new Vector3(0, (float)0.4813456, 0);
        Target.transform.localPosition=new Vector3(UnityEngine.Random.Range(-5,5), 9, UnityEngine.Random.Range(-5,5));
        //Target.transform.localPosition = new Vector3(3, 9, 3);

        reward = 0;
        episode += 1;
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

    private float power = 45;

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Vector3 currentRotation = rb.rotation.eulerAngles;
        
        // 각 축의 회전 각도 출력
        float pitch = currentRotation.x;
        float yaw = currentRotation.y;
        float roll = currentRotation.z;
        // 회전 각도 출력 예시
        //Debug.Log("Pitch: " + pitch + ", Yaw: " + yaw + ", Roll: " + roll);


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
        float dis = Vector3.Distance(Target.transform.localPosition, rb.transform.localPosition);
        Debug.Log("Dis: " + dis);
        Vector3 currentRotation = rb.rotation.eulerAngles;

        // 각 축의 회전 각도 출력
        double pitch = currentRotation.x;
        double yaw = currentRotation.y;
        double roll = currentRotation.z;

        double New_pitch = 0, New_roll = 0;

        if(pitch > 180)
        {
            New_pitch = 360 - pitch;
        }
        else
        {
            New_pitch = pitch;
        }
        if (roll > 180)
        {
            New_roll=360-roll;
        }
        else
        {
            New_roll = roll;
        }
        double GP = Gaussian(New_pitch, 0, 10) * 25;
        double GR = Gaussian(New_roll, 0, 10) * 25;
        double GD = Gaussian(dis, 0, 8) * 25;
        AddReward((float)(GP + GR + (1.5 * GD)));
        Debug.Log("GP:" + GP);
        Debug.Log("GR:" + GR);
        Debug.Log("GD:" + GD);
        Debug.Log("velocity:" + Rotor_bl.velocity);
        AddReward(-3f);

        // 90~270도만큼 회전하면 음수 보상&종료
        if ((pitch > 90 && pitch < 270) || (roll > 90 && roll < 270))
        {
            SetReward(-1000f);
            //logger.Log(episode, (float)New_pitch, (float)New_roll);
            EndEpisode();
        }
        else if (Mathf.Abs(rb.transform.localPosition.x) > 10 || Mathf.Abs(rb.transform.localPosition.z) > 10)
        {
            SetReward(-1000f);
            //logger.Log(episode, (float)New_pitch, (float)New_roll);
            EndEpisode();
        }
        else if (step > 5000)
        {
            AddReward(-1000f);
            //logger.Log(episode, (float)New_pitch, (float)New_roll);
            EndEpisode();
        }
        var statsRecorder = Academy.Instance.StatsRecorder;
        statsRecorder.Add("Distance", dis, StatAggregationMethod.Average);
        statsRecorder.Add("Pitch", (float)New_pitch, StatAggregationMethod.Average);
        statsRecorder.Add("Roll", (float)New_roll, StatAggregationMethod.Average);
    }

    static double Gaussian(double x, double mean, double standardDeviation)
    {
        double coefficient = 1 / (standardDeviation * Math.Sqrt(2 * Math.PI));
        double exponent = -0.5 * Math.Pow((x - mean) / standardDeviation, 2);
        double result = coefficient * Math.Pow(Math.E, exponent);

        return result;
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
            AddReward(1000f);
            Debug.Log("success");
            EndEpisode();
        }
    }
}

public class Logger
{
    private string filePath;

    public Logger(string filePath)
    {
        this.filePath = "C:/Users/leejehee/Desktop/Github/FindTarget/ml - agents/Project/Build/FindTarget/dronemodel";

        using (var writer = new StreamWriter(filePath, append: true))
        {
            writer.WriteLine("Episode,Reward,AnotherMetric");
        }
    }

    public void Log(int episode, float New_pitch, float New_roll)
    {
        using (var writer = new StreamWriter(filePath, append: true))
        {
            writer.WriteLine($"{episode},{New_pitch},{New_roll}");
        }
    }
}
