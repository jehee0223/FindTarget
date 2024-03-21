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

public class DroneAgent : Agent
{

    public int step = 0;
    public int total_step = 0;
    public double reward = 0;

    public Rigidbody rb;
    public Rigidbody Rotor_fr;
    public Rigidbody Rotor_fl;
    public Rigidbody Rotor_br;
    public Rigidbody Rotor_bl;
    public Transform Target;

    private Vector3 initPosition;
    private Quaternion initQuaternion;

    public double dis;
    public double fix_dis;
    public double TGP,TGR,TGD;
    double GP, GR, GD;
    public double New_pitch;
    public double New_roll;
    public int episode = 0;

    public Logger logger_epi;
    public Logger logger_step;
    public Logger logger;

    // Start is called before the first frame update
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        logger = new Logger("C:/Users/leejehee/Desktop/Github/FindTarget/ml-agents/Project/Build/FindTarget/dronemodel/log/drone13/log_epi", "C:/Users/leejehee/Desktop/Github/FindTarget/ml-agents/Project/Build/FindTarget/dronemodel/log/drone13/log_step");
    }

    public override void OnEpisodeBegin()
    {
        step = 0;
        TGP = 0;
        TGR = 0;
        TGD = 0;
        New_pitch = 0;
        New_roll = 0;

        rb.transform.localPosition = initPosition;
        rb.transform.localRotation = initQuaternion;

        Rotor_fr.velocity = Vector3.zero;
        Rotor_fl.velocity = Vector3.zero;
        Rotor_br.velocity = Vector3.zero;
        Rotor_bl.velocity = Vector3.zero;
        rb.velocity = Vector3.zero;

        // Move the target to a new spot
        rb.transform.localPosition = new Vector3(0, (float)0.4813456, 0);
        //Target.transform.localPosition = new Vector3(UnityEngine.Random.Range(-5, 5), 9, UnityEngine.Random.Range(-5, 5));
        Target.transform.localPosition = new Vector3(3, 9, 3);
        fix_dis = Vector3.Distance(Target.transform.localPosition, rb.transform.localPosition);

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

    private float power = 50;

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
        total_step++;
        SetReward();

    }


    public void SetReward()
    {
        /*-----------------------------------Target 찾아가기-------------------------------*/
        dis = Vector3.Distance(Target.transform.localPosition, rb.transform.localPosition);
        Vector3 currentRotation = rb.rotation.eulerAngles;

        // 각 축의 회전 각도 출력
        double pitch = currentRotation.x;
        double yaw = currentRotation.y;
        double roll = currentRotation.z;
        
        New_pitch = 0;
        New_roll = 0;

        if (pitch > 180)
        {
            New_pitch = 360 - pitch;
        }
        else
        {
            New_pitch = pitch;
        }
        if (roll > 180)
        {
            New_roll = 360 - roll;
        }
        else
        {
            New_roll = roll;
        }
        // rotation < 30이면 가우시안에 따름, >=30이면 -1
        if (New_pitch < 30)
        {
            GP = (Gaussian(New_pitch, 0, 17) * 43)-1;
        }
        else { GP = -4; }
        if (New_roll < 30)
        {
            GR = (Gaussian(New_roll, 0, 17) * 43)-1;
        }
        else { GR= -4; }

        //GD = Gaussian(dis, 0, 8) * 100;
        GD = -(4 / fix_dis * dis) + 4;
        

        AddReward((float)(GP + GR + GD));
        reward += (float)(GP + GR + GD);
        TGR += GR;
        TGP += GP;
        TGD += GD;
        AddReward(-0.1f);
        reward -= 0.1;

        // 90~270도만큼 회전하면 음수 보상&종료
        if (New_pitch>90 || New_roll>90)
        {
            SetReward(-1000f);
            reward -= 1000;
            logger.Log_epi(episode, step, New_pitch, New_roll, TGP, TGR, TGD, reward, 0);
            EndEpisode();
        }
        else if (Mathf.Abs(rb.transform.localPosition.x) > 10 || Mathf.Abs(rb.transform.localPosition.z) > 10)
        {
            SetReward(-1000f);
            reward -= 1000;
            logger.Log_epi(episode, step, New_pitch, New_roll, TGP, TGR, TGD, reward, 0);
            EndEpisode();
        }
        else if (step > 5000)
        {
            AddReward(-1000f);
            reward -= 1000;
            logger.Log_epi(episode, step, New_pitch, New_roll, TGP, TGR, TGD, reward, 0);
            EndEpisode();
        }
        var statsRecorder = Academy.Instance.StatsRecorder;
        statsRecorder.Add("Distance", (float)dis, StatAggregationMethod.Average);
        statsRecorder.Add("Pitch", (float)New_pitch, StatAggregationMethod.Average);
        statsRecorder.Add("Roll", (float)New_roll, StatAggregationMethod.Average);
        statsRecorder.Add("step", (float)step, StatAggregationMethod.Average);
        //logger.Log_step(total_step, (float)dis, (float)New_pitch, (float)New_roll, reward);
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("target"))
        {
            AddReward(1000f);
            Debug.Log("success");
            logger.Log_epi(episode, step, New_pitch, New_roll, TGP, TGR, TGD, reward, 1);
            EndEpisode();
        }
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
        else if (Input.GetKey(KeyCode.Z))
        {
            Rotor_fr.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_fl.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_br.AddRelativeForce(Vector3.up * 1 * power);
            Rotor_bl.AddRelativeForce(Vector3.up * 1 * power);
        }
    }

    
}

public class Logger
{
    private string baseFilePath_epi;
    private string baseFilePath_step;
    private string filePath_epi;
    private string filePath_step;
    private int epiFileCount = 0;
    private int stepFileCount = 0;
    private int epiLineCount = 0; // 현재 에피소드 파일의 라인 수
    private int stepLineCount = 0; // 현재 스텝 파일의 라인 수
    private const int maxLinesPerFile = 5000; // 파일 당 최대 라인 수

    public Logger(string baseFilePath_epi, string baseFilePath_step)
    {
        this.baseFilePath_epi = baseFilePath_epi;
        this.baseFilePath_step = baseFilePath_step;
        UpdateFilePaths(); // 초기 파일 경로 설정
    }

    private void UpdateFilePaths()
    {
        filePath_epi = $"{baseFilePath_epi}_{epiFileCount}.csv";
        filePath_step = $"{baseFilePath_step}_{stepFileCount}.csv";
        // 파일 초기화 및 헤더 작성
        InitializeFile(filePath_epi, "Episode,Step,New_pitch,New_roll,GP+GR,GD,reward,success");
        InitializeFile(filePath_step, "Step,success");
    }

    private void InitializeFile(string filePath, string header)
    {
        if (!File.Exists(filePath) || new FileInfo(filePath).Length == 0)
        {
            using (var writer = new StreamWriter(filePath, append: true))
            {
                writer.WriteLine(header);
            }
        }
    }

    public void Log_epi(int episode, int step, double New_pitch, double New_roll,double GP,double GR,double GD, double reward, int success)
    {
        if (epiLineCount >= maxLinesPerFile)
        {
            epiFileCount++; // 파일 카운트 증가
            epiLineCount = 0; // 라인 카운트 초기화
            UpdateFilePaths(); // 새 파일 경로 업데이트
        }
        using (var writer = new StreamWriter(filePath_epi, append: true))
        {
            writer.WriteLine($"{episode},{step},{New_pitch},{New_roll},{GP+GR},{GD},{reward},{success}");
            epiLineCount++; // 라인 카운트 증가
        }
    }

    public void Log_step(int total_step, float dis, float New_pitch, float New_roll, double reward)
    {
        if (stepLineCount >= maxLinesPerFile)
        {
            stepFileCount++; // 파일 카운트 증가
            stepLineCount = 0; // 라인 카운트 초기화
            UpdateFilePaths(); // 새 파일 경로 업데이트
        }
        using (var writer = new StreamWriter(filePath_step, append: true))
        {
            writer.WriteLine($"{total_step},{dis},{New_pitch},{New_roll},{reward}");
            stepLineCount++; // 라인 카운트 증가
        }
    }
}
