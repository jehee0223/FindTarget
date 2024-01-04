using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System.Collections;
using Unity.MLAgents.Sensors;

public class agent : Agent
{
    Rigidbody Agent;
    public GameObject Target;
    public float agentRunSpeed;
    public float agentRotationSpeed;

    public GameObject rangeObject;
    BoxCollider rangeCollider;
    //이제희 바보
    int count=0;
    // 랜덤 위치 선정
    public Vector3 Return_RandomPosition()
    {
        Vector3 originPosition = rangeObject.transform.position;
        float range_X = rangeCollider.bounds.size.x;
        float range_Z = rangeCollider.bounds.size.z;

        range_X = Random.Range((range_X / 2) * -1, range_X / 2);
        range_Z = Random.Range((range_Z / 2) * -1, range_Z / 2);
        Vector3 RandomPostion = new Vector3(range_X, 0f, range_Z);

        Vector3 respawnPosition = originPosition + RandomPostion;
        return respawnPosition;
    }

    // 초기화
    public override void Initialize()
    {
        Agent=GetComponent<Rigidbody>();
        rangeCollider = GetComponent<BoxCollider>();
    }

    // ray perception sensor 3d를 사용하고있으면 사용 불가능
    //public override void CollectObservations(VectorSensor sensor)
    //{
    //

    //    // 타겟의 위치 정보 수집
    //    if (Target != null)
    //    {
    //        Transform targetTransform = Target.transform;
    //        Vector3 targetPosition = targetTransform.position;
    //        sensor.AddObservation(targetPosition.x);
    //        sensor.AddObservation(targetPosition.y);
    //        sensor.AddObservation(targetPosition.z);
    //        //Debug.Log(targetPosition.x+" "+targetPosition.y+" "+targetPosition.z);
    //        Debug.Log("Is sensor null: " + sensor == null);
    //    }
    //}

    public override void OnEpisodeBegin()
    {
        Agent.transform.localPosition = new Vector3(Random.Range(-5, 5), 0.5f, Random.Range(-5, 5));
        Target.transform.localPosition = new Vector3(Random.Range(-5, 5), 0.5f, Random.Range(-5, 5));
        SetReward(0);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[0] = 3;
        }
        else if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[0] = 4;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = 2;
        }
    }

    public void MoveAgent(ActionSegment<int> act)
    {
        /*Descrete한 행동들*/

        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        var forward = act[0];
        //var rote = act[1];
        switch (forward)
        {
            case 1:
                dirToGo = transform.forward * 1f;
                break;
            case 2:
                dirToGo = transform.forward * -1f;
                break;
            case 3:
                rotateDir = transform.up * 1f;
                break;
            case 4:
                rotateDir = transform.up * -1f;
                break;
        }
        //b
        transform.Rotate(rotateDir, Time.deltaTime * 150f);
        Agent.AddForce(dirToGo * 1, ForceMode.VelocityChange);

        // 여기 위에서 1은 agentRunSpeed. 하지만 자꾸 0으로 바뀜. 수정 필요(12/30)
        //Debug.Log("agentRunSpeed"+agentRunSpeed);

        /*continuous 행동 시도*/
        //float moveX = act[0];
        //float moveZ = act[1];
        //float movespeed = 1f;
        //transform.position += new Vector3(moveX, 0, moveZ) * Time.deltaTime * movespeed;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        AddReward(-0.01f);
        MoveAgent(actions.DiscreteActions);
        count = count + 1;
        Vector3 center = transform.position;
        float halfWidth = transform.localScale.x / 2f;
        float halfHeight = transform.localScale.y / 2f;

        //Vector3 endPoint = center + new Vector3(halfWidth, halfHeight, 0f);
        //Debug.Log("도형의 끝점 좌표: " + endPoint);
    }


    public void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("target"))
        {
            SetReward(2f);
            Debug.Log("success");
            EndEpisode();
        }
        else if (collision.gameObject.CompareTag("wall"))
        {
            AddReward(-1f);
            Debug.Log("wall col");
        }
    }

    public void RayCastInfo(RayPerceptionSensorComponent3D rayComponent)
    {
        //rayComponent(파라미터)가 만들어내는 RayOutput을 rayOutputs변수에 저장
        var rayOutputs = RayPerceptionSensor
                .Perceive(rayComponent.GetRayPerceptionInput(),true)
                .RayOutputs;
        //감지한 물체(정보)가 있다면
        if (rayOutputs != null)
        {	//센서에 달린 Ray가 여러개 있으니 RayOutputs는 배열
            var outputLegnth = RayPerceptionSensor
                    .Perceive(rayComponent.GetRayPerceptionInput(),true)
                    .RayOutputs
                    .Length;
            for (int i = 0; i < outputLegnth; i++)
            {	//센서의 Ray에 충돌한(감지된) 물체가 있는 경우
                GameObject goHit = rayOutputs[i].HitGameObject;
                if (goHit != null)
                {	// 충돌한 물체까지의 거리 계산
                    var rayDirection = rayOutputs[i].EndPositionWorld - rayOutputs[i].StartPositionWorld;
                    var scaledRayLength = rayDirection.magnitude;
                    float rayHitDistance = rayOutputs[i].HitFraction * scaledRayLength;
                    // 낙하물(Obstacle)이 일정 거리 이내로 들어오면 -1점
                    if (goHit.tag == "Obstacle" && rayHitDistance < 2.4f)
                    {
                        AddReward(1.0f);
                        //에피소드 종료
                        EndEpisode();
                    }
                }
            }
        }
    }
}
