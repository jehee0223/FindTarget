using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.MLAgents;
using UnityEngine.Serialization;

/// <summary>
/// An example of how to use ML-Agents without inheriting from the Agent class.
/// Observations are generated by the attached SensorComponent, and the actions
/// are retrieved from the Agent.
/// </summary>
public class BasicController : MonoBehaviour
{
    public float timeBetweenDecisionsAtInference;
    float m_TimeSinceDecision;
    [FormerlySerializedAs("m_Position")]
    [HideInInspector]
    public int position;
    const int k_SmallGoalPosition = 7;
    const int k_LargeGoalPosition = 17;
    public GameObject largeGoal;
    public GameObject smallGoal;
    const int k_MinPosition = 0;
    const int k_MaxPosition = 20;
    public const int k_Extents = k_MaxPosition - k_MinPosition;

    Agent m_Agent;

    public void Awake()
    {
        // Since this example does not inherit from the Agent class, explicit registration
        // of the RpcCommunicator is required. The RPCCommunicator should only be compiled
        // for Standalone platforms (i.e. Windows, Linux, or Mac)
#if UNITY_EDITOR || UNITY_STANDALONE
        if (!CommunicatorFactory.CommunicatorRegistered)
        {
            Debug.Log("Registered Communicator.");
            CommunicatorFactory.Register<ICommunicator>(RpcCommunicator.Create);
        }
#endif
    }

    public void OnEnable()
    {
        m_Agent = GetComponent<Agent>();
        position = 10;
        transform.position = new Vector3(position - 10f, 0f, 0f);
        smallGoal.transform.position = new Vector3(k_SmallGoalPosition - 10f, 0f, 0f);
        largeGoal.transform.position = new Vector3(k_LargeGoalPosition - 10f, 0f, 0f);
    }

    /// <summary>
    /// Controls the movement of the GameObject based on the actions received.
    /// </summary>
    /// <param name="direction"></param>
    public void MoveDirection(int direction)
    {
        position += direction;
        if (position < k_MinPosition) { position = k_MinPosition; }
        if (position > k_MaxPosition) { position = k_MaxPosition; }

        gameObject.transform.position = new Vector3(position - 10f, 0f, 0f);

        m_Agent.AddReward(-0.01f);

        if (position == k_SmallGoalPosition)
        {
            m_Agent.AddReward(0.1f);
            m_Agent.EndEpisode();
            ResetAgent();
        }

        if (position == k_LargeGoalPosition)
        {
            m_Agent.AddReward(1f);
            m_Agent.EndEpisode();
            ResetAgent();
        }
    }

    public void ResetAgent()
    {
        // This is a very inefficient way to reset the scene. Used here for testing.
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        m_Agent = null; // LoadScene only takes effect at the next Update.
        // We set the Agent to null to avoid using the Agent before the reload
    }

    public void FixedUpdate()
    {
        WaitTimeInference();
    }

    void WaitTimeInference()
    {
        if (m_Agent == null)
        {
            return;
        }
        if (Academy.Instance.IsCommunicatorOn)
        {
            m_Agent?.RequestDecision();
        }
        else
        {
            if (m_TimeSinceDecision >= timeBetweenDecisionsAtInference)
            {
                m_TimeSinceDecision = 0f;
                m_Agent?.RequestDecision();
            }
            else
            {
                m_TimeSinceDecision += Time.fixedDeltaTime;
            }
        }
    }
}
