using System.Collections;
using UnityEngine;

public class Motor : MonoBehaviour
{
    // Start is called before the first frame update

    public float thrust;
    public Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        rb.AddRelativeForce(Vector3.up * thrust);
    }
}
