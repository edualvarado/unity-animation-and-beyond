using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThrowBall : MonoBehaviour
{
    public GameObject ball;
    public float force;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            var projectile = Instantiate(ball, transform.position, transform.rotation);
            projectile.GetComponent<Rigidbody>().AddForce(transform.right * force);//cannon's x axis
            Physics.IgnoreCollision(projectile.GetComponent<Collider>(), this.GetComponent<Collider>());
        }
    }
}
