using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DetectCollision : MonoBehaviour
{    
    // TODO - Class where to forward the hit
    
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision Detected: " + collision.gameObject.name);
        Debug.DrawRay(collision.contacts[0].point, collision.contacts[0].normal, Color.blue, 1f);

        // TODO - Forward to the parent
        //PlayerCollisionHelper parentScript = transform.parent.GetComponent<PlayerCollisionHelper>();

        // Let it know a collision happened:
        //parentScript.CollisionFromChild(hit);
    }
}
