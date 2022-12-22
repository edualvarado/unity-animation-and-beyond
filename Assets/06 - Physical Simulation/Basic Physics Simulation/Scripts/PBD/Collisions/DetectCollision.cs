using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PositionBasedDynamics.Collisions
{
    public class DetectCollision : MonoBehaviour
    {
        public Collision Hit { get; set; }     
       
        private void OnCollisionEnter(Collision collision)
        {
            // Save collision
            Hit = collision;
            
            //Debug.Log("[DetectCollision] Sphere: " + int.Parse(gameObject.name) + ": Hit " + Hit.GetContact(0).point);

            // Forward to the parent and let know a collision happened
            BasicPBDDemo parentScript = transform.parent.GetComponent<BasicPBDDemo>();
            parentScript.CollisionFromChild(Hit, this.gameObject);
        }

        private void OnCollisionExit(Collision collision)
        {
            // Save collision
            //Hit = collision;

            //Debug.Log("[DetectCollision] Sphere: " + int.Parse(gameObject.name) + ": Hit " + Hit.GetContact(0).point);

            // Forward to the parent and let know a collision happened
            BasicPBDDemo parentScript = transform.parent.GetComponent<BasicPBDDemo>();
            parentScript.ExitCollisionFromChild(this.gameObject);
        }
    } 
}
