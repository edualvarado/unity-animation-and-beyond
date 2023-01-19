using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PositionBasedDynamics.Collisions
{
    public class DetectionCollision : MonoBehaviour
    {
        public Collision Hit { get; set; }
        public float PenetrationDistance { get; set; }

        private GameObject root;

        private void Awake()
        {
            root = GameObject.Find("Root");
        }
        
        private void OnCollisionEnter(Collision other)
        {
            // Bring to parent script
            VegetationCreator parentScript = root.GetComponent<VegetationCreator>();
            
            // Save collision
            Hit = other;
            
            // To estimate penetration distance
            ContactPoint[] contactPoints = Hit.contacts;
            Vector3 center = GetComponent<Collider>().bounds.center;

            ContactPoint contactPoint = contactPoints[0];
            Vector3 normal = contactPoint.normal;

            RaycastHit hitInfo;
            if (Physics.Raycast(center, -normal, out hitInfo))
            {
                PenetrationDistance = Mathf.Abs((float)parentScript.diameter / 2 - hitInfo.distance);
            }

            Debug.DrawRay(center, -normal * hitInfo.distance, Color.blue);
            
            // Send information
            parentScript.CollisionFromChildBody(Hit, PenetrationDistance, this.gameObject);
        }

        private void OnCollisionExit(Collision other)
        {
            // Bring to parent script
            VegetationCreator parentScript = root.GetComponent<VegetationCreator>();

            // Send information
            parentScript.ExitCollisionFromChildBody(this.gameObject);
            
            //if (transform.parent.gameObject.name == "BodyPlant_0")
            //    parentScript.ExitCollisionFromChildBody(this.gameObject);

        }
    }
}
