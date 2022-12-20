using System;
using System.Collections.Generic;
using UnityEngine;


using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{
    public class BodyCollisionExternal3d : CollisionExternal3d
    {

        private Body3d Body1 { get; set; }

        private int Particle1 { get; set; }

        private Rigidbody ExtBody { get; set; }

        public BodyCollisionExternal3d(Body3d body1, Rigidbody extBody)
        {
            Body1 = body1;
            ExtBody = extBody;
        }

        internal override void FindExternalContacts(IList<Body3d> bodies, IList<Body3d> externalBodies, List<CollisionContact3d> contacts)
        {
            for (int j = 0; j < bodies.Count; j++)
            {
                for (int k = 0; k < externalBodies.Count; k++)
                {
                    Debug.Log("bodies[j]: " + bodies[j] + " with externalBodies[k]: " + externalBodies[k]);
                }
                
            }

        }

    }
}