using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Constraints
{

    public class BendingConstraint3d : Constraint3d
    {

        private double RestLength { get; set; }

        private double Stiffness { get; set; }

        // TEST
        private double Diff { get; set; }
        private double CurrentAngle { get; set; }
        private float CurrentAngleAlt { get; set; }
        private double ThresholdAngle { get; set; }

        private readonly int i0, i1, i2;

        internal BendingConstraint3d(Body3d body, int i0, int i1, int i2, double stiffness) : base(body)
        {
            this.i0 = i0;
            this.i1 = i1;
            this.i2 = i2;

            Stiffness = stiffness;

            Vector3d center = (Body.Positions[i0] + Body.Positions[i1] + Body.Positions[i2]) / 3.0;
            RestLength = (Body.Positions[i2] - center).Magnitude;
        }

        internal override void ConstrainPositions(double di)
        {

            Vector3d center = (Body.Predicted[i0] + Body.Predicted[i1] + Body.Predicted[i2]) / 3.0;
            Vector3d dirCenter = Body.Predicted[i2] - center;

            double distCenter = dirCenter.Magnitude;
            double diff = 1.0 - (RestLength / distCenter); // X
            Diff = diff;

            // TEST CURRENT ANGLE 1
            double currentAngle;
            if ((distCenter / RestLength) > 1)
                currentAngle = Math.Acos(1) * Mathf.Rad2Deg;
            else if ((distCenter / RestLength) < -1)
                currentAngle = Math.Acos(-1) * Mathf.Rad2Deg;
            else
                currentAngle = Math.Acos(distCenter / RestLength) * Mathf.Rad2Deg;

            CurrentAngle = currentAngle;

            // TEST CURRENT ANGLE 2
            float currentAngleAlt;
            currentAngleAlt = Vector3.Angle((Body.Predicted[i0] - Body.Predicted[i1]).ToVector3(), (Body.Predicted[i2] - Body.Predicted[i1]).ToVector3());
            CurrentAngleAlt = currentAngleAlt;

            // TEST
            //if (i0 == 0 && i1 == 3 && i2 == 6)
            //{
            //    Debug.Log("Between: " + i0 + ", " + i1 + " and " + i2);
            //    Debug.Log("distCenter: " + distCenter);
            //    Debug.Log("RestLength: " + RestLength);
            //    Debug.Log("(distCenter / RestLength): " + (distCenter / RestLength));
            //    Debug.Log("Arcos(distCenter / RestLength): " + CurrentAngle); // WRONG, look here!

            //    Debug.Log("1.0 - (RestLength / distCenter): " + Diff);
            //    Debug.Log("Arcos(diff): " + (Math.Acos(Diff) * 180.0 / Math.PI));

            //    Debug.Log("currentAngleAlt : " + (180f - CurrentAngleAlt));
            //}

            double thresholdAngle = 35; // TODO: MODIFY
            ThresholdAngle = thresholdAngle;

            double mass = Body.ParticleMass;

            double w = mass + mass * 2.0f + mass;

            Vector3d dirForce = dirCenter * diff;

            Vector3d fa = Stiffness * (2.0 * mass / w) * dirForce * di;
            Body.Predicted[i0] += fa;

            Vector3d fb = Stiffness * (2.0 * mass / w) * dirForce * di;
            Body.Predicted[i1] += fb;

            Vector3d fc = -Stiffness * (4.0 * mass / w) * dirForce * di;
            Body.Predicted[i2] += fc;
        }

        internal override void RemoveConstrainPositions()
        {
            //Debug.Log("Looking to remove constraint for: " + i0 + ", " + i1 + " and " + i2 + " - the angle (distCenter / RestLength) here is: " + CurrentAngle);

            // For testing: Check only first triple
            if (i0 == 0 && i1 == 3 && i2 == 6)
            {
                //Debug.Log("Between: " + i0 + ", " + i1 + " and " + i2);
                //Debug.Log("Arcos(distCenter / RestLength): " + CurrentAngle); // WRONG, look here!
                //Debug.Log("Arcos(diff): " + (Math.Acos(Diff) * 180.0 / Math.PI));
                //Debug.Log("currentAngleAlt : " + (180f - CurrentAngleAlt));

                //if (CurrentAngle > 15f)
                //{
                //    Body.Constraints.Remove(this);
                //}

                //if ((180f - CurrentAngleAlt) > ThresholdAngle)
                //{
                //    Debug.Log("REMOVE Between: " + i0 + ", " + i1 + " and " + i2);
                //    Body.IsBroken[i1] = true;
                //    Body.Constraints.Remove(this);
                //}
            }

            // For testing: Check only first row
            if ((i0 == 0 && i1 == 3 && i2 == 6) || (i0 == 1 && i1 == 4 && i2 == 7) || (i0 == 2 && i1 == 5 && i2 == 8))
            {
                //Debug.Log("Between: " + i0 + ", " + i1 + " and " + i2);
                //Debug.Log("Arcos(distCenter / RestLength): " + CurrentAngle); // WRONG, look here!
                //Debug.Log("Arcos(diff): " + (Math.Acos(Diff) * 180.0 / Math.PI));
                //Debug.Log("currentAngleAlt : " + (180f - CurrentAngleAlt));

                if ((180f - CurrentAngleAlt) > ThresholdAngle)
                {
                    //Debug.Log("REMOVE Between: " + i0 + ", " + i1 + " and " + i2);
                    //Body.IsBroken[i1] = true;
                    //Body.Constraints.Remove(this);
                }
            }
        }
    }
}
