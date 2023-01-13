using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Constraints;

namespace PositionBasedDynamics.Bodies
{

    public abstract class Body3d
    {
        public int NumParticles { get { return Positions.Length; } }

        public int NumConstraints { get { return Constraints.Count; } }

        public double Dampning { get; set; }

        public double ParticleRadius { get; protected set; }

        public double ParticleDiameter { get { return ParticleRadius * 2.0; } }

        public double ParticleMass { get; protected set; }

        public Vector3d[] Positions { get; private set; }

        public Vector3d[] Predicted { get; private set; }

        public Vector3d[] Velocities { get; private set; }
        
        // TEST
        public bool[] IsContact { get; set; }
        // TEST
        public ContactPoint[] ExternalHit { get; set; }
        // TEST
        public bool[] IsBroken { get; set; }
        // TEST
        public bool[] IsStatic { get; set; }

        public Box3d Bounds { get; private set; }

        public List<Constraint3d> Constraints { get; private set; }

        private List<StaticConstraint3d> StaticConstraints { get; set; }

        public Body3d(int numParticles, double radius, double mass)
        {
            Debug.Log("[Body3d] Arrays Initialized");
            Positions = new Vector3d[numParticles];
            Predicted = new Vector3d[numParticles];
            Velocities = new Vector3d[numParticles];
            IsContact = new bool[numParticles]; // TEST
            ExternalHit = new ContactPoint[numParticles]; // TEST
            IsBroken = new bool[numParticles]; // TEST
            IsStatic = new bool[numParticles]; // TEST
            Constraints = new List<Constraint3d>();
            StaticConstraints = new List<StaticConstraint3d>();

            ParticleRadius = radius;
            ParticleMass = mass;
            Dampning = 1;

            if (ParticleMass <= 0)
                throw new ArgumentException("Particles mass <= 0");

            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

        internal void ConstrainPositions(double di)
        {
            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainPositions(di);
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainPositions(di);
            }
        }

        internal void ConstrainVelocities()
        {

            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainVelocities();
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainVelocities();
            }

        }

        // TEST
        internal void RemoveConstrainPositions()
        {
            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].RemoveConstrainPositions();
            }
        }

        public void RandomizePositions(System.Random rnd, double amount)
        {
            for(int i = 0; i < NumParticles; i++)
            {
                double rx = rnd.NextDouble() * 2.0 - 1.0;
                double ry = rnd.NextDouble() * 2.0 - 1.0;
                double rz = rnd.NextDouble() * 2.0 - 1.0;

                Positions[i] += new Vector3d(rx, ry, rz) * amount;
            }
        }

        public void RandomizeConstraintOrder(System.Random rnd)
        {
            int count = Constraints.Count;
            if (count <= 1) return;

            List<Constraint3d> tmp = new List<Constraint3d>();

            while (tmp.Count != count)
            {
                int i = rnd.Next(0, Constraints.Count - 1);

                tmp.Add(Constraints[i]);
                Constraints.RemoveAt(i);
            }

            Constraints = tmp;
        }

        public void MarkAsStatic(Box3d bounds)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                if (bounds.Contains(Positions[i]))
                {
                    IsStatic[i] = true;
                    StaticConstraints.Add(new StaticConstraint3d(this, i));
                }
            }
        }

        public void UpdateBounds()
        {
            Vector3d min = new Vector3d(double.PositiveInfinity);
            Vector3d max = new Vector3d(double.NegativeInfinity);

            for (int i = 0; i < NumParticles; i++)
            {
                min.Min(Positions[i]);
                max.Max(Positions[i]);
            }

            min -= ParticleRadius;
            max += ParticleRadius;

            Bounds = new Box3d(min, max);
        }

    }

}