using UnityEngine;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using Common.Unity.Drawing;
using Common.Unity.Mathematics;

using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Bodies.Deformable;
using PositionBasedDynamics.Sources;
using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Solvers;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics
{
    public class BasicPBDDemo : MonoBehaviour
    {
        #region Instance Fields

        public Vector3 translation;
        public Vector3 rotation;

        public Vector3 minVector3;
        public Vector3 maxVector3;

        public float fxmin, fxmax;
        public float fymin, fymax;
        public float fzmin, fzmax;

        public int GRID_SIZE = 10;

        public int iterations = 4;

        public double radius = 0.25;
        public double stiffness = 0.2;
        public double mass = 1.0;

        public bool drawLines = true;
        public Material sphereMaterial;

        #endregion

        #region Instance Properties

        private List<GameObject> Spheres { get; set; }
        
        private DeformableBody3d Body { get; set; }

        private Solver3d Solver { get; set; }

        private Box3d StaticBounds { get; set; }

        #endregion

        #region Read-only & Static Fields

        // XPBD -> deltaTs = deltaT / n
        private const double timeStep = 1.0 / 60.0;

        private Vector3d min;
        private Vector3d max;

        #endregion

        // Start is called before the first frame update
        void Start()
        {
            // Create body
            Matrix4x4d T = Matrix4x4d.Translate(new Vector3d(translation.x, translation.y, translation.z));
            Matrix4x4d R = Matrix4x4d.Rotate(new Vector3d(rotation.x, rotation.y, rotation.z));
            Matrix4x4d TR = T * R;

            // Mesh properties
            System.Random rnd = new System.Random(0);
            min = new Vector3d(minVector3.x, minVector3.y, minVector3.z);
            max = new Vector3d(maxVector3.x, maxVector3.y, maxVector3.z);
            Box3d bounds = new Box3d(min, max);
            TetrahedronsFromBounds source = new TetrahedronsFromBounds(radius, bounds);
            Body = new DeformableBody3d(source, radius, mass, stiffness, TR);

            Body.Dampning = 1.0;
            Body.RandomizePositions(rnd, radius * 0.01);
            Body.RandomizeConstraintOrder(rnd);

            // Mesh properties
            Vector3d smin = new Vector3d(min.x + fxmin, min.y + fymin, min.z + fzmin);
            Vector3d smax = new Vector3d(min.x + fxmax, max.y + fymax, max.z + fzmax);
            StaticBounds = new Box3d(smin, smax);
            Body.MarkAsStatic(StaticBounds);

            // Constrainst Solver
            Solver = new Solver3d();
            Solver.AddBody(Body);
            Solver.AddForce(new GravitationalForce3d());
            Solver.AddCollision(new PlanarCollision3d(Vector3d.UnitY, 0));
            Solver.SolverIterations = 2;
            Solver.CollisionIterations = 2;
            Solver.SleepThreshold = 1;

            CreateSpheres();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            double dt = timeStep / iterations; // XPBD
            //double dt = Time.fixedDeltaTime / iterations; // XPBD

            for (int i = 0; i < iterations; i++)
                Solver.StepPhysics(dt);

            UpdateSpheres();
        }

        void OnDestroy()
        {

            if (Spheres != null)
            {
                for (int i = 0; i < Spheres.Count; i++)
                    DestroyImmediate(Spheres[i]);
            }
        }

        private void OnRenderObject()
        {
            if (drawLines)
            {
                Camera camera = Camera.current;

                Vector3 min = new Vector3(-GRID_SIZE, 0, -GRID_SIZE);
                Vector3 max = new Vector3(GRID_SIZE, 0, GRID_SIZE);

                DrawLines.DrawGrid(camera, Color.white, min, max, 1, transform.localToWorldMatrix);

                Matrix4x4d m = MathConverter.ToMatrix4x4d(transform.localToWorldMatrix);
                DrawLines.DrawVertices(LINE_MODE.TETRAHEDRON, camera, Color.red, Body.Positions, Body.Indices, m);

                DrawLines.DrawBounds(camera, Color.green, StaticBounds, Matrix4x4d.Identity);
            }
        }

        private void CreateSpheres()
        {
            if (sphereMaterial == null) return;

            Spheres = new List<GameObject>();

            int numParticles = Body.NumParticles;
            float diam = (float)Body.ParticleRadius * 2.0f;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                Spheres.Add(sphere);
            }

        }

        public void UpdateSpheres()
        {

            if (Spheres != null)
            {
                for (int i = 0; i < Spheres.Count; i++)
                {
                    Vector3d pos = Body.Positions[i];
                    Spheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                }
            }

        }
    } 
}
