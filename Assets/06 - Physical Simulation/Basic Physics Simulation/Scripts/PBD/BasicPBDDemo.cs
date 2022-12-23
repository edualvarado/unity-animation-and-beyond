using UnityEngine;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using Common.Unity.Drawing;
using Common.Unity.Mathematics;

using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Bodies.Deformable;
using PositionBasedDynamics.Bodies.Ridgid;
using PositionBasedDynamics.Sources;
using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Solvers;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics
{
    public class BasicPBDDemo : MonoBehaviour
    {
        #region Instance Fields

        // Debug
        [Header("Debug")]
        public bool printInformation;

        // External Obstacles
        [Header("External")]
        public GameObject obstacle;

        // Reference Grid (White)
        [Header("Grid")]
        public int GRID_SIZE = 2;

        // Global pos and rotation of mesh
        [Header("Mesh - Global pos and rot")]
        public Vector3 translation;
        public Vector3 rotation;

        // Mesh
        [Header("Mesh - Properties")]
        public Vector3 minVector3;
        public Vector3 maxVector3;
        public double radius = 0.25;
        public double stiffness = 0.2;
        public double mass = 1.0;
        public Vector3 fxMin;
        public Vector3 fxMax;

        [Header("Mesh - Debug")]
        public bool drawLines = true;
        public Material sphereMaterial;
        public Material sphereMaterialNoContact;
        public Material sphereMaterialContact;
        [Range(0f, 1f)] public float scaleRadius;

        [Header("Solver")]
        public int iterations = 1; // 4 before
        public int solverIterations = 1; // 2 before
        public int collisionIterations = 1; // 2 before

        #endregion

        #region Instance Properties

        private List<GameObject> Spheres { get; set; }
        private List<GameObject> Spheres2 { get; set; }
        private List<GameObject> Spheres3 { get; set; }

        private DeformableBody3d Body1 { get; set; }
        private DeformableBody3d Body2 { get; set; }
        private RidgidBody3d Body3 { get; set; }
        private Rigidbody ExtRigidBody { get; set; }

        private Solver3d Solver { get; set; }

        private Box3d StaticBounds { get; set; }

        #endregion

        #region Read-only & Static Fields

        //private double timeStep = 1.0 / 60.0; 
        private double timeStep; 

        private Vector3d min;
        private Vector3d max;

        #endregion

        // Start is called before the first frame update
        void Start()
        {
            // Global pos and rotation of mesh
            Matrix4x4d T = Matrix4x4d.Translate(new Vector3d(translation.x, translation.y, translation.z));
            Matrix4x4d R = Matrix4x4d.Rotate(new Vector3d(rotation.x, rotation.y, rotation.z));
            Matrix4x4d TR = T * R;

            // Mesh (body) - using PBD
            min = new Vector3d(minVector3.x, minVector3.y, minVector3.z);
            max = new Vector3d(maxVector3.x, maxVector3.y, maxVector3.z);
            Box3d bounds = new Box3d(min, max);
            TetrahedronsFromBounds source = new TetrahedronsFromBounds(radius, bounds);
            Body1 = new DeformableBody3d(source, radius, mass, stiffness, TR);

            // TEST: Second Body
            // -----------------

            // Global pos and rotation of mesh
            //Matrix4x4d T2 = Matrix4x4d.Translate(new Vector3d(-0.5f, 5f, 0.5f));
            //Matrix4x4d R2 = Matrix4x4d.Rotate(new Vector3d(0f, 0f, 0f));
            //Matrix4x4d TR2 = T2 * R2;

            //Vector3d min2 = new Vector3d(0.5f, 0.5f, 0.5f);
            //Vector3d max2 = new Vector3d(maxVector3.x, maxVector3.y, maxVector3.z);
            //Box3d bounds2 = new Box3d(min2, max2);
            //TetrahedronsFromBounds source2 = new TetrahedronsFromBounds(radius, bounds2);
            //Body2 = new DeformableBody3d(source2, radius, mass, stiffness, TR2);

            // TEST: Third Rigid Body with particles
            // -------------------------------------

            //Matrix4x4d T3 = Matrix4x4d.Translate(new Vector3d(0.0, 4.0f, 0.0));
            //Matrix4x4d R3 = Matrix4x4d.Rotate(new Vector3d(0.0, 0.0, 0.0));
            //double spacing3 = 0.5;
            //double radius3 = spacing3;
            //double mass3 = 1.0;
            //Vector3d min3 = new Vector3d(0f);
            //Vector3d max3 = new Vector3d(0.1f);
            //Box3d bounds3 = new Box3d(min, max);
            //ParticlesFromBounds source3 = new ParticlesFromBounds(spacing3, bounds3);
            //Body3 = new RidgidBody3d(source3, radius3, mass3, T3 * R3);

            // TEST: Fourth Rigid Body (Unity)
            // -------------------------------

            ExtRigidBody = obstacle.GetComponent<Rigidbody>();
            
            // Damping
            System.Random rnd = new System.Random(0);
            Body1.Dampning = 1.0;
            Body1.RandomizePositions(rnd, radius * 0.01);
            Body1.RandomizeConstraintOrder(rnd);

            //Body2.Dampning = 1.0;
            //Body2.RandomizePositions(rnd, radius * 0.01);
            //Body2.RandomizeConstraintOrder(rnd);

            //Body3.Dampning = 1.0;
            //Body3.RandomizePositions(rnd, radius3 * 0.01);

            // -------------------------

            // Static Bounds (Green) - Fix what is inside the bounds
            Vector3d smin = new Vector3d(min.x + fxMin.x, min.y + fxMin.y, min.z + fxMin.z);
            Vector3d smax = new Vector3d(min.x + fxMax.x, max.y + fxMax.y, max.z + fxMax.z);
            StaticBounds = new Box3d(smin, smax);
            Body1.MarkAsStatic(StaticBounds);

            //Vector3d smin = new Vector3d(0.5f, 0.5f, 0.5f);
            //Vector3d smax = new Vector3d(1f, 1f, 1f);
            //StaticBounds = new Box3d(smin, smax);
            //Body2.MarkAsStatic(StaticBounds);

            // -------------------------

            // Create Solver
            Solver = new Solver3d();

            // Add particle-based bodies
            Solver.AddBody(Body1);
            //Solver.AddBody(Body2);
            //Solver.AddBody(Body3);

            // Add external Unity bodies
            Solver.AddExternalBody(ExtRigidBody);

            // Add external forces
            Solver.AddForce(new GravitationalForce3d());

            // -------

            // Add collisions with ground
            Collision3d ground = new PlanarCollision3d(Vector3d.UnitY, 0);
            Solver.AddCollision(ground);

            // Add collision with particle-based bodies
            Collision3d bodyBody = new BodyCollision3d(Body1, Body2);
            Solver.AddCollision(bodyBody);
            //Collision3d bodyBody = new BodyCollision3d(Body1, Body3);
            //Solver.AddCollision(bodyBody);
            
            // Add collision with external bodies
            CollisionExternal3d bodyWithExt = new BodyCollisionExternal3d(Body1, ExtRigidBody);
            Solver.AddExternalCollision(bodyWithExt);

            // -------

            Solver.SolverIterations = solverIterations;
            Solver.CollisionIterations = collisionIterations;
            Solver.SleepThreshold = 1;

            // Create Spheres
            CreateSpheres();
            //CreateSpheres2();
            //CreateSpheres3();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            // Update time step
            timeStep = Time.fixedDeltaTime;
            
            // Update dts
            double dts = timeStep / iterations;

            for (int i = 0; i < iterations; i++)
                Solver.StepPhysics(dts);

            // Update Spheres
            UpdateSpheres();
            //UpdateSpheres2();
            //UpdateSpheres3();
        }

        void OnDestroy()
        {
            if (Spheres != null)
            {
                for (int i = 0; i < Spheres.Count; i++)
                    DestroyImmediate(Spheres[i]);
            }

            if (Spheres2 != null)
            {
                for (int i = 0; i < Spheres2.Count; i++)
                    DestroyImmediate(Spheres2[i]);
            }

            if (Spheres3 != null)
            {
                for (int i = 0; i < Spheres3.Count; i++)
                    DestroyImmediate(Spheres3[i]);
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
                DrawLines.DrawVertices(LINE_MODE.TETRAHEDRON, camera, Color.red, Body1.Positions, Body1.Indices, m);
                //DrawLines.DrawVertices(LINE_MODE.TETRAHEDRON, camera, Color.red, Body2.Positions, Body2.Indices, m);

                DrawLines.DrawBounds(camera, Color.green, StaticBounds, Matrix4x4d.Identity);
            }
        }

        private void CreateSpheres()
        {
            if (sphereMaterial == null) return;

            Spheres = new List<GameObject>();

            int numParticles = Body1.NumParticles;
            float diam = (float)Body1.ParticleRadius * 2.0f * scaleRadius;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body1.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = i.ToString(); // TEST
                sphere.transform.parent = transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = true;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                sphere.AddComponent<DetectCollision>();

                sphere.GetComponent<MeshRenderer>().material = sphereMaterialNoContact;

                Spheres.Add(sphere);
            }
        }

        private void CreateSpheres2()
        {
            if (sphereMaterial == null) return;

            Spheres2 = new List<GameObject>();

            int numParticles = Body2.NumParticles;
            float diam = (float)Body2.ParticleRadius * 2.0f;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body2.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                Spheres2.Add(sphere);
            }
        }

        private void CreateSpheres3()
        {
            if (sphereMaterial == null) return;

            Spheres3 = new List<GameObject>();

            int numParticles = Body3.NumParticles;
            float diam = (float)Body3.ParticleRadius * 2.0f;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body3.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                Spheres3.Add(sphere);
            }
        }

        public void UpdateSpheres()
        {
            if (Spheres != null)
            {
                for (int i = 0; i < Spheres.Count; i++)
                {
                    Vector3d pos = Body1.Positions[i];
                    Spheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);

                    if (Body1.IsContact[i])
                        Spheres[i].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                    else
                        Spheres[i].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                }
            }
        }

        public void UpdateSpheres2()
        {
            if (Spheres2 != null)
            {
                for (int i = 0; i < Spheres2.Count; i++)
                {
                    Vector3d pos = Body2.Positions[i];
                    Spheres2[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                }
            }
        }

        public void UpdateSpheres3()
        {
            if (Spheres3 != null)
            {
                for (int i = 0; i < Spheres3.Count; i++)
                {
                    Vector3d pos = Body3.Positions[i];
                    Spheres3[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                }
            }
        }

        public void CollisionFromChild(Collision hit, GameObject children)
        {
            if(printInformation)
                Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);

            if (printInformation)
                Debug.Log("CHANGING SPHERE: " + int.Parse(children.name) + " with the value " + Spheres[int.Parse(children.name)].GetComponent<DetectCollision>().Hit.GetContact(0).point);

            Body1.ExternalHit[int.Parse(children.name)] = hit.GetContact(0);
            Body1.IsContact[int.Parse(children.name)] = true;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body1.NumParticles; x++)
                {
                    if (Body1.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body1.IsContact[x] + " - Body1.ExternalHit[x]: " + Body1.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body1.IsContact[x] + " - Body1.ExternalHit[x]: " + Body1.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body1.ExternalHit[int.Parse(children.name)].point, Body1.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }

        public void ExitCollisionFromChild(GameObject children)
        {
            Body1.ExternalHit[int.Parse(children.name)] = new ContactPoint();
            Body1.IsContact[int.Parse(children.name)] = false;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body1.NumParticles; x++)
                {
                    if (Body1.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body1.IsContact[x] + " - Body1.ExternalHit[x]: " + Body1.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body1.IsContact[x] + " - Body1.ExternalHit[x]: " + Body1.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body1.ExternalHit[int.Parse(children.name)].point, Body1.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }
    } 
}
