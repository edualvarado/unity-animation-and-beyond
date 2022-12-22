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

        // Test
        public GameObject plane;
        public float distance;
        public GameObject obstacle;
        [Range(0f, 1f)] public float scaleRadius;
        
        // Reference Grid (White)
        public int GRID_SIZE = 2;

        // Global pos and rotation of mesh
        public Vector3 translation;
        public Vector3 rotation;

        // Mesh
        public Vector3 minVector3;
        public Vector3 maxVector3;
        public double radius = 0.25;
        public double stiffness = 0.2;
        public double mass = 1.0;

        public float fxmin, fxmax;
        public float fymin, fymax;
        public float fzmin, fzmax;

        public int iterations = 1; // 4 before


        public bool drawLines = true;
        public Material sphereMaterial;

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

        // XPBD -> deltaTs = deltaT / n
        private const double timeStep = 1.0 / 60.0;

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
            Matrix4x4d T2 = Matrix4x4d.Translate(new Vector3d(-0.5f, 5f, 0.5f));
            Matrix4x4d R2 = Matrix4x4d.Rotate(new Vector3d(0f, 0f, 0f));
            Matrix4x4d TR2 = T2 * R2;

            Vector3d min2 = new Vector3d(0.5f, 0.5f, 0.5f);
            Vector3d max2 = new Vector3d(maxVector3.x, maxVector3.y, maxVector3.z);
            Box3d bounds2 = new Box3d(min2, max2);
            TetrahedronsFromBounds source2 = new TetrahedronsFromBounds(radius, bounds2);
            Body2 = new DeformableBody3d(source2, radius, mass, stiffness, TR2);

            // TEST: Third Rigid Body
            // -------------------------

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

            // TEST: Fourth Rigid Body
            // -------------------------

            ExtRigidBody = obstacle.GetComponent<Rigidbody>();
            
            // -------------------------

            // ???
            System.Random rnd = new System.Random(0);
            Body1.Dampning = 1.0;
            Body1.RandomizePositions(rnd, radius * 0.01);
            Body1.RandomizeConstraintOrder(rnd);

            // TEST: Second Body
            //Body2.Dampning = 1.0;
            //Body2.RandomizePositions(rnd, radius * 0.01);
            //Body2.RandomizeConstraintOrder(rnd);

            // TEST: Third Body
            //Body3.Dampning = 1.0;
            //Body3.RandomizePositions(rnd, radius3 * 0.01);
            
            // -------------------------

            // Static Bounds (Green)
            //Vector3d smin = new Vector3d(min.x + fxmin, min.y + fymin, min.z + fzmin);
            //Vector3d smax = new Vector3d(min.x + fxmax, max.y + fymax, max.z + fzmax);
            //StaticBounds = new Box3d(smin, smax);
            //Body.MarkAsStatic(StaticBounds);

            //Vector3d smin = new Vector3d(0.5f, 0.5f, 0.5f);
            //Vector3d smax = new Vector3d(1f, 1f, 1f);
            //StaticBounds = new Box3d(smin, smax);
            //Body2.MarkAsStatic(StaticBounds);

            // -------------------------

            // Constrainst Solver
            Solver = new Solver3d();

            // TEST
            Solver.AddBody(Body1);
            //Solver.AddBody(Body2);
            //Solver.AddBody(Body3);

            // TEST
            Solver.AddExternalBody(ExtRigidBody); // Or include GameObject, let's see - Add in solver to list "ExternalBodies"

            Solver.AddForce(new GravitationalForce3d());

            // -------

            // Permanent Collisions
            Collision3d ground = new PlanarCollision3d(Vector3d.UnitY, 0);
            Solver.AddCollision(ground);

            // TEST TODO: Add COLLISION WITH EXTERNAL BODY
            CollisionExternal3d bodyWithExt = new BodyCollisionExternal3d(Body1, ExtRigidBody);
            Solver.AddExternalCollision(bodyWithExt);

            // TEST
            // Collision with Body2
            Collision3d bodyBody = new BodyCollision3d(Body1, Body2);
            Solver.AddCollision(bodyBody);

            // TEST
            // Collision with Body3
            //Collision3d bodyBody = new BodyCollision3d(Body1, Body3);
            //Solver.AddCollision(bodyBody);

            // -------

            Solver.SolverIterations = 2;
            Solver.CollisionIterations = 2;
            Solver.SleepThreshold = 1;

            // TEST
            CreateSpheres();
            CreateSpheres2();
            //CreateSpheres3();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            double dt = timeStep / iterations;
            //double dt = Time.fixedDeltaTime / iterations; // XPBD

            for (int i = 0; i < iterations; i++)
                Solver.StepPhysics(dt);

            // TEST
            UpdateSpheres();
            UpdateSpheres2();
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
            //Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);

            for (int x = 0; x < Body1.NumParticles; x++)
            {
                if (x == int.Parse(children.name))
                {
                    //Debug.Log("CHANGING SPHERE: " + x + " with the value " + Spheres[x].GetComponent<DetectCollision>().Hit.GetContact(0).point);

                    Body1.ExternalHit[x] = hit.GetContact(0);
                    Body1.IsContact[x] = true;
                }
            }

            //Debug.Log("==================");
            //for (int x = 0; x < Body1.NumParticles; x++)
            //{
            //    if (Body1.ExternalHit[x].point != Vector3.zero)
            //        Debug.Log("[INFO] Sphere: " + x + " - Body1.ExternalHit[x]: " + Body1.ExternalHit[x].point);
            //    else
            //        Debug.Log("[INFO] Sphere: " + x + " - Body1.ExternalHit[x]: 0");
            //}
            //Debug.Log("==================");
            
            // It draws multiple rays
            //Debug.DrawRay(Body1.ExternalHit[int.Parse(children.name)].point, Body1.ExternalHit[int.Parse(children.name)].normal, Color.blue, 1f);
        }

        public void ExitCollisionFromChild(GameObject children)
        {
            for (int x = 0; x < Body1.NumParticles; x++)
            {
                if (x == int.Parse(children.name))
                {
                    Body1.IsContact[x] = false;
                }
            }
        }

        //public void OnCollisionEnter(Collision other)
        //{
        //    Debug.Log("OnCollisionEnter (MAIN SPHERE): " + other.gameObject.name);
        //}
    } 
}
