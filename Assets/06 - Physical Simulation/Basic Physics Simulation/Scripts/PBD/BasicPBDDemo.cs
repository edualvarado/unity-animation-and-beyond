using UnityEngine;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using Common.Unity.Drawing;
using Common.Unity.Mathematics;

using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Bodies.Deformable;
using PositionBasedDynamics.Bodies.Cloth;
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
        [Header("Body 1 - Global pos and rot")]
        public GameObject tet;
        public Vector3 translation1;
        public Vector3 rotation1;

        // Meshes
        [Header("Body 1 - Properties")]
        public Vector3 minVector3;
        public Vector3 maxVector3;
        public double radius1 = 0.25;
        public double stiffness1 = 0.2;
        public double mass1 = 1.0;
        public Vector3 fxMin;
        public Vector3 fxMax;

        // Global pos and rotation of cloth
        [Header("Body 2 - Global pos and rot")]
        public GameObject cloth;
        public Vector3 translation2;
        public Vector3 rotation2;

        [Header("Body 2 - Properties")]
        public double radius2 = 0.25;
        public double mass2 = 1.0;
        public double width2 = 5.0;
        public double height2 = 4.0;
        public double depth2 = 5.0;
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;
        
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

        private DeformableBody3d Body1 { get; set; }
        private ClothBody3d Body2 { get; set; }
        private Rigidbody ExtRigidBody { get; set; }

        private Solver3d Solver { get; set; }

        private Box3d StaticBounds1 { get; set; }
        private Box3d StaticBounds2 { get; set; }

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
            // 1. Create Tetrahedron Body
            // ==========================
            
            // Global pos and rotation of mesh
            Matrix4x4d T = Matrix4x4d.Translate(new Vector3d(translation1.x, translation1.y, translation1.z));
            Matrix4x4d R = Matrix4x4d.Rotate(new Vector3d(rotation1.x, rotation1.y, rotation1.z));
            Matrix4x4d TR = T * R;

            // Mesh (body) - using PBD
            min = new Vector3d(minVector3.x, minVector3.y, minVector3.z);
            max = new Vector3d(maxVector3.x, maxVector3.y, maxVector3.z);
            Box3d bounds = new Box3d(min, max);

            TetrahedronsFromBounds source1 = new TetrahedronsFromBounds(radius1, bounds);

            Body1 = new DeformableBody3d(source1, radius1, mass1, stiffness1, TR);

            // 2. Create Cloth Body
            // ==========================

            Matrix4x4d TCloth = Matrix4x4d.Translate(new Vector3d(translation2.x, translation2.y + height2, translation2.z));
            Matrix4x4d RCloth = Matrix4x4d.Rotate(new Vector3d(rotation2.x, rotation2.y, rotation2.z));
            Matrix4x4d TRCloth = TCloth * RCloth;

            TrianglesFromGrid source2 = new TrianglesFromGrid(radius2, width2, depth2);
            
            Body2 = new ClothBody3d(source2, radius1, mass1, stretchStiffness, bendStiffness, TRCloth);

            // Add external body

            ExtRigidBody = obstacle.GetComponent<Rigidbody>();
            
            // Damping
            System.Random rnd = new System.Random(0);
            Body1.Dampning = 1.0;
            Body1.RandomizePositions(rnd, radius1 * 0.01);
            Body1.RandomizeConstraintOrder(rnd);

            Body2.Dampning = 1.0;
            //Body2.RandomizePositions(rnd, radius * 0.01);
            //Body2.RandomizeConstraintOrder(rnd);

            // -------------------------

            // Static Bounds (Green) - Fix what is inside the bounds
            Vector3d smin = new Vector3d(min.x + fxMin.x, min.y + fxMin.y, min.z + fxMin.z);
            Vector3d smax = new Vector3d(min.x + fxMax.x, max.y + fxMax.y, max.z + fxMax.z);
            StaticBounds1 = new Box3d(smin, smax);
            Body1.MarkAsStatic(StaticBounds1);

            Vector3d sminCloth = new Vector3d(-width2 / 2 - 0.1, height2 - 0.1, -depth2 / 2 - 0.1);
            Vector3d smaxCloth = new Vector3d(width2 / 2 + 0.1, height2 + 0.1, -depth2 / 2 + 0.1);
            StaticBounds2 = new Box3d(sminCloth, smaxCloth);
            Body2.MarkAsStatic(StaticBounds2);

            // -------------------------

            // Create Solver
            Solver = new Solver3d();

            // Add particle-based bodies
            Solver.AddBody(Body1);
            Solver.AddBody(Body2);

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
            
            // Add collision with external bodies
            CollisionExternal3d bodyWithExt = new BodyCollisionExternal3d(Body1, ExtRigidBody);
            Solver.AddExternalCollision(bodyWithExt);
            
            CollisionExternal3d bodyWithExt2 = new BodyCollisionExternal3d(Body2, ExtRigidBody);
            Solver.AddExternalCollision(bodyWithExt2);

            // -------

            Solver.SolverIterations = solverIterations;
            Solver.CollisionIterations = collisionIterations;
            Solver.SleepThreshold = 1;

            // Create Spheres
            CreateSpheres();
            CreateSpheres2();
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
            UpdateSpheres2();
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
                DrawLines.DrawVertices(LINE_MODE.TRIANGLES, camera, Color.red, Body2.Positions, Body2.Indices, m);

                DrawLines.DrawBounds(camera, Color.green, StaticBounds1, Matrix4x4d.Identity);

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
                sphere.transform.parent = tet.transform;
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
            float diam = (float)Body2.ParticleRadius * 2.0f * scaleRadius;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body2.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = i.ToString(); // TEST
                sphere.transform.parent = cloth.transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = true;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                sphere.AddComponent<DetectCollision>();

                sphere.GetComponent<MeshRenderer>().material = sphereMaterialNoContact;

                Spheres2.Add(sphere);
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

                    if (Body2.IsContact[i])
                        Spheres2[i].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                    else
                        Spheres2[i].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                }
            }
        }
        public void CollisionFromChildBody1(Collision hit, GameObject children)
        {
            if (printInformation)
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

        public void ExitCollisionFromChildBody1(GameObject children)
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

        public void CollisionFromChildBody2(Collision hit, GameObject children)
        {
            if(printInformation)
                Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);

            if (printInformation)
                Debug.Log("CHANGING SPHERE: " + int.Parse(children.name) + " with the value " + Spheres[int.Parse(children.name)].GetComponent<DetectCollision>().Hit.GetContact(0).point);

            Body2.ExternalHit[int.Parse(children.name)] = hit.GetContact(0);
            Body2.IsContact[int.Parse(children.name)] = true;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body2.NumParticles; x++)
                {
                    if (Body2.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body2.IsContact[x] + " - Body2.ExternalHit[x]: " + Body2.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body2.IsContact[x] + " - Body2.ExternalHit[x]: " + Body2.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body2.ExternalHit[int.Parse(children.name)].point, Body2.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }

        public void ExitCollisionFromChildBody2(GameObject children)
        {
            Body2.ExternalHit[int.Parse(children.name)] = new ContactPoint();
            Body2.IsContact[int.Parse(children.name)] = false;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body2.NumParticles; x++)
                {
                    if (Body2.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body2.IsContact[x] + " - Body2.ExternalHit[x]: " + Body2.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body2.IsContact[x] + " - Body2.ExternalHit[x]: " + Body2.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body2.ExternalHit[int.Parse(children.name)].point, Body2.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }        
    } 
}
