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
    public class VegetationCreator : MonoBehaviour
    {
        #region Instance Fields

        // Debug
        [Header("Debug")]
        public bool printInformation;

        // External Obstacles
        [Header("External")]
        public GameObject obstacle2;
        public GameObject terrain;

        // Reference Grid (White)
        [Header("Grid")]
        public int GRID_SIZE = 2;

        // Global pos and rotation of cloth
        [Header("Body - Global pos and rot")]
        public GameObject cloth;
        public Vector3 translation;
        public Vector3 rotation;

        [Header("Body - Properties")]
        public int numParticlesX = 3;
        public int numParticlesY = 3;
        public double mass = 1.0;
        public double diameter = 0.5;
        //public double width = 5.0;
        //public double depth = 5.0;
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;
        public Vector3 fxMinCloth;
        public Vector3 fxMaxCloth;

        [Header("Mesh - Debug")]
        public bool drawLines = true;
        public bool drawMesh = true;
        public bool drawSpheres = true;
        public Mesh mesh;
        public Material sphereMaterial;
        public Material sphereMaterialNoContact;
        public Material sphereMaterialContact;
        public Material sphereMaterialBroken;
        [Range(0f, 1f)] public float scaleRadius;

        [Header("Solver")]
        public int iterations = 1; // 4 before
        public int solverIterations = 1; // 2 before
        public int collisionIterations = 1; // 2 before

        #endregion

        #region Instance Properties

        private List<GameObject> Spheres2 { get; set; }

        private ClothBody3d Body2 { get; set; }

        private Rigidbody ExtRigidBody2 { get; set; }
        private Rigidbody Terrain { get; set; }

        private Solver3d Solver { get; set; }

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

            // 2. Create Cloth Body
            // ==========================

            // Global pos and rotation of mesh

            // Always translate upwards s.t. we leave a virtual row underground
            double height = ((numParticlesY * diameter) / 2) - diameter;

            Matrix4x4d TCloth = Matrix4x4d.Translate(new Vector3d(translation.x, height, translation.z)); // should be height
            Matrix4x4d RCloth = Matrix4x4d.Rotate(new Vector3d(rotation.x, rotation.y, rotation.z));
            Matrix4x4d TRCloth = TCloth * RCloth;

            // Create cloth body

            double width = (numParticlesX - 1) * diameter;
            double depth = (numParticlesY - 1) * diameter;

            TrianglesFromGrid source2 = new TrianglesFromGrid(diameter / 2, width, depth);
            Body2 = new ClothBody3d(source2, diameter / 2, mass, stretchStiffness, bendStiffness, TRCloth);

            ExtRigidBody2 = obstacle2.GetComponent<Rigidbody>();
            
            Body2.Dampning = 1.0;

            //Vector3d sminCloth = new Vector3d(fxMinCloth.x, fxMinCloth.y, fxMinCloth.z);
            //Vector3d smaxCloth = new Vector3d(fxMaxCloth.x, fxMaxCloth.y, fxMaxCloth.z);
            Vector3d sminCloth = new Vector3d(translation.x - (numParticlesX * diameter) / 2, -(float)diameter, translation.z - (diameter / 2));
            Vector3d smaxCloth = new Vector3d(translation.x + (numParticlesX * diameter) / 2, (float)diameter, translation.z + (diameter / 2));
            StaticBounds2 = new Box3d(sminCloth, smaxCloth);
            Body2.MarkAsStatic(StaticBounds2);

            // Create Solver
            Solver = new Solver3d();

            // Add particle-based bodies
            Solver.AddBody(Body2);

            // Add external Unity bodies
            Solver.AddExternalBody(ExtRigidBody2);

            // Add external forces
            Solver.AddForce(new GravitationalForce3d());

            // -------

            // Add collisions with ground
            //Collision3d ground = new PlanarCollision3d(Vector3d.UnitY, (float)diameter); // DEFAULT 0 - changed to counteract the scaling factor // NEED TO BE THE DIAMETER TO MAKE ONE SPHERE IN UNDERGROUND
            //Solver.AddCollision(ground);

            // TEST
            Collision3d realGround = new PlanarCollision3d(Vector3d.UnitY, terrain.transform.position.y);
            Solver.AddCollision(realGround);
            
            CollisionExternal3d bodyWithExt4 = new BodyCollisionExternal3d(Body2, ExtRigidBody2);
            Solver.AddExternalCollision(bodyWithExt4);

            // -------

            Solver.SolverIterations = solverIterations;
            Solver.CollisionIterations = collisionIterations;
            Solver.SleepThreshold = 1;

            // Create Spheres
            if (drawSpheres)
            {
                CreateSpheres2();
            }

        }

        private void FixedUpdate()
        {
            // Update time step
            timeStep = Time.fixedDeltaTime;

            // Update dts
            double dts = timeStep / iterations;

            for (int i = 0; i < iterations; i++)
                Solver.StepPhysics(dts);

            // Update Spheres
            if (drawSpheres)
            {
                UpdateSpheres2();
            }
        }

        private void OnDestroy()
        {
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
                DrawLines.DrawVertices(LINE_MODE.TRIANGLES, camera, Color.red, Body2.Positions, Body2.Indices, m);

                DrawLines.DrawBounds(camera, Color.green, StaticBounds2, Matrix4x4d.Identity);

            }

            if (drawMesh)
            {
                Vector3[] vertices = new Vector3[Body2.Positions.Length * 2]; // 24
                Vector3[] normals = new Vector3[(Body2.Indices.Length / 3) * 2]; // 24

                GetComponent<MeshFilter>().mesh = mesh = new Mesh();
                mesh.name = "TextureMesh";

                for (int i = 0; i < Body2.Positions.Length; i++) // From 0 to 12
                {
                    vertices[i] = Body2.Positions[i].ToVector3();
                }

                for (int i = Body2.Positions.Length; i < Body2.Positions.Length * 2; i++) // From 12 to 24 // TEST
                {
                    vertices[i] = Body2.Positions[i - Body2.Positions.Length].ToVector3();
                }

                mesh.vertices = vertices;

                int[] triangles = new int[Body2.Indices.Length * 2]; // 72

                for (int i = 0; i < Body2.Indices.Length; i++) // From 0 to 36
                {
                    triangles[i] = Body2.Indices[i];
                }

                for (int i = Body2.Indices.Length; i < Body2.Indices.Length * 2; i++) // From 36 to 72 // ERROR - FLIP NORMAL
                {
                    triangles[i] = Body2.Indices[i - Body2.Indices.Length];
                }

                mesh.triangles = triangles;

                //Debug.Log("Normals: " + normals.Length);

                //for (var n = 12; n < normals.Length; n++)
                //{
                //    normals[n] = -normals[n];
                //}

                //mesh.normals = normals;

                mesh.RecalculateNormals();
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
                sphere.AddComponent<DetectionCollision>();

                sphere.GetComponent<MeshRenderer>().material = sphereMaterialNoContact;

                Spheres2.Add(sphere);
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

                    if (Body2.IsContact[i] && !Body2.IsBroken[i])
                        Spheres2[i].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                    else if (!Body2.IsContact[i] && !Body2.IsBroken[i])
                        Spheres2[i].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                    else if (Body2.IsBroken[i])
                        Spheres2[i].GetComponent<MeshRenderer>().material = sphereMaterialBroken;
                }
            }
        }

        public void CollisionFromChildBody2(Collision hit, GameObject children)
        {
            if (printInformation)
                Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);

            if (printInformation)
                Debug.Log("CHANGING SPHERE: " + int.Parse(children.name) + " with the value " + Spheres2[int.Parse(children.name)].GetComponent<DetectCollision>().Hit.GetContact(0).point);

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
