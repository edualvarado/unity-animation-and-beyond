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
        public GameObject obstacle;
        public GameObject terrain;

        // Reference Grid (White)
        [Header("Grid")]
        public int GRID_SIZE = 2;

        // Global pos and rotation of cloth
        [Header("Body - Global position and orientation")]
        public GameObject cloth; // TODO For each body
        public Vector3 translation;
        public Vector3 rotation;

        [Header("Body Type - Properties")]
        public Vector2 plantSize;
        public double mass = 1.0;
        public double diameter = 0.5;
        [Range(0f, 1f)] public float scaleRadius;

        [Header("Body - Stiffness")]
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;

        [Header("Mesh - Debug")]
        public bool drawLines = true;
        public bool drawMesh = true;
        public bool drawSpheres = true;

        [Header("Mesh - Properties")]
        public Mesh mesh;
        public Material sphereMaterial;
        public Material sphereMaterialNoContact;
        public Material sphereMaterialContact;
        public Material sphereMaterialBroken;

        [Header("PBD Solver")]
        public int iterations = 1; // 4 before
        public int solverIterations = 1; // 2 before
        public int collisionIterations = 1; // 2 before

        #endregion

        #region Instance Properties

        private List<GameObject> Spheres { get; set; }
        private Plant PlantType { get; set; }
        private ClothBody3d Body { get; set; }
        private Rigidbody ExtRigidBody { get; set; }
        private Solver3d Solver { get; set; }
        private Box3d StaticBounds { get; set; }

        #endregion

        #region Read-only & Static Fields

        //private double timeStep = 1.0 / 60.0; 
        private double timeStep;
        
        #endregion

        // Start is called before the first frame update
        void Start()
        {
            // TODO: List of plants?
            PlantType = new Plant(plantSize, mass, diameter, scaleRadius, stretchStiffness, bendStiffness);
            Body = PlantType.CreatePlant(translation, rotation);

            // Solver
            // -------

            // Create Solver
            Solver = new Solver3d();

            // Add particle-based bodies
            Solver.AddBody(Body);

            // Add external Unity bodies
            ExtRigidBody = obstacle.GetComponent<Rigidbody>();
            Solver.AddExternalBody(ExtRigidBody);

            // Add external forces
            Solver.AddForce(new GravitationalForce3d());

            // Collisions
            // -------

            // Add collisions with ground
            //Collision3d ground = new PlanarCollision3d(Vector3d.UnitY, (float)diameter); // DEFAULT 0 - changed to counteract the scaling factor // NEED TO BE THE DIAMETER TO MAKE ONE SPHERE IN UNDERGROUND
            //Solver.AddCollision(ground);            
            Collision3d realGround = new PlanarCollision3d(Vector3d.UnitY, terrain.transform.position.y);
            Solver.AddCollision(realGround);
            
            // Add collisions with external bodies
            CollisionExternal3d bodyWithExternal = new BodyCollisionExternal3d(Body, ExtRigidBody);
            Solver.AddExternalCollision(bodyWithExternal);

            // Iterations
            // -------

            Solver.SolverIterations = solverIterations;
            Solver.CollisionIterations = collisionIterations;
            Solver.SleepThreshold = 1;

            // Create Spheres
            if (drawSpheres)
            {
                CreateSpheres();
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
                UpdateSpheres();
            }
        }

        private void OnDestroy()
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

                // Grid
                Vector3 min = new Vector3(-GRID_SIZE, 0, -GRID_SIZE);
                Vector3 max = new Vector3(GRID_SIZE, 0, GRID_SIZE);
                DrawLines.DrawGrid(camera, Color.white, min, max, 1, transform.localToWorldMatrix);
                
                // Vertices
                Matrix4x4d m = MathConverter.ToMatrix4x4d(transform.localToWorldMatrix);
                DrawLines.DrawVertices(LINE_MODE.TRIANGLES, camera, Color.red, Body.Positions, Body.Indices, m);

                // Static Bounds
                Vector3d sminCloth = new Vector3d(translation.x - (plantSize.x * diameter) / 2, -(float)diameter, translation.z - (diameter / 2));
                Vector3d smaxCloth = new Vector3d(translation.x + (plantSize.x * diameter) / 2, (float)diameter, translation.z + (diameter / 2));
                StaticBounds = new Box3d(sminCloth, smaxCloth);   
                DrawLines.DrawBounds(camera, Color.green, StaticBounds, Matrix4x4d.Identity);

            }

            if (drawMesh)
            {
                Vector3[] vertices = new Vector3[Body.Positions.Length * 2]; // 24
                Vector3[] normals = new Vector3[(Body.Indices.Length / 3) * 2]; // 24

                GetComponent<MeshFilter>().mesh = mesh = new Mesh();
                mesh.name = "TextureMesh";

                for (int i = 0; i < Body.Positions.Length; i++) // From 0 to 12
                {
                    vertices[i] = Body.Positions[i].ToVector3();
                }

                for (int i = Body.Positions.Length; i < Body.Positions.Length * 2; i++) // From 12 to 24 // TEST
                {
                    vertices[i] = Body.Positions[i - Body.Positions.Length].ToVector3();
                }

                mesh.vertices = vertices;

                int[] triangles = new int[Body.Indices.Length * 2]; // 72

                for (int i = 0; i < Body.Indices.Length; i++) // From 0 to 36
                {
                    triangles[i] = Body.Indices[i];
                }

                for (int i = Body.Indices.Length; i < Body.Indices.Length * 2; i++) // From 36 to 72 // ERROR - FLIP NORMAL
                {
                    triangles[i] = Body.Indices[i - Body.Indices.Length];
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

        private void CreateSpheres()
        {
            if (sphereMaterial == null) return;

            Spheres = new List<GameObject>();

            int numParticles = Body.NumParticles;
            float diam = (float)Body.ParticleRadius * 2.0f * scaleRadius;

            for (int i = 0; i < numParticles; i++)
            {
                Vector3 pos = MathConverter.ToVector3(Body.Positions[i]);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.name = i.ToString(); // TEST
                sphere.transform.parent = cloth.transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = true;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                sphere.AddComponent<DetectionCollision>();

                sphere.GetComponent<MeshRenderer>().material = sphereMaterialNoContact;

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

                    if (Body.IsContact[i] && !Body.IsBroken[i])
                        Spheres[i].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                    else if (!Body.IsContact[i] && !Body.IsBroken[i])
                        Spheres[i].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                    else if (Body.IsBroken[i])
                        Spheres[i].GetComponent<MeshRenderer>().material = sphereMaterialBroken;
                }
            }
        }

        public void CollisionFromChildBody2(Collision hit, GameObject children)
        {
            if (printInformation)
                Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);

            if (printInformation)
                Debug.Log("CHANGING SPHERE: " + int.Parse(children.name) + " with the value " + Spheres[int.Parse(children.name)].GetComponent<DetectCollision>().Hit.GetContact(0).point);

            Body.ExternalHit[int.Parse(children.name)] = hit.GetContact(0);
            Body.IsContact[int.Parse(children.name)] = true;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body.NumParticles; x++)
                {
                    if (Body.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body.IsContact[x] + " - Body2.ExternalHit[x]: " + Body.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body.IsContact[x] + " - Body2.ExternalHit[x]: " + Body.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body.ExternalHit[int.Parse(children.name)].point, Body.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }

        public void ExitCollisionFromChildBody2(GameObject children)
        {
            Body.ExternalHit[int.Parse(children.name)] = new ContactPoint();
            Body.IsContact[int.Parse(children.name)] = false;

            if (printInformation)
            {
                Debug.Log("==================");
                for (int x = 0; x < Body.NumParticles; x++)
                {
                    if (Body.IsContact[x])
                        Debug.Log("[INFO] Sphere: " + x + " is in contact: " + Body.IsContact[x] + " - Body2.ExternalHit[x]: " + Body.ExternalHit[x].point);
                    else
                        Debug.Log("[INFO] Sphere: " + x + " NOT in contact: " + Body.IsContact[x] + " - Body2.ExternalHit[x]: " + Body.ExternalHit[x].point);
                }
                Debug.Log("==================");

                // It draws multiple rays
                Debug.DrawRay(Body.ExternalHit[int.Parse(children.name)].point, Body.ExternalHit[int.Parse(children.name)].normal, Color.blue, 0.1f);
            }
        }
    } 
}
