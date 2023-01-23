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

        [Header("Randomizer")]
        public bool activateRandomization;
        public Vector2 limitsTerrain;
        public Vector3 testTranslation;
        public Vector3 testRotation;
        
        // Debug
        [Header("Debug")]
        public bool printInformation;

        // External Obstacles
        [Header("External")]
        public bool applyGravity;
        public GameObject obstacle;
        public GameObject obstacle2;
        public GameObject terrain;

        [Header("List")]
        public List<List<GameObject>> ListOfLists = new List<List<GameObject>>();

        // Global pos and rotation of cloth
        [Header("Body - Global position and orientation")]
        //public GameObject bodyPlant; // TODO For each body
        public Vector3 translation;
        public Vector3 rotation;

        [Header("Body Type - Properties")]
        public int numberOfPlants = 2;
        public List<ClothBody3d> plants = new List<ClothBody3d>();
        public Vector2 plantSize;
        public double mass = 1.0;
        public double diameter = 0.5;
        public double spaceBetween = 0;

        [Header("Body - Stiffness")]
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;

        [Header("Mesh - Debug")]
        public bool drawLines = true;
        public bool drawMesh = true;
        public bool drawSpheres = true;
        public bool makeSpheresVisible = false;

        [Header("Mesh - Properties")]
        public Material grass;
        public Material sphereMaterial;
        public Material sphereMaterialNoContact;
        public Material sphereMaterialContact;
        public Material sphereMaterialBroken;

        [Header("PBD Solver")]
        public int iterations = 2; // 4 before
        public int solverIterations = 1; // 2 before
        public int collisionIterations = 1; // 2 before

        [Header("Mesh Deformation")]
        public Mesh[] deformingMesh;
        public Vector3[] originalVertices, displacedVertices;
        public Vector3[] originalVerticesLocal, displacedVerticesLocal;
        public Vector2[] originalUV, displacedUV;
        public int[] triangles; 

        #endregion

        #region Instance Properties

        private GameObject[,] SpheresPlant { get; set; }

        private List<GameObject> Spheres { get; set; }
        private Plant PlantType { get; set; }
        private ClothBody3d Body { get; set; }
        private Rigidbody ExtRigidBody { get; set; }
        private Rigidbody ExtRigidBody2 { get; set; }

        private Solver3d Solver { get; set; }
        private Box3d StaticBounds { get; set; }

        #endregion

        #region Read-only & Static Fields

        //private double timeStep = 1.0 / 60.0; 
        private double timeStep;
  
        private int GRID_SIZE = 2;
        
        #endregion

        // Start is called before the first frame update
        void Start()
        {
            // Initialize Solver
            Solver = new Solver3d();

            // List of plants for certain type
            for (int i = 0; i < numberOfPlants; i++)
            {
                Vector3 randomTranslation = new Vector3(Random.Range(-limitsTerrain.x, limitsTerrain.x), 0, Random.Range(-limitsTerrain.y, limitsTerrain.y));
                float randomRotation = Random.Range(0, 360);

                PlantType = new Plant(plantSize, mass, diameter, spaceBetween, stretchStiffness, bendStiffness);
                Body = PlantType.CreatePlant(new Vector3(translation.x + i, translation.y, translation.z), new Vector3(rotation.x, rotation.y, rotation.z));
                //PlantType = new Plant(plantSize, mass, diameter, spaceBetween, stretchStiffness, bendStiffness);
                //Body = PlantType.CreatePlant(randomTranslation, new Vector3(rotation.x, randomRotation, rotation.z));

                plants.Add(Body);

                // Add particle-based bodies
                Solver.AddBody(Body);
            }

            // Initialize spheres for each plant of one type
            SpheresPlant = new GameObject[plants.Count, Body.NumParticles];
            deformingMesh = new Mesh[numberOfPlants];

            // Add external Unity bodies
            ExtRigidBody = obstacle.GetComponent<Rigidbody>();
            Solver.AddExternalBody(ExtRigidBody);

            ExtRigidBody2 = obstacle2.GetComponent<Rigidbody>();
            Solver.AddExternalBody(ExtRigidBody2);
            
            // Add external forces
            if (applyGravity)
                Solver.AddForce(new GravitationalForce3d());

            // Collisions
            // -------

            // Add collisions with ground
            Collision3d realGround = new PlanarCollision3d(Vector3d.UnitY, terrain.transform.position.y - (float)diameter / 2); 
            Solver.AddCollision(realGround);

            for (int i = 0; i < numberOfPlants; i++)
            {
                // Add collisions with external bodies // TODO for all bodies
                CollisionExternal3d bodyWithExternal = new BodyCollisionExternal3d(plants[i], ExtRigidBody);
                Solver.AddExternalCollision(bodyWithExternal);

                CollisionExternal3d bodyWithExternal2 = new BodyCollisionExternal3d(plants[i], ExtRigidBody2);
                Solver.AddExternalCollision(bodyWithExternal2); 
            }

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

            if(drawMesh)
            {
                CreateMesh();
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

            if (drawMesh)
            {
                UpdateMesh();
            }
        }

        private void CreateMesh()
        {
            for (int i = 0; i < numberOfPlants; i++)
            {
                // Get mesh
                GameObject parent = GameObject.Find("Parent_" + i.ToString());
                GameObject type = GameObject.Find("Type_" + i.ToString());

                deformingMesh[i] = new Mesh();
                deformingMesh[i].name = "TextureMesh_" + i.ToString();

                // Assign to parent
                parent.GetComponent<MeshFilter>().mesh = deformingMesh[i];
                
                // Initialize vertices
                originalVertices = new Vector3[plants[i].Positions.Length];
                originalVerticesLocal = new Vector3[plants[i].Positions.Length];

                for (int j = 0; j < plants[i].Positions.Length; j++)
                {
                    originalVertices[j] = plants[i].Positions[j].ToVector3();
                    originalVerticesLocal[j] = type.transform.InverseTransformPoint(plants[i].Positions[j].ToVector3());

                    Debug.Log("Positions: " + j + " is " + originalVertices[j]);
                    Debug.Log("Local Positions: " + j + " is " + originalVerticesLocal[j]);
                }

                displacedVertices = new Vector3[plants[i].Positions.Length];
                displacedVerticesLocal = new Vector3[plants[i].Positions.Length];

                // Initialize UV
                originalUV = new Vector2[originalVertices.Length];
                for (int j = 0; j < plants[i].Positions.Length; j++)
                {
                    originalUV[j] = new Vector2(originalVertices[j].x, originalVertices[j].y); // ERROR
                }

                displacedUV = new Vector2[displacedVertices.Length];

                // Initialize Triangles
                triangles = new int[plants[i].Indices.Length];
                for (int j = 0; j < plants[i].Indices.Length; j++)
                {
                    triangles[j] = plants[i].Indices[j];
                }

                // Set to mesh
                deformingMesh[i].vertices = originalVertices;
                deformingMesh[i].uv = originalUV;
                deformingMesh[i].triangles = triangles;
            }
        }

        private void UpdateMesh()
        {
            for (int i = 0; i < numberOfPlants; i++)
            {
                //Vector3[] normals = new Vector3[plants[i].Indices.Length / 3]; // Number of indices side

                for (int j = 0; j < plants[i].Positions.Length; j++)
                {
                    displacedVertices[j] = plants[i].Positions[j].ToVector3();
                    displacedUV[j] = new Vector2(displacedVertices[j].x, displacedVertices[j].y);
                }

                for (var j = 0; j < displacedUV.Length; j++)
                {
                    var rot = Quaternion.Euler(testRotation);
                    displacedUV[j] = rot * displacedUV[j];
                }

                // Define triangles
                triangles = new int[plants[i].Indices.Length];
                for (int j = 0; j < plants[i].Indices.Length; j++)
                {
                    triangles[j] = plants[i].Indices[j];
                }

                // Set to mesh
                deformingMesh[i].vertices = displacedVertices;
                deformingMesh[i].uv = displacedUV;
                deformingMesh[i].triangles = triangles;
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

                // Transformation
                Matrix4x4d m = MathConverter.ToMatrix4x4d(transform.localToWorldMatrix);

                for (int i = 0; i < numberOfPlants; i++)
                {
                    // Vertices
                    DrawLines.DrawVertices(LINE_MODE.TRIANGLES, camera, Color.red, plants[i].Positions, plants[i].Indices, m);

                    // Static Bounds
                    //Vector3d sminCloth = new Vector3d(translation.x + i - (plantSize.x / 2) - (diameter / 2), -((float)spaceBetween - (float)diameter) - (float)(diameter / 1), translation.z - (diameter / 2));
                    //Vector3d smaxCloth = new Vector3d(translation.x + i + (plantSize.x / 2) + (diameter / 2), (float)diameter, translation.z + (diameter / 2));
                    Vector3d sminCloth = new Vector3d(translation.x - 10, -((float)spaceBetween - (float)diameter) - (float)(diameter / 1), translation.z - 10); // For testing only
                    Vector3d smaxCloth = new Vector3d(translation.x + 10, (float)diameter, translation.z + 10); // For testing only
                    StaticBounds = new Box3d(sminCloth, smaxCloth);
                    DrawLines.DrawBounds(camera, Color.green, StaticBounds, Matrix4x4d.Identity); 
                }
            }  
        }

        private void CreateSpheres() // TODO Spheres list should be one per plant
        {
            if (sphereMaterial == null) return;

            Spheres = new List<GameObject>();
            GameObject[] bodyPlantParent = new GameObject[numberOfPlants];
            GameObject[] bodyPlantMaster = new GameObject[numberOfPlants];

            for (int i = 0; i < numberOfPlants; i++)
            {
                // Define spheres for each particle
                int numParticles = plants[i].NumParticles;
                float diam = (float)plants[i].ParticleRadius * 2.0f;

                // Create parent with mesh Filter + Mesh Renderer for each 
                //GameObject bodyPlant = new GameObject(i.ToString());
                bodyPlantParent[i] = new GameObject("Parent_" + i.ToString());
                bodyPlantMaster[i] = new GameObject("Type_" + i.ToString());

                bodyPlantMaster[i].transform.position = Vector3.zero + translation; // Correct
                //bodyPlantMaster[i].transform.rotation = Quaternion.LookRotation(bodyPlantMaster[i].transform.forward); // Not correct

                bodyPlantParent[i].transform.parent = bodyPlantMaster[i].transform;
                bodyPlantParent[i].AddComponent<MeshFilter>();
                bodyPlantParent[i].AddComponent<MeshRenderer>();
                bodyPlantParent[i].GetComponent<MeshRenderer>().material = grass;

                for (int j = 0; j < numParticles; j++)
                {
                    Vector3 pos = MathConverter.ToVector3(plants[i].Positions[j]);

                    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);

                    if (makeSpheresVisible)
                        sphere.GetComponent<MeshRenderer>().enabled = true;
                    else
                        sphere.GetComponent<MeshRenderer>().enabled = false;

                    sphere.name = j.ToString();
                    sphere.transform.parent = bodyPlantParent[i].transform;
                    sphere.transform.position = pos;
                    sphere.transform.localScale = new Vector3(diam, diam, diam);
                    sphere.GetComponent<Collider>().enabled = true;
                    sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                    sphere.AddComponent<DetectionCollision>();

                    sphere.GetComponent<MeshRenderer>().material = sphereMaterialNoContact;

                    Spheres.Add(sphere);
                    SpheresPlant[i,j] = sphere;
                } 
            }
        }

        public void UpdateSpheres()
        {
            if (Spheres != null)
            {
                for (int i = 0; i < numberOfPlants; i++)
                {
                    for (int j = 0; j < SpheresPlant.GetLength(1); j++)
                    {
                        Vector3d pos = plants[i].Positions[j];
                        //Spheres[j].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                        SpheresPlant[i,j].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);

                        if (plants[i].IsContact[j] && !plants[i].IsBroken[j])
                        {
                            //Spheres[j].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                            SpheresPlant[i,j].GetComponent<MeshRenderer>().material = sphereMaterialContact;
                        }
                        else if (!plants[i].IsContact[j] && !plants[i].IsBroken[j])
                        {
                            //Spheres[j].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                            SpheresPlant[i, j].GetComponent<MeshRenderer>().material = sphereMaterialNoContact;
                        }
                        else if (plants[i].IsBroken[j])
                        {
                            //Spheres[j].GetComponent<MeshRenderer>().material = sphereMaterialBroken;
                            SpheresPlant[i, j].GetComponent<MeshRenderer>().material = sphereMaterialBroken;
                        }
                    } 
                }
            }
        }

        public void CollisionFromChildBody(Collision hit, float penetrationDistance, GameObject children, GameObject parent)
        {
            if (printInformation)
            {
                Debug.Log("[BasicPBDDemo] Collision Detected with object: " + hit.gameObject.name + " with particle/sphere: " + children.name);
                Debug.Log("CHANGING SPHERE: " + int.Parse(children.name) + " with the value " + Spheres[int.Parse(children.name)].GetComponent<DetectCollision>().Hit.GetContact(0).point);
            }

            // Instead of just parent.name being the number               
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].ExternalHit[int.Parse(children.name)] = hit.GetContact(0);
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].PenetrationDistance[int.Parse(children.name)] = penetrationDistance;
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].IsContact[int.Parse(children.name)] = true;

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

        public void ExitCollisionFromChildBody(GameObject children, GameObject parent)
        {

            // Instead of just parent.name being the number               
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].ExternalHit[int.Parse(children.name)] = new ContactPoint();
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].PenetrationDistance[int.Parse(children.name)] = 0f;
            plants[int.Parse(parent.name.Substring(parent.name.Length - 1))].IsContact[int.Parse(children.name)] = false;

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
