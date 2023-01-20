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
    public class Plant
    {
        #region Instance Fields
        
        // Position and Orientation
        public Vector3 translation;
        public Vector3 rotation;

        // Plant properties
        public Vector2 plantSize;
        public double mass = 1.0;
        public double diameter = 0.5;
        public double spaceBetween = 0;

        // Stiffness
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;

        #endregion

        #region Instance Properties

        private ClothBody3d Body { get; set; }
        private Box3d StaticBounds { get; set; }

        #endregion

        #region Read-only & Static Fields

        #endregion

        #region Methods

        public Plant(Vector2 plantSize, double mass, double diameter, double spaceBetween, double stretchStiffness, double bendStiffness)
        {
            this.plantSize = plantSize;
            this.mass = mass;
            this.diameter = diameter;
            this.spaceBetween = spaceBetween;
            this.stretchStiffness = stretchStiffness;
            this.bendStiffness = bendStiffness;
        }
        
        public ClothBody3d CreatePlant(Vector3 translation, Vector3 rotation)
        {
            // Always translate upwards s.t. we leave a virtual row underground
            //double height = (plantSize.y / 2) - diameter / 2; 
            //double height = (plantSize.y / 2) - spaceBetween + diameter / 2;
            double height = (plantSize.y / 2) - ((diameter/2) + (spaceBetween - diameter));

            // Global pos and rotation of mesh
            Matrix4x4d TCloth = Matrix4x4d.Translate(new Vector3d(translation.x, height, translation.z)); // should be height
            Matrix4x4d RCloth = Matrix4x4d.Rotate(new Vector3d(rotation.x, rotation.y, rotation.z));
            Matrix4x4d TRCloth = TCloth * RCloth;

            // Create cloth body
            double width = plantSize.x;
            double depth = plantSize.y;

            // Create mesh given space in between and size
            TrianglesFromGrid sourceMesh = new TrianglesFromGrid(spaceBetween / 2, width, depth);

            // Build plant body given the specs
            Body = new ClothBody3d(sourceMesh, diameter / 2, mass, stretchStiffness, bendStiffness, TRCloth);
         
            // TEST TODO
            Body.Dampning = 1.0;

            // Volume fixed
            Vector3d sminCloth = new Vector3d(translation.x - (plantSize.x / 2) - (diameter / 2), -((float)spaceBetween - (float)diameter) - (float)(diameter / 1), translation.z - (diameter / 2));
            Vector3d smaxCloth = new Vector3d(translation.x + (plantSize.x / 2) + (diameter / 2), (float)diameter, translation.z + (diameter / 2));
            StaticBounds = new Box3d(sminCloth, smaxCloth);
            Body.MarkAsStatic(StaticBounds);

            return Body;
        }

        #endregion
    } 
}
