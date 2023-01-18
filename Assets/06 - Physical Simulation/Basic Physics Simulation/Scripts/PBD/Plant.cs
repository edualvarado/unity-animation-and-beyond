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

        [Header("Plant - Position and Orientation")]
        public Vector3 translation;
        public Vector3 rotation;

        [Header("Plant - Properties")]
        public Vector2 plantSize;
        public double mass = 1.0;
        public double diameter = 0.5;
        [Range(0f, 1f)] public float scaleRadius;

        [Header("Body - Stiffness")]
        public double stretchStiffness = 0.25;
        public double bendStiffness = 0.5;

        #endregion

        #region Instance Properties

        private ClothBody3d Body { get; set; }
        private Box3d StaticBounds2 { get; set; }

        #endregion

        #region Read-only & Static Fields

        #endregion

        #region Methods

        public Plant(Vector2 plantSize, double mass, double diameter, float scaleRadius, double stretchStiffness, double bendStiffness)
        {
            this.plantSize = plantSize;
            this.mass = mass;
            this.diameter = diameter;
            this.scaleRadius = scaleRadius;
            this.stretchStiffness = stretchStiffness;
            this.bendStiffness = bendStiffness;
        }

        public ClothBody3d CreatePlant(Vector3 translation, Vector3 rotation)
        {
            // Always translate upwards s.t. we leave a virtual row underground
            double height = ((plantSize.y * diameter) / 2) - diameter;

            // Global pos and rotation of mesh
            Matrix4x4d TCloth = Matrix4x4d.Translate(new Vector3d(translation.x, height, translation.z)); // should be height
            Matrix4x4d RCloth = Matrix4x4d.Rotate(new Vector3d(rotation.x, rotation.y, rotation.z));
            Matrix4x4d TRCloth = TCloth * RCloth;

            // Create cloth body
            double width = (plantSize.x - 1) * diameter;
            double depth = (plantSize.y - 1) * diameter;
            
            TrianglesFromGrid source2 = new TrianglesFromGrid(diameter / 2, width, depth);
            Body = new ClothBody3d(source2, diameter / 2, mass, stretchStiffness, bendStiffness, TRCloth);
            
            Body.Dampning = 1.0;

            //Vector3d sminCloth = new Vector3d(fxMinCloth.x, fxMinCloth.y, fxMinCloth.z);
            //Vector3d smaxCloth = new Vector3d(fxMaxCloth.x, fxMaxCloth.y, fxMaxCloth.z);
            Vector3d sminCloth = new Vector3d(translation.x - (plantSize.x * diameter) / 2, -(float)diameter, translation.z - (diameter / 2));
            Vector3d smaxCloth = new Vector3d(translation.x + (plantSize.x * diameter) / 2, (float)diameter, translation.z + (diameter / 2));
            StaticBounds2 = new Box3d(sminCloth, smaxCloth);
            Body.MarkAsStatic(StaticBounds2);

            return Body;
        }

        #endregion
    } 
}
