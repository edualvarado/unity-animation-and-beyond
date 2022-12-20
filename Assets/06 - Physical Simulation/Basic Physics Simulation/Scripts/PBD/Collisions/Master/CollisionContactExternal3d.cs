using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{

    internal abstract class CollisionContactExternal3d
    {

        internal abstract void ResolveContactExternal(double di);

    }

}