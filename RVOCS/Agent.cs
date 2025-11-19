/*
 * Agent.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace RVO
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal struct Agent : IEquatable<Agent>
    {
        internal float2 position_;
        internal float2 prefVelocity_;
        internal float2 velocity_;
        internal int id_;
        internal int maxNeighbors_;
        internal float maxSpeed_;
        internal float neighborDist_;
        internal float radius_;
        internal float timeHorizon_;
        internal float timeHorizonObst_;

        internal float2 newVelocity_;

        public override bool Equals(object obj) => base.Equals(obj);
        public override int GetHashCode() => base.GetHashCode();
        public bool Equals(Agent other) => id_ == other.id_;
        public static bool operator ==(Agent a, Agent b) => a.id_ == b.id_;
        public static bool operator !=(Agent a, Agent b) => a.id_ != b.id_;

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            velocity_ = newVelocity_;
            position_ += velocity_ * Simulator.Instance.timeStep_;
        }

    }
}
