/*
 * KdTree.cs
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
using System.Security;
using Unity.Collections;
using Unity.Mathematics;

namespace RVO
{
    /**
     * <summary>Defines k-D trees for agents and static obstacles in the
     * simulation.</summary>
     */
    internal class KdTree
    {
        /**
         * <summary>Defines a node of an agent k-D tree.</summary>
         */
        internal struct AgentTreeNode
        {
            internal int begin_;
            internal int end_;
            internal int left_;
            internal int right_;
            internal float maxX_;
            internal float maxY_;
            internal float minX_;
            internal float minY_;
        }

        /**
         * <summary>Defines a pair of scalar values.</summary>
         */
        private struct FloatPair
        {
            private readonly float a_;
            private readonly float b_;

            /**
             * <summary>Constructs and initializes a pair of scalar
             * values.</summary>
             *
             * <param name="a">The first scalar value.</param>
             * <param name="b">The second scalar value.</param>
             */
            internal FloatPair(float a, float b)
            {
                a_ = a;
                b_ = b;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is less
             * than the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is less than the
             * second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator <(FloatPair pair1, FloatPair pair2)
            {
                return pair1.a_ < pair2.a_ || !(pair2.a_ < pair1.a_) && pair1.b_ < pair2.b_;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is less
             * than or equal to the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is less than or
             * equal to the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator <=(FloatPair pair1, FloatPair pair2)
            {
                return (pair1.a_ == pair2.a_ && pair1.b_ == pair2.b_) || pair1 < pair2;
            }

            /**
             * <summary>Returns true if the first pair of scalar values is
             * greater than the second pair of scalar values.</summary>
             *
             * <returns>True if the first pair of scalar values is greater than
             * the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator >(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 <= pair2);
            }

            /**
             * <summary>Returns true if the first pair of scalar values is
             * greater than or equal to the second pair of scalar values.
             * </summary>
             *
             * <returns>True if the first pair of scalar values is greater than
             * or equal to the second pair of scalar values.</returns>
             *
             * <param name="pair1">The first pair of scalar values.</param>
             * <param name="pair2">The second pair of scalar values.</param>
             */
            public static bool operator >=(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 < pair2);
            }
        }

        /**
         * <summary>Defines a node of an obstacle k-D tree.</summary>
         */
        private struct ObstacleTreeNode
        {
            public int idx;
            internal int obstacleNo_;
            internal int left_;
            internal int right_;
        };

        /**
         * <summary>The maximum size of an agent k-D tree leaf.</summary>
         */
        private const int MAX_LEAF_SIZE = 10;

        private NativeArray<Agent> agents_;
        // private NativeArray<AgentTreeNode> agentTree_;

        #region Obstacles Tree
        private int obstacleTreeNodeIdx_;
        private NativeArray<ObstacleTreeNode> obstacleTreeNodes_;
        #endregion

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal void buildAgentTree(ref NativeArray<AgentTreeNode> agentTree, ref NativeArray<Agent> agents)
        {
            Simulator simulator = Simulator.Instance;
            // if (agents_ == null || agents_.Length != simulator.agents_.Count)
            // {
            //     agents_ = new NativeArray<Agent>(simulator.agents_.Count, Allocator.Persistent);

            // }

            for (int i = 0; i < agents.Length; ++i)
                agents[i] = simulator.agents_[i];

            if (agents.Length != 0)
            {
                buildAgentTreeRecursive(ref agentTree, ref agents, 0, agents_.Length, 0);
            }
        }

        /**
         * <summary>Builds an obstacle k-D tree.</summary>
         */
        internal void buildObstacleTree(Simulator simulator)
        {
            NativeArray<int> obstacleIds = new NativeArray<int>(simulator.obstacles_.Length, Allocator.Temp);
            for (int i = 0; i < simulator.obstacles_.Length; ++i) obstacleIds[i] = simulator.obstacles_[i].id_;

            obstacleTreeNodes_ = new NativeArray<ObstacleTreeNode>(simulator.obstacles_.Length, Allocator.Persistent);
            obstacleTreeNodeIdx_ = buildObstacleTreeRecursive(simulator, in obstacleIds, ref simulator.obstacles_, ref obstacleTreeNodes_);
            obstacleIds.Dispose();
        }

        /**
         * <summary>Computes the agent neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void computeAgentNeighbors(in Agent agent, in NativeArray<AgentTreeNode> agentTree, in NativeArray<Agent> agents, ref float rangeSq, ref NativeList<KeyValuePair<float, int>> agentNeighbors)
        {
            queryAgentTreeRecursive(in agent, in agentTree, in agents, ref rangeSq, 0, ref agentNeighbors);
        }

        /**
         * <summary>Computes the obstacle neighbors of the specified agent.
         * </summary>
         *
         * <param name="agentNo">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void computeObstacleNeighbors(ref Agent agent, float rangeSq, in NativeList<Obstacle> obstacles, ref NativeList<KeyValuePair<float, int>> obstacleNeighbors)
        {
            queryObstacleTreeRecursive(ref agent, rangeSq, obstacleTreeNodeIdx_, obstacles, ref obstacleNeighbors);
        }

        /**
         * <summary>Queries the visibility between two points within a specified
         * radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         */
        internal bool queryVisibility(float2 q1, float2 q2, float radius, NativeList<Obstacle> obstacles)
        {
            return queryVisibilityRecursive(q1, q2, radius, obstacleTreeNodeIdx_, obstacles);
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="nodeIndex">The current agent k-D tree node index.</param>
         */
        internal void buildAgentTreeRecursive(ref NativeArray<AgentTreeNode> agentTree, ref NativeArray<Agent> agents, int begin, int end, int nodeIndex)
        {
            AgentTreeNode node = agentTree[nodeIndex];
            node.begin_ = begin;
            node.end_ = end;
            node.minX_ = node.maxX_ = agents[begin].position_.x;
            node.minY_ = node.maxY_ = agents[begin].position_.y;

            for (int i = begin + 1; i < end; ++i)
            {
                node.maxX_ = Math.Max(node.maxX_, agents[i].position_.x);
                node.minX_ = Math.Min(node.minX_, agents[i].position_.x);
                node.maxY_ = Math.Max(node.maxY_, agents[i].position_.y);
                node.minY_ = Math.Min(node.minY_, agents[i].position_.y);
            }
            agentTree[nodeIndex] = node;

            if (end - begin > MAX_LEAF_SIZE)
            {
                /* No leaf node. */
                bool isVertical = node.maxX_ - node.minX_ > node.maxY_ - node.minY_;
                float splitValue = 0.5f * (isVertical ? node.maxX_ + node.minX_ : node.maxY_ + node.minY_);

                int left = begin;
                int right = end;

                while (left < right)
                {
                    while (left < right && (isVertical ? agents[left].position_.x : agents[left].position_.y) < splitValue)
                    {
                        ++left;
                    }

                    while (right > left && (isVertical ? agents[right - 1].position_.x : agents[right - 1].position_.y) >= splitValue)
                    {
                        --right;
                    }

                    if (left < right)
                    {
                        Agent tempAgent = agents[left];
                        agents[left] = agents[right - 1];
                        agents[right - 1] = tempAgent;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                }

                node.left_ = nodeIndex + 1;
                node.right_ = nodeIndex + 2 * leftSize;

                agentTree[nodeIndex] = node;
                buildAgentTreeRecursive(ref agentTree, ref agents, begin, left, node.left_);
                buildAgentTreeRecursive(ref agentTree, ref agents, left, end, node.right_);
            }
        }

        /**
         * <summary>Recursive method for building an obstacle k-D tree.
         * </summary>
         *
         * <returns>An obstacle k-D tree node.</returns>
         *
         * <param name="obstacles">A list of obstacles.</param>
         */
        private int buildObstacleTreeRecursive(Simulator simulator, in NativeArray<int> obstacleIds, ref NativeList<Obstacle> obstacles, ref NativeArray<ObstacleTreeNode> obstacleTreeNodes)
        {
            if (obstacleIds.Length == 0 || !obstacleIds.IsCreated)
            {
                return -1;
            }

            ObstacleTreeNode node = new ObstacleTreeNode();

            int optimalSplit = 0;
            int minLeft = obstacleIds.Length;
            int minRight = obstacleIds.Length;

            for (int i = 0; i < obstacleIds.Length; ++i)
            {
                int leftSize = 0;
                int rightSize = 0;

                int obstacleI1Id = obstacleIds[i];
                if (obstacleI1Id < 0) continue;

                Obstacle obstacleI1 = obstacles[obstacleI1Id];
                Obstacle obstacleI2 = obstacles[obstacleI1.next_];

                /* Compute optimal split node. */
                for (int j = 0; j < obstacleIds.Length; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    int obstacleJ1Id = obstacleIds[j];
                    if (obstacleJ1Id < 0) continue;

                    Obstacle obstacleJ1 = obstacles[obstacleJ1Id];
                    Obstacle obstacleJ2 = obstacles[obstacleJ1.next_];

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        ++rightSize;
                    }
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) >= new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                    {
                        break;
                    }
                }

                if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) < new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                NativeArray<int> leftObstacles = new NativeArray<int>(minLeft, Allocator.Temp);
                for (int n = 0; n < minLeft; ++n)
                    leftObstacles[n] = -1;

                NativeArray<int> rightObstacles = new NativeArray<int>(minRight, Allocator.Temp);
                for (int n = 0; n < minRight; ++n)
                    rightObstacles[n] = -1;

                int leftCounter = 0;
                int rightCounter = 0;
                int i = optimalSplit;

                Obstacle obstacleI1 = obstacles[obstacleIds[i]];
                Obstacle obstacleI2 = obstacles[obstacleI1.next_];

                for (int j = 0; j < obstacleIds.Length; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    int obstacleJ1Id = obstacleIds[j];
                    if (obstacleJ1Id < 0) continue;

                    Obstacle obstacleJ1 = obstacles[obstacleJ1Id];
                    Obstacle obstacleJ2 = obstacles[obstacleJ1.next_];

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        leftObstacles[leftCounter++] = obstacleIds[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        rightObstacles[rightCounter++] = obstacleIds[j];
                    }
                    else
                    {
                        /* Split obstacle j. */
                        float t = RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_);

                        float2 splitPoint = obstacleJ1.point_ + t * (obstacleJ2.point_ - obstacleJ1.point_);

                        Obstacle newObstacle = new Obstacle();
                        newObstacle.point_ = splitPoint;
                        newObstacle.previous_ = obstacleJ1.id_;
                        newObstacle.next_ = obstacleJ2.id_;
                        newObstacle.convex_ = true;
                        newObstacle.direction_ = obstacleJ1.direction_;

                        newObstacle.id_ = obstacles.Length;

                        obstacles.Add(newObstacle);

                        obstacleJ1.next_ = newObstacle.id_; // @mark
                        obstacleJ2.previous_ = newObstacle.id_; // @mark
                        obstacles[obstacleJ1.id_] = obstacleJ1;
                        obstacles[obstacleJ2.id_] = obstacleJ2;

                        if (j1LeftOfI > 0.0f)
                        {
                            leftObstacles[leftCounter++] = obstacleJ1.id_;
                            rightObstacles[rightCounter++] = newObstacle.id_;
                        }
                        else
                        {
                            rightObstacles[rightCounter++] = obstacleJ1.id_;
                            leftObstacles[leftCounter++] = newObstacle.id_;
                        }
                    }
                }
                node.obstacleNo_ = obstacleI1.id_;
                node.left_ = buildObstacleTreeRecursive(simulator, in leftObstacles, ref obstacles, ref obstacleTreeNodes);
                node.right_ = buildObstacleTreeRecursive(simulator, in rightObstacles, ref obstacles, ref obstacleTreeNodes);

                leftObstacles.Dispose();
                rightObstacles.Dispose();

                obstacleTreeNodes_[node.obstacleNo_] = node;
                return node.obstacleNo_;
            }
        }

        /**
         * <summary>Recursive method for computing the agent neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void queryAgentTreeRecursive(in Agent agent, in NativeArray<AgentTreeNode> agentTree, in NativeArray<Agent> agents, ref float rangeSq, int node, ref NativeList<KeyValuePair<float, int>> agentNeighbors)
        {
            if (agentTree[node].end_ - agentTree[node].begin_ <= MAX_LEAF_SIZE)
            {
                for (int i = agentTree[node].begin_; i < agentTree[node].end_; ++i)
                {
                    insertAgentNeighbor(in agent, agents[i].id_, agents[i].position_, ref rangeSq, ref agentNeighbors);
                }
            }
            else
            {
                float distSqLeft = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left_].minX_ - agent.position_.x)) +
                                        RVOMath.sqr(Math.Max(0.0f, agent.position_.x - agentTree[agentTree[node].left_].maxX_)) +
                                            RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].left_].minY_ - agent.position_.y)) +
                                                RVOMath.sqr(Math.Max(0.0f, agent.position_.y - agentTree[agentTree[node].left_].maxY_));

                float distSqRight = RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right_].minX_ - agent.position_.x)) +
                                        RVOMath.sqr(Math.Max(0.0f, agent.position_.x - agentTree[agentTree[node].right_].maxX_)) +
                                            RVOMath.sqr(Math.Max(0.0f, agentTree[agentTree[node].right_].minY_ - agent.position_.y)) +
                                                RVOMath.sqr(Math.Max(0.0f, agent.position_.y - agentTree[agentTree[node].right_].maxY_));

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        queryAgentTreeRecursive(in agent, in agentTree, in agents, ref rangeSq, agentTree[node].left_, ref agentNeighbors);

                        if (distSqRight < rangeSq)
                        {
                            queryAgentTreeRecursive(in agent, in agentTree, in agents, ref rangeSq, agentTree[node].right_, ref agentNeighbors);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        queryAgentTreeRecursive(in agent, in agentTree, in agents, ref rangeSq, agentTree[node].right_, ref agentNeighbors);

                        if (distSqLeft < rangeSq)
                        {
                            queryAgentTreeRecursive(in agent, in agentTree, in agents, ref rangeSq, agentTree[node].left_, ref agentNeighbors);
                        }
                    }
                }

            }
        }

        /**
         * <summary>Recursive method for computing the obstacle neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private void queryObstacleTreeRecursive(ref Agent agent, float rangeSq, int nodeIndex, in NativeList<Obstacle> obstacles, ref NativeList<KeyValuePair<float, int>> obstacleNeighbors)
        {
            if (nodeIndex < 0) return;
            ObstacleTreeNode node = obstacleTreeNodes_[nodeIndex];

            Obstacle obstacle1 = obstacles[node.obstacleNo_];
            Obstacle obstacle2 = obstacles[obstacle1.next_];

            float agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

            queryObstacleTreeRecursive(ref agent, rangeSq, agentLeftOfLine >= 0.0f ? node.left_ : node.right_, obstacles, ref obstacleNeighbors);

            float distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point_ - obstacle1.point_);

            if (distSqLine < rangeSq)
            {
                if (agentLeftOfLine < 0.0f)
                {
                    /*
                     * Try obstacle at this node only if agent is on right side of
                     * obstacle (and can see obstacle).
                     */
                    insertObstacleNeighbor(agent.position_, node.obstacleNo_, obstacles, rangeSq, ref obstacleNeighbors);
                }

                /* Try other side of line. */
                queryObstacleTreeRecursive(ref agent, rangeSq, agentLeftOfLine >= 0.0f ? node.right_ : node.left_, obstacles, ref obstacleNeighbors);
            }
        }

        /**
         * <summary>Recursive method for querying the visibility between two
         * points within a specified radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private bool queryVisibilityRecursive(float2 q1, float2 q2, float radius, int nodeIndex, in NativeList<Obstacle> obstacles)
        {
            if (nodeIndex < 0) return true;

            ObstacleTreeNode node = obstacleTreeNodes_[nodeIndex];
            Obstacle obstacle1 = obstacles[node.obstacleNo_];
            Obstacle obstacle2 = obstacles[obstacle1.next_];

            float q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
            float q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
            float invLengthI = 1.0f / RVOMath.absSq(obstacle2.point_ - obstacle1.point_);

            if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.left_, in obstacles) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.right_, in obstacles));
            }

            if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.right_, in obstacles) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.left_, in obstacles));
            }

            if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
            {
                /* One can see through obstacle from left to right. */
                return queryVisibilityRecursive(q1, q2, radius, node.left_, in obstacles) && queryVisibilityRecursive(q1, q2, radius, node.right_, in obstacles);
            }

            float point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
            float point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
            float invLengthQ = 1.0f / RVOMath.absSq(q2 - q1);

            return point1LeftOfQ * point2LeftOfQ >= 0.0f && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node.left_, in obstacles) && queryVisibilityRecursive(q1, q2, radius, node.right_, in obstacles);
        }

        internal void Clear()
        {
            // if (agents_ != null) Array.Clear(agents_, 0, agents_.Length);
            // if (agentTree_ != null) Array.Clear(agentTree_, 0, agentTree_.Length);

            // if (agentTree_.IsCreated) agentTree_.Dispose();
            if (agents_.IsCreated) agents_.Dispose();
            if (obstacleTreeNodes_.IsCreated) obstacleTreeNodes_.Dispose();

            // this.agentIds.Resize(0);
            // this.agentTree.Resize(0);
            // this.obstacleTreeNodes.Resize(0);
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal static void insertAgentNeighbor(in Agent agent, int agentNo, float2 position, ref float rangeSq, ref NativeList<KeyValuePair<float, int>> agentNeighbors)
        {
            if (agent.id_ != agentNo)
            {
                float distSq = RVOMath.absSq(agent.position_ - position);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors.Length < agent.maxNeighbors_)
                    {
                        agentNeighbors.Add(new KeyValuePair<float, int>(distSq, agentNo));
                    }

                    int i = agentNeighbors.Length - 1;

                    while (i != 0 && distSq < agentNeighbors[i - 1].Key)
                    {
                        agentNeighbors[i] = agentNeighbors[i - 1];
                        --i;
                    }

                    agentNeighbors[i] = new KeyValuePair<float, int>(distSq, agentNo);

                    if (agentNeighbors.Length == agent.maxNeighbors_)
                    {
                        rangeSq = agentNeighbors[agentNeighbors.Length - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal static void insertObstacleNeighbor(float2 position, int obstacleNo, in NativeList<Obstacle> obstacles, float rangeSq, ref NativeList<KeyValuePair<float, int>> obstacleNeighbors)
        {
            Obstacle obstacle = obstacles[obstacleNo];
            Obstacle nextObstacle = obstacles[obstacle.next_];

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new KeyValuePair<float, int>(distSq, obstacleNo));

                int i = obstacleNeighbors.Length - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].Key)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }
                obstacleNeighbors[i] = new KeyValuePair<float, int>(distSq, obstacleNo);
            }
        }
    }
}
