
using System;
using System.Collections.Generic;
using System.Security;
using Unity.Collections;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine.UIElements;

namespace RVO
{
    /**
     * <summary>Kd-tree for agents and obstacles.</summary>
     */
    public class GridTree : IRVOQuery, IDisposable
    {
        const int MAX_LEAF_SIZE = 10;
        public GridTree() { }
        internal float cellsize_;

        NativeArray<int> dirs;
        NativeHashMap<int, FixedList64Bytes<int>> gridCells;
        #region Obstacles Tree
        private int obstacleTreeNodeIdx_;
        private NativeArray<ObstacleTreeNode> obstacleTreeNodes_;
        #endregion


        public static int getCellHashCode(ref int x, ref int y) => x * 73856093 ^ y * 19349663;

        public float CalculateCellSize(float neighborDist)
        {
            return neighborDist * 0.6f;
        }
        public void Bind(float cellsize, ref NativeHashMap<int, FixedList64Bytes<int>> gridCells)
        {
            if (!dirs.IsCreated)
            {
                dirs = new NativeArray<int>(new int[] {
                    -1, 0, 1, 0,
                    0, -1, 0, 1,
                    -1, -1, 1, 1,
                    -1, 1, 1, -1
                    }, Allocator.Persistent);
            }

            cellsize_ = cellsize;
            this.gridCells = gridCells;
        }

        public void buildAgentTree(ref NativeArray<Agent> agents)
        {
            gridCells.Clear();
            for (int i = 0; i < agents.Length; i++)
            {
                Agent agent = agents[i];
                int cellX = (int)math.floor(agent.position_.x / cellsize_);
                int cellY = (int)math.floor(agent.position_.y / cellsize_);
                int cellId = getCellHashCode(ref cellX, ref cellY);

                if (!gridCells.ContainsKey(cellId))
                {
                    gridCells[cellId] = new FixedList64Bytes<int>();
                }

                if (gridCells[cellId].Length >= MAX_LEAF_SIZE)
                {
                    agent.valid_ = false;
                    continue;
                }
                else agent.valid_ = true;

                FixedList64Bytes<int> list = gridCells[cellId];
                list.Add(agent.id_);
                gridCells[cellId] = list;
            }
        }

        public void buildObstacleTree(ref NativeList<Obstacle> obstacles)
        {
            NativeArray<int> obstacleIds = new NativeArray<int>(obstacles.Length, Allocator.Temp);
            for (int i = 0; i < obstacles.Length; ++i) obstacleIds[i] = obstacles[i].id_;

            obstacleTreeNodes_ = new NativeArray<ObstacleTreeNode>(obstacles.Length, Allocator.Persistent);
            obstacleTreeNodeIdx_ = buildObstacleTreeRecursive(in obstacleIds, ref obstacles, ref obstacleTreeNodes_);
            obstacleIds.Dispose();
        }


        public void computeAgentNeighbors(in Agent agent, in NativeArray<Agent> agents, ref float rangeSq, ref NativeList<Pair> agentNeighbors)
        {
            agentNeighbors.Clear();
            int maxNeighbors = agent.maxNeighbors_;
            float neighborDist_ = agent.neighborDist_ * agent.neighborDist_;

            int cellX = (int)math.floor(agent.position_.x / cellsize_);
            int cellY = (int)math.floor(agent.position_.y / cellsize_);
            int cellId = getCellHashCode(ref cellX, ref cellY);
            FixedList64Bytes<int> cells = gridCells[cellId];


            for (int i = math.min(maxNeighbors, cells.Length - 1); i >= 0; i--)
            {
                if (cells[i] == agent.id_) continue;
                int agentId = cells[i];
                float distSq = math.distancesq(agent.position_, agents[agentId].position_);
                if (distSq < neighborDist_) agentNeighbors.Add(new Pair(distSq, cells[i]));
            }


            int index = 0;
            int round = agent.maxNeighbors_;

            while (round > 0 && agentNeighbors.Length < maxNeighbors)
            {
                bool hasMore = false;
                for (int i = dirs.Length - 2; i >= 0 && agentNeighbors.Length < maxNeighbors; i--)
                {
                    int neighborCellX = cellX + dirs[i];
                    int neighborCellY = cellY + dirs[i + 1];
                    int neighborCellId = getCellHashCode(ref neighborCellX, ref neighborCellY);

                    if (gridCells.ContainsKey(neighborCellId) && gridCells[neighborCellId].Length > index)
                    {
                        hasMore = true;
                        int agentId = gridCells[neighborCellId][index];
                        float distSq = math.distancesq(agent.position_, agents[agentId].position_);
                        if (distSq < neighborDist_) agentNeighbors.Add(new Pair(distSq, agentId));
                    }
                }
                round--;
                index++;

                if (!hasMore) break;
            }

            agentNeighbors.Sort();
        }

        public void computeObstacleNeighbors(ref Agent agent, in NativeList<Obstacle> obstacles, float rangeSq, ref NativeList<Pair> obstacleNeighbors)
        {
            queryObstacleTreeRecursive(ref agent, rangeSq, obstacleTreeNodeIdx_, obstacles, ref obstacleNeighbors);
        }

        /**
         * <summary>Recursive method for building an obstacle k-D tree.
         * </summary>
         *
         * <returns>An obstacle k-D tree node.</returns>
         *
         * <param name="obstacles">A list of obstacles.</param>
         */
        private int buildObstacleTreeRecursive(in NativeArray<int> obstacleIds, ref NativeList<Obstacle> obstacles, ref NativeArray<ObstacleTreeNode> obstacleTreeNodes)
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
                node.left_ = buildObstacleTreeRecursive(in leftObstacles, ref obstacles, ref obstacleTreeNodes);
                node.right_ = buildObstacleTreeRecursive(in rightObstacles, ref obstacles, ref obstacleTreeNodes);

                leftObstacles.Dispose();
                rightObstacles.Dispose();

                obstacleTreeNodes_[node.obstacleNo_] = node;
                return node.obstacleNo_;
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
        private void queryObstacleTreeRecursive(ref Agent agent, float rangeSq, int nodeIndex, in NativeList<Obstacle> obstacles, ref NativeList<Pair> obstacleNeighbors)
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
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal static void insertObstacleNeighbor(float2 position, int obstacleNo, in NativeList<Obstacle> obstacles, float rangeSq, ref NativeList<Pair> obstacleNeighbors)
        {
            Obstacle obstacle = obstacles[obstacleNo];
            Obstacle nextObstacle = obstacles[obstacle.next_];

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new Pair(distSq, obstacleNo));

                int i = obstacleNeighbors.Length - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].distSq)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }
                obstacleNeighbors[i] = new Pair(distSq, obstacleNo);
            }
        }

        private bool disposedValue = false;

        public void Clear()
        {
        }
        public void Dispose()
        {
            this.Dispose(true);
        }
        private void Dispose(bool disposing)
        {
            if (!this.disposedValue)
            {
                if (disposing)
                {
                    // Managed state
                    this.Clear();
                }
                dirs.Dispose();
                obstacleTreeNodes_.Dispose();
                // Unmanaged resources
                this.disposedValue = true;
            }
        }


    }
}