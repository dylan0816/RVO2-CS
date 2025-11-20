
namespace RVO
{
    using System.Collections.Generic;
    using Unity.Collections;
    using Unity.Mathematics;
    using UnityEngine;

    public static class GridRuntime
    {
        internal static GridTree tree;

        public static void processObstacles(Simulator simulator)
        {
            if (tree == null) tree = new GridTree();
            tree.buildObstacleTree(ref simulator.obstacles_);
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        internal static float doStep(Simulator simulator)
        {
            if (tree == null) tree = new GridTree();

            int agentCount = simulator.agents_.Length;

            // 构建 树

            UnityEngine.Profiling.Profiler.BeginSample("[RVO] Allocator.Temp");
            NativeArray<Agent> agentsReadOnly = new NativeArray<Agent>(agentCount, Allocator.Temp);
            NativeHashMap<int, FixedList64Bytes<int>> agentTreeReadOnly_ = new NativeHashMap<int, FixedList64Bytes<int>>(agentCount, Allocator.Temp);
            NativeList<Pair> obstacleNeighbors = new NativeList<Pair>(16, Allocator.Temp);
            NativeList<Pair> agentNeighbors = new NativeList<Pair>(16, Allocator.Temp);
            UnityEngine.Profiling.Profiler.EndSample();

            UnityEngine.Profiling.Profiler.BeginSample("[RVO] buildAgentTree");
            for (int i = 0; i < agentsReadOnly.Length; ++i) agentsReadOnly[i] = simulator.agents_[i];
            tree.Bind(math.ceil(2 * 1.5f), ref agentTreeReadOnly_);
            tree.buildAgentTree(ref agentsReadOnly);
            UnityEngine.Profiling.Profiler.EndSample();

            // 避障计算
            for (int agentNo = 0; agentNo < agentCount; ++agentNo)
            {
                Agent agent = simulator.agents_[agentNo];

                // 查找 邻居
                obstacleNeighbors.Clear();
                agentNeighbors.Clear();

                UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeObstacleNeighbors");
                float rangeSq = RVOMath.sqr(agent.timeHorizonObst_ * agent.maxSpeed_ + agent.radius_);
                tree.computeObstacleNeighbors(ref agent, in simulator.obstacles_, rangeSq, ref obstacleNeighbors);
                UnityEngine.Profiling.Profiler.EndSample();

                if (agent.maxNeighbors_ > 0)
                {
                    UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeAgentNeighbors");
                    rangeSq = RVOMath.sqr(agent.neighborDist_);
                    tree.computeAgentNeighbors(in agent, in agentsReadOnly, ref rangeSq, ref agentNeighbors);
                    UnityEngine.Profiling.Profiler.EndSample();
                }

                // 计算 ORCA 新速度
                UnityEngine.Profiling.Profiler.BeginSample("[RVO] ROCA.computeNewVelocity");
                ROCA.computeNewVelocity(ref agent, simulator.agents_, simulator.obstacles_, in obstacleNeighbors, in agentNeighbors);
                UnityEngine.Profiling.Profiler.EndSample();
                simulator.agents_[agentNo] = agent;
            }

            agentsReadOnly.Dispose();
            agentTreeReadOnly_.Dispose();
            obstacleNeighbors.Dispose();
            agentNeighbors.Dispose();

            for (int agentNo = 0; agentNo < agentCount; ++agentNo)
            {
                Agent agent = simulator.agents_[agentNo];
                simulator.agents_[agentNo] = agent.update(simulator);
            }

            return simulator.doStep();
        }

        public static void Clear()
        {
            if (tree != null)
            {
                tree.Dispose();
            }

        }
    }
}