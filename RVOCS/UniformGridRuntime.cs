// #define OPEN_PROFILER

namespace RVO
{
    using Unity.Collections;

    public static class UniformGridRuntime
    {
        internal static UniformGrid tree;

        public static void processObstacles(Simulator simulator)
        {
            if (tree == null) tree = new UniformGrid();
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
            if (tree == null) tree = new UniformGrid();

            int agentCount = simulator.agents_.Length;

            // 构建 树
#if OPEN_PROFILER
            UnityEngine.Profiling.Profiler.BeginSample("[RVO] Grid Allocator.Temp");
#endif
            NativeArray<Agent> agentsReadOnly = new NativeArray<Agent>(agentCount, Allocator.Temp);
            NativeMultiHashMap<int, int> agentTreeReadOnly_ = new NativeMultiHashMap<int, int>(agentCount, Allocator.Temp);
            NativeList<Pair> obstacleNeighbors = new NativeList<Pair>(32, Allocator.Temp);
            NativeList<Pair> agentNeighbors = new NativeList<Pair>(128, Allocator.Temp);

#if OPEN_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
            UnityEngine.Profiling.Profiler.BeginSample("[RVO] buildAgentTree");
#endif

            for (int i = 0; i < agentsReadOnly.Length; ++i) agentsReadOnly[i] = simulator.agents_[i];
            tree.Bind(tree.CalculateCellSize(simulator.getAgentRadius(), simulator.getAgentNeighborDist()), ref agentTreeReadOnly_);
            tree.buildAgentTree(ref agentsReadOnly);

#if OPEN_PROFILER
            UnityEngine.Profiling.Profiler.EndSample();
#endif

            // 避障计算
            ROCA.Allocate();
            for (int agentNo = 0; agentNo < agentCount; ++agentNo)
            {
                Agent agent = simulator.agents_[agentNo];
                if (agent.static_) continue;

                // 查找 邻居
                obstacleNeighbors.Clear();
                agentNeighbors.Clear();
#if OPEN_PROFILER
                UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeObstacleNeighbors");
#endif
                float rangeSq = RVOMath.sqr(agent.timeHorizonObst_ * agent.maxSpeed_ + agent.radius_);
                tree.computeObstacleNeighbors(ref agent, in simulator.obstacles_, rangeSq, ref obstacleNeighbors);

#if OPEN_PROFILER
                UnityEngine.Profiling.Profiler.EndSample();
#endif

                if (agent.maxNeighbors_ > 0)
                {
#if OPEN_PROFILER
                    UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeAgentNeighbors");
#endif
                    rangeSq = RVOMath.sqr(agent.neighborDist_);
                    tree.computeAgentNeighbors(in agent, in agentsReadOnly, ref rangeSq, ref agentNeighbors);
#if OPEN_PROFILER
                    UnityEngine.Profiling.Profiler.EndSample();
#endif

                }

                // 计算 ORCA 新速度
#if OPEN_PROFILER
                UnityEngine.Profiling.Profiler.BeginSample("[RVO] ROCA.computeNewVelocity");
#endif
                ROCA.computeNewVelocity(ref agent, simulator.agents_, simulator.obstacles_, in obstacleNeighbors, in agentNeighbors);
#if OPEN_PROFILER
                UnityEngine.Profiling.Profiler.EndSample();
#endif
                simulator.agents_[agentNo] = agent;
            }
            ROCA.Deallocate();

            agentsReadOnly.Dispose();
            agentTreeReadOnly_.Dispose();
            obstacleNeighbors.Dispose();
            agentNeighbors.Dispose();

            for (int agentNo = 0; agentNo < agentCount; ++agentNo)
            {
                Agent agent = simulator.agents_[agentNo];
                simulator.agents_[agentNo] = agent.update(simulator);
            }

            tree.Clear();
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