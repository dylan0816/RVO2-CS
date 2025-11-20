
namespace RVO
{
    using Unity.Collections;

    public static class KDRunntime
    {
        internal static KdTree kdTree_;

        public static void processObstacles(Simulator simulator)
        {
            if (kdTree_ == null) kdTree_ = new KdTree();
            kdTree_.buildObstacleTree(ref simulator.obstacles_);
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        internal static float doStep(Simulator simulator)
        {
            if (kdTree_ == null) kdTree_ = new KdTree();

            int agentCount = simulator.agents_.Length;

            // 构建 树
            UnityEngine.Profiling.Profiler.BeginSample("[RVO] Allocator.Temp");
            NativeArray<Agent> agentsReadOnly = new NativeArray<Agent>(agentCount, Allocator.Temp);
            NativeArray<KdTree.AgentTreeNode> agentTreeReadOnly_ = new NativeArray<KdTree.AgentTreeNode>(2 * agentCount, Allocator.Temp);
            NativeList<Pair> obstacleNeighbors = new NativeList<Pair>(16, Allocator.Temp);
            NativeList<Pair> agentNeighbors = new NativeList<Pair>(16, Allocator.Temp);
            UnityEngine.Profiling.Profiler.EndSample();

            UnityEngine.Profiling.Profiler.BeginSample("[RVO] buildAgentTree");
            for (int i = 0; i < agentsReadOnly.Length; ++i) agentsReadOnly[i] = simulator.agents_[i];
            kdTree_.Bind(ref agentTreeReadOnly_);
            kdTree_.buildAgentTree(ref agentsReadOnly);
            UnityEngine.Profiling.Profiler.EndSample();

            // 避障计算
            ROCA.Allocate();
            for (int agentNo = 0; agentNo < agentCount; ++agentNo)
            {
                Agent agent = simulator.agents_[agentNo];

                // 查找 邻居
                obstacleNeighbors.Clear();
                agentNeighbors.Clear();

                UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeObstacleNeighbors");
                float rangeSq = RVOMath.sqr(agent.timeHorizonObst_ * agent.maxSpeed_ + agent.radius_);
                kdTree_.computeObstacleNeighbors(ref agent, in simulator.obstacles_, rangeSq, ref obstacleNeighbors);
                UnityEngine.Profiling.Profiler.EndSample();

                if (agent.maxNeighbors_ > 0)
                {
                    UnityEngine.Profiling.Profiler.BeginSample("[RVO] computeAgentNeighbors");
                    rangeSq = RVOMath.sqr(agent.neighborDist_);
                    kdTree_.computeAgentNeighbors(in agent, in agentsReadOnly, ref rangeSq, ref agentNeighbors);
                    UnityEngine.Profiling.Profiler.EndSample();
                }

                // 计算 ORCA 新速度
                UnityEngine.Profiling.Profiler.BeginSample("[RVO] ROCA.computeNewVelocity");
                ROCA.computeNewVelocity(ref agent, simulator.agents_, simulator.obstacles_, in obstacleNeighbors, in agentNeighbors);
                UnityEngine.Profiling.Profiler.EndSample();
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

            return simulator.doStep();
        }

        public static void Clear()
        {
            if (kdTree_ != null) kdTree_.Clear();
        }
    }
}