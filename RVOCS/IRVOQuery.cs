

namespace RVO
{
    using Unity.Collections;

    public interface IRVOQuery
    {
        void buildObstacleTree(ref NativeList<Obstacle> obstacles);
        void buildAgentTree(ref NativeArray<Agent> agents);

        void computeAgentNeighbors(in Agent agent, in NativeArray<Agent> agents, ref float rangeSq, ref NativeList<Pair> agentNeighbors);
        void computeObstacleNeighbors(ref Agent agent, in NativeList<Obstacle> obstacles, float rangeSq, ref NativeList<Pair> obstacleNeighbors);
        void Clear();
    }
}