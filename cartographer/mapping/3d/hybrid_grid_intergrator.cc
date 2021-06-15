#include "cartographer/mapping/3d/hybrid_grid_intergrator.h"
#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

// 修改：定义HybridGridIntergrator类。此类的对象用于把多个子图的内容合并成单个地图。
void HybridGridIntergrator::InsertGrid(const HybridGrid &grid,
                                       const transform::Rigid3d &grid_pose)
{
    if(!base_hybrid_grid_)
    {
        // create a new hybrid grid
        base_hybrid_grid_ = std::make_unique<HybridGrid>(grid.resolution());
    }

    const transform::Rigid3d pose_int(
                grid_pose.translation() / grid.resolution(),
                grid_pose.rotation());

    for(auto item : grid)
    {
        const Eigen::Array3d global_pose = pose_int * item.first.cast<double>();
        uint16* const cell = base_hybrid_grid_
                ->mutable_value({common::RoundToInt(global_pose.x()),
                                 common::RoundToInt(global_pose.y()),
                                 common::RoundToInt(global_pose.z())});

        if(*cell == 0)
        {
            *cell = item.second;
        }
        else
        {
            *cell = ProbabilityToValue(ProbabilityFromOdds(
                                       Odds((*kValueToProbability)[item.second])
                                       * Odds((*kValueToProbability)[*cell])));
        }
    }
}

std::unique_ptr<HybridGrid>& HybridGridIntergrator::Get()
{
    return base_hybrid_grid_;
}

}
}
