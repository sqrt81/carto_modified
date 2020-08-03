#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

class HybridGridIntergrator
{
public:
    HybridGridIntergrator() = default;

    void InsertGrid(const HybridGrid& grid,
                    const transform::Rigid3d &grid_pose);

    std::unique_ptr<HybridGrid>& Get();

private:
    std::unique_ptr<HybridGrid> base_hybrid_grid_;
};

}
}
