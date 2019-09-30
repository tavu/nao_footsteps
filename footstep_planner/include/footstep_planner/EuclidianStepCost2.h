#ifndef EUCLIDIAD_STEP_COST_H 
#define EUCLIDIAD_STEP_COST_H

#include <footstep_planner/Heuristic.h>

namespace footstep_planner
{
class EuclidianStepCost2: public Heuristic
{
    public:
        EuclidianStepCost2(double cell_size, int num_angle_bins,
                            double step_cost, double diff_angle_cost,
                            double max_step_width, double inflation_radius);
        virtual ~EuclidianStepCost2(){};

        /**
        * @return The estimated costs needed to reach the state 'to' from within the
        * current state.
        */
        virtual double getHValue(const PlanningState& current,
                                const PlanningState& to) const;
                                
    private:
        double ivStepCost;
        double ivDiffAngleCost;
        double ivMaxX;
        double ivMaxY;
        double ivMaxTheta;
};

}

#endif
