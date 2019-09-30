#include <footstep_planner/EuclidianStepCost2.h>

namespace footstep_planner
{
EuclidianStepCost2::EuclidianStepCost2(double cell_size, 
                                       int num_angle_bins,
                                        double step_cost, 
                                       double diff_angle_cost,
                                       double max_x, 
                                       double max_y)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_STEPCOST2),
//   ivpGrid(NULL),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),  
  ivMaxX(max_x),
  ivMaxY(max_y)
//   ivGoalX(-1),
//   ivGoalY(-1)
{
    ivMaxTheta=atan(max_x/max_x);
}


double EuclidianStepCost2::getHValue(const PlanningState& current, const PlanningState& to) const
{
    if (current == to)
        return 0.0;

    unsigned int from_x=current.getX();
    unsigned int from_y=current.getY();   
    unsigned int to_x=to.getX();
    unsigned int to_y=to.getY();
  
    double dist = cont_val(euclidean_distance(
        current.getX(), current.getY(), to.getX(), to.getY()) -ivMaxX, ivCellSize);
    double expected_steps =dist/0.02 -1;
  
    double diff_angle = 0.0;
    if (ivDiffAngleCost > 0.0)
    {
        double bin_size = TWO_PI / ivNumAngleBins;
//         double to_th=(double)to.getTheta();
        double from_th=(double)current.getTheta();
        double x=(double)to.getX()-(double)current.getX();
        double y=(double)to.getY()-(double)current.getY();        
        double current_theta, to_theta;
        
        current_theta=angles::normalize_angle( (double)current.getTheta()*bin_size );
        to_theta=angles::normalize_angle( (double)to.getTheta()*bin_size );

        double d1=std::atan2(y,x)-current_theta;
        int d1_disc=angle_state_2_cell(std::fabs(d1),ivNumAngleBins);
        
        double d2=to_theta-current_theta-d1;
        d2=std::fabs(angles::normalize_angle(d2));
        d2 -= ivMaxTheta;
        
        if(d2>0)
        {
            int d2_disc=angle_state_2_cell(std::fabs(d2),ivNumAngleBins);
            diff_angle=d1_disc+d2_disc;
        }
        else
        {
            diff_angle=d1_disc;
        }
        
//         diff_angle =d1_disc+d2_disc;
        
//         ROS_ERROR("x:%f, y:%f", x,y);
//         ROS_ERROR("curr_th:%f to_th:%f", current_theta,to_theta);
//         ROS_ERROR("angle:%f d1:%f d1_disc:%d d2:%f d2_disc:%d",diff_angle,d1,d1_disc,d2,d2_disc);
    }
//     return expected_steps * ivStepCost;
    return ( expected_steps * ivStepCost +  diff_angle * ivDiffAngleCost - ivDiffAngleCost);
//     return ( expected_steps * ivStepCost +  diff_angle * ivDiffAngleCost );
}


}