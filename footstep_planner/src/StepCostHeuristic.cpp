 #include <footstep_planner/StepCostHeuristic.h>
#include <math.h>
#include <cmath>
#include <footstep_planner/helper.h>
#include <angles/angles.h>
namespace footstep_planner
{
StepCostHeuristic::StepCostHeuristic(double cell_size,
                                     int    num_angle_bins,
                                     double step_cost,
                                     double diff_angle_cost,
                                     double max_step_width,
                                     double inflation_radius)
: Heuristic(cell_size, num_angle_bins, STEP_COST),
  ivpGrid(NULL),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivMaxStepWidth(max_step_width),
  ivInflationRadius(inflation_radius),
  ivGoalX(-1),
  ivGoalY(-1)
{}


StepCostHeuristic::~StepCostHeuristic()
{
    if (ivpGrid)
        resetGrid();
}


double StepCostHeuristic::getHValue(const PlanningState& current,const PlanningState& to) const
{
    assert(ivGoalX >= 0 && ivGoalY >= 0);

    if (current == to)
        return 0.0;

    unsigned int from_x;
    unsigned int from_y;
    // could be removed after more testing (then use ...noBounds... again)
    ivMapPtr->worldToMapNoBounds(cell_2_state(current.getX(), ivCellSize),
                               cell_2_state(current.getY(), ivCellSize),
                               from_x, from_y);

    unsigned int to_x;
    unsigned int to_y;
    // could be removed after more testing (then use ...noBounds... again)
    ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

    // cast to unsigned int is safe since ivGoalX/ivGoalY are checked to be >= 0
    if ((unsigned int)ivGoalX != to_x || (unsigned int)ivGoalY != to_y)
    {
        ROS_ERROR("StepCostHeuristic::getHValue to a different value than "
              "precomputed, heuristic values will be wrong. You need to call "
              "calculateDistances() before!");
    }
    assert((unsigned int)ivGoalX == to_x && (unsigned int)ivGoalY == to_y);

    double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
         from_x, from_y)) / 1000.0;
        
//     double expected_steps = dist / ivMaxStepWidth;

//      double dist = cont_val(euclidean_distance(
//          current.getX(), current.getY(), to.getX(), to.getY()) - ivMaxStepWidth, ivCellSize);

//     dist -=ivMaxStepWidth;
    double expected_steps =dist/0.02 -1;
  
    double diff_angle = 0.0;
    if (ivDiffAngleCost > 0.0)
    {
        double tmp=to_y;
        to_y=from_y;
        from_y=tmp;
        
        tmp=to_x;
        to_x=from_x;
        from_x=tmp;
        
        double bin_size = TWO_PI / ivNumAngleBins;
        double y=(double)to_y - (double)from_y;
        double x=double(to_x) - (double)from_x;
//         double to_theta=(double)to.getTheta();
//         double from_theta=(double)current.getTheta();
        double to_th=(double)to.getTheta();
        double from_th=(double)current.getTheta();
        
//         double x=(double)current.getX()-(double)to.getX();
//         double y=(double)current.getY()-(double)to.getY();
        
        double current_theta, to_theta;
        /*
        if(current.getTheta() > ivNumAngleBins/2)
        {
            current_theta = (current.getTheta()-ivNumAngleBins)*bin_size;
        }
        else
        {
            current_theta=current.getTheta()*bin_size;
        }
        
        if(to.getTheta() > ivNumAngleBins/2)
        {
            to_theta = (to.getTheta()-ivNumAngleBins)*bin_size;
        }
        else
        {
            to_theta=to.getTheta()*bin_size;
        }
        */
        current_theta=angles::normalize_angle( from_th*bin_size );
        to_theta=angles::normalize_angle( to_theta*bin_size );
//         double d1=std::atan2(y,x)-current_theta;
        double d1=std::atan2(y,x)-current_theta;
        int d1_disc=angle_state_2_cell(std::fabs(d1),ivNumAngleBins);
        
        double d2=to_theta-current_theta-d1;
        d2=angles::normalize_angle(d2);
        int d2_disc=angle_state_2_cell(std::fabs(d2),ivNumAngleBins);
        
        diff_angle =d1_disc+d2_disc;
        diff_angle =d1_disc;
//         ROS_ERROR("x:%f, y:%f", x,y);
//         ROS_ERROR("curr_th:%f to_th:%f", current_theta,to_theta);
         ROS_ERROR("angle:%f d1:%f d1_disc:%d d2:%f d2_disc:%d",diff_angle,d1,d1_disc,d2,d2_disc);
    }
    
    return ( expected_steps * ivStepCost +  diff_angle * ivDiffAngleCost - ivDiffAngleCost);
}


bool
StepCostHeuristic::calculateDistances(const PlanningState& from,
                                      const PlanningState& to)
{
  assert(ivMapPtr);

  unsigned int from_x;
  unsigned int from_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(from.getX(), ivCellSize),
                               cell_2_state(from.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ivGoalX = to_x;
    ivGoalY = to_y;
    ivGridSearchPtr->search(ivpGrid, cvObstacleThreshold,
                            ivGoalX, ivGoalY, from_x, from_y,
                            SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }

  return true;
}


void
StepCostHeuristic::updateMap(gridmap_2d::GridMap2DPtr map)
{
  if (ivpGrid) // reset map before change it's sizes (in other case we will get SEGMENT ERROR)
    resetGrid();

  ivMapPtr.reset();
  ivMapPtr = map;

  ivGoalX = ivGoalY = -1;

  unsigned width = ivMapPtr->getInfo().width;
  unsigned height = ivMapPtr->getInfo().height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();
  ivGridSearchPtr.reset(new SBPL2DGridSearch(width, height,
                                             ivMapPtr->getResolution()));

  ivpGrid = new unsigned char* [width];

  for (unsigned x = 0; x < width; ++x)
    ivpGrid[x] = new unsigned char [height];
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      float dist = ivMapPtr->distanceMapAtCell(x,y);
      if (dist < 0.0f)
        ROS_ERROR("Distance map at %d %d out of bounds", x, y);
      else if (dist <= ivInflationRadius)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
    }
  }
}


void
StepCostHeuristic::resetGrid()
{
  // CvSize size = ivMapPtr->size(); // here we get (height; width) instead of (width; height)
  int width = ivMapPtr->getInfo().width;
  for (int x = 0; x < width; ++x)
  {
    if (ivpGrid[x])
    {
      delete[] ivpGrid[x];
      ivpGrid[x] = NULL;
    }
  }
  delete[] ivpGrid;
  ivpGrid = NULL;
}
} // end of namespace
