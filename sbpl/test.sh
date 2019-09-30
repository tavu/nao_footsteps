#!/bin/sh

test_sbpl="./build/./test_sbpl"

env_folder="env_examples/nav3d/"
# env_file="env1.cfg"
env_file="willow-25mm-inflated-env.cfg"

enviroment="$env_folder$env_file"


# direction="backward"
direction="forward"

# planner="arastar"
planner="adstar"
#  planner="rstar"
# planner="anastar"
planner="lazyARA"

#  env_type="xythetamlev"
env_type="xytheta"
#  env_type="robarm"
 
$test_sbpl  --env=$env_type --planner=$planner --search-dir=$direction   $enviroment
echo $enviroment
