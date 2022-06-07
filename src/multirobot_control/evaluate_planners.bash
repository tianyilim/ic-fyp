#!bin/bash
# Run this in the multirobot_control directory

# remove all files in the test directories
rm -r /home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_scenarios
rm -r /home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_params

# Creates a few variations of test parameters in 
python3 create_test_scenarios.py

# Loop through test_scenarios and test_params
for file in /home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_scenarios/*
do
    TEST_SCENARIO_FILE=$file
    NAME=$(basename $file)
    TEST_PARAM_FILE="/home/tianyilim/fyp/ic-fyp/src/multirobot_control/test_params/${NAME}"
    
    cp $TEST_SCENARIO_FILE "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml"
    cp $TEST_PARAM_FILE "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/params_custom.yaml"
    
    # Create goal creation as a background process
    ros2 run multirobot_control goal_creation --ros-args --params-file "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml" &
    GOAL_CREATION_PID=$!

    ros2 launch multirobot_control test_scenario.launch.py rviz:=false &

    wait $GOAL_CREATION_PID # Wait for goal_creation to terminate.
    echo "Goal creation completed. Killing all ROS nodes."

    # Find all PIDs of running ROS processes
    pid_array=$(ps aux | grep [^a-zA-Z]ros[^a-zA-Z] | awk '{print $2}')
    for i in "${pid_array[@]}"
    do
        kill $i
    done
done

rm "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml"
rm "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/params_custom.yaml"
