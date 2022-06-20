#!bin/bash
# Run this in the evaluation directory

EVALUATION_DIR="/home/tianyilim/fyp/ic-fyp/evaluation"
cd $EVALUATION_DIR

TEST_SCENARIO_DIR="${EVALUATION_DIR}/test_scenarios"
TEST_PARAMS_DIR="${EVALUATION_DIR}/test_params"
TEST_RESULT_DIR="${EVALUATION_DIR}/result"

mkdir -p result/log

# Regenerates test cases
# remove all files in the test directories
rm -r $TEST_SCENARIO_DIR
rm -r $TEST_PARAMS_DIR
python3 create_test_scenarios.py

NUM_TEST_FILES=$(ls -1q $TEST_SCENARIO_DIR/* | wc -l)
TEST_FILE_CTR=1

# Loop through test_scenarios and test_params
for file in $TEST_SCENARIO_DIR/*
do

    NAME=$(basename $file)
    
    if test -f "$TEST_RESULT_DIR/$NAME"; then
        echo "$TEST_RESULT_DIR/$NAME exists. Not running test."
    else
        echo "$TEST_RESULT_DIR/$NAME does not exist. Running test."

        TEST_SCENARIO_FILE=$file
        TEST_PARAM_FILE="$TEST_PARAMS_DIR/${NAME}"
        
        cp $TEST_SCENARIO_FILE "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml"
        cp $TEST_PARAM_FILE "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/params_custom.yaml"
        
        # Create goal creation as a background process
        LOGFILE=$(basename $file .yaml)
        ros2 launch multirobot_control test_scenario.launch.py rviz:=false &> "result/log/${LOGFILE}.txt" &

        ros2 run multirobot_control goal_creation --ros-args --params-file "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml" &
        GOAL_CREATION_PID=$!

        wait $GOAL_CREATION_PID # Wait for goal_creation to terminate.

        echo "Test [$TEST_FILE_CTR/$NUM_TEST_FILES] complete. Killing all ROS nodes."
        TEST_FILE_CTR=$(($TEST_FILE_CTR+1)) # Increment test counter

        # Find all PIDs of running ROS processes
        pid_array=$(ps aux | grep [^a-zA-Z]ros[^a-zA-Z] | awk '{print $2}')
        for PID in "${pid_array[@]}"; do kill $PID; done

        sleep 1 # Wait for cleanup to happen if needed
    fi
done

rm -f "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/scenario_custom.yaml"
rm -f "/home/tianyilim/fyp/ic-fyp/src/multirobot_control/params/params_custom.yaml"
