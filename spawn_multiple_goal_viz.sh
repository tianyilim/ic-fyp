#!bin/bash

Y=(-5.4 -3.8 -3.1 -1.5 -0.8 0.8 1.5 3.1 3.8 5.4)
X=(-6.5 -5.5 -4.5 -1.0 0.0 1.0 4.5 5.5 6.5)

id_ctr=1

for x in "${X[@]}"
do
    for y in "${Y[@]}"
    do
        # echo "$x, $y, $id_ctr"
        ros2 run multirobot_control spawn_single_bot -z 0.05 --urdf "src/multirobot_control/urdf/goal.sdf" -x $x -y $y -n "marker$id_ctr"

        id_ctr=$(($id_ctr+1))

    done
done