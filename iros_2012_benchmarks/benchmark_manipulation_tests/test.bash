#!/bin/bash

roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_sbl.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/sbl


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_sbl.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/sbl


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_sbl.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/sbl


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_sbl.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/sbl







roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_rrt.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/rrt


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_rrt.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/rrt


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_rrt.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/rrt


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_rrt.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/rrt




roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_rrtconnect.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/rrtconnect


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_rrtconnect.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/rrtconnect


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_rrtconnect.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/rrtconnect


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_rrtconnect.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/rrtconnect









roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_rrtstar.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/rrtstar


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_rrtstar.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/rrtstar


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_rrtstar.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/rrtstar


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_rrtstar.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/rrtstar











roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_kpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/kpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_kpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/kpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_kpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/kpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_kpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/kpiece










roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tabletop_tests_lbkpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/tabletop planner_stats/tabletop/lbkpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests industrial_tests_lbkpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/industrial planner_stats/industrial/lbkpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests tunnel_tests_lbkpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/tunnel planner_stats/tunnel/lbkpiece


roslaunch benchmark_manipulation_tests server.launch &
sleep 30
roslaunch benchmark_manipulation_tests countertop_tests_lbkpiece.launch 
killall -INT roslaunch
sleep 30
cp -r results/countertop planner_stats/countertop/lbkpiece










