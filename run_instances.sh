#!/bin/sh
for instance in "instances/random-8-8-10/teams_experiment/2/test_1.txt" "instances/random-8-8-10/teams_experiment/2/test_2.txt" "instances/random-8-8-10/teams_experiment/2/test_3.txt" "instances/random-8-8-10/teams_experiment/2/test_4.txt" "instances/random-8-8-10/teams_experiment/2/test_5.txt" "instances/random-8-8-10/teams_experiment/4/test_1.txt" "instances/random-8-8-10/teams_experiment/4/test_2.txt" "instances/random-8-8-10/teams_experiment/4/test_3.txt" "instances/random-8-8-10/teams_experiment/4/test_4.txt" "instances/random-8-8-10/teams_experiment/4/test_5.txt" "instances/random-8-8-10/teams_experiment/6/test_1.txt" "instances/random-8-8-10/teams_experiment/6/test_2.txt" "instances/random-8-8-10/teams_experiment/6/test_3.txt" "instances/random-8-8-10/teams_experiment/6/test_4.txt" "instances/random-8-8-10/teams_experiment/6/test_5.txt" "instances/random-8-8-10/agents_experiment/15/test_1.txt" "instances/random-8-8-10/agents_experiment/15/test_2.txt" "instances/random-8-8-10/agents_experiment/15/test_3.txt" "instances/random-8-8-10/agents_experiment/15/test_4.txt" "instances/random-8-8-10/agents_experiment/15/test_5.txt" "instances/warehouse-12-21/agents_experiment/6/test_1.txt" "instances/warehouse-12-21/agents_experiment/6/test_2.txt" "instances/warehouse-12-21/agents_experiment/6/test_3.txt" "instances/warehouse-12-21/agents_experiment/6/test_4.txt" "instances/warehouse-12-21/agents_experiment/6/test_5.txt" "instances/warehouse-12-21/agents_experiment/9/test_1.txt" "instances/warehouse-12-21/agents_experiment/9/test_2.txt" "instances/warehouse-12-21/agents_experiment/9/test_3.txt" "instances/warehouse-12-21/agents_experiment/9/test_4.txt" "instances/warehouse-12-21/agents_experiment/9/test_5.txt" "instances/warehouse-12-21/agents_experiment/12/test_1.txt" "instances/warehouse-12-21/agents_experiment/12/test_2.txt" "instances/warehouse-12-21/agents_experiment/12/test_3.txt" "instances/warehouse-12-21/agents_experiment/12/test_4.txt" "instances/warehouse-12-21/agents_experiment/12/test_5.txt" "instances/warehouse-12-21/teams_experiment/2/test_1.txt" "instances/warehouse-12-21/teams_experiment/2/test_2.txt" "instances/warehouse-12-21/teams_experiment/2/test_3.txt" "instances/warehouse-12-21/teams_experiment/2/test_4.txt" "instances/warehouse-12-21/teams_experiment/2/test_5.txt" "instances/warehouse-12-21/teams_experiment/4/test_1.txt" "instances/warehouse-12-21/teams_experiment/4/test_2.txt" "instances/warehouse-12-21/teams_experiment/4/test_3.txt" "instances/warehouse-12-21/teams_experiment/4/test_4.txt" "instances/warehouse-12-21/teams_experiment/4/test_5.txt" "instances/warehouse-12-21/teams_experiment/6/test_1.txt" "instances/warehouse-12-21/teams_experiment/6/test_2.txt" "instances/warehouse-12-21/teams_experiment/6/test_3.txt" "instances/warehouse-12-21/teams_experiment/6/test_4.txt" "instances/warehouse-12-21/teams_experiment/6/test_5.txt" "instances/warehouse-12-21/agents_experiment/15/test_1.txt" "instances/warehouse-12-21/agents_experiment/15/test_2.txt" "instances/warehouse-12-21/agents_experiment/15/test_3.txt" "instances/warehouse-12-21/agents_experiment/15/test_4.txt" "instances/warehouse-12-21/agents_experiment/15/test_5.txt" "instances/random-8-8-10/agents_experiment/6/test_1.txt" "instances/random-8-8-10/agents_experiment/6/test_2.txt" "instances/random-8-8-10/agents_experiment/6/test_3.txt" "instances/random-8-8-10/agents_experiment/6/test_4.txt" "instances/random-8-8-10/agents_experiment/6/test_5.txt" "instances/random-8-8-10/agents_experiment/9/test_1.txt" "instances/random-8-8-10/agents_experiment/9/test_2.txt" "instances/random-8-8-10/agents_experiment/9/test_3.txt" "instances/random-8-8-10/agents_experiment/9/test_4.txt" "instances/random-8-8-10/agents_experiment/9/test_5.txt" "instances/random-8-8-10/agents_experiment/12/test_1.txt" "instances/random-8-8-10/agents_experiment/12/test_2.txt" "instances/random-8-8-10/agents_experiment/12/test_3.txt" "instances/random-8-8-10/agents_experiment/12/test_4.txt" "instances/random-8-8-10/agents_experiment/12/test_5.txt" "instances/random-8-8-10/teams_experiment/2/test_1.txt" "instances/random-8-8-10/teams_experiment/2/test_2.txt" "instances/random-8-8-10/teams_experiment/2/test_3.txt" "instances/random-8-8-10/teams_experiment/2/test_4.txt" "instances/random-8-8-10/teams_experiment/2/test_5.txt" "instances/random-8-8-10/teams_experiment/4/test_1.txt" "instances/random-8-8-10/teams_experiment/4/test_2.txt" "instances/random-8-8-10/teams_experiment/4/test_3.txt" "instances/random-8-8-10/teams_experiment/4/test_4.txt" "instances/random-8-8-10/teams_experiment/4/test_5.txt" "instances/random-8-8-10/teams_experiment/6/test_1.txt" "instances/random-8-8-10/teams_experiment/6/test_2.txt" "instances/random-8-8-10/teams_experiment/6/test_3.txt" "instances/random-8-8-10/teams_experiment/6/test_4.txt" "instances/random-8-8-10/teams_experiment/6/test_5.txt" "instances/random-8-8-10/agents_experiment/15/test_1.txt" "instances/random-8-8-10/agents_experiment/15/test_2.txt" "instances/random-8-8-10/agents_experiment/15/test_3.txt" "instances/random-8-8-10/agents_experiment/15/test_4.txt" "instances/random-8-8-10/agents_experiment/15/test_5.txt"; do
python3 run_experiments.py --instance $instance --solver makespan
done;
