#!/bin/sh
for instance in '"instances/random-32-32-20/agents_experiment/16/*"' '"instances/random-32-32-20/agents_experiment/24/*"' '"instances/random-32-32-20/agents_experiment/32/*"' '"instances/random-32-32-20/agents_experiment/40/*"'; do
python3 run_experiments.py --instance $instance --solver Prioritized --replanner prioritized
done;
git add .
git commit -m "bash script updating results"
git push
