# Coloured Multi-Agent Pathfinding with Large Neighborhood Search

## Overview
This Python project is an implementation of a solution for Coloured Multi-Agent Pathfinding (CMAPF) problems using Large Neighborhood Search (LNS). CMAPF involves finding collision-free paths for multiple agents in a dynamic environment where each agent has a unique color, and agents must avoid collisions while navigating through the space. The use of Large Neighborhood Search introduces an efficient optimization approach to tackle complex CMAPF instances.

## Features
- **Coloured Agents:** Each agent is assigned a unique color, introducing an additional dimension to the traditional Multi-Agent Pathfinding problem.
- **Dynamic Environment:** Agents navigate through a dynamic environment, adapting their paths in real-time to avoid collisions and achieve their goals.
- **Large Neighborhood Search (LNS):** The project utilizes the LNS technique to iteratively optimize agent paths, balancing efficiency and solution quality.

## Usage
1. Clone the repository: `git clone https://github.com/diarmuidmckenna/coloured_mapf/`
2. Navigate to the project directory: `cd cmapf-lns`
3. Install dependencies: `pip install -r requirements.txt`
4. Run the main script: `./run_instances.sh`

## Requirements
- Python 3.x

## Acknowledgements
This project was developed as part of a final year project by Diarmuid McKenna. Special thanks to Ken Brown for guidance and support.
