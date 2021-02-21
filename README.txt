########## NOTE ##########
This project was a group project for SFU's CMPT 417/824 intellegent systems class that is reloaded to github. Not everything in this project is done by me.
##########################

General Description:
The project's goal is to test different heuristics for conflict-based search. We tested three different heuristics for this: CG, DG, and WDG. 
- CG measures the heuristic based on the cardinal conflict of the agents and is found using a Multi-Valued Decision Diagram (MDD). 
- DG measures the heuristic based on the dependency (two agents are dependent if and only if every pair of their optimal path has at least one conflict) and is found using a joint-MDD.
- WDG is the weighted version of DG and measures heuristics based on the difference between their conflict-free paths and their optimal paths.

Files:
Worked on by me:
- mvc.py - brute force implementation of a minimum vertex cover.
- joint_mdd.py - a joint multi-valued decision diagram.

Worked on by other members:
- run_experiments.py - Runs the code on different test files.
- mdd.py - a multi-valued decision diagram.

Test files:
- instances - test examples given by the professor.
- custominstances - a few custom test sample created by us.
- largeinstance - test examples with larger maps to test run speed.

Based code from previous assignment:
- prioritize.py
- visualize.py
- single_agent_planner.py
- cbs.py
- independent.py

libraries used:
• argparse
• glob
• itertools
• copy
• math
• time
• heapq
• random
• numpy
• matplotlib
These libraries are also listed in the requirements.txt.
##############
TO RUN
##############
A simple example to run the code:
python run_experiments.py --instance instances/exp0.txt --solver CBS --heuristic CG

--instance: 
Directory of the map to be solved. These can be any of the text files inside the instances, custominstances, or largeinstances folder. To run a batch, use the --batch arguement and reference the directory for the set of maps to solve.

--solver:
Finds agent's path 
CBS - using conflict based search.
Independent - independently from other agents.
Prioritized - by randomly prioritizing certain agents over others. (may fail based on which agents are prioritized first.)

--heuristic:
Solves using the three heuristics described above.
CG
DG
WDG


