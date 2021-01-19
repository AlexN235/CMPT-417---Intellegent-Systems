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