Emergency services logistics
Automated planning project made with pddl, PDDL4J and planutils to tackle an emergency services logistics problem where the robotic agents must deliver some goods (food, medicine, tools) to people in need using a carrier and some boxes, and have to come up with the optimal plan. The strategy uses A-star search with a domain-specific heuristic.

Overview
The repository contains three directories.

Task_1 and Task_2 Classical Planning
Three instances of the problem are modeled using pddl and are passed to a solver (coded in Java) with the PDDL4J library. The solver uses the A-star algorithm to search the solution space and a domain-specific heuristic tailored to the use case. The heuristic is based on the SUM heuristic of the PDDL4J library, but before returning the result of the SUM heuristic makes some checks to massively prune the search tree, speeding up the computation. This pruning takes place thanks to a mapping between the object types specified in the pddl files and the instance passed to the solver, so in this sense the heuristic is domain-specific.

Task_3: Temporal Planning
In the temporal planning setting, every action has now its duration, based on the weight of the carrier and/or the boxes and their content. The directory also contains the files needed to execute the plan on the PlanSys2 tool.
