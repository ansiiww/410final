import sys
import os
import time
import importlib
import numpy as np

from pddl_parser import planner
original_cwd = os.getcwd()

project_path = os.path.dirname(os.path.abspath(__file__))
module1_path = os.path.join(project_path, 'padm_project_2023f')
src_path = os.path.join(module1_path, 'src')
os.chdir(module1_path)

sys.path.extend([module1_path, src_path])

try:
    from padm_project_2023f import execution_engine as engine
finally:
    os.chdir(original_cwd)

# padm_name = 'padm-project-2023f'
# padm = importlib.import_module(padm_name)

# execution_name = 'execution_engine.py'  # Replace with your submodule/file name
# full_submodule_name = f"{padm_name}.{execution_name}"
# execution = importlib.import_module(full_submodule_name)

start_time = time.time()
domain = 'kitchen.pddl'
problem = 'pb.pddl'
verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
planner = planner.Planner()

plan = planner.solve_plan_bfs(domain, problem)
# print(plan)
print('Time: ' + str(time.time() - start_time) + 's')
if plan is not None:
    print('plan:')
    for act in plan:
        print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
else:
    sys.exit('No plan was found')

engine.execute_plan(plan)
