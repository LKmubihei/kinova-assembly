import sys


def forall(seq,cond):
    """True if cond(x) holds for all x in seq, otherwise False."""
    for x in seq:
        if not cond(x): return False
    return True

def find_if(cond,seq):
    """
    Return the first x in seq such that cond(x) holds, if there is one.
    Otherwise return None.
    """
    for x in seq:
        if cond(x): return x
    return None

def is_done(b1,state,goal, done_state):
    if b1 == done_state: return True
    if b1 in goal.pos and goal.pos[b1] != state.pos[b1]:
        return False
    if state.pos[b1] == done_state: return True
    return is_done(state.pos[b1],state,goal,done_state)

def all(state):
    return state.clear.keys()

def print_state(state,indent=4):
    """Print each variable in state, indented by indent spaces."""
    if state != False:
        for (name,val) in vars(state).items():
            if name != '__name__':
                for x in range(indent): sys.stdout.write(' ')
                sys.stdout.write(state.__name__ + '.' + name)
                print(' =', val)
    else: print('False')

print_goal = print_state

def print_operators(olist):
    """Print out the names of the operators"""
    print('OPERATORS:', ', '.join(olist))

def print_methods(mlist):
    """Print out a table of what the methods are for each task"""
    print('{:<14}{}'.format('TASK:','METHODS:'))
    for task in mlist:
        print('{:<14}'.format(task) + ', '.join(
            [f.__name__ for f in mlist[task]]))

"""
Helper functions for printing HTN states, operators, methods, and goals.
"""

def print_state(state):
    """Print the state in a readable format."""
    if hasattr(state, '__name__'):
        print(f"    state.name = {state.__name__}")
    if hasattr(state, 'bindings'):
        print(f"    state.bindings = {state.bindings}")
    
    # Print all other attributes
    for attr in dir(state):
        if not attr.startswith('_') and attr not in ['name', 'bindings', 'copy']:
            value = getattr(state, attr)
            if not callable(value):
                print(f"    state.{attr} = {value}")

def print_goal(goal):
    """Print the goal in a readable format."""
    if hasattr(goal, '__name__'):
        print(f"    goal.name = {goal.__name__}")
    if hasattr(goal, 'bindings'):
        print(f"    goal.bindings = {goal.bindings}")
    
    # Print all other attributes
    for attr in dir(goal):
        if not attr.startswith('_') and attr not in ['name', 'bindings']:
            value = getattr(goal, attr)
            if not callable(value):
                print(f"    goal.{attr} = {value}")

def print_operators(operators):
    """Print all registered operators."""
    if operators:
        operator_names = ', '.join(sorted(operators.keys()))
        print(f"OPERATORS: {operator_names}")
    else:
        print("OPERATORS: (none)")

def print_methods(methods):
    """Print all registered methods."""
    if methods:
        print("TASK:         METHODS:")
        for task_name in sorted(methods.keys()):
            method_list = methods[task_name]
            method_names = ', '.join([m.__name__ for m in method_list])
            print(f"{task_name:<13} {method_names}")
    else:
        print("METHODS: (none)")
