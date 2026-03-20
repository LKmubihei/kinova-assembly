from robot_htn import htn

def pickup(state,obj,place):
    """Each robot_htn planning operator is a Python function. The 1st argument is
    the current state, and the others are the planning operator's usual arguments.
    obj is the object to be picked up, and place is where it is located.
    The robot operators use three state variables:
    - pos[obj] = object obj's position, which may be 'table', 'hand', or another object.
    - clear[obj] = False if a block is on obj or the hand is holding obj, else True.
    - holding = name of the object being held, or False if the hand is empty."""
    if (state.pos[obj] == place
        and state.pos["robot"] == place
        and state.clear[obj] == True
        and state.holding == False):
        # If the object is on the table and clear, pick it up
        state.pos[obj] = 'hand'
        state.clear[obj] = False
        state.holding = obj
        return state
    else: return False


def moveto(state,placeA,placeB):
    """
    Move the robot from placeA to placeB if the robot is at placeA  and
    placeB is clear.
    The robot operators use three state variables:
    - pos["robot"] = robot's position, which may be 'table', 'hand'
    - clear[place] = False if a block is on place or the hand is holding place, else True.
    - holding = name of the object being held, or False if the hand is empty.
    """
    if (state.pos["robot"] == placeA
        and state.clear[placeB] == True):
        state.clear[placeA] = True
        state.clear[placeB] = False
        state.pos["robot"] = placeB
        return state
    else:
        return False


def putdown(state,obj,place):
    """Put down the object obj at the specified place."""
    if state.pos[obj] == 'hand'and state.pos["robot"] == place:
        # If the object is in the hand and the robot is at the place, put it down
        state.pos[obj] = place
        state.clear[obj] = True
        state.holding = False
        return state
    else: return False      




"""
Below, 'declare_operators(pickup, unstack, putdown, stack)' tells robot_htn
what the operators are. Note that the operator names are *not* quoted.
"""

htn.declare_operators(pickup, moveto, putdown)
