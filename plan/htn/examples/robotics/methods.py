"""
methods for robot_htn 1.1.
"""
from robot_htn import helpers, htn



### methods for "get"
def get_obj(state,obj):
    """
    Generate a pickup .
    """
    if state.clear[obj] and state.holding == False:
        if state.pos["robot"] == 'initial':
            return [('moveto', 'initial', state.pos[obj]),('pickup', obj, state.pos[obj])]
        elif state.pos["robot"] == state.pos[obj]:
            return [('pickup', obj, state.pos[obj])]
        else:
            return [('moveto', state.pos["robot"], state.pos[obj]),('pickup', obj, state.pos[obj])]
    else:
        return False
htn.declare_methods('get_obj', get_obj)


def put_obj(state,obj,place):
    """
    Generate a putdown.
    """
    if state.holding == obj:
        if state.pos["robot"] == place:
            return [('putdown', obj, place)]
        else:
            return [('moveto', state.pos["robot"], place), ('putdown', obj, place)]
    else:
        return False
htn.declare_methods('put_obj', put_obj) 


def assemble(state, obj,  place):
    """
    Generate a sequence of actions to assemble an object.
    This method checks if the object is already at the desired place.       
    If not, it will generate the necessary actions to get the object and put it down at the specified place.

    """
    if state.pos[obj] != place:
        if state.holding == False:
            return[("get_obj", obj),("put_obj",  obj, place)] 
        elif state.holding == obj:
            return[("put_obj", obj, place)] 
        else:
            return False

    else:
            return False

      
htn.declare_methods('assemble', assemble)

     
     








