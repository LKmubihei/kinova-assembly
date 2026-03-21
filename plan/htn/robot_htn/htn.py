from __future__ import print_function
import copy
from typing import List, Dict, Tuple, Union, Callable, Optional, Any
from dataclasses import dataclass, field
from functools import lru_cache
import logging

from robot_htn.helpers import (
    print_goal, print_methods, print_operators, print_state)

# Configure logging
logger = logging.getLogger(__name__)

############################################################
# States and goals

@dataclass
class State:
    """A state is a collection of variable bindings with efficient comparison."""
    name: str
    bindings: Dict[str, Any] = field(default_factory=dict)

    def _freeze_value(self, value: Any) -> Any:
        """Convert nested containers into hashable structures."""
        if isinstance(value, dict):
            return tuple(sorted((k, self._freeze_value(v)) for k, v in value.items()))
        if isinstance(value, (list, tuple)):
            return tuple(self._freeze_value(v) for v in value)
        if isinstance(value, set):
            return tuple(sorted(self._freeze_value(v) for v in value))
        return value

    def _comparison_items(self) -> Tuple[Tuple[str, Any], ...]:
        """Include dynamic attributes so planner caches remain correct."""
        return tuple(
            sorted(
                (key, self._freeze_value(value))
                for key, value in vars(self).items()
                if not callable(value)
            )
        )
    
    def __hash__(self):
        """Make State hashable for caching."""
        return hash(self._comparison_items())
    
    def __eq__(self, other):
        """Efficient equality comparison."""
        if not isinstance(other, State):
            return False
        return self._comparison_items() == other._comparison_items()
    
    def copy(self):
        """Deep copy that preserves all dynamic attributes."""
        return copy.deepcopy(self)
    
    @property
    def __name__(self):
        """Backward compatibility."""
        return self.name

@dataclass
class Goal:
    """A goal is a collection of variable bindings."""
    name: str
    bindings: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def __name__(self):
        """Backward compatibility."""
        return self.name

############################################################
# Operator and Method Registry

class TaskRegistry:
    """Centralized registry for operators and methods with validation."""
    
    def __init__(self):
        self.operators: Dict[str, Callable] = {}
        self.methods: Dict[str, List[Callable]] = {}
        self._operator_cache: Dict[Tuple[str, ...], Optional[State]] = {}
    
    def declare_operators(self, *op_list: Callable) -> Dict[str, Callable]:
        """Register operators with validation."""
        for op in op_list:
            if not callable(op):
                raise ValueError(f"Operator {op} must be callable")
            self.operators[op.__name__] = op
        return self.operators
    
    def declare_methods(self, task_name: str, *method_list: Callable) -> List[Callable]:
        """Register methods for a task with validation."""
        if not isinstance(task_name, str):
            raise ValueError("Task name must be a string")
        
        validated_methods = []
        for method in method_list:
            if not callable(method):
                raise ValueError(f"Method {method} must be callable")
            validated_methods.append(method)
        
        self.methods[task_name] = validated_methods
        return validated_methods
    
    def get_operators(self) -> Dict[str, Callable]:
        """Get all registered operators."""
        return self.operators.copy()
    
    def get_methods(self) -> Dict[str, List[Callable]]:
        """Get all registered methods."""
        return {k: v.copy() for k, v in self.methods.items()}
    
    def clear_cache(self):
        """Clear operator result cache."""
        self._operator_cache.clear()

# Global registry instance
registry = TaskRegistry()

# Backward compatibility functions
def declare_operators(*op_list):
    """Legacy function for operator declaration."""
    return registry.declare_operators(*op_list)

def declare_methods(task_name, *method_list):
    """Legacy function for method declaration."""
    return registry.declare_methods(task_name, *method_list)

def get_operators():
    """Legacy function to get operators."""
    return registry.get_operators()

def get_methods():
    """Legacy function to get methods."""
    return registry.get_methods()

# Module-level references for backward compatibility
operators = registry.operators
methods = registry.methods

############################################################
# Optimized Planner

class HTNPlanner:
    """Hierarchical Task Network planner with optimizations."""
    
    def __init__(self, operators: Dict[str, Callable], methods: Dict[str, List[Callable]]):
        self.operators = operators
        self.methods = methods
        self._plan_cache: Dict[Tuple[State, Tuple], Optional[List]] = {}
        self._method_cache: Dict[Tuple[str, State], List] = {}
    
    def plan(self, state: State, tasks: List[Tuple], verbose: int = 0) -> Union[List[Tuple], bool]:
        """
        Find a plan that accomplishes tasks in state.
        Returns the plan if successful, False otherwise.
        """
        if verbose > 0:
            logger.info(f'** HTN Planning, verbose={verbose}: **\n   state = {state.name}\n   tasks = {tasks}')
        
        # Convert tasks to tuple for caching
        tasks_tuple = tuple(tasks)
        
        # Check cache first
        cache_key = (state, tasks_tuple)
        if cache_key in self._plan_cache:
            if verbose > 0:
                logger.info(f'** Cache hit for {cache_key}')
            return self._plan_cache[cache_key]
        
        result = self._seek_plan(state, tasks, [], 0, verbose)
        
        # Cache the result
        self._plan_cache[cache_key] = result
        
        if verbose > 0:
            logger.info(f'** result = {result}\n')
        return result
    
    def _seek_plan(self, state: State, tasks: List[Tuple], plan: List[Tuple], 
                   depth: int, verbose: int = 0) -> Union[List[Tuple], bool]:
        """Optimized recursive planning with early termination."""
        if verbose > 1:
            logger.debug(f'depth {depth} tasks {tasks}')
        
        # Base case: all tasks completed
        if not tasks:
            if verbose > 2:
                logger.debug(f'depth {depth} returns plan {plan}')
            return plan
        
        task = tasks[0]
        task_name = task[0]
        remaining_tasks = tasks[1:]
        
        # Try operators first (usually faster)
        if task_name in self.operators:
            result = self._try_operator(state, task, remaining_tasks, plan, depth, verbose)
            if result is not False:
                return result
        
        # Try methods if no operator found or operator failed
        if task_name in self.methods:
            result = self._try_methods(state, task, remaining_tasks, plan, depth, verbose)
            if result is not False:
                return result
        
        if verbose > 2:
            logger.debug(f'depth {depth} returns failure')
        return False
    
    def _try_operator(self, state: State, task: Tuple, remaining_tasks: List[Tuple],
                      plan: List[Tuple], depth: int, verbose: int) -> Union[List[Tuple], bool]:
        """Apply operator with minimal state copying."""
        if verbose > 2:
            logger.debug(f'depth {depth} action {task}')
        
        operator = self.operators[task[0]]
        newstate = operator(state.copy(), *task[1:])
        
        if verbose > 2:
            logger.debug(f'depth {depth} new state:')
            print_state(newstate)
        
        if newstate:
            return self._seek_plan(newstate, remaining_tasks, plan + [task], depth + 1, verbose)
        
        return False
    
    @lru_cache(maxsize=256)
    def _get_applicable_methods(self, task_name: str, state_hash: int) -> List[Callable]:
        """Cache applicable methods for a task and state."""
        return self.methods.get(task_name, [])
    
    def _try_methods(self, state: State, task: Tuple, remaining_tasks: List[Tuple],
                     plan: List[Tuple], depth: int, verbose: int) -> Union[List[Tuple], bool]:
        """Try all applicable methods with caching."""
        if verbose > 2:
            logger.debug(f'depth {depth} method instance {task}')
        
        task_name = task[0]
        applicable_methods = self._get_applicable_methods(task_name, hash(state))
        
        for method in applicable_methods:
            try:
                subtasks = method(state, *task[1:])
                
                if verbose > 2:
                    logger.debug(f'depth {depth} method {method.__name__} subtasks: {subtasks}')
                
                if subtasks is not False:
                    # Combine subtasks with remaining tasks
                    new_tasks = subtasks + remaining_tasks if subtasks else remaining_tasks
                    
                    solution = self._seek_plan(state, new_tasks, plan, depth + 1, verbose)
                    if solution is not False:
                        return solution
            
            except Exception as e:
                logger.error(f"Error in method {method.__name__}: {e}")
                if verbose > 1:
                    raise
        
        return False
    
    def clear_cache(self):
        """Clear all caches."""
        self._plan_cache.clear()
        self._method_cache.clear()
        self._get_applicable_methods.cache_clear()

# Global planner instance
_planner = None

def plan(state: State, tasks: List[Tuple], operators: Dict[str, Callable], 
         methods: Dict[str, List[Callable]], verbose: int = 0) -> Union[List[Tuple], bool]:
    """
    Legacy planning function with caching support.
    """
    global _planner
    
    # Create or update planner if operators/methods changed
    if _planner is None or _planner.operators != operators or _planner.methods != methods:
        _planner = HTNPlanner(operators, methods)
    
    return _planner.plan(state, tasks, verbose)

# Optimized search functions for backward compatibility
def search_operators(state, tasks, operators, methods, plan, task, depth, verbose):
    """Legacy operator search function."""
    if _planner is None:
        return plan(state, tasks, operators, methods, verbose)
    return _planner._try_operator(state, task, tasks[1:], plan, depth, verbose)

def search_methods(state, tasks, operators, methods, plan, task, depth, verbose):
    """Legacy method search function."""
    if _planner is None:
        return plan(state, tasks, operators, methods, verbose)
    return _planner._try_methods(state, task, tasks[1:], plan, depth, verbose)

def seek_plan(state, tasks, operators, methods, plan, depth, verbose=0):
    """Legacy planning function."""
    if _planner is None:
        return plan(state, tasks, operators, methods, verbose)
    return _planner._seek_plan(state, tasks, plan, depth, verbose)
