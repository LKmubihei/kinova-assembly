"""
Utility functions for HTN planner performance monitoring and debugging.
"""

import time
import functools
from typing import Callable, Any
import logging

logger = logging.getLogger(__name__)


def timeit(func: Callable) -> Callable:
    """Decorator to measure function execution time."""
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed = end_time - start_time
        logger.info(f"{func.__name__} took {elapsed:.4f} seconds")
        return result
    return wrapper


def profile_memory(func: Callable) -> Callable:
    """Decorator to profile memory usage (requires memory_profiler)."""
    try:
        from memory_profiler import profile
        return profile(func)
    except ImportError:
        logger.warning("memory_profiler not installed, skipping memory profiling")
        return func


class PlannerStats:
    """Collect and report planner statistics."""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """Reset all statistics."""
        self.operator_calls = 0
        self.method_calls = 0
        self.cache_hits = 0
        self.cache_misses = 0
        self.planning_time = 0.0
        self.max_depth = 0
    
    def report(self) -> str:
        """Generate statistics report."""
        total_calls = self.operator_calls + self.method_calls
        cache_total = self.cache_hits + self.cache_misses
        hit_rate = (self.cache_hits / cache_total * 100) if cache_total > 0 else 0
        
        return f"""
HTN Planner Statistics:
----------------------
Total planning time: {self.planning_time:.4f}s
Operator calls: {self.operator_calls}
Method calls: {self.method_calls}
Total calls: {total_calls}
Max recursion depth: {self.max_depth}
Cache hits: {self.cache_hits}
Cache misses: {self.cache_misses}
Cache hit rate: {hit_rate:.1f}%
"""

# Global stats instance
stats = PlannerStats()


def validate_state(state: Any) -> bool:
    """Validate that a state object has required attributes."""
    from .htn import State
    return isinstance(state, State) and hasattr(state, 'name') and hasattr(state, 'bindings')


def validate_task(task: Any) -> bool:
    """Validate task format."""
    return (isinstance(task, (list, tuple)) and 
            len(task) > 0 and 
            isinstance(task[0], str))


def validate_plan(plan: Any) -> bool:
    """Validate that a plan is properly formatted."""
    if not isinstance(plan, list):
        return False
    return all(validate_task(task) for task in plan)
