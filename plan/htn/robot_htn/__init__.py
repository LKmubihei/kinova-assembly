"""
Hierarchical Task Network (HTN) Planner

An optimized implementation of HTN planning for robotics and AI applications.
"""

from .htn import (
    # Core classes
    State, Goal,
    
    # Registry functions
    declare_operators, declare_methods,
    get_operators, get_methods,
    
    # Planning functions
    plan, HTNPlanner,
    
    # Global registry
    registry,
    
    # Module level compatibility
    operators, methods
)

__version__ = "2.0.0"
__all__ = [
    'State', 'Goal',
    'declare_operators', 'declare_methods',
    'get_operators', 'get_methods',
    'plan', 'HTNPlanner',
    'registry',
    'operators', 'methods'
]
