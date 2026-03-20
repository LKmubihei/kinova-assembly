"""
HTN Planner API - Programmatic interface for HTN configuration management
"""

import json
import os
from typing import Dict, List, Any, Optional, Union
from pathlib import Path
from datetime import datetime
import copy

from robot_htn import htn, State, declare_operators, declare_methods


class HTNConfig:
    """HTN Configuration container"""
    
    def __init__(self, operators: Dict = None, methods: Dict = None, metadata: Dict = None):
        self.operators = operators or {}
        self.methods = methods or {}
        self.metadata = metadata or {
            "version": "2.0",
            "created": datetime.now().isoformat(),
            "author": os.getenv("USERNAME", "Unknown"),
            "description": "HTN Planning Configuration"
        }
    
    def to_dict(self) -> Dict:
        """Convert to dictionary"""
        return {
            "version": self.metadata.get("version", "2.0"),
            "created": self.metadata.get("created", datetime.now().isoformat()),
            "operators": self.operators,
            "methods": self.methods,
            "metadata": self.metadata
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'HTNConfig':
        """Create from dictionary"""
        return cls(
            operators=data.get("operators", {}),
            methods=data.get("methods", {}),
            metadata=data.get("metadata", {})
        )


class HTNConfigAPI:
    """API for HTN configuration management"""
    
    def __init__(self):
        self._current_config = HTNConfig()
        self._current_file = None
    
    # File Operations
    
    def new_config(self) -> HTNConfig:
        """Create a new empty configuration"""
        self._current_config = HTNConfig()
        self._current_file = None
        return self._current_config
    
    def load_config(self, filepath: Union[str, Path]) -> HTNConfig:
        """Load configuration from JSON file"""
        filepath = Path(filepath)
        
        if not filepath.exists():
            raise FileNotFoundError(f"Configuration file not found: {filepath}")
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Handle different format versions
            if isinstance(data, dict):
                if "version" in data:
                    # New format
                    self._current_config = HTNConfig.from_dict(data)
                else:
                    # Old format - assume it only has operators and methods
                    self._current_config = HTNConfig(
                        operators=data.get("operators", {}),
                        methods=data.get("methods", {})
                    )
            else:
                raise ValueError("Invalid configuration format")
            
            self._current_file = filepath
            return self._current_config
            
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in configuration file: {e}")
    
    def save_config(self, filepath: Union[str, Path] = None) -> Path:
        """Save configuration to JSON file"""
        if filepath is None and self._current_file is None:
            raise ValueError("No filepath specified and no current file")
        
        filepath = Path(filepath) if filepath else self._current_file
        
        # Update metadata
        self._current_config.metadata["modified"] = datetime.now().isoformat()
        
        try:
            # Create directory if needed
            filepath.parent.mkdir(parents=True, exist_ok=True)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self._current_config.to_dict(), f, indent=2)
            
            self._current_file = filepath
            return filepath
            
        except Exception as e:
            raise IOError(f"Failed to save configuration: {e}")
    
    def export_operators(self, filepath: Union[str, Path]) -> Path:
        """Export only operators to JSON file"""
        filepath = Path(filepath)
        
        data = {
            "version": "2.0",
            "type": "operators",
            "exported": datetime.now().isoformat(),
            "operators": self._current_config.operators
        }
        
        try:
            filepath.parent.mkdir(parents=True, exist_ok=True)
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            return filepath
        except Exception as e:
            raise IOError(f"Failed to export operators: {e}")
    
    def export_methods(self, filepath: Union[str, Path]) -> Path:
        """Export only methods to JSON file"""
        filepath = Path(filepath)
        
        data = {
            "version": "2.0",
            "type": "methods",
            "exported": datetime.now().isoformat(),
            "methods": self._current_config.methods
        }
        
        try:
            filepath.parent.mkdir(parents=True, exist_ok=True)
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            return filepath
        except Exception as e:
            raise IOError(f"Failed to export methods: {e}")
    
    def import_operators(self, filepath: Union[str, Path], merge: bool = True) -> Dict:
        """Import operators from JSON file"""
        filepath = Path(filepath)
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            imported_ops = data.get("operators", {})
            
            if merge:
                self._current_config.operators.update(imported_ops)
            else:
                self._current_config.operators = imported_ops
            
            return imported_ops
            
        except Exception as e:
            raise IOError(f"Failed to import operators: {e}")
    
    def import_methods(self, filepath: Union[str, Path], merge: bool = True) -> Dict:
        """Import methods from JSON file"""
        filepath = Path(filepath)
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            imported_methods = data.get("methods", {})
            
            if merge:
                self._current_config.methods.update(imported_methods)
            else:
                self._current_config.methods = imported_methods
            
            return imported_methods
            
        except Exception as e:
            raise IOError(f"Failed to import methods: {e}")
    
    # Operator Management
    
    def add_operator(self, name: str, params: List[str], 
                     preconditions: str, effects: str) -> None:
        """Add or update an operator"""
        self._current_config.operators[name] = {
            "params": params,
            "preconditions": preconditions,
            "effects": effects
        }
    
    def get_operator(self, name: str) -> Optional[Dict]:
        """Get operator by name"""
        return self._current_config.operators.get(name)
    
    def list_operators(self) -> List[str]:
        """List all operator names"""
        return list(self._current_config.operators.keys())
    
    def remove_operator(self, name: str) -> bool:
        """Remove an operator"""
        if name in self._current_config.operators:
            del self._current_config.operators[name]
            return True
        return False
    
    def clear_operators(self) -> None:
        """Remove all operators"""
        self._current_config.operators.clear()
    
    # Method Management
    
    def add_method(self, name: str, task: str, params: List[str],
                   preconditions: str, subtasks: str) -> None:
        """Add or update a method"""
        self._current_config.methods[name] = {
            "task": task,
            "name": name,
            "params": params,
            "preconditions": preconditions,
            "subtasks": subtasks
        }
    
    def get_method(self, name: str) -> Optional[Dict]:
        """Get method by name"""
        return self._current_config.methods.get(name)
    
    def list_methods(self) -> List[str]:
        """List all method names"""
        return list(self._current_config.methods.keys())
    
    def list_methods_for_task(self, task: str) -> List[str]:
        """List all methods that handle a specific task"""
        return [name for name, method in self._current_config.methods.items()
                if method.get("task") == task]
    
    def remove_method(self, name: str) -> bool:
        """Remove a method"""
        if name in self._current_config.methods:
            del self._current_config.methods[name]
            return True
        return False
    
    def clear_methods(self) -> None:
        """Remove all methods"""
        self._current_config.methods.clear()
    
    # Configuration Properties
    
    @property
    def current_config(self) -> HTNConfig:
        """Get current configuration"""
        return self._current_config
    
    @property
    def current_file(self) -> Optional[Path]:
        """Get current file path"""
        return self._current_file
    
    @property
    def metadata(self) -> Dict:
        """Get configuration metadata"""
        return self._current_config.metadata
    
    def set_metadata(self, key: str, value: Any) -> None:
        """Set metadata value"""
        self._current_config.metadata[key] = value
    
    # HTN System Integration
    
    def apply_to_htn(self) -> None:
        """Apply current configuration to HTN system"""
        # Clear existing
        htn.operators.clear()
        htn.methods.clear()
        
        # Apply operators
        for name, op_def in self._current_config.operators.items():
            self._create_and_register_operator(name, op_def)
        
        # Apply methods
        task_methods = {}
        for name, method_def in self._current_config.methods.items():
            task = method_def.get("task")
            if task not in task_methods:
                task_methods[task] = []
            task_methods[task].append((name, method_def))
        
        for task, method_list in task_methods.items():
            funcs = []
            for name, method_def in method_list:
                func = self._create_method_function(name, method_def)
                if func:
                    funcs.append(func)
            if funcs:
                declare_methods(task, *funcs)
    
    def _create_and_register_operator(self, name: str, op_def: Dict) -> None:
        """Create and register an operator function"""
        params = op_def["params"]
        precond = op_def["preconditions"].replace('\n', ' ').strip()
        effects_lines = op_def["effects"].split('\n')
        effects_indented = '\n        '.join(effects_lines)
        
        func_code = f"""
def {name}(state, {', '.join(params)}):
    if ({precond}):
        {effects_indented}
        return state
    else:
        return False
"""
        
        namespace = {"State": State}
        exec(func_code, namespace)
        declare_operators(namespace[name])
    
    def _create_method_function(self, name: str, method_def: Dict):
        """Create a method function"""
        params = method_def["params"]
        precond = method_def["preconditions"].replace('\n', ' ').strip()
        subtasks = method_def["subtasks"]
        
        func_code = f"""
def {name}(state, {', '.join(params)}):
    if ({precond}):
        return {subtasks}
    else:
        return False
"""
        
        namespace = {"State": State}
        try:
            exec(func_code, namespace)
            return namespace[name]
        except:
            return None
    
    # Validation
    
    def validate_config(self) -> Dict[str, List[str]]:
        """Validate current configuration"""
        errors = {
            "operators": [],
            "methods": [],
            "references": []
        }
        
        # Validate operators
        for name, op_def in self._current_config.operators.items():
            if not op_def.get("params"):
                errors["operators"].append(f"{name}: Missing parameters")
            if not op_def.get("preconditions"):
                errors["operators"].append(f"{name}: Missing preconditions")
            if not op_def.get("effects"):
                errors["operators"].append(f"{name}: Missing effects")
        
        # Validate methods
        for name, method_def in self._current_config.methods.items():
            if not method_def.get("task"):
                errors["methods"].append(f"{name}: Missing task")
            if not method_def.get("params"):
                errors["methods"].append(f"{name}: Missing parameters")
            if not method_def.get("subtasks"):
                errors["methods"].append(f"{name}: Missing subtasks")
            
            # Check subtask references
            try:
                subtasks = eval(method_def.get("subtasks", "[]"))
                if isinstance(subtasks, list):
                    for subtask in subtasks:
                        if isinstance(subtask, tuple) and len(subtask) > 0:
                            subtask_name = subtask[0]
                            # Check if subtask exists as task or operator
                            task_exists = any(m.get("task") == subtask_name 
                                            for m in self._current_config.methods.values())
                            op_exists = subtask_name in self._current_config.operators
                            
                            if not task_exists and not op_exists:
                                errors["references"].append(
                                    f"{name}: References undefined task/operator '{subtask_name}'"
                                )
            except:
                errors["methods"].append(f"{name}: Invalid subtasks format")
        
        return errors


# Convenience functions

def create_api() -> HTNConfigAPI:
    """Create a new HTN Config API instance"""
    return HTNConfigAPI()


def load_config_file(filepath: Union[str, Path]) -> HTNConfig:
    """Load configuration from file"""
    api = HTNConfigAPI()
    return api.load_config(filepath)


def create_example_config() -> HTNConfig:
    """Create an example configuration"""
    config = HTNConfig()
    
    # Add example operators
    config.operators["pickup"] = {
        "params": ["obj", "place"],
        "preconditions": "state.pos[obj] == place and state.holding == False",
        "effects": "state.pos[obj] = 'hand'\nstate.holding = obj"
    }
    
    config.operators["putdown"] = {
        "params": ["obj", "place"],
        "preconditions": "state.pos[obj] == 'hand'",
        "effects": "state.pos[obj] = place\nstate.holding = False"
    }
    
    # Add example methods
    config.methods["get_object"] = {
        "task": "get",
        "name": "get_object",
        "params": ["obj"],
        "preconditions": "True",
        "subtasks": "[('pickup', obj, state.pos[obj])]"
    }
    
    return config
