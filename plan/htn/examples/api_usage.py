"""
Example usage of HTN Config API
"""

import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from robot_htn.api import HTNConfigAPI, create_example_config


def example_basic_usage():
    """Basic API usage example"""
    print("=== Basic API Usage ===\n")
    
    # Create API instance
    api = HTNConfigAPI()
    
    # Create new configuration
    config = api.new_config()
    print(f"Created new config: {config.metadata}")
    
    # Add operator
    api.add_operator(
        name="move",
        params=["from_loc", "to_loc"],
        preconditions="state.location == from_loc",
        effects="state.location = to_loc"
    )
    print(f"\nAdded operator 'move'")
    
    # Add method
    api.add_method(
        name="travel_method",
        task="travel",
        params=["destination"],
        preconditions="state.location != destination",
        subtasks="[('move', state.location, destination)]"
    )
    print(f"Added method 'travel_method'")
    
    # List operators and methods
    print(f"\nOperators: {api.list_operators()}")
    print(f"Methods: {api.list_methods()}")
    
    # Save configuration
    save_path = Path("temp_config.json")
    api.save_config(save_path)
    print(f"\nSaved to: {save_path}")
    
    # Load configuration
    api2 = HTNConfigAPI()
    loaded_config = api2.load_config(save_path)
    print(f"\nLoaded config with {len(loaded_config.operators)} operators")
    
    # Clean up
    save_path.unlink()


def example_validation():
    """Configuration validation example"""
    print("\n=== Configuration Validation ===\n")
    
    api = HTNConfigAPI()
    
    # Add invalid operator (missing effects)
    api.add_operator(
        name="invalid_op",
        params=["param1"],
        preconditions="True",
        effects=""
    )
    
    # Add method with undefined reference
    api.add_method(
        name="invalid_method",
        task="do_something",
        params=[],
        preconditions="True",
        subtasks="[('undefined_operator', 'arg1')]"
    )
    
    # Validate
    errors = api.validate_config()
    
    print("Validation errors:")
    for category, error_list in errors.items():
        if error_list:
            print(f"\n{category}:")
            for error in error_list:
                print(f"  - {error}")


def example_import_export():
    """Import/Export example"""
    print("\n=== Import/Export Example ===\n")
    
    # Create example config
    config = create_example_config()
    
    api = HTNConfigAPI()
    api._current_config = config
    
    # Export operators
    op_file = Path("operators_only.json")
    api.export_operators(op_file)
    print(f"Exported operators to: {op_file}")
    
    # Export methods
    method_file = Path("methods_only.json")
    api.export_methods(method_file)
    print(f"Exported methods to: {method_file}")
    
    # Create new API and import
    api2 = HTNConfigAPI()
    imported_ops = api2.import_operators(op_file, merge=False)
    print(f"\nImported {len(imported_ops)} operators")
    
    imported_methods = api2.import_methods(method_file, merge=True)
    print(f"Imported {len(imported_methods)} methods")
    
    # Clean up
    op_file.unlink()
    method_file.unlink()


def example_htn_integration():
    """HTN system integration example"""
    print("\n=== HTN Integration Example ===\n")
    
    from robot_htn import htn, State
    
    # Create API and load example
    api = HTNConfigAPI()
    api._current_config = create_example_config()
    
    # Apply to HTN system
    api.apply_to_htn()
    print("Applied configuration to HTN system")
    
    # Check what was registered
    print(f"\nRegistered operators: {list(htn.operators.keys())}")
    print(f"Registered methods: {list(htn.methods.keys())}")
    
    # Test planning
    state = State("test_state")
    state.pos = {"obj1": "table"}
    state.holding = False
    
    tasks = [("get", "obj1")]
    
    print(f"\nTesting planning with tasks: {tasks}")
    result = htn.plan(state, tasks, htn.operators, htn.methods, verbose=0)
    
    if result:
        print("Plan found:")
        for i, action in enumerate(result):
            print(f"  {i+1}. {action}")
    else:
        print("No plan found")


if __name__ == "__main__":
    example_basic_usage()
    example_validation()
    example_import_export()
    example_htn_integration()

