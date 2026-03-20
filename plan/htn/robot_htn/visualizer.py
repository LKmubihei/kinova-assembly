"""
HTN Task Network Visualizer - Visualize task decomposition hierarchy
"""

import tkinter as tk
from tkinter import ttk, Canvas
import math
from typing import Dict, List, Tuple, Set, Optional
import json
import sys

# Import ScaleManager for font scaling
try:
    from robot_htn.gui import ScaleManager, AppleFonts, AppleColors
except ImportError:
    # Fallback if import fails
    class ScaleManager:
        @classmethod
        def s(cls, val): return val
        @classmethod
        def get_scale(cls): return 1.0
        @classmethod
        def register_callback(cls, cb): pass
    
    class AppleFonts:
        FAMILY = "Segoe UI" if sys.platform == "win32" else "SF Pro Display"
        BODY = (FAMILY, 14)
        BODY_BOLD = (FAMILY, 14, "bold")
        CAPTION = (FAMILY, 12)
    
    class AppleColors:
        BG_PRIMARY = "#FFFFFF"
        TEXT_PRIMARY = "#1D1D1F"


class HTNVisualizer:
    """Visualize HTN task decomposition as a hierarchical graph"""
    
    def __init__(self, parent, methods: Dict, operators: Dict):
        self.parent = parent
        self.methods = methods
        self.operators = operators
        
        # Create main frame
        self.frame = ttk.Frame(parent)
        self.frame.pack(fill=tk.BOTH, expand=True)
        
        # Create toolbar
        self._create_toolbar()
        
        # Create canvas
        self.canvas = Canvas(self.frame, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbars
        v_scrollbar = ttk.Scrollbar(self.frame, orient='vertical', command=self.canvas.yview)
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        h_scrollbar = ttk.Scrollbar(self.frame, orient='horizontal', command=self.canvas.xview)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.canvas.configure(
            yscrollcommand=v_scrollbar.set,
            xscrollcommand=h_scrollbar.set
        )
        
        # Bind events
        self.canvas.bind('<ButtonPress-1>', self.on_mouse_press)
        self.canvas.bind('<B1-Motion>', self.on_mouse_drag)
        self.canvas.bind('<MouseWheel>', self.on_mouse_wheel)
        self.canvas.bind('<Button-4>', lambda e: self.on_mouse_wheel(e, delta=120))
        self.canvas.bind('<Button-5>', lambda e: self.on_mouse_wheel(e, delta=-120))
        
        # Drawing parameters - use scaled values
        s = ScaleManager.s
        self.node_width = s(120)
        self.node_height = s(40)
        self.level_height = s(100)
        self.node_spacing = s(30)
        self.zoom_level = 1.0
        
        # Node positions and info
        self.nodes = {}  # {node_id: {'x': x, 'y': y, 'type': type, 'name': name}}
        self.edges = []  # [(from_id, to_id)]
        
        # Colors
        self.colors = {
            'task': '#4A90E2',      # Blue for tasks
            'method': '#7ED321',    # Green for methods
            'operator': '#F5A623',  # Orange for operators
            'selected': '#E94B3C',  # Red for selected
            'edge': '#333333'       # Dark gray for edges
        }
        
        # Selection
        self.selected_node = None
        self.drag_start = None
        
        # Build and draw network
        self.build_network()
        self.draw_network()
    
    def _create_toolbar(self):
        """Create toolbar with controls"""
        s = ScaleManager.s
        self.toolbar = tk.Frame(self.frame, bg=AppleColors.BG_PRIMARY)
        self.toolbar.pack(side=tk.TOP, fill=tk.X, padx=s(10), pady=s(10))
        
        # Store widgets for scaling
        self.toolbar_widgets = []
        
        # Layout options
        self.layout_label = tk.Label(
            self.toolbar, text="Layout:", 
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        self.layout_label.pack(side=tk.LEFT, padx=s(5))
        self.toolbar_widgets.append(('label', self.layout_label))
        
        self.layout_var = tk.StringVar(value="hierarchical")
        self.layout_combo = ttk.Combobox(
            self.toolbar, 
            textvariable=self.layout_var,
            values=["hierarchical", "circular", "force-directed"],
            width=15,
            state='readonly',
            font=AppleFonts.BODY
        )
        self.layout_combo.pack(side=tk.LEFT, padx=s(5))
        self.layout_combo.bind('<<ComboboxSelected>>', lambda e: self.update_layout())
        self.toolbar_widgets.append(('combo', self.layout_combo))
        
        # Zoom controls
        self.zoom_in_btn = tk.Button(
            self.toolbar, text="Zoom In", 
            command=self.zoom_in,
            font=AppleFonts.BODY,
            padx=s(10), pady=s(4)
        )
        self.zoom_in_btn.pack(side=tk.LEFT, padx=s(2))
        self.toolbar_widgets.append(('button', self.zoom_in_btn))
        
        self.zoom_out_btn = tk.Button(
            self.toolbar, text="Zoom Out", 
            command=self.zoom_out,
            font=AppleFonts.BODY,
            padx=s(10), pady=s(4)
        )
        self.zoom_out_btn.pack(side=tk.LEFT, padx=s(2))
        self.toolbar_widgets.append(('button', self.zoom_out_btn))
        
        self.reset_zoom_btn = tk.Button(
            self.toolbar, text="Reset Zoom", 
            command=self.reset_zoom,
            font=AppleFonts.BODY,
            padx=s(10), pady=s(4)
        )
        self.reset_zoom_btn.pack(side=tk.LEFT, padx=s(2))
        self.toolbar_widgets.append(('button', self.reset_zoom_btn))
        
        # Export button
        self.export_btn = tk.Button(
            self.toolbar, text="Export as Image", 
            command=self.export_image,
            font=AppleFonts.BODY,
            padx=s(10), pady=s(4)
        )
        self.export_btn.pack(side=tk.LEFT, padx=s(10))
        self.toolbar_widgets.append(('button', self.export_btn))
        
        # Filter options
        self.show_label = tk.Label(
            self.toolbar, text="Show:", 
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        self.show_label.pack(side=tk.LEFT, padx=(s(20), s(5)))
        self.toolbar_widgets.append(('label', self.show_label))
        
        self.show_methods = tk.BooleanVar(value=True)
        self.show_operators = tk.BooleanVar(value=True)
        
        self.methods_check = tk.Checkbutton(
            self.toolbar, text="Methods", 
            variable=self.show_methods,
            command=self.update_visibility,
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY,
            activebackground=AppleColors.BG_PRIMARY
        )
        self.methods_check.pack(side=tk.LEFT)
        self.toolbar_widgets.append(('check', self.methods_check))
        
        self.operators_check = tk.Checkbutton(
            self.toolbar, text="Operators", 
            variable=self.show_operators,
            command=self.update_visibility,
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY,
            activebackground=AppleColors.BG_PRIMARY
        )
        self.operators_check.pack(side=tk.LEFT)
        self.toolbar_widgets.append(('check', self.operators_check))
        
        # Register for scale updates
        ScaleManager.register_callback(self._on_scale_change)
    
    def _on_scale_change(self):
        """Handle scale change for toolbar widgets"""
        s = ScaleManager.s
        for widget_type, widget in self.toolbar_widgets:
            try:
                if widget_type in ('label', 'button', 'check'):
                    widget.config(font=AppleFonts.BODY)
                elif widget_type == 'combo':
                    widget.config(font=AppleFonts.BODY)
            except:
                pass
        
        # Update node drawing parameters
        self.node_width = s(120)
        self.node_height = s(40)
        self.level_height = s(100)
        self.node_spacing = s(30)
        
        # Redraw network
        self.draw_network()
    
    def build_network(self):
        """Build the task network from methods and operators"""
        self.nodes.clear()
        self.edges.clear()
        
        # Track all tasks and their decompositions
        tasks = {}  # {task_name: [method_names]}
        
        # Collect all tasks from methods
        for method_name, method_def in self.methods.items():
            task_name = method_def.get('task', 'unknown')
            if task_name not in tasks:
                tasks[task_name] = []
            tasks[task_name].append(method_name)
        
        # Create nodes for tasks
        node_id = 0
        task_nodes = {}
        for task_name in tasks:
            node_id += 1
            self.nodes[node_id] = {
                'type': 'task',
                'name': task_name,
                'x': 0,
                'y': 0
            }
            task_nodes[task_name] = node_id
        
        # Create nodes for methods and connect to tasks
        method_nodes = {}
        for method_name, method_def in self.methods.items():
            node_id += 1
            self.nodes[node_id] = {
                'type': 'method',
                'name': method_name,
                'x': 0,
                'y': 0,
                'def': method_def
            }
            method_nodes[method_name] = node_id
            
            # Connect task to method
            task_name = method_def.get('task', 'unknown')
            if task_name in task_nodes:
                self.edges.append((task_nodes[task_name], node_id))
        
        # Create nodes for operators
        operator_nodes = {}
        for op_name in self.operators:
            node_id += 1
            self.nodes[node_id] = {
                'type': 'operator',
                'name': op_name,
                'x': 0,
                'y': 0
            }
            operator_nodes[op_name] = node_id
        
        # Parse subtasks and create edges from methods to subtasks
        for method_name, method_def in self.methods.items():
            method_node = method_nodes[method_name]
            subtasks_str = method_def.get('subtasks', '[]')
            
            try:
                # Parse subtasks - handle different formats
                if subtasks_str.strip():
                    # Try to evaluate as Python expression
                    namespace = {'state': type('obj', (object,), {'pos': {}})()}
                    subtasks = eval(subtasks_str, {"__builtins__": {}}, namespace)
                    
                    if isinstance(subtasks, list):
                        for subtask in subtasks:
                            if isinstance(subtask, tuple) and len(subtask) > 0:
                                subtask_name = subtask[0]
                                
                                # Connect method to subtask (task or operator)
                                if subtask_name in task_nodes:
                                    self.edges.append((method_node, task_nodes[subtask_name]))
                                elif subtask_name in operator_nodes:
                                    self.edges.append((method_node, operator_nodes[subtask_name]))
                                else:
                                    # Create a placeholder node for unknown subtasks
                                    node_id += 1
                                    self.nodes[node_id] = {
                                        'type': 'unknown',
                                        'name': f"{subtask_name}(?)",
                                        'x': 0,
                                        'y': 0
                                    }
                                    self.edges.append((method_node, node_id))
            except Exception as e:
                # If parsing fails, try to extract task names with regex
                import re
                matches = re.findall(r'\("([^"]+)"', subtasks_str)
                for match in matches:
                    if match in task_nodes:
                        self.edges.append((method_node, task_nodes[match]))
                    elif match in operator_nodes:
                        self.edges.append((method_node, operator_nodes[match]))
        
        # Add edges for operators that might be referenced in other ways
        self._add_implicit_edges(task_nodes, method_nodes, operator_nodes)
        
        # Calculate positions
        self.calculate_positions()
    
    def _add_implicit_edges(self, task_nodes, method_nodes, operator_nodes):
        """Add edges for implicit relationships"""
        # Look for operator references in method parameters or conditions
        for method_name, method_def in self.methods.items():
            method_node = method_nodes[method_name]
            
            # Check preconditions for operator references
            precond = method_def.get('preconditions', '')
            for op_name in self.operators:
                if op_name in precond and op_name in operator_nodes:
                    # Add a dashed edge to show dependency
                    self.edges.append((method_node, operator_nodes[op_name], 'dependency'))

    def calculate_positions(self):
        """Calculate node positions based on selected layout"""
        layout = self.layout_var.get()
        
        if layout == "hierarchical":
            self.hierarchical_layout()
        elif layout == "circular":
            self.circular_layout()
        else:  # force-directed
            self.force_directed_layout()
    
    def _edge_endpoints(self, edge):
        """Extract (from_node, to_node) from an edge regardless of tuple length."""
        return edge[0], edge[1]

    def hierarchical_layout(self):
        """Arrange nodes in hierarchical layers"""
        # Find root tasks (tasks with no incoming edges)
        incoming = {node_id: 0 for node_id in self.nodes}
        for edge in self.edges:
            _, to_node = self._edge_endpoints(edge)
            incoming[to_node] += 1
        
        roots = [node_id for node_id, count in incoming.items() if count == 0]
        
        # BFS to assign levels
        levels = {}
        visited = set()
        queue = [(root, 0) for root in roots]
        
        while queue:
            node_id, level = queue.pop(0)
            if node_id in visited:
                continue
            
            visited.add(node_id)
            if level not in levels:
                levels[level] = []
            levels[level].append(node_id)
            
            # Add children to queue
            for edge in self.edges:
                from_node, to_node = self._edge_endpoints(edge)
                if from_node == node_id and to_node not in visited:
                    queue.append((to_node, level + 1))
        
        # Position nodes
        y_offset = 50
        for level, nodes_in_level in levels.items():
            x_offset = 50
            total_width = len(nodes_in_level) * (self.node_width + self.node_spacing)
            x_start = (800 - total_width) / 2  # Center on 800px width
            
            for i, node_id in enumerate(nodes_in_level):
                self.nodes[node_id]['x'] = x_start + i * (self.node_width + self.node_spacing)
                self.nodes[node_id]['y'] = y_offset + level * self.level_height
    
    def circular_layout(self):
        """Arrange nodes in circular layout"""
        center_x, center_y = 400, 300
        radius = 200
        
        # Separate nodes by type
        tasks = [n for n, info in self.nodes.items() if info['type'] == 'task']
        methods = [n for n, info in self.nodes.items() if info['type'] == 'method']
        operators = [n for n, info in self.nodes.items() if info['type'] == 'operator']
        
        # Place tasks in inner circle
        for i, node_id in enumerate(tasks):
            angle = 2 * math.pi * i / len(tasks) if tasks else 0
            self.nodes[node_id]['x'] = center_x + radius * 0.5 * math.cos(angle)
            self.nodes[node_id]['y'] = center_y + radius * 0.5 * math.sin(angle)
        
        # Place methods in middle circle
        for i, node_id in enumerate(methods):
            angle = 2 * math.pi * i / len(methods) if methods else 0
            self.nodes[node_id]['x'] = center_x + radius * math.cos(angle)
            self.nodes[node_id]['y'] = center_y + radius * math.sin(angle)
        
        # Place operators in outer circle
        for i, node_id in enumerate(operators):
            angle = 2 * math.pi * i / len(operators) if operators else 0
            self.nodes[node_id]['x'] = center_x + radius * 1.5 * math.cos(angle)
            self.nodes[node_id]['y'] = center_y + radius * 1.5 * math.sin(angle)
    
    def force_directed_layout(self):
        """Simple force-directed layout"""
        # Initialize random positions
        import random
        for node_id in self.nodes:
            self.nodes[node_id]['x'] = random.randint(100, 700)
            self.nodes[node_id]['y'] = random.randint(100, 500)
        
        # Apply forces for several iterations
        iterations = 50
        for _ in range(iterations):
            forces = {node_id: {'x': 0, 'y': 0} for node_id in self.nodes}
            
            # Repulsive forces between all nodes
            for n1 in self.nodes:
                for n2 in self.nodes:
                    if n1 != n2:
                        dx = self.nodes[n2]['x'] - self.nodes[n1]['x']
                        dy = self.nodes[n2]['y'] - self.nodes[n1]['y']
                        dist = math.sqrt(dx*dx + dy*dy)
                        if dist > 0:
                            force = 1000 / (dist * dist)
                            forces[n1]['x'] -= force * dx / dist
                            forces[n1]['y'] -= force * dy / dist
            
            # Attractive forces along edges
            for edge in self.edges:
                from_node, to_node = self._edge_endpoints(edge)
                dx = self.nodes[to_node]['x'] - self.nodes[from_node]['x']
                dy = self.nodes[to_node]['y'] - self.nodes[from_node]['y']
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0:
                    force = dist / 100
                    forces[from_node]['x'] += force * dx / dist
                    forces[from_node]['y'] += force * dy / dist
                    forces[to_node]['x'] -= force * dx / dist
                    forces[to_node]['y'] -= force * dy / dist
            
            # Apply forces
            for node_id in self.nodes:
                self.nodes[node_id]['x'] += forces[node_id]['x'] * 0.1
                self.nodes[node_id]['y'] += forces[node_id]['y'] * 0.1
                
                # Keep within bounds
                self.nodes[node_id]['x'] = max(50, min(750, self.nodes[node_id]['x']))
                self.nodes[node_id]['y'] = max(50, min(550, self.nodes[node_id]['y']))
    
    def draw_network(self):
        """Draw the network on canvas"""
        self.canvas.delete('all')
        
        # Draw edges first
        for edge_info in self.edges:
            if len(edge_info) == 2:
                from_node, to_node = edge_info
                edge_type = 'normal'
            else:
                from_node, to_node, edge_type = edge_info
                
            if from_node in self.nodes and to_node in self.nodes:
                x1 = self.nodes[from_node]['x'] + self.node_width/2
                y1 = self.nodes[from_node]['y'] + self.node_height/2
                x2 = self.nodes[to_node]['x'] + self.node_width/2
                y2 = self.nodes[to_node]['y'] + self.node_height/2
                
                # Apply zoom
                x1, y1 = x1 * self.zoom_level, y1 * self.zoom_level
                x2, y2 = x2 * self.zoom_level, y2 * self.zoom_level
                
                # Draw arrow with different styles
                self.draw_arrow(x1, y1, x2, y2, edge_type)
        
        # Draw nodes
        for node_id, node_info in self.nodes.items():
            self.draw_node(node_id, node_info)
        
        # Update scroll region
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))
    
    def draw_node(self, node_id, node_info):
        """Draw a single node"""
        node_type = node_info['type']
        
        # Check visibility
        if node_type == 'method' and not self.show_methods.get():
            return
        if node_type == 'operator' and not self.show_operators.get():
            return
        
        x = node_info['x'] * self.zoom_level
        y = node_info['y'] * self.zoom_level
        w = self.node_width * self.zoom_level
        h = self.node_height * self.zoom_level
        
        # Choose color
        if node_type == 'unknown':
            color = '#CCCCCC'  # Gray for unknown nodes
        else:
            color = self.colors.get(node_type, '#CCCCCC')
            
        if node_id == self.selected_node:
            color = self.colors['selected']
        
        # Draw shape based on type
        if node_type == 'task':
            # Rectangle for tasks
            rect = self.canvas.create_rectangle(
                x, y, x + w, y + h,
                fill=color, outline='black', width=2,
                tags=('node', f'node_{node_id}')
            )
        elif node_type == 'method':
            # Rounded rectangle for methods
            rect = self.create_rounded_rectangle(
                x, y, x + w, y + h, 10,
                fill=color, outline='black', width=2,
                tags=('node', f'node_{node_id}')
            )
        elif node_type == 'operator':
            # Hexagon for operators
            points = []
            cx, cy = x + w/2, y + h/2
            for i in range(6):
                angle = math.pi / 3 * i
                px = cx + w/2 * math.cos(angle)
                py = cy + h/2.5 * math.sin(angle)
                points.extend([px, py])
            
            rect = self.canvas.create_polygon(
                points,
                fill=color, outline='black', width=2,
                tags=('node', f'node_{node_id}')
            )
        else:  # unknown
            # Diamond for unknown nodes
            points = [x + w/2, y, x + w, y + h/2, x + w/2, y + h, x, y + h/2]
            rect = self.canvas.create_polygon(
                points,
                fill=color, outline='black', width=2, dash=(5, 5),
                tags=('node', f'node_{node_id}')
            )
        
        # Draw text with scaled font
        s = ScaleManager.s
        font_size = max(8, int(s(12) * self.zoom_level))
        text = self.canvas.create_text(
            x + w/2, y + h/2,
            text=node_info['name'],
            fill='white' if node_type != 'unknown' else 'black',
            font=(AppleFonts.FAMILY, font_size, 'bold'),
            width=w * 0.9,
            tags=('node', f'node_{node_id}')
        )
        
        # Bind events
        for item in [rect, text]:
            self.canvas.tag_bind(item, '<Button-1>', lambda e, nid=node_id: self.on_node_click(nid))
            self.canvas.tag_bind(item, '<Double-Button-1>', lambda e, nid=node_id: self.on_node_double_click(nid))
    
    def create_rounded_rectangle(self, x1, y1, x2, y2, radius, **kwargs):
        """Create a rounded rectangle"""
        points = []
        for x, y in [(x1, y1 + radius), (x1, y1), (x1 + radius, y1),
                     (x2 - radius, y1), (x2, y1), (x2, y1 + radius),
                     (x2, y2 - radius), (x2, y2), (x2 - radius, y2),
                     (x1 + radius, y2), (x1, y2), (x1, y2 - radius)]:
            points.extend([x, y])
        
        return self.canvas.create_polygon(points, smooth=True, **kwargs)
    
    def draw_arrow(self, x1, y1, x2, y2, edge_type='normal'):
        """Draw an arrow from (x1,y1) to (x2,y2)"""
        # Calculate arrow head position
        angle = math.atan2(y2 - y1, x2 - x1)
        arrow_length = 15 * self.zoom_level
        
        # Shorten line to not overlap with node
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        if dist > 0:
            shorten = (self.node_height * self.zoom_level) / 2
            x2 = x2 - (shorten * (x2 - x1) / dist)
            y2 = y2 - (shorten * (y2 - y1) / dist)
        
        # Choose line style based on edge type
        if edge_type == 'dependency':
            # Dashed line for dependencies
            line = self.canvas.create_line(
                x1, y1, x2, y2,
                fill='#999999',
                width=1 * self.zoom_level,
                dash=(5, 5),
                arrow=tk.LAST,
                arrowshape=(arrow_length, arrow_length, arrow_length/2)
            )
        else:
            # Solid line for normal edges
            line = self.canvas.create_line(
                x1, y1, x2, y2,
                fill=self.colors['edge'],
                width=2 * self.zoom_level,
                arrow=tk.LAST,
                arrowshape=(arrow_length, arrow_length, arrow_length/2)
            )

    def on_mouse_press(self, event):
        """Handle mouse press"""
        self.drag_start = (event.x, event.y)
    
    def on_mouse_drag(self, event):
        """Handle mouse drag for panning"""
        if self.drag_start:
            dx = event.x - self.drag_start[0]
            dy = event.y - self.drag_start[1]
            
            self.canvas.scan_dragto(event.x, event.y, gain=1)
            self.drag_start = (event.x, event.y)
    
    def on_mouse_wheel(self, event, delta=None):
        """Handle mouse wheel for zooming"""
        if delta is None:
            delta = event.delta
        
        # Zoom in or out
        if delta > 0:
            self.zoom_in()
        else:
            self.zoom_out()
    
    def on_node_click(self, node_id):
        """Handle node click"""
        self.selected_node = node_id
        self.draw_network()
    
    def on_node_double_click(self, node_id):
        """Handle node double-click - show details"""
        node_info = self.nodes[node_id]
        
        # Create detail window
        detail_window = tk.Toplevel(self.parent)
        detail_window.title(f"Node Details: {node_info['name']}")
        detail_window.geometry("500x400")
        
        # Show node information
        text = tk.Text(detail_window, wrap=tk.WORD)
        text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        text.insert(tk.END, f"Name: {node_info['name']}\n")
        text.insert(tk.END, f"Type: {node_info['type']}\n\n")
        
        if node_info['type'] == 'method' and 'def' in node_info:
            method_def = node_info['def']
            text.insert(tk.END, f"Task: {method_def.get('task', 'N/A')}\n")
            text.insert(tk.END, f"Parameters: {', '.join(method_def.get('params', []))}\n")
            text.insert(tk.END, f"\nPreconditions:\n{method_def.get('preconditions', 'N/A')}\n")
            text.insert(tk.END, f"\nSubtasks:\n{method_def.get('subtasks', 'N/A')}\n")
        elif node_info['type'] == 'operator' and node_info['name'] in self.operators:
            op_def = self.operators[node_info['name']]
            text.insert(tk.END, f"Parameters: {', '.join(op_def.get('params', []))}\n")
            text.insert(tk.END, f"\nPreconditions:\n{op_def.get('preconditions', 'N/A')}\n")
            text.insert(tk.END, f"\nEffects:\n{op_def.get('effects', 'N/A')}\n")
        
        # Show connections
        text.insert(tk.END, "\n\nConnections:\n")
        text.insert(tk.END, "Incoming edges from:\n")
        for from_node, to_node, *_ in self.edges:
            if to_node == node_id and from_node in self.nodes:
                text.insert(tk.END, f"  ← {self.nodes[from_node]['name']} ({self.nodes[from_node]['type']})\n")
        
        text.insert(tk.END, "\nOutgoing edges to:\n")
        for from_node, to_node, *_ in self.edges:
            if from_node == node_id and to_node in self.nodes:
                text.insert(tk.END, f"  → {self.nodes[to_node]['name']} ({self.nodes[to_node]['type']})\n")
        
        text.config(state='disabled')
    
    def zoom_in(self):
        """Zoom in"""
        self.zoom_level *= 1.2
        self.draw_network()
    
    def zoom_out(self):
        """Zoom out"""
        self.zoom_level /= 1.2
        self.draw_network()
    
    def reset_zoom(self):
        """Reset zoom to 100%"""
        self.zoom_level = 1.0
        self.draw_network()
    
    def update_layout(self):
        """Update layout when changed"""
        self.calculate_positions()
        self.draw_network()
    
    def update_visibility(self):
        """Update node visibility"""
        self.draw_network()
    
    def export_image(self):
        """Export canvas as PostScript"""
        from tkinter import filedialog
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".ps",
            filetypes=[("PostScript files", "*.ps"), ("All files", "*.*")]
        )
        
        if filename:
            self.canvas.postscript(file=filename)
            tk.messagebox.showinfo("Export", f"Network exported to {filename}")
    
    def update_data(self, methods, operators):
        """Update with new methods and operators"""
        self.methods = methods
        self.operators = operators
        self.build_network()
        self.draw_network()
