"""
HTN Planner GUI - Apple-style Visual Interface for HTN Planning
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import json
import os
from typing import Dict, List, Any
import threading
import queue
import sys
from pathlib import Path
from datetime import datetime

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from robot_htn import htn, State
from robot_htn.helpers import print_state, print_operators, print_methods
from robot_htn.visualizer import HTNVisualizer
from robot_htn.api import HTNConfigAPI


# ============================================================================
# Apple Design System Colors & Styles
# ============================================================================
class AppleColors:
    """Apple Human Interface Guidelines inspired color palette"""
    # Primary backgrounds
    BG_PRIMARY = "#FFFFFF"
    BG_SECONDARY = "#F5F5F7"
    BG_TERTIARY = "#E8E8ED"
    
    # Sidebar & navigation
    SIDEBAR_BG = "#F5F5F7"
    SIDEBAR_HOVER = "#E8E8ED"
    SIDEBAR_SELECTED = "#007AFF"
    
    # Text colors
    TEXT_PRIMARY = "#1D1D1F"
    TEXT_SECONDARY = "#86868B"
    TEXT_TERTIARY = "#AEAEB2"
    TEXT_ON_ACCENT = "#FFFFFF"
    
    # Accent colors (Apple System Blue)
    ACCENT_BLUE = "#007AFF"
    ACCENT_BLUE_HOVER = "#0056CC"
    ACCENT_BLUE_LIGHT = "#E3F2FF"
    
    # Status colors
    SUCCESS = "#34C759"
    WARNING = "#FF9F0A"
    ERROR = "#FF3B30"
    
    # Borders & separators
    BORDER = "#D2D2D7"
    BORDER_LIGHT = "#E5E5EA"
    SEPARATOR = "#C6C6C8"
    
    # Input fields
    INPUT_BG = "#FFFFFF"
    INPUT_BORDER = "#D2D2D7"
    INPUT_FOCUS = "#007AFF"
    
    # Cards & elevated surfaces
    CARD_BG = "#FFFFFF"
    CARD_SHADOW = "#00000014"


class ScaleManager:
    """Global scale manager for proportional UI scaling"""
    # Reference window size
    REF_WIDTH = 1400
    REF_HEIGHT = 900
    
    # Current scale factor
    _scale = 1.0
    
    # Registered callbacks for scale updates
    _callbacks = []
    
    @classmethod
    def update_scale(cls, window_width, window_height):
        """Update scale based on window size"""
        # Calculate scale based on window diagonal for proportional scaling
        ref_diagonal = (cls.REF_WIDTH ** 2 + cls.REF_HEIGHT ** 2) ** 0.5
        current_diagonal = (window_width ** 2 + window_height ** 2) ** 0.5
        
        new_scale = current_diagonal / ref_diagonal
        new_scale = max(0.6, min(2.0, new_scale))  # Limit scale between 0.6x and 2.0x
        
        # Only update if scale changed significantly
        if abs(new_scale - cls._scale) > 0.03:
            cls._scale = new_scale
            cls._notify_callbacks()
            return True
        return False
    
    @classmethod
    def get_scale(cls):
        return cls._scale
    
    @classmethod
    def s(cls, base_value):
        """Get scaled value (shorthand)"""
        return int(base_value * cls._scale)
    
    @classmethod
    def register_callback(cls, callback):
        """Register a callback to be called when scale changes"""
        cls._callbacks.append(callback)
    
    @classmethod
    def unregister_callback(cls, callback):
        """Unregister a callback"""
        if callback in cls._callbacks:
            cls._callbacks.remove(callback)
    
    @classmethod
    def _notify_callbacks(cls):
        """Notify all registered callbacks of scale change"""
        for callback in cls._callbacks:
            try:
                callback()
            except:
                pass


class AppleFonts:
    """Apple-style typography (using system fonts) with dynamic scaling"""
    # On Windows, use Segoe UI as fallback for SF Pro
    FAMILY = "Segoe UI" if sys.platform == "win32" else "SF Pro Display"
    FAMILY_MONO = "Cascadia Code" if sys.platform == "win32" else "SF Mono"
    
    # Base font sizes (at reference resolution)
    BASE_SIZES = {
        'title_large': 32,
        'title': 24,
        'headline': 18,
        'body': 14,
        'caption': 12,
        'code': 14,
        'code_small': 13
    }
    
    # Font configurations - will be updated dynamically
    TITLE_LARGE = (FAMILY, 32, "bold")
    TITLE = (FAMILY, 24, "bold")
    HEADLINE = (FAMILY, 18, "bold")
    BODY = (FAMILY, 14)
    BODY_BOLD = (FAMILY, 14, "bold")
    CAPTION = (FAMILY, 12)
    CAPTION_BOLD = (FAMILY, 12, "bold")
    CODE = (FAMILY_MONO, 14)
    CODE_SMALL = (FAMILY_MONO, 13)
    
    @classmethod
    def update_fonts(cls):
        """Update all font tuples with current scale"""
        scale = ScaleManager.get_scale()
        
        cls.TITLE_LARGE = (cls.FAMILY, max(16, int(cls.BASE_SIZES['title_large'] * scale)), "bold")
        cls.TITLE = (cls.FAMILY, max(14, int(cls.BASE_SIZES['title'] * scale)), "bold")
        cls.HEADLINE = (cls.FAMILY, max(12, int(cls.BASE_SIZES['headline'] * scale)), "bold")
        cls.BODY = (cls.FAMILY, max(10, int(cls.BASE_SIZES['body'] * scale)))
        cls.BODY_BOLD = (cls.FAMILY, max(10, int(cls.BASE_SIZES['body'] * scale)), "bold")
        cls.CAPTION = (cls.FAMILY, max(9, int(cls.BASE_SIZES['caption'] * scale)))
        cls.CAPTION_BOLD = (cls.FAMILY, max(9, int(cls.BASE_SIZES['caption'] * scale)), "bold")
        cls.CODE = (cls.FAMILY_MONO, max(10, int(cls.BASE_SIZES['code'] * scale)))
        cls.CODE_SMALL = (cls.FAMILY_MONO, max(9, int(cls.BASE_SIZES['code_small'] * scale)))
    
    @classmethod
    def get_scaled_size(cls, base_size):
        """Get a scaled size value"""
        return ScaleManager.s(base_size)


# ============================================================================
# Custom Styled Widgets
# ============================================================================
class AppleButton(tk.Canvas):
    """Custom Apple-style button with rounded corners and scaling support"""
    
    def __init__(self, parent, text="", command=None, style="default", width=None, **kwargs):
        self.style = style
        self.command = command
        self.text = text
        self.base_width = width  # Store base width for scaling
        self.parent = parent
        
        # Calculate initial dimensions with scaling
        self._calculate_dimensions()
        
        super().__init__(parent, width=self._btn_width, height=self._btn_height, 
                        highlightthickness=0, bg=AppleColors.BG_PRIMARY, **kwargs)
        
        # Style configurations
        self.styles = {
            "default": {
                "bg": AppleColors.BG_TERTIARY,
                "bg_hover": AppleColors.BORDER,
                "fg": AppleColors.TEXT_PRIMARY,
                "border": AppleColors.BORDER
            },
            "primary": {
                "bg": AppleColors.ACCENT_BLUE,
                "bg_hover": AppleColors.ACCENT_BLUE_HOVER,
                "fg": AppleColors.TEXT_ON_ACCENT,
                "border": AppleColors.ACCENT_BLUE
            },
            "destructive": {
                "bg": AppleColors.ERROR,
                "bg_hover": "#CC2F26",
                "fg": AppleColors.TEXT_ON_ACCENT,
                "border": AppleColors.ERROR
            }
        }
        
        self._draw_button()
        
        # Bindings
        self.bind("<Enter>", self._on_hover)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_click)
        self.bind("<ButtonRelease-1>", self._on_release)
        
        # Register for scale updates
        ScaleManager.register_callback(self._on_scale_change)
    
    def _calculate_dimensions(self):
        """Calculate button dimensions based on current scale"""
        s = ScaleManager.s
        font = AppleFonts.BODY_BOLD
        temp_label = tk.Label(self.parent, text=self.text, font=font)
        temp_label.update_idletasks()
        text_width = temp_label.winfo_reqwidth()
        temp_label.destroy()
        
        if self.base_width:
            self._btn_width = s(self.base_width)
        else:
            # Ensure minimum width with extra padding for scaled fonts
            self._btn_width = max(text_width + s(40), s(80))
        self._btn_height = s(38)
    
    def _on_scale_change(self):
        """Handle scale change"""
        self._calculate_dimensions()
        self.config(width=self._btn_width, height=self._btn_height)
        self._draw_button()
        
    def _draw_button(self, hover=False):
        self.delete("all")
        
        w = self.winfo_reqwidth()
        h = self.winfo_reqheight()
        r = ScaleManager.s(8)  # Scaled corner radius
        
        style_config = self.styles.get(self.style, self.styles["default"])
        bg = style_config["bg_hover"] if hover else style_config["bg"]
        fg = style_config["fg"]
        
        # Draw rounded rectangle
        self._create_rounded_rect(2, 2, w-2, h-2, r, fill=bg, outline="")
        
        # Draw text with current font
        self.create_text(w//2, h//2, text=self.text, fill=fg, font=AppleFonts.BODY_BOLD)
        
    def _create_rounded_rect(self, x1, y1, x2, y2, r, **kwargs):
        """Create a rounded rectangle"""
        points = [
            x1+r, y1, x2-r, y1, x2, y1, x2, y1+r,
            x2, y2-r, x2, y2, x2-r, y2, x1+r, y2,
            x1, y2, x1, y2-r, x1, y1+r, x1, y1, x1+r, y1
        ]
        return self.create_polygon(points, smooth=True, **kwargs)
    
    def _on_hover(self, event):
        self._draw_button(hover=True)
        self.config(cursor="hand2")
        
    def _on_leave(self, event):
        self._draw_button(hover=False)
        self.config(cursor="")
        
    def _on_click(self, event):
        pass
        
    def _on_release(self, event):
        if self.command:
            self.command()
            
    def configure(self, **kwargs):
        if "state" in kwargs:
            state = kwargs.pop("state")
            if state == "disabled":
                self.unbind("<Button-1>")
                self.config(cursor="")
            else:
                self.bind("<Button-1>", self._on_click)
        super().configure(**kwargs)
        
    config = configure


class AppleEntry(tk.Frame):
    """Custom Apple-style entry with rounded border and scaling support"""
    
    def __init__(self, parent, textvariable=None, width=30, placeholder="", **kwargs):
        super().__init__(parent, bg=AppleColors.BG_PRIMARY)
        
        self.placeholder = placeholder
        self.placeholder_shown = False
        self.base_width = width
        
        # Create inner frame for border effect
        self.border_frame = tk.Frame(self, bg=AppleColors.INPUT_BORDER, padx=1, pady=1)
        self.border_frame.pack(fill=tk.BOTH, expand=True)
        
        self.inner_frame = tk.Frame(self.border_frame, bg=AppleColors.INPUT_BG)
        self.inner_frame.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)
        
        # Entry widget
        self.entry = tk.Entry(
            self.inner_frame, 
            textvariable=textvariable,
            font=AppleFonts.BODY,
            bg=AppleColors.INPUT_BG,
            fg=AppleColors.TEXT_PRIMARY,
            insertbackground=AppleColors.TEXT_PRIMARY,
            relief="flat",
            width=width,
            highlightthickness=0,
            **kwargs
        )
        self._update_padding()
        
        # Register for scale updates
        ScaleManager.register_callback(self._on_scale_change)
    
    def _update_padding(self):
        """Update padding based on current scale"""
        s = ScaleManager.s
        self.entry.pack_forget()
        self.entry.pack(padx=s(12), pady=s(10), fill=tk.BOTH, expand=True)
    
    def _on_scale_change(self):
        """Handle scale change"""
        self.entry.config(font=AppleFonts.BODY)
        self._update_padding()
        
        # Bindings for focus effects
        self.entry.bind("<FocusIn>", self._on_focus_in)
        self.entry.bind("<FocusOut>", self._on_focus_out)
        
        # Placeholder handling
        if placeholder:
            self._show_placeholder()
            self.entry.bind("<FocusIn>", self._on_focus_in_placeholder, add="+")
            self.entry.bind("<FocusOut>", self._on_focus_out_placeholder, add="+")
    
    def _on_focus_in(self, event):
        self.border_frame.config(bg=AppleColors.ACCENT_BLUE)
        
    def _on_focus_out(self, event):
        self.border_frame.config(bg=AppleColors.INPUT_BORDER)
        
    def _show_placeholder(self):
        if not self.entry.get():
            self.entry.insert(0, self.placeholder)
            self.entry.config(fg=AppleColors.TEXT_TERTIARY)
            self.placeholder_shown = True
            
    def _on_focus_in_placeholder(self, event):
        if self.placeholder_shown:
            self.entry.delete(0, tk.END)
            self.entry.config(fg=AppleColors.TEXT_PRIMARY)
            self.placeholder_shown = False
            
    def _on_focus_out_placeholder(self, event):
        self._show_placeholder()
        
    def get(self):
        if self.placeholder_shown:
            return ""
        return self.entry.get()
    
    def insert(self, index, string):
        if self.placeholder_shown:
            self.entry.delete(0, tk.END)
            self.entry.config(fg=AppleColors.TEXT_PRIMARY)
            self.placeholder_shown = False
        self.entry.insert(index, string)
        
    def delete(self, first, last=None):
        self.entry.delete(first, last)


class AppleCodeEditor(tk.Frame):
    """Apple-style code editor with syntax highlighting appearance and scaling support"""
    
    def __init__(self, parent, height=8, **kwargs):
        super().__init__(parent, bg=AppleColors.BORDER, padx=1, pady=1)
        
        self.base_height = height
        s = ScaleManager.s
        
        self.text = tk.Text(
            self,
            font=AppleFonts.CODE,
            bg="#1E1E1E",  # Dark code editor background
            fg="#D4D4D4",  # Light text
            insertbackground="#FFFFFF",
            relief="flat",
            height=height,
            wrap=tk.NONE,
            padx=s(14),
            pady=s(12),
            highlightthickness=0,
            **kwargs
        )
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Register for scale updates
        ScaleManager.register_callback(self._on_scale_change)
    
    def _on_scale_change(self):
        """Handle scale change"""
        s = ScaleManager.s
        self.text.config(font=AppleFonts.CODE, padx=s(14), pady=s(12))
        
    def get(self, start, end):
        return self.text.get(start, end)
    
    def insert(self, index, chars):
        self.text.insert(index, chars)
        
    def delete(self, start, end=None):
        self.text.delete(start, end)


class AppleCard(tk.Frame):
    """Apple-style card container with subtle shadow effect and scaling support"""
    
    def __init__(self, parent, title=None, **kwargs):
        super().__init__(parent, bg=AppleColors.BG_PRIMARY, **kwargs)
        
        s = ScaleManager.s
        
        # Main card container
        self.card = tk.Frame(self, bg=AppleColors.CARD_BG, padx=s(20), pady=s(16))
        self.card.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        
        # Store title label reference for scaling
        self.title_label = None
        
        # Optional title
        if title:
            self.title_label = tk.Label(
                self.card, 
                text=title, 
                font=AppleFonts.HEADLINE,
                bg=AppleColors.CARD_BG,
                fg=AppleColors.TEXT_PRIMARY,
                anchor="w"
            )
            self.title_label._font_type = 'headline'
            self.title_label.pack(fill=tk.X, pady=(0, s(12)))
            
            # Separator
            self.sep = tk.Frame(self.card, bg=AppleColors.SEPARATOR, height=1)
            self.sep.pack(fill=tk.X, pady=(0, s(12)))
        
        # Content frame
        self.content = tk.Frame(self.card, bg=AppleColors.CARD_BG)
        self.content.pack(fill=tk.BOTH, expand=True)
        
        # Register for scale updates
        ScaleManager.register_callback(self._on_scale_change)
    
    def _on_scale_change(self):
        """Handle scale change"""
        s = ScaleManager.s
        self.card.config(padx=s(20), pady=s(16))
        if self.title_label:
            self.title_label.config(font=AppleFonts.HEADLINE)


# ============================================================================
# Main Application
# ============================================================================
class HTNPlannerGUI:
    """Main GUI application for HTN Planner - Apple Style"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("HTN Planner")
        self.root.geometry("1400x900")
        self.root.configure(bg=AppleColors.BG_PRIMARY)
        
        # Configure minimum size
        self.root.minsize(1200, 700)
        
        # Initialize API
        self.api = HTNConfigAPI()
        
        # Data storage
        self.operators = {}
        self.methods = {}
        self.current_state = None
        self.plan_queue = queue.Queue()
        
        # File management
        self.is_modified = False
        self.recent_files = self._load_recent_files()
        
        # Configure ttk styles for Apple look
        self._configure_styles()
        
        # Create GUI components
        self._create_header()
        self._create_menu()
        self._create_main_layout()
        
        # Set window close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Load example data
        self._load_example_data()
        
        # Bind window resize event for font scaling
        self._resize_job = None
        self.root.bind('<Configure>', self._on_window_resize)
        
    def _configure_styles(self):
        """Configure ttk styles for Apple appearance"""
        style = ttk.Style()
        
        # Use clam theme as base (only set once)
        try:
            current_theme = style.theme_use()
            if current_theme != 'clam':
                style.theme_use('clam')
        except:
            style.theme_use('clam')
        
        # Configure Notebook (tabs) - use current scaled fonts
        tab_padding = [AppleFonts.get_scaled_size(20), AppleFonts.get_scaled_size(10)]
        style.configure("TNotebook", 
                       background=AppleColors.BG_PRIMARY,
                       borderwidth=0)
        style.configure("TNotebook.Tab", 
                       background=AppleColors.BG_SECONDARY,
                       foreground=AppleColors.TEXT_SECONDARY,
                       padding=tab_padding,
                       font=AppleFonts.BODY_BOLD)
        style.map("TNotebook.Tab",
                 background=[("selected", AppleColors.BG_PRIMARY)],
                 foreground=[("selected", AppleColors.TEXT_PRIMARY)])
        
        # Configure Frame
        style.configure("TFrame", background=AppleColors.BG_PRIMARY)
        style.configure("Card.TFrame", background=AppleColors.CARD_BG)
        style.configure("Sidebar.TFrame", background=AppleColors.SIDEBAR_BG)
        
        # Configure Label
        style.configure("TLabel", 
                       background=AppleColors.BG_PRIMARY,
                       foreground=AppleColors.TEXT_PRIMARY,
                       font=AppleFonts.BODY)
        style.configure("Title.TLabel", font=AppleFonts.TITLE)
        style.configure("Headline.TLabel", font=AppleFonts.HEADLINE)
        style.configure("Caption.TLabel", 
                       font=AppleFonts.CAPTION,
                       foreground=AppleColors.TEXT_SECONDARY)
        
        # Configure Button - use scaled padding
        btn_padding = [AppleFonts.get_scaled_size(16), AppleFonts.get_scaled_size(8)]
        style.configure("TButton",
                       background=AppleColors.BG_TERTIARY,
                       foreground=AppleColors.TEXT_PRIMARY,
                       padding=btn_padding,
                       font=AppleFonts.BODY_BOLD)
        style.map("TButton",
                 background=[("active", AppleColors.BORDER)])
        
        # Primary button
        style.configure("Primary.TButton",
                       background=AppleColors.ACCENT_BLUE,
                       foreground=AppleColors.TEXT_ON_ACCENT)
        style.map("Primary.TButton",
                 background=[("active", AppleColors.ACCENT_BLUE_HOVER)])
        
        # Configure Entry - use scaled padding
        s = ScaleManager.s
        entry_padding = [s(10), s(8)]
        style.configure("TEntry",
                       fieldbackground=AppleColors.INPUT_BG,
                       foreground=AppleColors.TEXT_PRIMARY,
                       padding=entry_padding)
        
        # Configure Listbox-like Treeview - scaled row height
        style.configure("Treeview",
                       background=AppleColors.BG_PRIMARY,
                       foreground=AppleColors.TEXT_PRIMARY,
                       fieldbackground=AppleColors.BG_PRIMARY,
                       font=AppleFonts.BODY,
                       rowheight=s(42))
        style.map("Treeview",
                 background=[("selected", AppleColors.ACCENT_BLUE)],
                 foreground=[("selected", AppleColors.TEXT_ON_ACCENT)])
        
        # Configure LabelFrame
        style.configure("TLabelframe",
                       background=AppleColors.BG_PRIMARY,
                       foreground=AppleColors.TEXT_PRIMARY)
        style.configure("TLabelframe.Label", 
                       font=AppleFonts.HEADLINE,
                       foreground=AppleColors.TEXT_PRIMARY,
                       background=AppleColors.BG_PRIMARY)
        
        # Configure Progressbar
        style.configure("TProgressbar",
                       background=AppleColors.ACCENT_BLUE,
                       troughcolor=AppleColors.BG_TERTIARY,
                       borderwidth=0,
                       lightcolor=AppleColors.ACCENT_BLUE,
                       darkcolor=AppleColors.ACCENT_BLUE)
        
        # Configure Spinbox - scaled arrow size
        style.configure("TSpinbox",
                       fieldbackground=AppleColors.INPUT_BG,
                       foreground=AppleColors.TEXT_PRIMARY,
                       arrowsize=s(12))
        
        # Configure Scrollbar
        style.configure("TScrollbar",
                       background=AppleColors.BG_TERTIARY,
                       troughcolor=AppleColors.BG_PRIMARY,
                       borderwidth=0,
                       arrowsize=0)
    
    def _create_header(self):
        """Create app header with title"""
        s = ScaleManager.s
        self.header_frame = tk.Frame(self.root, bg=AppleColors.BG_PRIMARY, height=s(70))
        self.header_frame.pack(fill=tk.X, padx=s(24), pady=(s(16), 0))
        self.header_frame.pack_propagate(False)
        
        # App icon and title
        title_frame = tk.Frame(self.header_frame, bg=AppleColors.BG_PRIMARY)
        title_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        # Icon (using Unicode symbol)
        self.header_icon = tk.Label(
            title_frame, 
            text="⚙️", 
            font=(AppleFonts.FAMILY, s(24)),
            bg=AppleColors.BG_PRIMARY
        )
        self.header_icon._font_type = 'icon'
        self.header_icon.pack(side=tk.LEFT, padx=(0, s(12)))
        
        # Title and subtitle
        text_frame = tk.Frame(title_frame, bg=AppleColors.BG_PRIMARY)
        text_frame.pack(side=tk.LEFT)
        
        self.header_title = tk.Label(
            text_frame, 
            text="HTN Planner", 
            font=AppleFonts.TITLE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        self.header_title._font_type = 'title'
        self.header_title.pack(anchor="w")
        
        self.subtitle_label = tk.Label(
            text_frame, 
            text="Hierarchical Task Network Planning", 
            font=AppleFonts.CAPTION,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_SECONDARY
        )
        self.subtitle_label._font_type = 'caption'
        self.subtitle_label.pack(anchor="w")
        
        # Status indicator
        self.status_frame = tk.Frame(self.header_frame, bg=AppleColors.BG_PRIMARY)
        self.status_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 8))
        
        self.status_dot = tk.Canvas(
            self.status_frame, 
            width=10, height=10, 
            bg=AppleColors.BG_PRIMARY,
            highlightthickness=0
        )
        self.status_dot.pack(side=tk.LEFT, padx=(0, 8), pady=20)
        self.status_dot.create_oval(0, 0, 10, 10, fill=AppleColors.SUCCESS, outline="")
        
        self.status_label = tk.Label(
            self.status_frame,
            text="Ready",
            font=AppleFonts.CAPTION,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_SECONDARY
        )
        self.status_label._font_type = 'caption'
        self.status_label.pack(side=tk.LEFT)

    def _create_menu(self):
        """Create menu bar"""
        # Configure menu style
        self.root.option_add('*Menu.background', AppleColors.BG_PRIMARY)
        self.root.option_add('*Menu.foreground', AppleColors.TEXT_PRIMARY)
        self.root.option_add('*Menu.activeBackground', AppleColors.ACCENT_BLUE)
        self.root.option_add('*Menu.activeForeground', AppleColors.TEXT_ON_ACCENT)
        self.root.option_add('*Menu.font', AppleFonts.BODY)
        
        menubar = tk.Menu(self.root, bg=AppleColors.BG_PRIMARY, 
                         fg=AppleColors.TEXT_PRIMARY, 
                         activebackground=AppleColors.ACCENT_BLUE,
                         activeforeground=AppleColors.TEXT_ON_ACCENT,
                         font=AppleFonts.BODY)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="New Configuration", command=self.new_config, accelerator="Ctrl+N")
        file_menu.add_command(label="Open Configuration...", command=self.load_config, accelerator="Ctrl+O")
        file_menu.add_command(label="Save Configuration", command=self.save_config, accelerator="Ctrl+S")
        file_menu.add_command(label="Save Configuration As...", command=self.save_config_as, accelerator="Ctrl+Shift+S")
        file_menu.add_separator()
        
        # Recent files submenu
        self.recent_menu = tk.Menu(file_menu, tearoff=0)
        file_menu.add_cascade(label="Recent Files", menu=self.recent_menu)
        self._update_recent_menu()
        
        file_menu.add_separator()
        file_menu.add_command(label="Export Operators...", command=self.export_operators)
        file_menu.add_command(label="Export Methods...", command=self.export_methods)
        file_menu.add_command(label="Import Operators...", command=self.import_operators)
        file_menu.add_command(label="Import Methods...", command=self.import_methods)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_closing)
        
        # Edit menu
        edit_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Edit", menu=edit_menu)
        edit_menu.add_command(label="Clear All Operators", command=self.clear_operators)
        edit_menu.add_command(label="Clear All Methods", command=self.clear_methods)
        edit_menu.add_command(label="Reset to Examples", command=self.reset_to_examples)
        
        # View menu
        view_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_command(label="Show Statistics", command=self.show_statistics)
        view_menu.add_command(label="Show Plan Visualization", command=self.show_plan_visualization)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Usage Guide", command=self.show_guide)
        help_menu.add_command(label="HTN Tutorial", command=self.show_tutorial)
        
        # Bind keyboard shortcuts
        self.root.bind('<Control-n>', lambda e: self.new_config())
        self.root.bind('<Control-o>', lambda e: self.load_config())
        self.root.bind('<Control-s>', lambda e: self.save_config())
        self.root.bind('<Control-Shift-S>', lambda e: self.save_config_as())

    def _create_main_layout(self):
        """Create main layout with notebook tabs"""
        s = ScaleManager.s
        # Main container with scaled padding
        self.main_container = tk.Frame(self.root, bg=AppleColors.BG_PRIMARY)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=s(24), pady=s(16))
        
        # Separator line
        separator = tk.Frame(self.main_container, bg=AppleColors.SEPARATOR, height=1)
        separator.pack(fill=tk.X, pady=(0, s(16)))
        
        # Notebook with custom styling
        self.notebook = ttk.Notebook(self.main_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Create tab frames
        self.operator_frame = tk.Frame(self.notebook, bg=AppleColors.BG_PRIMARY)
        self.method_frame = tk.Frame(self.notebook, bg=AppleColors.BG_PRIMARY)
        self.planning_frame = tk.Frame(self.notebook, bg=AppleColors.BG_PRIMARY)
        self.visualization_frame = tk.Frame(self.notebook, bg=AppleColors.BG_PRIMARY)
        
        self.notebook.add(self.operator_frame, text="  ⚡ Operators  ")
        self.notebook.add(self.method_frame, text="  📋 Methods  ")
        self.notebook.add(self.planning_frame, text="  🚀 Planning  ")
        self.notebook.add(self.visualization_frame, text="  📊 Visualization  ")
        
        # Create panel contents
        self._create_operator_panel()
        self._create_method_panel()
        self._create_planning_panel()
        
        # Create visualizer
        self.visualizer = None
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

    def _create_operator_panel(self):
        """Create operator definition panel"""
        s = ScaleManager.s
        # Split into left (list) and right (editor) with draggable sash
        self.operator_paned = tk.PanedWindow(
            self.operator_frame, 
            orient=tk.HORIZONTAL, 
            bg=AppleColors.SEPARATOR,
            sashwidth=s(6),
            sashrelief=tk.RAISED,
            sashpad=s(2),
            opaqueresize=True,
            showhandle=False
        )
        self.operator_paned.pack(fill=tk.BOTH, expand=True, pady=s(16))
        
        # Bind cursor change on sash hover
        self.operator_paned.bind('<Enter>', lambda e: self._on_sash_enter(e, self.operator_paned))
        
        # === Left Panel: Operator List ===
        left_panel = tk.Frame(self.operator_paned, bg=AppleColors.SIDEBAR_BG, width=s(280))
        self.operator_paned.add(left_panel, minsize=s(200), width=s(280))
        
        # List header
        list_header = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        list_header.pack(fill=tk.X, padx=s(16), pady=(s(16), s(12)))
        
        op_list_label = tk.Label(
            list_header, 
            text="Operators", 
            font=AppleFonts.HEADLINE,
            bg=AppleColors.SIDEBAR_BG,
            fg=AppleColors.TEXT_PRIMARY
        )
        op_list_label._font_type = 'headline'
        op_list_label.pack(side=tk.LEFT)
        
        # Item count badge
        self.operator_count_label = tk.Label(
            list_header,
            text="0",
            font=AppleFonts.CAPTION_BOLD,
            bg=AppleColors.ACCENT_BLUE,
            fg=AppleColors.TEXT_ON_ACCENT,
            padx=8,
            pady=2
        )
        self.operator_count_label._font_type = 'caption_bold'
        self.operator_count_label.pack(side=tk.RIGHT)
        
        # Operator listbox with custom styling
        list_container = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        list_container.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        
        self.operator_listbox = tk.Listbox(
            list_container, 
            font=AppleFonts.BODY,
            bg=AppleColors.SIDEBAR_BG,
            fg=AppleColors.TEXT_PRIMARY,
            selectbackground=AppleColors.ACCENT_BLUE,
            selectforeground=AppleColors.TEXT_ON_ACCENT,
            highlightthickness=0,
            borderwidth=0,
            activestyle='none',
            relief='flat'
        )
        self.operator_listbox.pack(fill=tk.BOTH, expand=True, padx=8)
        self.operator_listbox.bind('<<ListboxSelect>>', self.on_operator_select)
        
        # Action buttons
        btn_frame = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        btn_frame.pack(fill=tk.X, padx=16, pady=16)
        
        # New button - use larger base width
        new_btn = AppleButton(btn_frame, text="＋ New", command=self.new_operator, 
                             style="primary", width=90)
        new_btn.pack(side=tk.LEFT, padx=(0, s(8)))
        new_btn.config(bg=AppleColors.SIDEBAR_BG)
        
        # Delete button  
        del_btn = AppleButton(btn_frame, text="Delete", command=self.delete_operator,
                             style="default", width=85)
        del_btn.pack(side=tk.LEFT, padx=(0, s(8)))
        del_btn.config(bg=AppleColors.SIDEBAR_BG)
        
        # Duplicate button
        dup_btn = AppleButton(btn_frame, text="Duplicate", command=self.duplicate_operator,
                             style="default", width=100)
        dup_btn.pack(side=tk.LEFT)
        dup_btn.config(bg=AppleColors.SIDEBAR_BG)
        
        # === Right Panel: Operator Editor ===
        right_panel = tk.Frame(self.operator_paned, bg=AppleColors.BG_PRIMARY)
        self.operator_paned.add(right_panel, minsize=s(400))
        
        # Editor container with padding
        editor = tk.Frame(right_panel, bg=AppleColors.BG_PRIMARY)
        editor.pack(fill=tk.BOTH, expand=True, padx=24, pady=16)
        
        # Title
        op_title = tk.Label(
            editor, 
            text="Operator Definition", 
            font=AppleFonts.TITLE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        op_title._font_type = 'title'
        op_title.pack(anchor=tk.W, pady=(0, 20))
        
        # Form fields
        form = tk.Frame(editor, bg=AppleColors.BG_PRIMARY)
        form.pack(fill=tk.BOTH, expand=True)
        
        # Name field
        self._create_form_field(form, "Name", "operator_name")
        self.operator_name_var = tk.StringVar()
        name_entry = AppleEntry(form, textvariable=self.operator_name_var, 
                               width=40, placeholder="e.g., pickup")
        name_entry.pack(fill=tk.X, pady=(0, 16))
        
        # Parameters field
        self._create_form_field(form, "Parameters", "operator_params", 
                               hint="Comma separated, e.g., obj, place")
        self.operator_params_var = tk.StringVar()
        params_entry = AppleEntry(form, textvariable=self.operator_params_var,
                                 width=60, placeholder="obj, place")
        params_entry.pack(fill=tk.X, pady=(0, 16))
        
        # Preconditions
        self._create_form_field(form, "Preconditions", "operator_precond",
                               hint="Python expression that must be True")
        self.operator_precond_text = AppleCodeEditor(form, height=6)
        self.operator_precond_text.pack(fill=tk.BOTH, expand=True, pady=(0, 16))
        
        # Effects
        self._create_form_field(form, "Effects", "operator_effects",
                               hint="Python code to modify state")
        self.operator_effect_text = AppleCodeEditor(form, height=6)
        self.operator_effect_text.pack(fill=tk.BOTH, expand=True, pady=(0, 20))
        
        # Save button
        save_frame = tk.Frame(form, bg=AppleColors.BG_PRIMARY)
        save_frame.pack(fill=tk.X)
        
        save_btn = AppleButton(save_frame, text="Save Operator", 
                              command=self.save_operator, style="primary", width=140)
        save_btn.pack(side=tk.RIGHT)
    
    def _create_method_panel(self):
        """Create method definition panel"""
        s = ScaleManager.s
        # Split into left (list) and right (editor) with draggable sash
        self.method_paned = tk.PanedWindow(
            self.method_frame, 
            orient=tk.HORIZONTAL,
            bg=AppleColors.SEPARATOR,
            sashwidth=s(6),
            sashrelief=tk.RAISED,
            sashpad=s(2),
            opaqueresize=True,
            showhandle=False
        )
        self.method_paned.pack(fill=tk.BOTH, expand=True, pady=s(16))
        
        # Bind cursor change on sash hover
        self.method_paned.bind('<Enter>', lambda e: self._on_sash_enter(e, self.method_paned))
        
        # === Left Panel: Method List ===
        left_panel = tk.Frame(self.method_paned, bg=AppleColors.SIDEBAR_BG, width=s(280))
        self.method_paned.add(left_panel, minsize=s(200), width=s(280))
        
        # List header
        list_header = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        list_header.pack(fill=tk.X, padx=16, pady=(16, 12))
        
        method_list_label = tk.Label(
            list_header, 
            text="Methods", 
            font=AppleFonts.HEADLINE,
            bg=AppleColors.SIDEBAR_BG,
            fg=AppleColors.TEXT_PRIMARY
        )
        method_list_label._font_type = 'headline'
        method_list_label.pack(side=tk.LEFT)
        
        # Item count badge
        self.method_count_label = tk.Label(
            list_header,
            text="0",
            font=AppleFonts.CAPTION_BOLD,
            bg=AppleColors.ACCENT_BLUE,
            fg=AppleColors.TEXT_ON_ACCENT,
            padx=8,
            pady=2
        )
        self.method_count_label._font_type = 'caption_bold'
        self.method_count_label.pack(side=tk.RIGHT)
        
        # Method listbox
        list_container = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        list_container.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        
        self.method_listbox = tk.Listbox(
            list_container,
            font=AppleFonts.BODY,
            bg=AppleColors.SIDEBAR_BG,
            fg=AppleColors.TEXT_PRIMARY,
            selectbackground=AppleColors.ACCENT_BLUE,
            selectforeground=AppleColors.TEXT_ON_ACCENT,
            highlightthickness=0,
            borderwidth=0,
            activestyle='none',
            relief='flat'
        )
        self.method_listbox.pack(fill=tk.BOTH, expand=True, padx=8)
        self.method_listbox.bind('<<ListboxSelect>>', self.on_method_select)
        
        # Action buttons
        btn_frame = tk.Frame(left_panel, bg=AppleColors.SIDEBAR_BG)
        btn_frame.pack(fill=tk.X, padx=16, pady=16)
        
        new_btn = AppleButton(btn_frame, text="＋ New", command=self.new_method,
                             style="primary", width=90)
        new_btn.pack(side=tk.LEFT, padx=(0, s(8)))
        new_btn.config(bg=AppleColors.SIDEBAR_BG)
        
        del_btn = AppleButton(btn_frame, text="Delete", command=self.delete_method,
                             style="default", width=85)
        del_btn.pack(side=tk.LEFT)
        del_btn.config(bg=AppleColors.SIDEBAR_BG)
        
        # === Right Panel: Method Editor ===
        right_panel = tk.Frame(self.method_paned, bg=AppleColors.BG_PRIMARY)
        self.method_paned.add(right_panel, minsize=s(400))
        
        editor = tk.Frame(right_panel, bg=AppleColors.BG_PRIMARY)
        editor.pack(fill=tk.BOTH, expand=True, padx=24, pady=16)
        
        method_title = tk.Label(
            editor, 
            text="Method Definition", 
            font=AppleFonts.TITLE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        method_title._font_type = 'title'
        method_title.pack(anchor=tk.W, pady=(0, 20))
        
        form = tk.Frame(editor, bg=AppleColors.BG_PRIMARY)
        form.pack(fill=tk.BOTH, expand=True)
        
        # Two-column layout for task and method name
        row1 = tk.Frame(form, bg=AppleColors.BG_PRIMARY)
        row1.pack(fill=tk.X, pady=(0, 16))
        
        # Task name
        task_col = tk.Frame(row1, bg=AppleColors.BG_PRIMARY)
        task_col.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 16))
        self._create_form_field(task_col, "Task Name", "method_task")
        self.method_task_var = tk.StringVar()
        task_entry = AppleEntry(task_col, textvariable=self.method_task_var,
                               width=25, placeholder="e.g., get_obj")
        task_entry.pack(fill=tk.X)
        
        # Method name
        name_col = tk.Frame(row1, bg=AppleColors.BG_PRIMARY)
        name_col.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._create_form_field(name_col, "Method Name", "method_name")
        self.method_name_var = tk.StringVar()
        name_entry = AppleEntry(name_col, textvariable=self.method_name_var,
                               width=25, placeholder="e.g., get_by_pickup")
        name_entry.pack(fill=tk.X)
        
        # Parameters
        self._create_form_field(form, "Parameters", "method_params",
                               hint="Comma separated")
        self.method_params_var = tk.StringVar()
        params_entry = AppleEntry(form, textvariable=self.method_params_var,
                                 width=60, placeholder="obj, place")
        params_entry.pack(fill=tk.X, pady=(0, 16))
        
        # Preconditions
        self._create_form_field(form, "Preconditions", "method_precond",
                               hint="Python expression")
        self.method_precond_text = AppleCodeEditor(form, height=5)
        self.method_precond_text.pack(fill=tk.BOTH, expand=True, pady=(0, 16))
        
        # Subtasks
        self._create_form_field(form, "Subtasks", "method_subtasks",
                               hint="Python list of tuples")
        self.method_subtasks_text = AppleCodeEditor(form, height=6)
        self.method_subtasks_text.pack(fill=tk.BOTH, expand=True, pady=(0, 20))
        
        # Save button
        save_frame = tk.Frame(form, bg=AppleColors.BG_PRIMARY)
        save_frame.pack(fill=tk.X)
        
        save_btn = AppleButton(save_frame, text="Save Method",
                              command=self.save_method, style="primary", width=130)
        save_btn.pack(side=tk.RIGHT)
    
    def _create_planning_panel(self):
        """Create planning panel with state, tasks, and output"""
        s = ScaleManager.s
        container = tk.Frame(self.planning_frame, bg=AppleColors.BG_PRIMARY)
        container.pack(fill=tk.BOTH, expand=True, padx=s(16), pady=s(16))
        
        # Top section: State and Tasks in two columns
        top_section = tk.Frame(container, bg=AppleColors.BG_PRIMARY)
        top_section.pack(fill=tk.X, pady=(0, s(16)))
        
        # === State Card ===
        state_card = AppleCard(top_section, title="Initial State")
        state_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        
        tk.Label(
            state_card.content,
            text="State Definition (JSON format)",
            font=AppleFonts.CAPTION,
            bg=AppleColors.CARD_BG,
            fg=AppleColors.TEXT_SECONDARY
        ).pack(anchor=tk.W, pady=(0, 8))
        
        self.state_text = AppleCodeEditor(state_card.content, height=8)
        self.state_text.pack(fill=tk.BOTH, expand=True, pady=(0, 12))
        
        # Default state
        example_state = """{
    "name": "initial_state",
    "pos": {"robot": "initial", "regulator": "table"},
    "clear": {"regulator": true, "table": true, "placement_regulator": true},
    "holding": false
}"""
        self.state_text.insert(tk.END, example_state)
        
        state_btn_frame = tk.Frame(state_card.content, bg=AppleColors.CARD_BG)
        state_btn_frame.pack(fill=tk.X)

        config_btn = AppleButton(state_btn_frame, text="Visual Config",
                                 command=self.open_state_configurator,
                                 style="primary", width=130)
        config_btn.pack(side=tk.LEFT)
        config_btn.config(bg=AppleColors.CARD_BG)

        parse_btn = AppleButton(state_btn_frame, text="Parse State",
                               command=self.parse_state, style="default", width=110)
        parse_btn.pack(side=tk.RIGHT)
        parse_btn.config(bg=AppleColors.CARD_BG)
        
        # === Tasks Card ===
        task_card = AppleCard(top_section, title="Planning Tasks")
        task_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))
        
        tk.Label(
            task_card.content,
            text="Tasks (Python list format)",
            font=AppleFonts.CAPTION,
            bg=AppleColors.CARD_BG,
            fg=AppleColors.TEXT_SECONDARY
        ).pack(anchor=tk.W, pady=(0, 8))
        
        self.task_entry = AppleEntry(task_card.content, width=50,
                                    placeholder='[("assemble", "regulator", "placement_regulator")]')
        self.task_entry.pack(fill=tk.X, pady=(0, 16))
        self.task_entry.insert(0, '[("assemble", "regulator", "placement_regulator")]')
        
        # Verbose level
        verbose_frame = tk.Frame(task_card.content, bg=AppleColors.CARD_BG)
        verbose_frame.pack(fill=tk.X, pady=(0, s(16)))
        
        self.verbose_label = tk.Label(
            verbose_frame,
            text="Verbose Level:",
            font=AppleFonts.BODY,
            bg=AppleColors.CARD_BG,
            fg=AppleColors.TEXT_PRIMARY
        )
        self.verbose_label._font_type = 'body'
        self.verbose_label.pack(side=tk.LEFT)
        
        self.verbose_var = tk.IntVar(value=1)
        self.verbose_spin = tk.Spinbox(
            verbose_frame, from_=0, to=3,
            textvariable=self.verbose_var, 
            width=5,
            font=AppleFonts.BODY,
            bg=AppleColors.INPUT_BG,
            fg=AppleColors.TEXT_PRIMARY,
            highlightthickness=1,
            highlightbackground=AppleColors.BORDER
        )
        self.verbose_spin.pack(side=tk.LEFT, padx=(s(8), 0))
        
        # Control buttons
        btn_frame = tk.Frame(task_card.content, bg=AppleColors.CARD_BG)
        btn_frame.pack(fill=tk.X)
        
        self.plan_button = AppleButton(btn_frame, text="🚀 Start Planning",
                                      command=self.start_planning, style="primary", width=150)
        self.plan_button.pack(side=tk.LEFT, padx=(0, 12))
        self.plan_button.config(bg=AppleColors.CARD_BG)
        
        clear_btn = AppleButton(btn_frame, text="Clear Output",
                               command=self.clear_output, style="default", width=110)
        clear_btn.pack(side=tk.LEFT)
        clear_btn.config(bg=AppleColors.CARD_BG)
        
        # === Output Section ===
        output_card = AppleCard(container, title="Planning Output")
        output_card.pack(fill=tk.BOTH, expand=True)
        
        # Output text with custom styling
        output_frame = tk.Frame(output_card.content, bg=AppleColors.BORDER, padx=1, pady=1)
        output_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 12))
        
        self.output_text = tk.Text(
            output_frame,
            font=AppleFonts.CODE,
            bg="#1E1E1E",
            fg="#D4D4D4",
            insertbackground="#FFFFFF",
            relief="flat",
            wrap=tk.WORD,
            padx=16,
            pady=12,
            highlightthickness=0
        )
        
        output_scroll = ttk.Scrollbar(output_frame, orient=tk.VERTICAL,
                                      command=self.output_text.yview)
        self.output_text.configure(yscrollcommand=output_scroll.set)
        
        self.output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        output_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Progress bar
        self.progress = ttk.Progressbar(output_card.content, mode='indeterminate')
        self.progress.pack(fill=tk.X)
    
    def _create_form_field(self, parent, label, name, hint=None):
        """Create a form field label with optional hint"""
        frame = tk.Frame(parent, bg=AppleColors.BG_PRIMARY)
        frame.pack(fill=tk.X, pady=(0, 6))
        
        label_widget = tk.Label(
            frame,
            text=label,
            font=AppleFonts.BODY_BOLD,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        )
        label_widget._font_type = 'body_bold'
        label_widget.pack(side=tk.LEFT)
        
        if hint:
            hint_widget = tk.Label(
                frame,
                text=f"  •  {hint}",
                font=AppleFonts.CAPTION,
                bg=AppleColors.BG_PRIMARY,
                fg=AppleColors.TEXT_TERTIARY
            )
            hint_widget._font_type = 'caption'
            hint_widget.pack(side=tk.LEFT)
    
    def _on_sash_enter(self, event, paned_widget):
        """Handle mouse entering PanedWindow sash area"""
        # Change cursor to indicate resizable
        def check_sash(event):
            try:
                # Check if mouse is near sash
                x, y = event.x, event.y
                sash_pos = paned_widget.sash_coord(0)
                if sash_pos:
                    sash_x = sash_pos[0]
                    s = ScaleManager.s
                    # If within sash area, change cursor
                    if abs(x - sash_x) < s(10):
                        paned_widget.config(cursor="sb_h_double_arrow")
                    else:
                        paned_widget.config(cursor="")
            except:
                pass
        
        paned_widget.bind('<Motion>', check_sash)
        paned_widget.bind('<Leave>', lambda e: paned_widget.config(cursor=""))
    
    def _update_status(self, text, status="ready"):
        """Update status indicator"""
        colors = {
            "ready": AppleColors.SUCCESS,
            "working": AppleColors.WARNING,
            "error": AppleColors.ERROR
        }
        
        s = ScaleManager.s
        dot_size = s(10)
        self.status_dot.config(width=dot_size, height=dot_size)
        self.status_dot.delete("all")
        self.status_dot.create_oval(0, 0, dot_size, dot_size, fill=colors.get(status, AppleColors.SUCCESS), outline="")
        self.status_label.config(text=text)
    
    def _on_window_resize(self, event):
        """Handle window resize for proportional scaling"""
        # Only respond to root window resize events
        if event.widget != self.root:
            return
        
        # Debounce resize events
        if self._resize_job:
            self.root.after_cancel(self._resize_job)
        
        self._resize_job = self.root.after(100, lambda: self._apply_scaling(event.width, event.height))
    
    def _apply_scaling(self, width, height):
        """Apply proportional scaling based on window size"""
        if ScaleManager.update_scale(width, height):
            # Update fonts first
            AppleFonts.update_fonts()
            # Then update all widgets (callbacks will be triggered by ScaleManager)
            self._update_all_fonts()
    
    def _update_all_fonts(self):
        """Update all widget fonts after scale change"""
        # Update all widgets recursively
        for widget in self.root.winfo_children():
            self._update_widget_fonts(widget)
        
        # Reconfigure ttk styles with new font sizes
        self._configure_styles()
        
        # Update specific tracked widgets with their correct fonts
        self._update_tracked_fonts()
    
    def _update_tracked_fonts(self):
        """Update fonts and sizes for specifically tracked widgets"""
        s = ScaleManager.s
        
        # Update header frame height
        if hasattr(self, 'header_frame') and self.header_frame:
            self.header_frame.config(height=s(70))
        
        # Update header widgets if they exist
        if hasattr(self, 'header_title') and self.header_title:
            self.header_title.config(font=AppleFonts.TITLE)
        if hasattr(self, 'header_icon') and self.header_icon:
            self.header_icon.config(font=(AppleFonts.FAMILY, s(24)))
        if hasattr(self, 'subtitle_label') and self.subtitle_label:
            self.subtitle_label.config(font=AppleFonts.CAPTION)
        if hasattr(self, 'status_label') and self.status_label:
            self.status_label.config(font=AppleFonts.CAPTION)
        
        # Update status dot size
        if hasattr(self, 'status_dot') and self.status_dot:
            dot_size = s(10)
            self.status_dot.config(width=dot_size, height=dot_size)
            self.status_dot.delete("all")
            self.status_dot.create_oval(0, 0, dot_size, dot_size, fill=AppleColors.SUCCESS, outline="")
        
        # Update count labels with padding
        if hasattr(self, 'operator_count_label') and self.operator_count_label:
            self.operator_count_label.config(font=AppleFonts.CAPTION_BOLD, padx=s(8), pady=s(2))
        if hasattr(self, 'method_count_label') and self.method_count_label:
            self.method_count_label.config(font=AppleFonts.CAPTION_BOLD, padx=s(8), pady=s(2))
        
        # Update listboxes
        if hasattr(self, 'operator_listbox') and self.operator_listbox:
            self.operator_listbox.config(font=AppleFonts.BODY)
        if hasattr(self, 'method_listbox') and self.method_listbox:
            self.method_listbox.config(font=AppleFonts.BODY)
        
        # Update output text
        if hasattr(self, 'output_text') and self.output_text:
            self.output_text.config(font=AppleFonts.CODE, padx=s(16), pady=s(12))
        
        # Update verbose spinbox
        if hasattr(self, 'verbose_spin') and self.verbose_spin:
            self.verbose_spin.config(font=AppleFonts.BODY)
        if hasattr(self, 'verbose_label') and self.verbose_label:
            self.verbose_label.config(font=AppleFonts.BODY)
        
        # Update PanedWindow sash widths
        if hasattr(self, 'operator_paned') and self.operator_paned:
            self.operator_paned.config(sashwidth=s(6), sashpad=s(2))
        if hasattr(self, 'method_paned') and self.method_paned:
            self.method_paned.config(sashwidth=s(6), sashpad=s(2))
    
    def _update_widget_fonts(self, widget):
        """Recursively update fonts for a widget and its children"""
        try:
            widget_class = widget.winfo_class()
            
            # Update based on widget type and stored font type
            if widget_class == 'Label':
                # Check if widget has a specific font type stored
                font_type = getattr(widget, '_font_type', None)
                
                if font_type == 'title_large':
                    widget.config(font=AppleFonts.TITLE_LARGE)
                elif font_type == 'title':
                    widget.config(font=AppleFonts.TITLE)
                elif font_type == 'headline':
                    widget.config(font=AppleFonts.HEADLINE)
                elif font_type == 'body_bold':
                    widget.config(font=AppleFonts.BODY_BOLD)
                elif font_type == 'caption':
                    widget.config(font=AppleFonts.CAPTION)
                elif font_type == 'caption_bold':
                    widget.config(font=AppleFonts.CAPTION_BOLD)
                elif font_type == 'icon':
                    widget.config(font=(AppleFonts.FAMILY, AppleFonts.get_scaled_size(24)))
                else:
                    # Try to detect font type from current font
                    try:
                        current_font = widget.cget('font')
                        if current_font:
                            font_str = str(current_font).lower()
                            if 'bold' in font_str:
                                widget.config(font=AppleFonts.BODY_BOLD)
                            else:
                                widget.config(font=AppleFonts.BODY)
                    except:
                        widget.config(font=AppleFonts.BODY)
            
            elif widget_class == 'Text':
                widget.config(font=AppleFonts.CODE)
            
            elif widget_class == 'Listbox':
                widget.config(font=AppleFonts.BODY)
            
            elif widget_class == 'Entry':
                widget.config(font=AppleFonts.BODY)
        
        except tk.TclError:
            pass
        
        # Recursively update children
        for child in widget.winfo_children():
            self._update_widget_fonts(child)

    # =========================================================================
    # Data Loading and Management
    # =========================================================================
    
    def _load_example_data(self):
        """Load example operators and methods"""
        self.operators = {
            "pickup": {
                "params": ["obj", "place"],
                "preconditions": """state.pos[obj] == place and 
state.pos["robot"] == place and 
state.clear[obj] == True and 
state.holding == False""",
                "effects": """state.pos[obj] = 'hand'
state.clear[obj] = False
state.holding = obj"""
            },
            "moveto": {
                "params": ["placeA", "placeB"],
                "preconditions": """state.pos["robot"] == placeA and 
state.clear[placeB] == True""",
                "effects": """state.clear[placeA] = True
state.clear[placeB] = False
state.pos["robot"] = placeB"""
            },
            "putdown": {
                "params": ["obj", "place"],
                "preconditions": """state.pos[obj] == 'hand' and 
state.pos["robot"] == place""",
                "effects": """state.pos[obj] = place
state.clear[obj] = True
state.holding = False"""
            }
        }
        
        self.methods = {
            "get_obj": {
                "task": "get_obj",
                "name": "get_by_pickup",
                "params": ["obj", "place"],
                "preconditions": "state.pos[obj] == place",
                "subtasks": '[("moveto", "initial", place), ("pickup", obj, place)]'
            },
            "put_obj": {
                "task": "put_obj",
                "name": "put_by_putdown",
                "params": ["obj", "place"],
                "preconditions": "state.holding == obj",
                "subtasks": '[("moveto", state.pos["robot"], place), ("putdown", obj, place)]'
            },
            "assemble": {
                "task": "assemble",
                "name": "assemble_parts",
                "params": ["part", "target"],
                "preconditions": "True",
                "subtasks": '[("get_obj", part, state.pos[part]), ("put_obj", part, target)]'
            }
        }
        
        self.update_operator_list()
        self.update_method_list()
        self.apply_operators()
        self.apply_methods()
    
    def update_operator_list(self):
        """Update operator listbox"""
        self.operator_listbox.delete(0, tk.END)
        for name in sorted(self.operators.keys()):
            self.operator_listbox.insert(tk.END, f"  {name}")
        self.operator_count_label.config(text=str(len(self.operators)))
    
    def update_method_list(self):
        """Update method listbox"""
        self.method_listbox.delete(0, tk.END)
        for name in sorted(self.methods.keys()):
            method = self.methods[name]
            display_name = f"  {method['task']} → {name}"
            self.method_listbox.insert(tk.END, display_name)
        self.method_count_label.config(text=str(len(self.methods)))
    
    def on_operator_select(self, event):
        """Handle operator selection"""
        selection = self.operator_listbox.curselection()
        if selection:
            name = self.operator_listbox.get(selection[0]).strip()
            if name in self.operators:
                op = self.operators[name]
                
                self.operator_name_var.set(name)
                self.operator_params_var.set(", ".join(op["params"]))
                self.operator_precond_text.delete(1.0, tk.END)
                self.operator_precond_text.insert(1.0, op["preconditions"])
                self.operator_effect_text.delete(1.0, tk.END)
                self.operator_effect_text.insert(1.0, op["effects"])
    
    def on_method_select(self, event):
        """Handle method selection"""
        selection = self.method_listbox.curselection()
        if selection:
            display_name = self.method_listbox.get(selection[0]).strip()
            for name, method in self.methods.items():
                check_name = f"{method['task']} → {name}"
                if check_name == display_name:
                    self.method_task_var.set(method["task"])
                    self.method_name_var.set(name)
                    self.method_params_var.set(", ".join(method["params"]))
                    self.method_precond_text.delete(1.0, tk.END)
                    self.method_precond_text.insert(1.0, method["preconditions"])
                    self.method_subtasks_text.delete(1.0, tk.END)
                    self.method_subtasks_text.insert(1.0, method["subtasks"])
                    break
    
    def new_operator(self):
        """Create new operator"""
        name = "new_operator"
        i = 1
        while f"{name}{i}" in self.operators:
            i += 1
        name = f"{name}{i}"
        
        self.operators[name] = {
            "params": ["param1"],
            "preconditions": "True",
            "effects": "# Add effects here"
        }
        
        self.update_operator_list()
        self.operator_listbox.selection_clear(0, tk.END)
        items = self.operator_listbox.get(0, tk.END)
        for idx, item in enumerate(items):
            if item.strip() == name:
                self.operator_listbox.selection_set(idx)
                self.on_operator_select(None)
                break
        self.mark_modified()
    
    def delete_operator(self):
        """Delete selected operator"""
        selection = self.operator_listbox.curselection()
        if selection:
            name = self.operator_listbox.get(selection[0]).strip()
            if messagebox.askyesno("Delete Operator", 
                                   f"Delete operator '{name}'?",
                                   icon='warning'):
                del self.operators[name]
                self.update_operator_list()
                self.apply_operators()
                self.mark_modified()
    
    def duplicate_operator(self):
        """Duplicate selected operator"""
        selection = self.operator_listbox.curselection()
        if selection:
            name = self.operator_listbox.get(selection[0]).strip()
            new_name = f"{name}_copy"
            i = 1
            while new_name in self.operators:
                new_name = f"{name}_copy{i}"
                i += 1
            
            self.operators[new_name] = self.operators[name].copy()
            self.update_operator_list()
            self.mark_modified()
    
    def save_operator(self):
        """Save current operator"""
        name = self.operator_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Operator name cannot be empty")
            return
        
        params = [p.strip() for p in self.operator_params_var.get().split(",") if p.strip()]
        precond = self.operator_precond_text.get(1.0, tk.END).strip()
        effects = self.operator_effect_text.get(1.0, tk.END).strip()
        
        self.operators[name] = {
            "params": params,
            "preconditions": precond,
            "effects": effects
        }
        
        self.update_operator_list()
        self.apply_operators()
        self.mark_modified()
        self._update_status("Operator saved", "ready")
    
    def new_method(self):
        """Create new method"""
        name = "new_method"
        i = 1
        while f"{name}{i}" in self.methods:
            i += 1
        name = f"{name}{i}"
        
        self.methods[name] = {
            "task": "new_task",
            "name": name,
            "params": ["param1"],
            "preconditions": "True",
            "subtasks": "[]"
        }
        
        self.update_method_list()
        self.mark_modified()
    
    def delete_method(self):
        """Delete selected method"""
        selection = self.method_listbox.curselection()
        if selection:
            display_name = self.method_listbox.get(selection[0]).strip()
            for name in list(self.methods.keys()):
                check_name = f"{self.methods[name]['task']} → {name}"
                if check_name == display_name:
                    if messagebox.askyesno("Delete Method", 
                                          f"Delete method '{name}'?",
                                          icon='warning'):
                        del self.methods[name]
                        self.update_method_list()
                        self.apply_methods()
                        self.mark_modified()
                    break
    
    def save_method(self):
        """Save current method"""
        name = self.method_name_var.get().strip()
        task = self.method_task_var.get().strip()
        
        if not name or not task:
            messagebox.showerror("Error", "Method name and task cannot be empty")
            return
        
        params = [p.strip() for p in self.method_params_var.get().split(",") if p.strip()]
        precond = self.method_precond_text.get(1.0, tk.END).strip()
        subtasks = self.method_subtasks_text.get(1.0, tk.END).strip()
        
        self.methods[name] = {
            "task": task,
            "name": name,
            "params": params,
            "preconditions": precond,
            "subtasks": subtasks
        }
        
        self.update_method_list()
        self.apply_methods()
        self.mark_modified()
        self._update_status("Method saved", "ready")

    # =========================================================================
    # File Operations
    # =========================================================================
    
    def new_config(self):
        """Create a new configuration"""
        if self.is_modified:
            response = messagebox.askyesnocancel(
                "Save Changes",
                "Do you want to save changes to the current configuration?"
            )
            if response is None:
                return
            elif response:
                if not self.save_config():
                    return
        
        self.api.new_config()
        self.operators = self.api.current_config.operators
        self.methods = self.api.current_config.methods
        self.is_modified = False
        
        self.update_operator_list()
        self.update_method_list()
        self.apply_operators()
        self.apply_methods()
        self.update_title()
        self.clear_output()
        self._update_status("New configuration created", "ready")
    
    def save_config(self):
        """Save configuration to current file or prompt for new file"""
        if self.api.current_file:
            return self._save_to_file(self.api.current_file)
        else:
            return self.save_config_as()
    
    def save_config_as(self):
        """Save configuration to a new file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("HTN Config files", "*.json"), ("All files", "*.*")],
            initialfile=f"htn_config_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            if self._save_to_file(filename):
                self._add_to_recent(filename)
                return True
        return False
    
    def _save_to_file(self, filename):
        """Save configuration to specified file"""
        try:
            self.api.current_config.operators = self.operators
            self.api.current_config.methods = self.methods
            saved_path = self.api.save_config(filename)
            
            self.is_modified = False
            self.update_title()
            self._update_status(f"Saved to {os.path.basename(saved_path)}", "ready")
            return True
            
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save configuration: {e}")
            return False
    
    def load_config(self, filename=None):
        """Load configuration from file"""
        if not filename:
            filename = filedialog.askopenfilename(
                filetypes=[("HTN Config files", "*.json"), ("All files", "*")]
            )
        
        if filename and os.path.exists(filename):
            if self.is_modified:
                response = messagebox.askyesnocancel(
                    "Save Changes",
                    "Do you want to save changes to the current configuration?"
                )
                if response is None:
                    return
                elif response:
                    if not self.save_config():
                        return
            
            try:
                config = self.api.load_config(filename)
                
                self.operators = config.operators
                self.methods = config.methods
                self.is_modified = False
                self._add_to_recent(filename)
                
                self.update_operator_list()
                self.update_method_list()
                self.apply_operators()
                self.apply_methods()
                self.update_title()
                
                # Update Planning tab if config contains planning section
                self._update_planning_from_config(filename)
                
                self._update_status(f"Loaded {os.path.basename(filename)}", "ready")
                
            except Exception as e:
                messagebox.showerror("Load Error", f"Failed to load configuration: {e}")
    
    def _update_planning_from_config(self, filename):
        """Update Planning tab state/tasks from config file's planning section"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            planning = data.get("planning")
            if not planning:
                return
            
            initial_state = planning.get("initial_state")
            if initial_state:
                state_json = json.dumps(initial_state, indent=4, ensure_ascii=False)
                self.state_text.delete("1.0", tk.END)
                self.state_text.insert(tk.END, state_json)
                self.current_state = None
            
            tasks_str = planning.get("tasks")
            if tasks_str:
                self.task_entry.delete(0, tk.END)
                self.task_entry.insert(0, tasks_str)
        except Exception:
            pass
    
    def export_operators(self):
        """Export only operators to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialfile=f"operators_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            try:
                self.api.current_config.operators = self.operators
                self.api.export_operators(filename)
                self._update_status("Operators exported", "ready")
            except Exception as e:
                messagebox.showerror("Export Error", f"Failed to export operators: {e}")
    
    def export_methods(self):
        """Export only methods to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialfile=f"methods_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            try:
                self.api.current_config.methods = self.methods
                self.api.export_methods(filename)
                self._update_status("Methods exported", "ready")
            except Exception as e:
                messagebox.showerror("Export Error", f"Failed to export methods: {e}")
    
    def import_operators(self):
        """Import operators from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*")]
        )
        
        if filename:
            try:
                merge = True
                if self.operators:
                    response = messagebox.askyesno(
                        "Import Operators",
                        "Merge with existing operators?\n\n(No will replace all)"
                    )
                    merge = response
                
                imported_ops = self.api.import_operators(filename, merge=merge)
                
                self.operators = self.api.current_config.operators
                self.update_operator_list()
                self.apply_operators()
                self.mark_modified()
                
                self._update_status(f"Imported {len(imported_ops)} operators", "ready")
                
            except Exception as e:
                messagebox.showerror("Import Error", f"Failed to import operators: {e}")
    
    def import_methods(self):
        """Import methods from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*")]
        )
        
        if filename:
            try:
                merge = True
                if self.methods:
                    response = messagebox.askyesno(
                        "Import Methods",
                        "Merge with existing methods?\n\n(No will replace all)"
                    )
                    merge = response
                
                imported_methods = self.api.import_methods(filename, merge=merge)
                
                self.methods = self.api.current_config.methods
                self.update_method_list()
                self.apply_methods()
                self.mark_modified()
                
                self._update_status(f"Imported {len(imported_methods)} methods", "ready")
                
            except Exception as e:
                messagebox.showerror("Import Error", f"Failed to import methods: {e}")
    
    def clear_operators(self):
        """Clear all operators"""
        if self.operators:
            if messagebox.askyesno("Clear Operators", "Remove all operators?"):
                self.operators.clear()
                self.update_operator_list()
                self.apply_operators()
                self.mark_modified()
    
    def clear_methods(self):
        """Clear all methods"""
        if self.methods:
            if messagebox.askyesno("Clear Methods", "Remove all methods?"):
                self.methods.clear()
                self.update_method_list()
                self.apply_methods()
                self.mark_modified()
    
    def reset_to_examples(self):
        """Reset to example configuration"""
        if messagebox.askyesno("Reset", "Reset to example configuration?\n\nThis will discard current changes."):
            self._load_example_data()
            self.api._current_file = None
            self.is_modified = False
            self.update_title()
    
    def mark_modified(self):
        """Mark current configuration as modified"""
        if not self.is_modified:
            self.is_modified = True
            self.update_title()
    
    def update_title(self):
        """Update window title with current file and modified status"""
        title = "HTN Planner"
        if self.api.current_file:
            title += f" — {os.path.basename(self.api.current_file)}"
        else:
            title += " — Untitled"
        
        if self.is_modified:
            title += " (Edited)"
        
        self.root.title(title)
    
    def on_closing(self):
        """Handle window closing"""
        if self.is_modified:
            response = messagebox.askyesnocancel(
                "Save Changes",
                "Do you want to save changes before closing?"
            )
            if response is None:
                return
            elif response:
                if not self.save_config():
                    return
        
        self._save_recent_files()
        self.root.destroy()
    
    def _load_recent_files(self):
        """Load recent files list"""
        try:
            config_dir = Path.home() / ".htn_planner"
            config_dir.mkdir(exist_ok=True)
            recent_file = config_dir / "recent.json"
            
            if recent_file.exists():
                with open(recent_file, 'r') as f:
                    return json.load(f)
        except:
            pass
        return []
    
    def _save_recent_files(self):
        """Save recent files list"""
        try:
            config_dir = Path.home() / ".htn_planner"
            config_dir.mkdir(exist_ok=True)
            recent_file = config_dir / "recent.json"
            
            with open(recent_file, 'w') as f:
                json.dump(self.recent_files[:10], f)
        except:
            pass
    
    def _add_to_recent(self, filename):
        """Add file to recent files list"""
        filename = os.path.abspath(filename)
        if filename in self.recent_files:
            self.recent_files.remove(filename)
        self.recent_files.insert(0, filename)
        self.recent_files = self.recent_files[:10]
        self._update_recent_menu()
    
    def _update_recent_menu(self):
        """Update recent files menu"""
        self.recent_menu.delete(0, tk.END)
        
        if not self.recent_files:
            self.recent_menu.add_command(label="(No recent files)", state='disabled')
        else:
            for i, filename in enumerate(self.recent_files):
                if os.path.exists(filename):
                    label = f"{i+1}. {os.path.basename(filename)}"
                    self.recent_menu.add_command(
                        label=label,
                        command=lambda f=filename: self.load_config(f)
                    )

    # =========================================================================
    # Planning Functions
    # =========================================================================
    
    def apply_operators(self):
        """Apply operators to HTN system"""
        self.api.current_config.operators = self.operators
        self.api.current_config.methods = self.methods
        self.api.apply_to_htn()
        
        if hasattr(self, 'visualizer') and self.visualizer:
            self.visualizer.update_data(self.methods, self.operators)
    
    def apply_methods(self):
        """Apply methods to HTN system"""
        htn.methods.clear()
        
        task_methods = {}
        for name, method_def in self.methods.items():
            task = method_def["task"]
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
                htn.declare_methods(task, *funcs)
        
        if hasattr(self, 'visualizer') and self.visualizer:
            self.visualizer.update_data(self.methods, self.operators)
    
    def _create_operator_function(self, name, op_def):
        """Create operator function dynamically"""
        params = op_def["params"]
        precond = op_def["preconditions"]
        effects = op_def["effects"]
        
        precond_cleaned = precond.replace('\n', ' ').strip()
        effects_lines = effects.split('\n')
        effects_indented = '\n        '.join(effects_lines)
        
        func_code = f"""
def {name}(state, {', '.join(params)}):
    if ({precond_cleaned}):
        {effects_indented}
        return state
    else:
        return False
"""
        
        namespace = {"State": State}
        try:
            exec(func_code, namespace)
            htn.declare_operators(namespace[name])
        except SyntaxError as e:
            self.output_text.insert(tk.END, f"Syntax error in operator {name}: {e}\n")
            self.output_text.insert(tk.END, f"Generated code:\n{func_code}\n")
        except Exception as e:
            self.output_text.insert(tk.END, f"Error creating operator {name}: {e}\n")
    
    def _create_method_function(self, name, method_def):
        """Create method function dynamically"""
        params = method_def["params"]
        precond = method_def["preconditions"]
        subtasks = method_def["subtasks"]
        
        precond_cleaned = precond.replace('\n', ' ').strip()
        
        func_code = f"""
def {name}(state, {', '.join(params)}):
    if ({precond_cleaned}):
        return {subtasks}
    else:
        return False
"""
        
        namespace = {"State": State}
        try:
            exec(func_code, namespace)
            return namespace[name]
        except SyntaxError as e:
            self.output_text.insert(tk.END, f"Syntax error in method {name}: {e}\n")
            self.output_text.insert(tk.END, f"Generated code:\n{func_code}\n")
            return None
        except Exception as e:
            self.output_text.insert(tk.END, f"Error creating method {name}: {e}\n")
            return None
    
    def _detect_scenario(self):
        """Detect current scenario from loaded config metadata or file path"""
        # 1. Check config file metadata for scenario field
        if self.api.current_file:
            try:
                with open(self.api.current_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                scenario = data.get("metadata", {}).get("scenario", "")
                if scenario:
                    return scenario
            except Exception:
                pass

            # 2. Fallback: detect from file name
            fname = os.path.basename(str(self.api.current_file)).lower()
            if "wm_display_board" in fname:
                return "wm_display_board"
            elif "tv_power_module" in fname:
                return "tv_power_module"

        # 3. Fallback: detect from state content
        try:
            state_str = self.state_text.get(1.0, tk.END).strip()
            if state_str:
                state_dict = json.loads(state_str)
                name = state_dict.get("name", "")
                if "wm_display_board" in name:
                    return "wm_display_board"
                elif "tv_power_module" in name:
                    return "tv_power_module"
                # Detect by characteristic state fields
                if "board_on_conveyor" in state_dict or "board_returned" in state_dict:
                    return "wm_display_board"
                if "power_com_in_box" in state_dict:
                    return "tv_power_module"
        except Exception:
            pass

        return None

    def open_state_configurator(self):
        """Open visual state configurator dialog based on current scenario"""

        def on_apply(state_json, tasks_str):
            self.state_text.delete("1.0", tk.END)
            self.state_text.insert(tk.END, state_json)
            self.task_entry.delete(0, tk.END)
            self.task_entry.insert(0, tasks_str)
            self.current_state = None
            self._update_status("State configured", "ready")

        scenario = self._detect_scenario()

        if scenario == "wm_display_board":
            from examples.wm_display_board.state_configurator import WMStateConfiguratorDialog
            WMStateConfiguratorDialog(self.root, on_apply=on_apply)
        elif scenario == "tv_power_module":
            from robot_htn.state_configurator import StateConfiguratorDialog
            StateConfiguratorDialog(self.root, on_apply=on_apply)
        else:
            # Show scenario selection dialog, then open the corresponding configurator
            self._show_scenario_selector(on_apply)

    def _show_scenario_selector(self, on_apply):
        """Show a dialog to select which scenario configurator to open"""
        selector = tk.Toplevel(self.root)
        selector.title("选择场景 - Select Scenario")
        selector.geometry("420x280")
        selector.configure(bg=AppleColors.BG_PRIMARY)
        selector.resizable(False, False)
        selector.transient(self.root)
        selector.grab_set()

        s = ScaleManager.s
        pad = s(24)

        tk.Label(selector, text="请选择场景配置器",
                 font=AppleFonts.TITLE, bg=AppleColors.BG_PRIMARY,
                 fg=AppleColors.TEXT_PRIMARY).pack(pady=(pad, s(4)))
        tk.Label(selector, text="Select a scenario to configure initial state",
                 font=AppleFonts.CAPTION, bg=AppleColors.BG_PRIMARY,
                 fg=AppleColors.TEXT_SECONDARY).pack(pady=(0, s(16)))

        btn_frame = tk.Frame(selector, bg=AppleColors.BG_PRIMARY)
        btn_frame.pack(fill=tk.X, padx=pad)

        def open_tv():
            selector.destroy()
            from robot_htn.state_configurator import StateConfiguratorDialog
            StateConfiguratorDialog(self.root, on_apply=on_apply)

        def open_wm():
            selector.destroy()
            from examples.wm_display_board.state_configurator import WMStateConfiguratorDialog
            WMStateConfiguratorDialog(self.root, on_apply=on_apply)

        tv_btn = tk.Button(btn_frame, text="🖥  电视机电源模块安装\nTV Power Module",
                           font=AppleFonts.BODY_BOLD,
                           bg=AppleColors.ACCENT_BLUE, fg=AppleColors.TEXT_ON_ACCENT,
                           activebackground=AppleColors.ACCENT_BLUE_HOVER,
                           activeforeground=AppleColors.TEXT_ON_ACCENT,
                           relief=tk.FLAT, padx=s(16), pady=s(12), cursor="hand2",
                           command=open_tv)
        tv_btn.pack(fill=tk.X, pady=s(4))

        wm_btn = tk.Button(btn_frame, text="🔧  洗衣机显示面板安装\nWM Display Board",
                           font=AppleFonts.BODY_BOLD,
                           bg=AppleColors.SUCCESS, fg=AppleColors.TEXT_ON_ACCENT,
                           activebackground="#2AA048",
                           activeforeground=AppleColors.TEXT_ON_ACCENT,
                           relief=tk.FLAT, padx=s(16), pady=s(12), cursor="hand2",
                           command=open_wm)
        wm_btn.pack(fill=tk.X, pady=s(4))

        tk.Button(btn_frame, text="取消 Cancel",
                  font=AppleFonts.BODY,
                  bg=AppleColors.BG_TERTIARY, fg=AppleColors.TEXT_PRIMARY,
                  relief=tk.FLAT, padx=s(16), pady=s(8), cursor="hand2",
                  command=selector.destroy).pack(fill=tk.X, pady=(s(8), 0))

    def parse_state(self):
        """Parse state definition"""
        try:
            state_str = self.state_text.get(1.0, tk.END).strip()
            state_dict = json.loads(state_str)
            
            self.current_state = State(state_dict.get("name", "state"))
            
            for key, value in state_dict.items():
                if key != "name":
                    setattr(self.current_state, key, value)
            
            self.output_text.insert(tk.END, "✓ State parsed successfully\n")
            self.output_text.insert(tk.END, "─" * 50 + "\n")
            
            import io
            from contextlib import redirect_stdout
            
            f = io.StringIO()
            with redirect_stdout(f):
                print_state(self.current_state)
            
            self.output_text.insert(tk.END, f.getvalue())
            self.output_text.insert(tk.END, "─" * 50 + "\n\n")
            self.output_text.see(tk.END)
            
            self._update_status("State parsed", "ready")
            
        except Exception as e:
            messagebox.showerror("Parse Error", f"Failed to parse state: {e}")
            self._update_status("Parse failed", "error")
    
    def start_planning(self):
        """Start planning in a separate thread"""
        if self.current_state is None:
            self.parse_state()
            if self.current_state is None:
                return
        
        try:
            tasks_str = self.task_entry.get().strip()
            tasks = eval(tasks_str)
            
            self.plan_button.config(state='disabled')
            self.progress.start()
            self._update_status("Planning...", "working")
            
            thread = threading.Thread(target=self._run_planning, args=(tasks,))
            thread.daemon = True
            thread.start()
            
            self.root.after(100, self._check_planning_result)
            
        except Exception as e:
            messagebox.showerror("Task Error", f"Failed to parse tasks: {e}")
            self.plan_button.config(state='normal')
    
    def _run_planning(self, tasks):
        """Run planning in background thread"""
        try:
            import io
            from contextlib import redirect_stdout
            
            f = io.StringIO()
            with redirect_stdout(f):
                result = htn.plan(
                    self.current_state,
                    tasks,
                    htn.get_operators(),
                    htn.get_methods(),
                    verbose=self.verbose_var.get()
                )
            
            output = f.getvalue()
            self.plan_queue.put(("success", result, output))
            
        except Exception as e:
            self.plan_queue.put(("error", str(e), ""))
    
    def _check_planning_result(self):
        """Check if planning is complete"""
        try:
            status, result, output = self.plan_queue.get_nowait()
            
            self.progress.stop()
            self.plan_button.config(state='normal')
            
            self.output_text.insert(tk.END, output)
            
            if status == "success":
                if result:
                    self.output_text.insert(tk.END, "\n" + "═"*50 + "\n")
                    self.output_text.insert(tk.END, "✓ PLANNING SUCCESSFUL\n")
                    self.output_text.insert(tk.END, "═"*50 + "\n\n")
                    self.output_text.insert(tk.END, "Plan Steps:\n")
                    for i, action in enumerate(result):
                        self.output_text.insert(tk.END, f"  {i+1}. {action}\n")
                    self._update_status("Planning complete", "ready")
                else:
                    self.output_text.insert(tk.END, "\n" + "═"*50 + "\n")
                    self.output_text.insert(tk.END, "✗ PLANNING FAILED: No valid plan found\n")
                    self.output_text.insert(tk.END, "═"*50 + "\n")
                    self._update_status("No plan found", "error")
            else:
                self.output_text.insert(tk.END, f"\n✗ ERROR: {result}\n")
                self._update_status("Planning error", "error")
            
            self.output_text.see(tk.END)
            
        except queue.Empty:
            self.root.after(100, self._check_planning_result)
    
    def clear_output(self):
        """Clear output text"""
        self.output_text.delete(1.0, tk.END)

    # =========================================================================
    # View Functions
    # =========================================================================
    
    def show_statistics(self):
        """Show planning statistics"""
        stats_window = tk.Toplevel(self.root)
        stats_window.title("Statistics")
        stats_window.geometry("450x350")
        stats_window.configure(bg=AppleColors.BG_PRIMARY)
        
        # Container
        container = tk.Frame(stats_window, bg=AppleColors.BG_PRIMARY, padx=24, pady=20)
        container.pack(fill=tk.BOTH, expand=True)
        
        # Title
        tk.Label(
            container,
            text="Configuration Statistics",
            font=AppleFonts.TITLE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        ).pack(anchor=tk.W, pady=(0, 20))
        
        # Stats
        stats_frame = tk.Frame(container, bg=AppleColors.BG_PRIMARY)
        stats_frame.pack(fill=tk.BOTH, expand=True)
        
        # Operator count
        self._create_stat_row(stats_frame, "Operators", str(len(self.operators)), AppleColors.ACCENT_BLUE)
        
        # Method count
        self._create_stat_row(stats_frame, "Methods", str(len(self.methods)), AppleColors.SUCCESS)
        
        # Tasks covered
        tasks = {}
        for method in self.methods.values():
            task = method.get('task', 'Unknown')
            tasks[task] = tasks.get(task, 0) + 1
        
        task_summary = ", ".join([f"{t} ({c})" for t, c in sorted(tasks.items())])
        if task_summary:
            tk.Label(
                stats_frame,
                text="Tasks covered:",
                font=AppleFonts.BODY_BOLD,
                bg=AppleColors.BG_PRIMARY,
                fg=AppleColors.TEXT_PRIMARY
            ).pack(anchor=tk.W, pady=(16, 4))
            
            tk.Label(
                stats_frame,
                text=task_summary,
                font=AppleFonts.BODY,
                bg=AppleColors.BG_PRIMARY,
                fg=AppleColors.TEXT_SECONDARY,
                wraplength=380
            ).pack(anchor=tk.W)
        
        if self.api.current_file:
            tk.Label(
                stats_frame,
                text=f"\nFile: {os.path.basename(self.api.current_file)}",
                font=AppleFonts.CAPTION,
                bg=AppleColors.BG_PRIMARY,
                fg=AppleColors.TEXT_TERTIARY
            ).pack(anchor=tk.W, pady=(16, 0))
    
    def _create_stat_row(self, parent, label, value, color):
        """Create a statistics row"""
        row = tk.Frame(parent, bg=AppleColors.BG_PRIMARY)
        row.pack(fill=tk.X, pady=4)
        
        tk.Label(
            row,
            text=label,
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        ).pack(side=tk.LEFT)
        
        tk.Label(
            row,
            text=value,
            font=AppleFonts.HEADLINE,
            bg=AppleColors.BG_PRIMARY,
            fg=color
        ).pack(side=tk.RIGHT)
    
    def show_plan_visualization(self):
        """Show plan visualization window"""
        for i in range(self.notebook.index("end")):
            if "Visualization" in self.notebook.tab(i, "text"):
                self.notebook.select(i)
                break
        
        if self.visualizer is None:
            self.visualizer = HTNVisualizer(
                self.visualization_frame,
                self.methods,
                self.operators
            )

    def show_tutorial(self):
        """Show HTN tutorial"""
        tutorial_window = tk.Toplevel(self.root)
        tutorial_window.title("HTN Tutorial")
        tutorial_window.geometry("700x550")
        tutorial_window.configure(bg=AppleColors.BG_PRIMARY)
        
        container = tk.Frame(tutorial_window, bg=AppleColors.BG_PRIMARY, padx=32, pady=24)
        container.pack(fill=tk.BOTH, expand=True)
        
        tk.Label(
            container,
            text="HTN Planning Tutorial",
            font=AppleFonts.TITLE_LARGE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        ).pack(anchor=tk.W, pady=(0, 20))
        
        # Scrollable content
        text_frame = tk.Frame(container, bg=AppleColors.BORDER, padx=1, pady=1)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        text = tk.Text(
            text_frame,
            font=AppleFonts.BODY,
            bg=AppleColors.CARD_BG,
            fg=AppleColors.TEXT_PRIMARY,
            relief="flat",
            wrap=tk.WORD,
            padx=20,
            pady=16,
            highlightthickness=0
        )
        text.pack(fill=tk.BOTH, expand=True)
        
        tutorial_text = """Hierarchical Task Network (HTN) Planning

HTN planning is a powerful approach to automated planning that decomposes high-level tasks into simpler subtasks.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

BASIC CONCEPTS

1. OPERATORS (Primitive Tasks)
   • Basic actions that can be directly executed
   • Have preconditions and effects
   • Example: pickup(obj, place)

2. METHODS (Compound Tasks)
   • Recipes for accomplishing complex tasks
   • Decompose tasks into subtasks
   • Can have multiple methods for the same task

3. STATE
   • Current world configuration
   • Set of variable-value pairs
   • Modified by operator effects

4. PLANNING PROCESS
   • Start with initial state and goal tasks
   • Decompose compound tasks using methods
   • Execute primitive tasks (operators)
   • Continue until all tasks are completed

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

EXAMPLE

Task: assemble(part1, part2)
Method: 
  1. get_obj(part1, location1)
  2. put_obj(part1, part2)

The planner will further decompose these subtasks until it reaches primitive operators.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

TIPS
• Start with simple operators
• Build methods incrementally
• Test with small examples first
• Use meaningful names
"""
        
        text.insert(tk.END, tutorial_text)
        text.config(state='disabled')
    
    def show_about(self):
        """Show about dialog"""
        about_window = tk.Toplevel(self.root)
        about_window.title("About")
        about_window.geometry("400x300")
        about_window.configure(bg=AppleColors.BG_PRIMARY)
        about_window.resizable(False, False)
        
        container = tk.Frame(about_window, bg=AppleColors.BG_PRIMARY, padx=32, pady=32)
        container.pack(fill=tk.BOTH, expand=True)
        
        # Icon
        tk.Label(
            container,
            text="⚙️",
            font=(AppleFonts.FAMILY, 48),
            bg=AppleColors.BG_PRIMARY
        ).pack(pady=(0, 16))
        
        # Title
        tk.Label(
            container,
            text="HTN Planner",
            font=AppleFonts.TITLE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        ).pack()
        
        tk.Label(
            container,
            text="Version 2.0",
            font=AppleFonts.CAPTION,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_SECONDARY
        ).pack(pady=(4, 16))
        
        tk.Label(
            container,
            text="A visual interface for Hierarchical\nTask Network planning.",
            font=AppleFonts.BODY,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY,
            justify=tk.CENTER
        ).pack(pady=(0, 16))
        
        tk.Label(
            container,
            text="© 2024 HTN Planning Team",
            font=AppleFonts.CAPTION,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_TERTIARY
        ).pack()
    
    def show_guide(self):
        """Show usage guide"""
        guide_window = tk.Toplevel(self.root)
        guide_window.title("Usage Guide")
        guide_window.geometry("650x500")
        guide_window.configure(bg=AppleColors.BG_PRIMARY)
        
        container = tk.Frame(guide_window, bg=AppleColors.BG_PRIMARY, padx=32, pady=24)
        container.pack(fill=tk.BOTH, expand=True)
        
        tk.Label(
            container,
            text="Usage Guide",
            font=AppleFonts.TITLE_LARGE,
            bg=AppleColors.BG_PRIMARY,
            fg=AppleColors.TEXT_PRIMARY
        ).pack(anchor=tk.W, pady=(0, 20))
        
        text_frame = tk.Frame(container, bg=AppleColors.BORDER, padx=1, pady=1)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        text = tk.Text(
            text_frame,
            font=AppleFonts.BODY,
            bg=AppleColors.CARD_BG,
            fg=AppleColors.TEXT_PRIMARY,
            relief="flat",
            wrap=tk.WORD,
            padx=20,
            pady=16,
            highlightthickness=0
        )
        text.pack(fill=tk.BOTH, expand=True)
        
        guide_text = """HTN Planner Usage Guide

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. OPERATORS
   Define basic actions that can be performed
   • Name: Unique identifier
   • Parameters: Input parameters
   • Preconditions: Conditions that must be true
   • Effects: Changes to the state

2. METHODS
   Define how to decompose tasks into subtasks
   • Task name: The task it decomposes
   • Method name: Unique identifier
   • Preconditions: When this method applies
   • Subtasks: List of tasks to perform

3. PLANNING
   • Define initial state in JSON format
   • Specify tasks to accomplish
   • Click "Start Planning" to find a solution
   • View the plan and execution trace

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

KEYBOARD SHORTCUTS

   Ctrl+N     New configuration
   Ctrl+O     Open configuration
   Ctrl+S     Save configuration
   Ctrl+Shift+S   Save as new file

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

TIPS
• Use Python syntax for conditions and effects
• State variables are accessed as state.variable
• Save/load configurations for reuse
• Increase verbose level for detailed output
"""
        
        text.insert(tk.END, guide_text)
        text.config(state='disabled')
    
    def on_tab_changed(self, event):
        """Handle tab change event"""
        selected_tab = self.notebook.select()
        tab_text = self.notebook.tab(selected_tab, "text")
        
        if "Visualization" in tab_text:
            if self.visualizer is None:
                self.visualizer = HTNVisualizer(
                    self.visualization_frame,
                    self.methods,
                    self.operators
                )
            else:
                self.visualizer.update_data(self.methods, self.operators)


def main():
    """Main entry point"""
    root = tk.Tk()
    
    # Set DPI awareness on Windows
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except:
        pass
    
    app = HTNPlannerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
