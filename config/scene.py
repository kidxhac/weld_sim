# config/scene.py
"""
Scene Configuration

Defines the physical setup of the welding system:
- Gantry dimensions and speeds
- Robot specifications and workspaces
- Collision zones
- Weld pattern generation
"""

SCENE = {
    "gantry": {
        "x_length": 6000.0,      # Total X-axis travel (mm)
        "x_speed": 300.0,        # Gantry speed along X (mm/s)
        "y_span": 9000.0         # Total Y-axis span (mm) - 9 meters
    },

    "robots": [
        # X-plus side (top in diagram)
        {"id": "R1", "side": "x_plus", "y_range": (0, 3000), "tcp_speed": 120},
        {"id": "R3", "side": "x_plus", "y_range": (6000, 9000), "tcp_speed": 120},
        
        # X-minus side (bottom in diagram)  
        {"id": "R2", "side": "x_minus", "y_range": (0, 3000), "tcp_speed": 120},
        {"id": "R4", "side": "x_minus", "y_range": (9000, 9000), "tcp_speed": 120},
    ],

    # Collision zones (gap between robot pairs)
    # R1 (0-1000) and R3 (2000-3000) have a GAP at Y=1000-2000
    # This is NOT a collision zone - it's a separation zone!
    # Only R3/R4 can reach this area, R1/R2 cannot
    "interference": {
        "s1": (3000, 6000),      # Gap zone between R1-R3 (only R3 can reach)
        "s2": (3000, 6000)       # Gap zone between R2-R4 (only R4 can reach)
    },

    # Weld pattern configurations for testing
    "weld_config": {
        # Long welds along X-axis (WOM-suitable)
        "x_long": {
            "count": 4,
            "length": (1500, 3000),  # Min-max length range
            "y_positions": [1000, 3000, 5000, 7000]
        },
        
        # Short welds (SAW-suitable)
        "y_long": {
            "count": 6,
            "length": (600, 1200),
            "y_positions": [200, 500, 900, 1100, 1500, 1800]
        },
        
        # Mixed pattern
        "mixed": {
            "x_long_count": 2,
            "y_long_count": 4
        }
    }
}


# Alternative scene configurations for testing

SCENE_SMALL = {
    """Smaller workspace for quick testing"""
    "gantry": {
        "x_length": 3000.0,
        "x_speed": 200.0,
        "y_span": 2000.0
    },
    "robots": [
        {"id": "R1", "side": "x_plus", "y_range": (0, 800), "tcp_speed": 100},
        {"id": "R3", "side": "x_plus", "y_range": (800, 1600), "tcp_speed": 100},
        {"id": "R2", "side": "x_minus", "y_range": (0, 800), "tcp_speed": 100},
        {"id": "R4", "side": "x_minus", "y_range": (800, 1600), "tcp_speed": 100},
    ],
    "interference": {
        "s1": (600, 1000),
        "s2": (600, 1000)
    }
}

SCENE_LARGE = {
    """Larger workspace for complex scenarios"""
    "gantry": {
        "x_length": 10000.0,
        "x_speed": 400.0,
        "y_span": 4000.0
    },
    "robots": [
        {"id": "R1", "side": "x_plus", "y_range": (0, 1200), "tcp_speed": 150},
        {"id": "R3", "side": "x_plus", "y_range": (1200, 2400), "tcp_speed": 150},
        {"id": "R2", "side": "x_minus", "y_range": (0, 1200), "tcp_speed": 150},
        {"id": "R4", "side": "x_minus", "y_range": (1200, 2400), "tcp_speed": 150},
    ],
    "interference": {
        "s1": (1000, 1400),
        "s2": (1000, 1400)
    }
}
