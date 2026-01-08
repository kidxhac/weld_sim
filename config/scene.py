# config/scene.py

SCENE = {
    "gantry": {
        "x_length": 6000.0,
        "x_speed": 300.0,
        "y_span": 3000.0
    },

    "robots": [
        {"id": "R1", "side": "x_plus", "y_range": (0, 1000), "tcp_speed": 120},
        {"id": "R3", "side": "x_plus", "y_range": (1000, 2000), "tcp_speed": 120},
        {"id": "R2", "side": "x_minus", "y_range": (0, 1000), "tcp_speed": 120},
        {"id": "R4", "side": "x_minus", "y_range": (1000, 2000), "tcp_speed": 120},
    ],

    "interference": {
        "s1": (800, 1200),
        "s2": (800, 1200)
    },

    "weld_config": {
        "x_long": {"count": 4, "length": (1500, 3000)},
        "y_long": {"count": 3, "length": (600, 1200)}
    }
}
