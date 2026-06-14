"""Entry point: ``python -m skate_commander`` → start the cockpit.

Auto-detects the skt_v3 model, builds the sim/guard model on first run, and
opens the browser. See ``python -m skate_commander --help`` for options.
"""
from .server import main

if __name__ == "__main__":
    main()
