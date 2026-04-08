# Current mission state published by Central Command
string current_mode   # e.g. action feedback phase or high-level mode
string last_error
string current_step   # mission step id from YAML (empty if not in a mission)
uint32 step_index     # 0-based index of current step
uint32 total_steps    # number of steps in loaded mission (0 if none)
