# name: identifier for epoch, should be unique within an experiment
# type: type of epoch, generally gain trajectory. e.g. constant, ramp etc.
string name
string type

# Variable-length list of parameters and values, depends on epoch type
string[] params
float64[] values

# Status of epoch
# -1 - not run yet
# 0 - running
# 1 - completed
int32 status
