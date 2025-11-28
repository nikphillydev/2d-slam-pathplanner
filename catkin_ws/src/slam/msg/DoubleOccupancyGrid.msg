# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

std_msgs/Header header

# MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0, 1].  Unknown is -1.
float64[] data
