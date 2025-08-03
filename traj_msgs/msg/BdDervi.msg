# Boundary derivatives at trajectory start and end

# Time offsets (durations) from start_time for each sample point
float32[] durations  

geometry_msgs/Point start_pos
geometry_msgs/Point start_vel
geometry_msgs/Point start_acc

geometry_msgs/Point end_pos
geometry_msgs/Point end_vel
geometry_msgs/Point end_acc

geometry_msgs/Point[]  inner_pos   #MINCO trajectories only has this
geometry_msgs/Point[]  inner_vel
geometry_msgs/Point[]  inner_acc

