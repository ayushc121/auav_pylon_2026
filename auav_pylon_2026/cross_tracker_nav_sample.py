import numpy as np

######################
# Waypoint Target Algorithm with cross-track flow field and Alongtrack switching modes
######################


def angle_rad_wrapper(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class XTrack_NAV_lookAhead:
    def __init__(self, dt, waypoints, start_WP_ind):
        self.dt = dt

        ## CURRENT_WP index might need to be parsed from the class variable in "init" and cycle directly from ros script
        self.current_WP_ind = start_WP_ind  # Current Active Next Waypoint Index (TARGET)
        self.next_wpt = None  # Current Active Next Waypoint Coordinate
        self.prev_wpt = (0, 0, 0)  # Previous Waypoint Coordinate
        self.last_WP = len(waypoints) - 1  # Last Waypoint Index
        self.current_pose_est = [0, 0, 0]  # Filtered pose position
        self.waypoints_list = waypoints

        self.v_max_vert = 0.5  # maximum vertical velocity (positive up) m/s
        self.v_max_horz = 0.55  # maximum horizontal velocity m/s
        self.v_min_horz = (
            0.5  # minimum horizontal velocity m/s enforce to prevent stall
        )
        self.v_cruise = 10.0  # cruise airspeed (scaled)
        self.wpt_rad = 3.0  # allowable error from target waypoint (m)

        self.wpt_switching_distance = (
            1.0  # Look ahead for x meters along track and jump to next waypoint
        )
        self.path_distance_buf = 5.0  # Cross-track distance buffer

        self.lookahead_time_s = 2.0  # seconds to look ahead along path
        self.lookahead_min_m = 2.0  # never look ahead less than this distance
        self.lookahead_max_m = 20.0  # cap look-ahead to prevent cutting corners


    

    def get_desired_flight(self, next_wpt, current_pose):
        x_err = next_wpt[0] - current_pose[0]
        y_err = next_wpt[1] - current_pose[1]
        z_err = next_wpt[2] - current_pose[2]
        
        wp_err = [x_err, y_err, z_err]

        horz_dist_err = np.sqrt(x_err**2 + y_err**2)  # horizontal distance from next waypoint

        # Compute desired pitch
        if horz_dist_err == 0:
            des_gamma = 0
        else:
            des_gamma = np.arctan(z_err / horz_dist_err)     #radians, will be measured from the horizontal

        des_heading = np.arctan2(y_err, x_err)
        
        return des_gamma, des_heading, wp_err


    def wp_tracker(self, waypoint):

        self.next_wpt = waypoints[self.current_WP_ind]
        
        if self.current_WP_ind != 0:
            self.prev_wpt = waypoints[self.current_WP_ind - 1]

        # Compute Desired Speed, Desired Heading, Desired Glide Path
        des_gamma, des_heading, wp_err = (self.get_desired_flight(self.next_wpt, self.current_pose_est))

        return des_gamma, des_heading, wp_err

    
    def check_arrived(self, wp_err, V_array):
        # Lateral Waypoint checker
        # Check based on along-track error

        scalar_proj = np.dot(V_array, wp_err) / np.linalg.norm(wp_err)   #velocity towards waypoint 

        time_to_waypoint = np.linalg.norm(wp_err) / scalar_proj
        
        # Switch when remaining distance to wayopoint is less than the current look ahead time 
        threshold = 2;

        # along_remaining is the distance to the next waypoint along the segment
        if time_to_waypoint < threshold:
            self.current_WP_ind += 1
            
        return self.current_WP_ind
