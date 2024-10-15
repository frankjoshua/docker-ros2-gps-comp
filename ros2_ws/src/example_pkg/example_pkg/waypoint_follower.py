from math import atan2, degrees, radians, sin, cos, sqrt
from geometry_msgs.msg import Twist

class WaypointFollower:
        def __init__(self, twist_msg, logger) -> None:
                self.logger = logger
                self.state = "Waiting"
                self.active_waypoint = 0
                self.twist_msg = twist_msg

        def update(self, waypoint_list, current_waypoint, heading):
                self.logger.info(f"Current state {self.state}")

                if self.state == "Waiting":
                        self.handle_waiting(waypoint_list)

                if self.state == "Running":
                        self.handle_running(waypoint_list, current_waypoint, heading)

                if self.state == "Searching":
                        self.handle_searching(waypoint_list, current_waypoint, heading)

                if self.state == "Returning":
                        self.handle_returning(waypoint_list, current_waypoint, heading)
        
        def handle_returning(self, waypoint_list, current_waypoint, heading):
                pass

        def handle_searching(self, waypoint_list, current_waypoint, heading):
                desired_heading = self.get_heading(waypoint_list[self.active_waypoint], current_waypoint)
                turn_rate = self.get_turn_rate(desired_heading, heading)
                self.update_speed(0,turn_rate)
                # Check if desired_heading is within 30 degrees of heading
                heading_diff = abs(desired_heading - heading)
                if heading_diff > 180:
                        heading_diff = 360 - heading_diff
                
                if heading_diff <= 30:
                        self.state = "Running"
                        self.logger.info(f"Aligned with waypoint. Switching to Running state.")
                else:
                        self.logger.info(f"Still searching. Heading difference: {heading_diff} degrees")

        def handle_running(self, waypoint_list, current_waypoint, heading):
                distance = self.get_distance(waypoint_list[self.active_waypoint], current_waypoint)
                if distance < 1:
                        self.update_speed(0,0)
                        self.active_waypoint += 1
                        if self.active_waypoint == len(waypoint_list):
                                self.active_waypoint = 0
                                self.state = "Returning"
                        else:
                                self.state = "Searching"
                        return
                desired_heading = self.get_heading(waypoint_list[self.active_waypoint], current_waypoint)
                turn_rate = self.get_turn_rate(desired_heading, heading)
                self.logger.info(f"Distance: {distance}, Turn rate {turn_rate}")
                self.update_speed(0.2, turn_rate)

        def update_speed(self, x, z):
                # Convert x and z to doubles
                x = float(x)
                z = float(z)
                self.twist_msg.linear.x = x
                self.twist_msg.angular.z = z

        def get_distance(self, dst_waypoint, current_waypoint):
                # Calculate the distance between two waypoints using the Haversine formula
                R = 6371000  # Earth's radius in meters

                # Convert latitude and longitude to radians
                lat1, lon1 = radians(current_waypoint.latitude), radians(current_waypoint.longitude)
                lat2, lon2 = radians(dst_waypoint.latitude), radians(dst_waypoint.longitude)

                # Calculate differences
                dlat = lat2 - lat1
                dlon = lon2 - lon1

                # Haversine formula
                a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
                c = 2 * atan2(sqrt(a), sqrt(1-a))

                # Calculate the distance
                distance = R * c

                return distance

        def get_turn_rate(self, desired_heading, heading):
                # Calculate the difference between desired and current heading
                heading_diff = desired_heading - heading
                
                # Normalize the difference to be between -180 and 180 degrees
                if heading_diff > 180:
                        heading_diff -= 360
                elif heading_diff < -180:
                        heading_diff += 360
                
                # Convert the difference to radians
                heading_diff_rad = radians(heading_diff)
                
                # Calculate the turn rate, with a maximum of 1 radian per second
                turn_rate = max(min(heading_diff_rad, 1), -1)
                
                return turn_rate
                
        def get_heading(self, dst_waypoint, current_waypoint):

                # Convert latitude and longitude to radians
                lat1, lon1 = radians(current_waypoint.latitude), radians(current_waypoint.longitude)
                lat2, lon2 = radians(dst_waypoint.latitude), radians(dst_waypoint.longitude)

                # Calculate the difference in coordinates
                dlon = lon2 - lon1

                # Calculate the heading
                y = sin(dlon) * cos(lat2)
                x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
                heading = atan2(y, x)

                # Convert heading to degrees and normalize to 0-360
                heading_degrees = (degrees(heading) + 360) % 360

                return heading_degrees

        def handle_waiting(self, waypoint_list):
                if len(waypoint_list) > 0:
                        self.state = "Running"
                

        