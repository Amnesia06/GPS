class RoverAPIWrapper:
    """
    Wrapper class for the real rover's navigation API that handles coordinate conversion
    between UTM and lat/lon systems.
    """
    
    def __init__(self, rover, coord_converter):
        self.rover = rover
        self.coord_converter = coord_converter
        self.debug = False
        
    def set_debug(self, debug_mode):
        """Enable or disable debug mode for API wrapper"""
        self.debug = debug_mode
        
    def send_waypoints_to_rover(self, utm_waypoints):
        """
        Convert UTM waypoints to lat/lon and send to the real rover's API
        
        Args:
            utm_waypoints (list): List of (easting, northing) tuples in UTM
            
        Returns:
            bool: Success status
        """
        # Convert UTM waypoints to lat/lon for the real rover API
        latlon_waypoints = self.coord_converter.convert_waypoints_to_latlon(utm_waypoints)
        
        if self.debug:
            print(f"Converting {len(utm_waypoints)} UTM waypoints to lat/lon")
            for i in range(min(3, len(utm_waypoints))):
                print(f"  Point {i}: UTM {utm_waypoints[i]} → Lat/Lon {latlon_waypoints[i]}")
                
            if len(utm_waypoints) > 3:
                print(f"  ... and {len(utm_waypoints) - 3} more points")
                
        # Here you would call the actual rover API with the lat/lon coordinates
        # For example:
        # success = self.rover.api.send_navigation_waypoints(latlon_waypoints)
        
        # For simulation purposes, assume success
        return True
        
    def receive_gps_position(self, lat, lon):
        """
        Process incoming GPS position from the real rover (lat/lon)
        and convert to UTM for use in simulation
        
        Args:
            lat (float): Latitude in decimal degrees
            lon (float): Longitude in decimal degrees
            
        Returns:
            tuple: (easting, northing) in UTM coordinates
        """
        # Convert the lat/lon position to UTM
        easting, northing = self.coord_converter.latlon_to_utm(lat, lon)
        
        if self.debug:
            print(f"GPS received: {lat:.6f}, {lon:.6f} → UTM: {easting:.3f}, {northing:.3f}")
            
        # Update rover position using UTM coordinates
        self.rover.set_position(easting, northing)
        
        return easting, northing