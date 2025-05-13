import utm
import math

class CoordinateConverter:
    """
    Handles conversion between WGS84 lat/lon coordinates (EPSG:4326) and 
    UTM coordinates (EPSG:32645 - UTM zone 45N) for the rover navigation system.
    """
    
    def __init__(self):
        # Default UTM zone for EPSG:32645 is zone 45N
        self.default_zone = 45
        self.default_zone_letter = 'N'
        
    def latlon_to_utm_coord(self, lat, lon):
        """
        Convert latitude/longitude (EPSG:4326) to UTM coordinates (EPSG:32645).
        
        Args:
            lat (float): Latitude in decimal degrees
            lon (float): Longitude in decimal degrees
            
        Returns:
            tuple: (easting, northing) UTM coordinates in meters
        """
        try:
            zone_number = int((lon + 180) // 6) + 1
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            return easting, northing, zone_number, zone_letter
        except Exception as e:
            print(f"Error converting lat/lon to UTM: {e}")
            return None, None, None, None
            
    def utm_to_latlon_coord(self, easting, northing, zone_number=None, zone_letter=None):
        """
        Convert UTM coordinates (EPSG:32645) to latitude/longitude (EPSG:4326).
        
        Args:
            easting (float): UTM easting coordinate in meters
            northing (float): UTM northing coordinate in meters
            zone_number (int, optional): UTM zone number. Defaults to 45.
            zone_letter (str, optional): UTM zone letter. Defaults to 'N'.
            
        Returns:
            tuple: (latitude, longitude) in decimal degrees
        """
        try:
            # Use default zone if not specified
            zone_number = zone_number if zone_number is not None else self.default_zone
            zone_letter = zone_letter if zone_letter is not None else self.default_zone_letter
            
            lat, lon = utm.to_latlon(easting, northing, zone_number, zone_letter)
            return lat, lon
        except Exception as e:
            print(f"Error converting UTM to lat/lon: {e}")
            return None, None
    
    def get_distance_between_coords(self, lat1, lon1, lat2, lon2):
        """
        Calculate the great circle distance between two lat/lon points
        using the haversine formula.
        
        Args:
            lat1, lon1: First point coordinates in decimal degrees
            lat2, lon2: Second point coordinates in decimal degrees
            
        Returns:
            float: Distance in meters
        """
        # Radius of earth in meters
        R = 6371000
        
        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
    
    def get_bearing_between_coords(self, lat1, lon1, lat2, lon2):
        """
        Calculate the initial bearing between two lat/lon points.
        
        Args:
            lat1, lon1: First point coordinates in decimal degrees
            lat2, lon2: Second point coordinates in decimal degrees
            
        Returns:
            float: Bearing in degrees (0-360)
        """
        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing
        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        bearing_rad = math.atan2(y, x)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = math.degrees(bearing_rad)
        bearing_normalized = (bearing_deg + 360) % 360
        
        return bearing_normalized