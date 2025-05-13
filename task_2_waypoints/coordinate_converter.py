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
        self.zone_number = None
        self.zone_letter = None
    def latlon_to_utm_coord(self, lat, lon):
        """
        Convert latitude/longitude (EPSG:4326) to UTM coordinates (EPSG:32645).

        Args:
            lat (float): Latitude in decimal degrees
            lon (float): Longitude in decimal degrees

        Returns:
            tuple: (easting, northing, zone_number, zone_letter)
        """
        try:
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            self.zone_number = zone_number
            self.zone_letter = zone_letter
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
            zone_number = zone_number or self.zone_number or self.default_zone
            zone_letter = zone_letter or self.zone_letter or self.default_zone_letter
            lat, lon = utm.to_latlon(easting, northing, zone_number, zone_letter)
            return lat, lon
        except Exception as e:
            print(f"Error converting UTM to lat/lon: {e}")
            return None, None

    def get_distance_between_coords(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def get_bearing_between_coords(self, lat1, lon1, lat2, lon2):
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360