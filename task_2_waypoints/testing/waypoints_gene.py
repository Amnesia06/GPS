import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import csv
import os

def parse_coordinates(coord_text):
    """Parse coordinates from the input text format"""
    coordinates = []
    lines = coord_text.strip().split('\n')
    
    for line in lines:
        if line.strip():
            clean_line = line.strip('() ')
            if clean_line and ',' in clean_line:
                try:
                    parts = clean_line.split(',')
                    if len(parts) >= 2:
                        lat = float(parts[0].strip())
                        lon = float(parts[1].strip())
                        coordinates.append((lat, lon))
                except (ValueError, IndexError):
                    continue
    
    return coordinates

def create_dense_grid_waypoints(boundary_coords, spacing_meters=3.1):
    """Generate dense grid of waypoints within the boundary for real-life field coverage"""
    if len(boundary_coords) < 3:
        return []
    
    try:
        polygon = Polygon(boundary_coords)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        
        min_lat, min_lon, max_lat, max_lon = polygon.bounds
        
        # Very precise conversion for your location (18.56°N, 73.82°E)
        lat_spacing = spacing_meters / 111000.0
        lon_spacing = spacing_meters / (111000.0 * np.cos(np.radians(18.566)))
        
        waypoints = []
        
        # Create dense grid starting from boundary edges
        lat = min_lat
        while lat <= max_lat:
            lon = min_lon
            while lon <= max_lon:
                point = Point(lat, lon)
                if polygon.contains(point):
                    waypoints.append((lat, lon))
                lon += lon_spacing
            lat += lat_spacing
        
        print(f"Initial waypoints before filtering: {len(waypoints)}")
        
        # Only filter out truly isolated points (with no neighbors at all)
        if len(waypoints) > 1:
            filtered_waypoints = []
            for i, (lat, lon) in enumerate(waypoints):
                # Count nearby waypoints within 2 * spacing distance
                neighbor_count = 0
                search_distance = 2.0 * spacing_meters / 111000.0
                
                for j, (other_lat, other_lon) in enumerate(waypoints):
                    if i != j:
                        distance = np.sqrt((lat - other_lat)**2 + (lon - other_lon)**2)
                        if distance <= search_distance:
                            neighbor_count += 1
                            
                # Keep points that have at least 1 neighbor (not completely isolated)
                if neighbor_count >= 1:
                    filtered_waypoints.append((lat, lon))
            
            print(f"Waypoints after filtering: {len(filtered_waypoints)}")
            return filtered_waypoints
        else:
            return waypoints
        
    except Exception as e:
        print(f"Error creating waypoints: {e}")
        return []

def save_waypoints_csv(waypoints, filepath=r"F:\GPS\task_2_waypoints\testing"):
    """Save waypoints to CSV file with real GPS coordinates"""
    # Ensure the directory exists
    os.makedirs(filepath, exist_ok=True)
    
    # Create full file path
    filename = os.path.join(filepath, "field_waypoints.csv")
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Point_ID', 'Latitude', 'Longitude'])
            
            for i, (lat, lon) in enumerate(waypoints, 1):
                writer.writerow([f'WP{i:03d}', f'{lat:.8f}', f'{lon:.8f}'])
        
        print(f"Successfully saved {len(waypoints)} waypoints to {filename}")
        return filename
        
    except Exception as e:
        print(f"Error saving file: {e}")
        # Try saving to current directory as fallback
        try:
            fallback_filename = "field_waypoints.csv"
            with open(fallback_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Point_ID', 'Latitude', 'Longitude'])
                
                for i, (lat, lon) in enumerate(waypoints, 1):
                    writer.writerow([f'WP{i:03d}', f'{lat:.8f}', f'{lon:.8f}'])
            
            print(f"Fallback: Saved {len(waypoints)} waypoints to current directory: {fallback_filename}")
            return fallback_filename
            
        except Exception as e2:
            print(f"Failed to save file even to current directory: {e2}")
            return None

def visualize_field_waypoints(boundary_coords, waypoints):
    """Visualize the field boundary and waypoints"""
    plt.figure(figsize=(15, 12))
    
    # Plot boundary
    boundary_lats = [coord[0] for coord in boundary_coords]
    boundary_lons = [coord[1] for coord in boundary_coords]
    plt.plot(boundary_lons, boundary_lats, 'b-', linewidth=2, label='Field Boundary')
    plt.plot([boundary_lons[-1], boundary_lons[0]], [boundary_lats[-1], boundary_lats[0]], 'b-', linewidth=2)
    
    # Plot waypoints
    if waypoints:
        waypoint_lats = [wp[0] for wp in waypoints]
        waypoint_lons = [wp[1] for wp in waypoints]
        plt.scatter(waypoint_lons, waypoint_lats, c='red', s=8, alpha=0.8, label=f'Waypoints ({len(waypoints)})')
    
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(f'Field Waypoints - {len(waypoints)} Points')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

# Your coordinate data
coord_data = """(18.565826378333334, 73.82119413833334)
(18.565826306666665, 73.82119416)
(18.565826338333334, 73.821194175)
(18.565826221666665, 73.82119425333333)
(18.565826221666665, 73.82119425333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582609, 73.82119434333333)
(18.56582597, 73.82119442)
(18.565825998333334, 73.82119449)
(18.565826018333333, 73.82119434)
(18.565826018333333, 73.82119434)
(18.565826018333333, 73.82119434)
(18.565825896666666, 73.82119452666667)
(18.565825846666666, 73.82119450333333)
(18.56582579333333, 73.82119452333333)
(18.56582579333333, 73.82119452333333)
(18.56582579333333, 73.82119452333333)
(18.56582579333333, 73.82119452333333)
(18.56582579333333, 73.82119452333333)
(18.565825853333333, 73.821194675)
(18.565825776666667, 73.82119481833334)
(18.565825743333335, 73.821194785)
(18.565825743333335, 73.821194785)
(18.565825743333335, 73.821194785)
(18.565825743333335, 73.821194785)
(18.56582567, 73.82119481666666)
(18.565825588333333, 73.821194835)
(18.56582557833333, 73.821194845)
(18.565825591666666, 73.82119484833333)
(18.565825591666666, 73.82119484833333)
(18.565825591666666, 73.82119484833333)
(18.565825591666666, 73.82119484833333)
(18.565825591666666, 73.82119484833333)
(18.565825608333334, 73.821194695)
(18.565825051666668, 73.821195185)
(18.565824646666666, 73.82119509666667)
(18.56582454, 73.82119519166666)
(18.565824051666667, 73.82119571333334)
(18.565823981666668, 73.82119577166667)
(18.565823913333332, 73.821195845)
(18.565823913333332, 73.82119576166667)
(18.565823706666666, 73.82119574833334)
(18.565823681666668, 73.82119569166667)
(18.565823681666668, 73.82119569166667)
(18.565823681666668, 73.82119569166667)
(18.565823635, 73.82119531666666)
(18.56582368666667, 73.821193465)
(18.565824223333333, 73.82118862166666)
(18.565824311666667, 73.82118719666667)
(18.565824311666667, 73.82118719666667)
(18.565824806666665, 73.821184455)
(18.565825205, 73.82118307666667)
(18.565825813333333, 73.82117864833333)
(18.565825813333333, 73.82117864833333)
(18.565826686666668, 73.82117178833333)
(18.565826236666666, 73.82116873833333)
(18.565826715, 73.82116153666666)
(18.565826601666668, 73.82116006833333)
(18.565826601666668, 73.82116006833333)
(18.565826601666668, 73.82116006833333)
(18.565826601666668, 73.82116006833333)
(18.565826601666668, 73.82116006833333)
(18.565826638333334, 73.82115871)
(18.565826796666666, 73.82115727833333)
(18.565826978333334, 73.82115582166666)
(18.565827913333333, 73.82113755833333)
(18.565827913333333, 73.82113755833333)
(18.565827913333333, 73.82113755833333)
(18.565827913333333, 73.82113755833333)
(18.565827913333333, 73.82113755833333)
(18.565828326666665, 73.82113313333333)
(18.565830663333333, 73.82111138166667)
(18.565830863333332, 73.82110971)
(18.565830956666666, 73.82110813666667)
(18.565831628333335, 73.82110048166666)
(18.565831628333335, 73.82110048166666)
(18.565831628333335, 73.82110048166666)
(18.565831628333335, 73.82110048166666)
(18.565831628333335, 73.82110048166666)
(18.565832111666666, 73.82109728833333)
(18.56583210166667, 73.82109567)
(18.565831991666666, 73.82109421333334)
(18.565832741666668, 73.82108316666667)
(18.565833771666668, 73.82107678666667)
(18.565834196666668, 73.82106848166667)
(18.565834141666667, 73.82106326)
(18.565834365, 73.821059815)
(18.565834396666666, 73.82105833166666)
(18.565833543333333, 73.82105217666667)
(18.565833378333334, 73.82105087333333)
(18.565831101666667, 73.82104773333333)
(18.565826551666667, 73.82104434666667)
(18.565823898333335, 73.82104400833333)
(18.565819448333333, 73.82104299166667)
(18.565817575, 73.82104275833333)
(18.565815838333332, 73.82104278333334)
(18.565811011666668, 73.82104261333333)
(18.565807958333334, 73.82104240166667)
(18.565800723333332, 73.82104158333334)
(18.565798833333332, 73.82104148333333)
(18.565797081666666, 73.82104138166666)
(18.565793466666666, 73.82104100833334)
(18.565791538333333, 73.82104075666666)
(18.565789846666668, 73.82104083333333)
(18.56578488, 73.82104015666667)
(18.565783278333335, 73.82103983833333)
(18.565774961666666, 73.82103940166667)
(18.565773258333333, 73.821039175)
(18.565768636666668, 73.82103841666667)
(18.565767316666665, 73.82103828166667)
(18.565765716666668, 73.82103823166666)
(18.565764085, 73.82103805833333)
(18.565759516666667, 73.82103746166666)
(18.565757978333334, 73.82103713833334)
(18.56575176, 73.82103646)
(18.565737748333333, 73.82103498166667)
(18.565736405, 73.82103537166667)
(18.56573509, 73.82103552833334)
(18.565732738333335, 73.82103611666666)
(18.565731771666666, 73.82103634166667)
(18.565724125, 73.82104969)
(18.565723856666665, 73.82105136666667)
(18.565723598333335, 73.82105345166667)
(18.56572286833333, 73.82105732166667)
(18.565720296666665, 73.82107986333334)
(18.565720011666667, 73.82108161666666)
(18.565719466666668, 73.82108743)
(18.565718516666667, 73.82109475666667)
(18.56571831666667, 73.82109860166666)
(18.565717946666666, 73.82110048)
(18.565717395, 73.82110424666666)
(18.565716971666667, 73.82110825)
(18.565716476666665, 73.8211102)
(18.565715211666667, 73.82111752)
(18.565714958333334, 73.82111950833334)
(18.565714233333335, 73.82112313833333)
(18.565713766666665, 73.82112505833334)
(18.56571355166667, 73.82112691833333)
(18.565713668333334, 73.821128735)
(18.565712973333333, 73.82113447166667)
(18.565712623333333, 73.82113633666667)
(18.565712581666666, 73.82114017666666)
(18.565712203333334, 73.82114383)
(18.565712058333332, 73.82114574666667)
(18.565711883333332, 73.82114765333333)
(18.565711368333332, 73.821153285)
(18.56571131666667, 73.82115525)
(18.565710921666668, 73.82115712166667)
(18.565710751666668, 73.82116062833333)
(18.565710663333334, 73.82116596166667)
(18.5657111, 73.82117149333334)
(18.565711143333335, 73.82117351)
(18.565711341666667, 73.82117873166666)
(18.565711838333332, 73.82118225833334)
(18.565712016666666, 73.82118576833334)
(18.565712628333333, 73.82118887666667)
(18.565713055, 73.82119045333333)
(18.565713765, 73.82119192666667)
(18.5657159, 73.82119538166667)
(18.565716816666665, 73.82119639666666)
(18.565720503333335, 73.82119844333333)
(18.565730226666666, 73.82120099166667)
(18.565732033333333, 73.82120132833333)
(18.565735645, 73.82120192166667)
(18.565739456666666, 73.82120216833333)
(18.565741415, 73.821202445)
(18.565746773333334, 73.82120289833334)
(18.56574882, 73.82120284166666)
(18.565750751666666, 73.82120304333333)
(18.565752656666668, 73.82120313833333)
(18.565754583333334, 73.82120324)
(18.565758478333333, 73.82120317333333)
(18.565767606666668, 73.821203875)
(18.56576952, 73.82120384166667)
(18.565771396666666, 73.82120425666666)
(18.565776815, 73.82120463666666)
(18.565778681666668, 73.821204735)
(18.565782418333335, 73.82120503833333)
(18.565785776666665, 73.82120528166666)
(18.56578927, 73.82120530666667)
(18.565794763333333, 73.82120578333334)
(18.565796363333334, 73.82120598333333)
(18.56579813, 73.82120608)
(18.565803283333334, 73.82120669333334)
(18.565804903333333, 73.82120691666667)
(18.56581132333333, 73.82120764333334)
(18.565812811666667, 73.821207875)
(18.565815701666665, 73.82120824666667)
(18.565820016666667, 73.82120840166667)"""

# Main execution
if __name__ == "__main__":
    # Parse boundary coordinates
    boundary_coords = parse_coordinates(coord_data)
    print(f"Parsed {len(boundary_coords)} boundary points")
    
    # Remove duplicates
    unique_coords = []
    for coord in boundary_coords:
        if coord not in unique_coords:
            unique_coords.append(coord)
    
    print(f"Unique boundary points: {len(unique_coords)}")
    
    # Generate dense waypoints for real field coverage (3 meter spacing)
    print("Generating dense waypoints for field coverage...")
    waypoints = create_dense_grid_waypoints(unique_coords, spacing_meters=3.1)
    print(f"Generated {len(waypoints)} waypoints")
    
    if len(waypoints) > 0:
        # Save to CSV file with your specified path
        saved_file = save_waypoints_csv(waypoints, r"F:\GPS\task_2_waypoints\testing")
        
        if saved_file:
            print(f"File successfully saved to: {saved_file}")
        
        # Show visualization
        visualize_field_waypoints(unique_coords, waypoints)
        
        # Print first 10 waypoints as sample
        print(f"\nFirst 10 waypoints:")
        for i, (lat, lon) in enumerate(waypoints[:10]):
            print(f"WP{i+1:03d}: {lat:.8f}, {lon:.8f}")
        
        print(f"\nTotal waypoints generated: {len(waypoints)}")
    else:
        print("No waypoints generated - check boundary coordinates")