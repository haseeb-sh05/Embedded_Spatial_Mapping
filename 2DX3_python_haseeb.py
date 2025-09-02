import serial
import numpy as np
import open3d as o3d
import math

# Open serial connection (modify COM port if needed)
s = serial.Serial('COM4', 115200, timeout=10)
print("Opening: " + s.name)

# Reset buffers to clear any leftover data
s.reset_output_buffer()
s.reset_input_buffer()

# Storing data variables
Angle = 0
distanceValues = []
xcoords = []
NumScans = 3 #default number of scans is set to 3

for scanNum in range(NumScans):                                              
    
    temp_xcoords = []
    temp_distanceValues = []
    
    print(f"Data set {scanNum + 1}...")
    input("Press Enter to start...")
    s.write('s'.encode())  

    for i in range(35): # reading 35 lines but ignore the first 3 
        x = s.readline()
        try:
            lineDecoded = x.decode().strip()
            print(f"Data Received was raw: {lineDecoded}")  
            toSplit = lineDecoded.split(",")
            if len(toSplit) != 2:
                raise ValueError("data format is incorrect")
            
            # float conversion
            temp_xcoords.append(float(toSplit[0]))
            temp_distanceValues.append(float(toSplit[1]))
        except (ValueError, IndexError) as e:
            print(f"Parsing line Error: {lineDecoded} - {e}")

    # need 32 valid data points
    if len(temp_xcoords) < 32 or len(temp_distanceValues) < 32:
        print("Error: Not enough valid data points received!")
        s.close()
        exit()
    
    xcoords.extend(temp_xcoords[:32])
    distanceValues.extend(temp_distanceValues[:32])

# Saving data to .txt file
with open("dataPoints3DScan", "w") as f:
    
    for i in range(NumScans*32):  # Now we have 3 sets of 32 points                          
        Angle = (i % 32) * 11.25
        x = xcoords[i]
        y = distanceValues[i] * math.cos(math.radians(Angle))
        z = distanceValues[i] * math.sin(math.radians(Angle))
        print(f"Point {i}: x={x}, y={y}, z={z}")  # Debugging
        f.write(f"{x} {y} {z}\n")

# to close serial transmission
print("Closing: " + s.name)
s.close()

# point cloud
try:
    print("Read the 3D data points(pcd)")
    pcd = o3d.io.read_point_cloud("dataPoints3DScan", format="xyz")
    print("PCD Array for 3D Scan:")
    print(np.asarray(pcd.points))
    
    print("Visualizing scan in 3D.....")
    o3d.visualization.draw_geometries([pcd])
except Exception as e:
    print(f"Error: point cloud not loading: {e}")


# giving each vertex a unique number
yz_slice_vertex = list(range(NumScans*32))                                                   
lines = []

# to connect each yz slice 
for scanNum in range(NumScans):                                                          
    indexStart = scanNum * 32
    for i in range(indexStart, indexStart + 31):
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i + 1]])
    lines.append([yz_slice_vertex[indexStart + 31], yz_slice_vertex[indexStart]])

# connecting points between current and next slices for better 3D visualization 
for scanNum in range(NumScans - 1):  # all sets being connected                               
    indexStart = scanNum * 32
    indexNext = (scanNum + 1) * 32
    for i in range(32):
        lines.append([yz_slice_vertex[indexStart + i], yz_slice_vertex[indexNext + i]])


# LineSet
try:
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
        lines=o3d.utility.Vector2iVector(lines)
    )
    print("point cloud Visualization")
    o3d.visualization.draw_geometries([line_set])
except Exception as e:
    print(f"Error due to line displacement: {e} ")
