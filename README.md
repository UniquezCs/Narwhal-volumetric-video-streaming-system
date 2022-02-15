# Narwhal-volumetric-video-streaming-system
# Requirement
VS2017+PCL1.9.1+QT 5.12
# Function Description
## The main file is mainly included in PCLVISUALIZER.CPP:
### 1. MPD file resolution module (MPD file needs to be generated in a certain format)
### 2. Cut block storage information module
### 3. Judge whether there is any existence of the cut block in the FOV (IS_TILEINFOV function)
### 4. Optimize solve function (need to configure a CPLEX environment)
### 5. Decoding function (requires V-PCC link library)
### 6.Buffer Read (QT Multithread)
### 7. Server communication function (the server uses Nginx configuration)

#Refrence

