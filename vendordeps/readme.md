This folder contains vendor libraries that should be installed.

If `wpi.vendor.loadFrom(project(":meanlib"))` line in build.gradle is present,
then meanlib's vendordep libraries are also being used in this project. So it is unnecessary to add duplicates 
