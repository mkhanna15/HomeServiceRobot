Introduced two nodes as part of this project:
1. pick_objects
2. add_markers

Main Script:
```./src/scripts/home_service.sh```

Test mapping using 
```./src/scripts/test_slam.sh```

Test navigation using
```./src/scripts/test_navigation.sh```

This exercise includes:
* Robot first localizes in the map/env provided.
* Takes in first location from where an object has to be picked.
* Simulates a robot autonomously traversing to this first location and picking up the object.
* The robot then receives second location coordinates.
* Robot then autonomously navigates to the second location.
* Simulates dropping off the picked object at this second location.

