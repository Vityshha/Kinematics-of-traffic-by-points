# Move control system with Bezier curve trajectory

This code simulates the movement of a model along a Bezier curve trajectory. It allows setting the acceleration of the model and correcting the direction of the model if it goes outside a safe zone.

## Dependencies
  - Python 3.5 or above
  - numpy
  - matplotlib

## Classes

### Point
A class that represents a point in 2D space.

### BezierCurve
A class that represents a Bezier curve with given control points. It can calculate the position of a point on the curve for a given parameter t.

### Tractor
A class that represents a tractor with the ability to move along a Bezier curve trajectory. It can set the trajectory, set the acceleration, check if it is inside a safe zone, and correct its direction if it goes outside the safe zone.

### Functions
simulate_tractor(tractor, dt): a function that simulates the movement of a tractor for a given time dt. It also prints the parameters of the tractor for each step of the simulation and returns the trajectory data for plotting.


## Example


