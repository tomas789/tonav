BezierCurve
===========

### Algorithm taken from here:
https://www.cs.tcd.ie/publications/tech-reports/reports.94/TCD-CS-94-18.pdf
### Requirements:
Eigen3
### Description:
Generates a Bezier Curve in euclidean or quaternion space with continous velocities and differentiable acceleration.
## Vector Version
```c++
std::vector<Vector3f> waypoints; /* Length N */

std::vector<float> timeForEachSegment; /* Length N - 1 */
// or
float timeForEachSegment; /* if each segment has the same time length */

curve = new BezierCurve(waypoints, timeForEachSegment);

Vector3f p = curve->positionAtTime(t);
Vector3f v = curve->velocityAtTime(t);
Vector3f a = curve->accelerationAtTime(t);
/* Note: v and a are local (of course) */
```
## Quaternion Version
```c++
std::vector<Quaternionf> waypoints; /* Length N */

std::vector<float> timeForEachSegment; /* Length N - 1 */
// or
float timeForEachSegment; /* if each segment has the same time length */

curve = new BezierCurve(waypoints,timeForEachSegment);

Quaternionf q = curve->quaternionAtTime(t);
Vector3f w = curve->angularVelocityAtTime(t); /* NOTE: this is a Vector3f */
```
