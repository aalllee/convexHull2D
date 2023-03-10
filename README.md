
<img align="left" width="64" height="64" src="images/convexHull2D_image.png">

# Convex Hull 2D


### A Cinema 4D plugin to compute 2-dimensional convex hulls.
### Currently built for R25.120


### What is a Convex Hull:
	Let's first take a look at the definition of a convex hull before exploring the possible use cases of the plugin.
	
	Given a set of coplanar points S, a 2-dimensional convex hull C is a smallest convex polygon that
	encloses S.
	
![image](images/chull_def.png)

### Installing the plugin:
	Create a "plugins" folder inside your Cinema 4D R25 directory (by default it is at "C:\Program Files\Maxon Cinema 4D R25").
	Inside the plugins folder create a folder named "Convex Hulls 2D" and paste the "res" folder and "convexhull2d.xdl64" file inside it.
	You will find the plugin under Extensions tab after reloading Cinema 4D.


## How to use the plugin:
	
The Convex Hull 2D object can be found under the "Extensions" tab. 

The Convex Hull is computed on it's children object's data. The data is interpreted differently depending
on the selected mode in the object properties.There are currently two convex hull modes:
**Pivot point based** and **Geometry based**.

### Pivot point based
	By default the Convex Hull 2D object snaps it's childern objects to it's local XZ plane and
	computes convex hull on it's child objects' pivot points. The child objects are free to move around
	in Convex Hull 2D object's XZ plane.

<p float="left">
  <img src="images/Object.png" width="178" />
  <img src="images/chull_default.png" width="600" /> 
</p>

### Geometry based
	To use Geometry based mode we need to enable it first in the Object settings by checking on
	"Geometry based" option. This mode computes the convex hull on the points produced by intersecting
	the world X-Z plane with the childern objects. Note that the children objects must be in editable
	mode to be considered for computation.
	
<p float="left">
  <img src="images/editable.png" width="201" />
  <img src="images/geometry_based.png" width="784" /> 
</p>

### Define custom plane of intersection
	We can also overwrite the default "world X-Z" plane of intersection and define a custom plane
 	of intersection for our convex hull calculations. We do so by linking any object to the "Plane
	of intersection target" linker in the Object properties. Simply create a new object
	(for example a quad polygon), drag and drop it into the link area. Once the object has been linked
	it should have a green bounding box surrounding it. The orientation and the position of the linked 
	object now defines the new orientation and center of the plane of intersection for geometry based mode.
<p float="left">
  <img src="images/linkedobj.png" width="414" />
</p>

<p float="left">
  <img src="images/linked1.png" width="314" />
  <img src="images/linked2.png" width="314" />
  <img src="images/linked3.png" width="314" />
</p>

### Bounding plane
	For a better visualization we can turn on the "Show Bounding Plane" option. This will display
	a green bounding plane of the convex hull points.

<p float="left">
  <img src="images/linked1b.png" width="314" />
  <img src="images/linked2b.png" width="314" />
  <img src="images/linked3b.png" width="314" />
</p>

### Expansion
	The expansion slider allows us to expand/contract the convex hull, similar but not the same as
	to scaling it up or down. More precisely, the expansion direction is based on the sum of a point's
	tangent vectors, which are pointing to the next and previous points within the convex hull. From my
	testing this type of expansion gives the most uniform and stable results, especially when dealing with
	closed geometry. Currently the "Expansion mode" contains "Tangent Based" mode only. I am planning on
	including more modes in the future.
<p float="left">
  <img src="images/expansionMode.png" width="314" />
 
</p>
<p float="left">
  <img src="images/expand0.png" width="314" />
  <img src="images/expand100.png" width="314" />
</p>

### Precision
	Precision parameter can be thought of as a distance threshold between two points. If the distance
	between two given points is smaller than the value of the precision parameter, then the two points
	will be considered as a single point for the convex Hull computation. By default it is set to 0.001.
	You can adjust the precision parameter based on the topology of your geometry and the respective distances
	between child objects. 
	
	Sometimes when dealing with dense geometry it could be helpful to increase the value of the precision
	parameter to increase performance of the algorithm. But as always, it is a trade off between performance and accuracy.

### Future updates
	1) Update current gift wrapping algorithm to a more efficient Graham's Scan Algorithm.
	2) Add a feature to create multiple convex hull splines along geometry by offseting current plane of intersection.
	3) Add bezier and bspline support.
	
