
#define MAXON_METHOD

#include "convexhull2Dobject.h"
#include "c4d_symbols.h"
#include "oconvexhull2d.h"
#include <algorithm>
#include <functional>
#include <maxon/convexhull.h>
#include <resultbase.h>
#include <maxon/block.h>
#include <maxon/sort.h>
#include <chrono>
#include <lib_ngon.h>
#include <maxon/matrix.h>

#define ID_CONVEXHULL2D 1060345

//helper fn
Float dot(Vector v1, Vector v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

ConvexHull2dObject::ConvexHull2dObject() {
	childNum = 0;
	linkedObjDirtyCt = 0;
	_opDirtySum_old = 0;
}

ConvexHull2dObject::~ConvexHull2dObject() {
	
	
}

Bool ConvexHull2dObject::Init(GeListNode* node) {
	BaseObject* pObj = (BaseObject*)node;
	BaseContainer* bc = pObj->GetDataInstance();
	bc->SetFloat(CONVEXHULL2D_PRECISION, 0.001f);
	bc->SetBool(GEOMETRYBASED, false);
	bc->SetFloat(EXPANSION_FLOAT, 0.0f);
	bc->SetInt32(EXPANSION_MODE, EXPANSION_MODE_TANGENT);
	return true;
}

BaseObject* ConvexHull2dObject::GetVirtualObjects(BaseObject* op, HierarchyHelp* hh) {
	Bool dirty = false;
	BaseContainer* bc = op->GetDataInstance();
	Bool geometryBased = bc->GetBool(GEOMETRYBASED);
	BaseObject* linkObj = bc->GetObjectLink(INTERSECTION_PLANE_TARGET, op->GetDocument());


	Bool checkMatrix = op->IsDirty(DIRTYFLAGS::MATRIX);
	Bool checkDATA = op->IsDirty(DIRTYFLAGS::DATA);
	Bool checkcache = op->CheckCache(hh);
	Bool childDirty = childrenNumber(op);
	Bool linkedObjDirty = linkedObjectDirty(linkObj, geometryBased, op);
	Bool parentDirtyMatrix = parentDirty(op);
	
	if (checkMatrix || checkDATA || checkcache || linkedObjDirty || childDirty || parentDirtyMatrix) {
		dirty = true;
	}

	if (!dirty)
		return op->GetCache(hh);

	
	//Prepare points for Convex Hull calculation
	std::vector<convexHullPoint> convexHullPoints;
	if (!geometryBased) {
		//Convex Hull on childern pivot points in local XZ plane
		updatePlaneOfIntersection(op, geometryBased, linkObj);
		preparePivotPoints(convexHullPoints, op);
	}
	else {
		//Convex hull on world-plane intersection points
		updatePlaneOfIntersection(op, geometryBased, linkObj);
		prepareGeometryPoints(convexHullPoints, op, linkObj);
	}
	

	//Calculate Convex Hull
	ep = bc->GetFloat(CONVEXHULL2D_PRECISION);
	filterPoints(convexHullPoints, ep);
	giftWrap(convexHullPoints, convexHull);

	//recalculate expansion and bounding plane
	Float expansion = bc->GetFloat(EXPANSION_FLOAT);
	if (expansion != 0) {
		updateExpandDirection(convexHull, bc->GetInt32(EXPANSION_MODE));
	}

	if (bc->GetBool(BOUNDINGPLANE)) {
		CalcBoundingPlane(convexHull, op, expansion, geometryBased);
	}

	

		worldToObject.sqmat = op->GetMg().sqmat.GetTransposed();
		worldToObject.off = -1 * worldToObject.sqmat * op->GetMg().off;
		Matrix convexHullTransform = worldToObject.sqmat * planeMat.sqmat;

		if (!geometryBased) {
			convexHullTransform.SetIdentity();
			worldToObject.off = Vector(0, 0, 0);
		}

		for (Int32 i = 0; i < convexHull.size(); i++) {
			convexHull[i].position = (convexHullTransform * (convexHull[i].position + convexHull[i].scaleDirection * expansion)) + worldToObject.off;
		}

	//convex hull to spline//////////////////////////////////////////////////////////
	Int32 vertexCount = convexHull.size();
	SplineObject* sp = SplineObject::Alloc(vertexCount, SPLINETYPE::LINEAR);
	sp->GetDataInstance()->SetBool(SPLINEOBJECT_CLOSED, true);
	Vector* padr = sp->GetPointW();
	
	if (padr && vertexCount>1) {
		for (Int32 i = 0; i < vertexCount; i++) {
			padr[i] = convexHull[i % vertexCount].position;
		}
	}

	sp->Message(MSG_UPDATE);
	return sp;
}

void ConvexHull2dObject::giftWrap(std::vector<convexHullPoint>& points, std::vector<convexHullPoint>& hullpoints) {

	hullpoints.clear();

	if (points.size() == 0 || points.size() == 1) {
		hullpoints = {};
		return;
	}
	
	//sort by z
	std::sort(points.begin(), points.end(), [](convexHullPoint a, convexHullPoint b) {return a.position.z < b.position.z;});

	//start point is bottom-most point and belongs in convex hull
	convexHullPoint startPoint = points[0];
	convexHullPoint nextPoint = {Vector(0,0,0), Vector(0,0,0)};
	Vector startDir(1.0f, 0.0f, 0.0f);
	Vector nextDir(0.0f);
	Vector pt(0, 0, 0);
	Vector toPoint(0, 0, 0);
	Float angle = maxon::PI;
	Float angleTemp = 0.0;
	Float dot = 0;
	Int32 ptCt = 0;

	do {
		angle = maxon::PI2;
		hullpoints.push_back(startPoint);

		for (Int32 i = 0; i < points.size(); i++) {
		
		
			if (points[i].Id == startPoint.Id)
				continue;

			pt = points[i].position;
			toPoint = (pt - startPoint.position).GetNormalized();
			
			dot = startDir.x * toPoint.x  + startDir.z * toPoint.z;
			
			dot = std::clamp(dot,-1.0,1.0);
			
			angleTemp = acos(dot);

			//find smallest counter clockwise angle to other point
			if (angle > angleTemp ) {
				angle = angleTemp;
				nextPoint.position = pt;
				nextPoint.Id = points[i].Id;
				nextDir = (pt - startPoint.position);
			}
		}

		ptCt++;

		if (ptCt > (points.size() + 1)) {
			// the convex hull should be at most the size of the input data
			//keep to find unstable cases
			ApplicationOutput("While condition wasn't met, adjust precision parameter.");
			break;
		}

		startPoint = nextPoint;
		startDir = nextDir.GetNormalized();
		startPoint.Id = nextPoint.Id;

	} while (startPoint.Id != points[0].Id);


	
}

Bool ConvexHull2dObject::EdgePlaneIntersection(Vector& planeN, Vector &planeC, Vector &p1,Vector& p2, Vector& intersection) {
	//plane center to Point direction
	Vector CP1 = p1 - planeC;
	Vector CP2 = p2 - planeC;

	Float len1 = CP1.GetLength();
	Float len2 = CP2.GetLength();
	
	CP1.Normalize();
	CP2.Normalize();
	planeN.Normalize();

	Float p1DotN = dot(planeN, CP1);
	Float p2DotN = dot(planeN, CP2);

	if (abs(p1DotN) < 0.00001f) {
		intersection = p1;
		return true;
	}
	if (abs(p2DotN) < 0.00001f) {
		intersection = p2;
		return true;
	}

	if (p1DotN * p2DotN > 0) {
		return false;
	}

	

	Float theta1 = acos(p1DotN);
	Float theta2 = acos(p2DotN);

	if (theta1 > maxon::PI && !(abs(theta1 - maxon::PI) < 0.0000001)) {
		theta1 -= maxon::PI;
	}
	else if (theta2 > maxon::PI && !(abs(theta2 - maxon::PI) < 0.0000001)) {
		
		theta2 -= maxon::PI;
	}

	Float dist1 = len1 * sin(maxon::PI05 - theta1);

	Float dist2 = len2 * sin(maxon::PI05 - theta2);
	
	Float diff = dist1 - dist2;

	Float t;
	if (abs(diff) < 0.0000001) {
		t = 0.5f;
	}
	else {
		t = dist1 / (dist1 - dist2);
	}

	intersection = p1 + t * (p2 - p1);

	return true;
}

void ConvexHull2dObject::CalcBoundingPlane(const std::vector<convexHullPoint>& hullpoints, BaseObject* op,Float expansion, Bool geometryBased) {

	if (hullpoints.size() == 0) {
		bounds[0] = Vector(0);
		bounds[1] = Vector(0);
		bounds[2] = Vector(0);
		bounds[3] = Vector(0);

	}
	else {
		//find bounding rectangle
		for (Int32 i = 0; i < hullpoints.size(); i++) {


			if (i == 0) {
				bounds[0] = hullpoints[0].position + hullpoints[0].scaleDirection * expansion;
				bounds[1] = hullpoints[0].position + hullpoints[0].scaleDirection * expansion;
				bounds[2] = hullpoints[0].position + hullpoints[0].scaleDirection * expansion;
				bounds[3] = hullpoints[0].position + hullpoints[0].scaleDirection * expansion;

			}

			//leftmost
			if (bounds[0].x > (hullpoints[i].position + hullpoints[i].scaleDirection * expansion).x) {
				bounds[0] = (hullpoints[i].position + hullpoints[i].scaleDirection * expansion);
			}
			//rightmost
			if (bounds[1].x < (hullpoints[i].position + hullpoints[i].scaleDirection * expansion).x) {
				bounds[1] = (hullpoints[i].position + hullpoints[i].scaleDirection * expansion);
			}
			if (bounds[2].z > (hullpoints[i].position + hullpoints[i].scaleDirection * expansion).z) {
				bounds[2] = (hullpoints[i].position + hullpoints[i].scaleDirection * expansion);
			}
			if (bounds[3].z < (hullpoints[i].position + hullpoints[i].scaleDirection * expansion).z) {
				bounds[3] = (hullpoints[i].position + hullpoints[i].scaleDirection * expansion);
			}
		}

		bounds[0].z = bounds[3].z; //top-left corner
		bounds[1].z =  bounds[3].z; // top-right
		bounds[2].x = bounds[0].x; // bottom-left
		bounds[3].x = bounds[1].x;//bottom-right x
		bounds[3].z = bounds[2].z;//bottom-right z


		
		
		Vector offset(0, 0, 0);
		if (!geometryBased) {
			offset = op->GetMg().off;
			planeMat.sqmat = op->GetMg().sqmat;
		}
		//bound points to world coords
		bounds[0] = planeMat.sqmat * bounds[0] + offset;
		bounds[1] = planeMat.sqmat * bounds[1] + offset;
		bounds[2] = planeMat.sqmat * bounds[2] + offset;
		bounds[3] = planeMat.sqmat * bounds[3] + offset;
		
	}


}

DRAWRESULT ConvexHull2dObject::Draw(BaseObject* op, DRAWPASS drawpass, BaseDraw* bd, BaseDrawHelp* bh) {
	if (drawpass == DRAWPASS::OBJECT )
	{
		BaseContainer* data = op->GetDataInstance();
		BaseObject* linkObj = data->GetObjectLink(INTERSECTION_PLANE_TARGET, op->GetDocument());
		Bool geometryBased = data->GetBool(GEOMETRYBASED);
		Matrix m1 = bh->GetMg();
		Matrix m = op->GetMg();
		
		if (linkObj && geometryBased) {
			m1 = linkObj->GetMg();
		}
		else if (!linkObj && geometryBased) {
			m1.SetIdentity();
		}
		else if (!geometryBased) {
			m1 = op->GetMg();
		}

		//plane of intersection marker points
		Vector p[4];
		p[0] = (m1 * (Vector(100, 0, 100)));
		p[1] = (m1 * (Vector(-100, 0, 100)));
		p[2] = (m1 * (Vector(-100, 0, -100)));
		p[3] = (m1 * (Vector(100, 0, -100)));

		bd->SetMatrix_Matrix(nullptr, Matrix());

		//draw plane of intersection
		bd->SetPen(Vector(0.0, 1.0, 0.5));
		bd->DrawLine(p[0], p[1], NOCLIP_D);
		bd->DrawLine(p[1], p[2], NOCLIP_D);
		bd->DrawLine(p[2], p[3], NOCLIP_D);
		bd->DrawLine(p[3], p[0], NOCLIP_D);
		
		if (linkObj) {
			m1 = linkObj->GetMg();
			//link plane to convex hull object
			bd->DrawLine(m.off, m1.off, NOCLIP_D);
		}
		
		//draw bounding plane
		if (data->GetBool(BOUNDINGPLANE)) {
			bd->SetPen(Vector(0.0, 1.0, 0.5));
			bd->DrawLine(bounds[0], bounds[1], NOCLIP_D);
			bd->DrawLine(bounds[2], bounds[3], NOCLIP_D);
			bd->DrawLine(bounds[1], bounds[3], NOCLIP_D);
			bd->DrawLine(bounds[0], bounds[2], NOCLIP_D);
		}
		
		//convex hull object global marker
		bd->SetPen(Vector(1.0, 1.0, 0.0));
		bd->DrawLine(m * Vector(100, 0, 0), m * Vector(-100, 0, 0), NOCLIP_D);
		bd->DrawLine(m * Vector(0, 0, 100), m * Vector(0, 0, -100), NOCLIP_D);
		bd->DrawLine(m * Vector( 0, 100, 0), m * Vector( 0, -100, 0), NOCLIP_D);
	}
	
	return DRAWRESULT::OK;
}

Bool ConvexHull2dObject::linkedObjectDirty(BaseObject* linkedObject, Bool geometryBased , BaseObject* op) {
	
	if (linkedObject && geometryBased) {
		linkedObject->SetName(op->GetName() + "_plane link");
		UInt32 dirtyCount = linkedObject->GetDirty(DIRTYFLAGS::MATRIX);

		//check if parent of the plane was moved
		BaseObject* planeParent = linkedObject->GetUp();
		while (planeParent) {
			dirtyCount += planeParent->GetDirty(DIRTYFLAGS::MATRIX);
			planeParent = planeParent->GetUp();
		}

		if (linkedObjDirtyCt != dirtyCount) {
			linkedObjDirtyCt = dirtyCount;
			return true;
		}
	}
	return false;
}

Bool ConvexHull2dObject::childrenNumber(BaseObject* op) {
	BaseObject* child = op->GetDown();
	Int32 childCount = 0;
	Bool childDirty = false;
	

	while (child) {
		childCount++;
		childDirty = childDirty || child->IsDirty(DIRTYFLAGS::MATRIX) || child->IsDirty(DIRTYFLAGS::DATA) || child->IsDirty(DIRTYFLAGS::CACHE);
		child = child->GetNext();
	}
	
	if (childNum != childCount) {
		childNum = childCount;
		return true;
	}

	return childDirty;
}

void ConvexHull2dObject::preparePivotPoints(std::vector<convexHullPoint>& convexHullPoints, BaseObject* op) {

	convexHullPoint childPivotPoint;
	PolygonObject* child = (PolygonObject*)op->GetDown();
	Vector temp(0, 0, 0);
	Int32 pointId = 0;

	while (child) {
		temp = child->GetAbsPos();
		temp.y = 0;
		child->SetAbsPos(temp);
		childPivotPoint.position = temp;
		childPivotPoint.Id = ++pointId;
		convexHullPoints.push_back(childPivotPoint);
		
		child = (PolygonObject*)child->GetNext();
	}

}

void ConvexHull2dObject::prepareGeometryPoints(std::vector<convexHullPoint>& convexHullPoints, BaseObject* op, BaseObject* linkObj) {
	
	Vector intersection;
	PolygonObject* child = (PolygonObject*)op->GetDown();
	BaseObject* parent = op->GetUp();
	const Vector* vertices;
	const CPolygon* polyArray;
	convexHullPoints = {};
	Int32 pointId = 0;
	Int32 polyct = 0;

	convexHullPoint chp;
	chp.scaleDirection = Vector(0, 0, 0);
	chp.position = Vector(0, 0, 0);
	chp.Id = 0;

	
	Matrix childWorldMat;
	Vector p1(0,0,0);
	Vector p2(0, 0, 0);
	Vector p3(0, 0, 0);
	Vector p4(0, 0, 0);


	

	while (child) {
			vertices = child->GetPointR();
			polyArray = child->GetPolygonR();
			polyct = child->GetPolygonCount();	
			childWorldMat = child->GetMg();
			
			//Handle tris and quads
			for (Int32 i = 0; i < polyct; i++) {
				p1 = (childWorldMat * vertices[polyArray[i].a]);
				p2 = (childWorldMat * vertices[polyArray[i].b]);
				p3 = (childWorldMat * vertices[polyArray[i].c]);
				p4 = (childWorldMat * vertices[polyArray[i].d]);

				

				//edge a-b
				if (EdgePlaneIntersection(planeN, planeC, p1, p2, intersection)) {
					if (linkObj) {
						intersection = planeMatInv * intersection;
					}
					chp.position = intersection;
					chp.Id = ++pointId;
					convexHullPoints.push_back(chp);
				}
				//edge b-c
				if (EdgePlaneIntersection(planeN, planeC, p2, p3, intersection)) {
					if (linkObj) {
						intersection = planeMatInv * intersection;
					
					}

					chp.position = intersection;
					chp.Id = ++pointId;
					convexHullPoints.push_back(chp);

				}
				//edge c-d
				if (!(polyArray[i].IsTriangle())) {
					if (EdgePlaneIntersection(planeN, planeC, p3, p4, intersection)) {
						if (linkObj) {
							intersection = planeMatInv * intersection;
						}
						chp.position = intersection;
						chp.Id = ++pointId;
						convexHullPoints.push_back(chp);
					}
				}

				//edge d-a
				if (EdgePlaneIntersection(planeN, planeC, p4, p1, intersection)) {
					if (linkObj) {
						intersection = planeMatInv * intersection;
					}
					chp.position = intersection;
					chp.Id = ++pointId;
					convexHullPoints.push_back(chp);
				}
			}
			child = (PolygonObject*)child->GetNext();
		}
	



}

void ConvexHull2dObject::updatePlaneOfIntersection(BaseObject* op, Bool geometryBased, BaseObject* linkObj) {

	//if no linked object then the plane is set to be world X-Z plane
	planeMat = op->GetMgn();
	planeMat.SetIdentity();
	planeMatInv = planeMat.sqmat.GetTransposed();
	planeN = Vector(0, 1, 0);
	planeC= Vector(0, 0, 0);

	
	//set intersection plane based on linked objects orientation
	if (linkObj && geometryBased) {
		planeMat = linkObj->GetMgn();
		planeMatInv = planeMat.sqmat.GetTransposed();

		planeN = planeMat.sqmat * planeN; //orient plane normal along linked objects Y axis
		planeC = planeMat.off;
	}
}

void ConvexHull2dObject::updateExpandDirection(std::vector<convexHullPoint>& convexHullPoints, Int32 expansionMode) {
	Int32 ptNum = convexHull.size();
	std::list<Int32> eraseSelection = {};
	Vector expand(0, 0, 0);

	for (Int32 i = 1; i < ptNum + 1; i++) {

		Vector dir1 = (convexHullPoints[(i + 1) % ptNum].position - convexHullPoints[i % ptNum].position).GetNormalized();
		Vector dir2 = (convexHullPoints[i - 1].position - convexHullPoints[i % ptNum].position).GetNormalized();

		Float t = dot(dir1, dir2);
		if (t < -0.99999) {
			if (i == ptNum) {
				eraseSelection.push_front(0);
			}
			else {
				eraseSelection.push_back(i);
			}
			continue;
		}

		//tangent based
		if (expansionMode == EXPANSION_MODE_TANGENT) {
			expand = -(dir1 + dir2);
			expand.y = 0;
		}
		
		expand.Normalize();
		convexHullPoints[i % ptNum].scaleDirection = expand;
	}

	//delete non extreme colinear points
	std::list<Int32>::reverse_iterator iter = eraseSelection.rbegin();
	for (; iter != eraseSelection.rend(); ++iter)
	{
		convexHull.erase(convexHull.begin() + *iter);
	}


}

/////stackoverflow.com/questions/38693528/remove-points-from-vector-depending-on-distance-between-pairs-of-points
void ConvexHull2dObject::filterPoints(std::vector<convexHullPoint>& convexHullPoints, Float threshold) {
	UInt nb_removed = 0;
	Float threshold_distance = threshold;

	for (Int32 i = 0; i < convexHullPoints.size() - nb_removed; ++i) {
		for (Int32 j = i + 1; j < convexHullPoints.size() - nb_removed; ++j) {
			
			if ((convexHullPoints[i].position - convexHullPoints[j].position).GetLength() < threshold_distance)
			{

				if (convexHullPoints[i].position.GetLength() < convexHullPoints[j].position.GetLength()) {
					std::iter_swap(convexHullPoints.begin() + i, convexHullPoints.end() - nb_removed - 1);
					nb_removed += 1;
					--i;
					break;
				}
				else {
					std::iter_swap(convexHullPoints.begin() + j, convexHullPoints.end() - nb_removed - 1);
					nb_removed += 1;
					--j;
					break;
				}
			}
		}
	}
	convexHullPoints.erase(convexHullPoints.end() - nb_removed, convexHullPoints.end());
}

Bool ConvexHull2dObject::parentDirty(BaseObject* op) {
	BaseObject* parent = op->GetUp();
	UInt32  opDirtySum = 0;

	while (parent) {
		opDirtySum = opDirtySum + parent->GetDirty(DIRTYFLAGS::MATRIX);
		parent = parent->GetUp();
	}

	if (this->_opDirtySum_old != opDirtySum) {
		_opDirtySum_old = opDirtySum;
		return true;
	}

	return false;
}

Bool RegisterFirstObject() {
	return RegisterObjectPlugin(ID_CONVEXHULL2D, GeLoadString(IDS_CONVEXHULL2D), OBJECT_GENERATOR, ConvexHull2dObject::Alloc, "Oconvexhull2d"_s, AutoBitmap("chull.png"_s), 0);
}