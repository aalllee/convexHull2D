
#pragma once
#include "c4d.h"
#include <vector>

struct convexHullPoint {

	Vector position;
	Vector scaleDirection;
	Int32 Id;

};

class ConvexHull2dObject : public ObjectData {
private:
	Int32 childNum;
	Int32 linkedObjDirtyCt;
	Int32 _opDirtySum_old;
	Vector bounds[4];
	Float ep;

	std::vector<convexHullPoint> convexHull;

	//plane of intersection data for geometry based mode
	Vector planeN;
	Vector planeC;
	Matrix planeMat;
	Matrix planeMatInv;
	Matrix worldToObject;
public:
	ConvexHull2dObject();
	virtual ~ConvexHull2dObject();
	virtual Bool Init(GeListNode *node);
	static NodeData* Alloc() { return NewObj(ConvexHull2dObject) iferr_ignore("ConvexHull2dObject plugin not instanced"); }
	virtual BaseObject* GetVirtualObjects(BaseObject* op, HierarchyHelp* hh);
	void giftWrap(std::vector<convexHullPoint>& points, std::vector<convexHullPoint>& hullpoints);
	Bool EdgePlaneIntersection(Vector& planeN, Vector& planeC, Vector& p1, Vector& p2, Vector& intersection);
	virtual DRAWRESULT Draw(BaseObject* op, DRAWPASS type, BaseDraw* bd, BaseDrawHelp* bh);
	void CalcBoundingPlane(const std::vector<convexHullPoint>& hullpoints, BaseObject* op, Float expansion, Bool geometryBased);
	Bool linkedObjectDirty(BaseObject* linkedObject, Bool geometryBased, BaseObject* op);
	Bool childrenNumber(BaseObject* op);
	void preparePivotPoints(std::vector<convexHullPoint>& convexHullPoints, BaseObject* op);
	void prepareGeometryPoints(std::vector<convexHullPoint>& convexHullPoints, BaseObject* op, BaseObject* linkObj);
	void updatePlaneOfIntersection(BaseObject* op, Bool geometryBased, BaseObject* linkObj);
	void updateExpandDirection(std::vector<convexHullPoint>& convexHullPoints, Int32 expansionMode);
	void filterPoints(std::vector<convexHullPoint>& convexHullPoints, Float threshold);

	Bool parentDirty(BaseObject* op);
	
};






