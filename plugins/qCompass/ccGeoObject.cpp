#include "ccGeoObject.h"

#include "ccTopologyRelation.h"

ccGeoObject::ccGeoObject(QString name, ccMainAppInterface* app)
	: ccHObject(name)
{
	init(name, app);

	//add "interior", "upper" and "lower" HObjects
	generateInterior();
	generateUpper();
	generateLower();
}

ccGeoObject::ccGeoObject(ccHObject* obj, ccMainAppInterface* app)
	: ccHObject(obj->getName())
{
	init(obj->getName(), app);

	//n.b. object should already have upper, inner and lower children

}

void ccGeoObject::init(QString name, ccMainAppInterface* app)
{
	setName(name);

	//store reference to app so we can manipulate the database
	m_app = app;

	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoObject");
	setMetaData(*map, true);
}

ccPointCloud* ccGeoObject::getAssociatedCloud()
{
	return m_associatedCloud;
}

ccHObject* ccGeoObject::getRegion(int mappingRegion)
{
	switch (mappingRegion)
	{
	case ccGeoObject::INTERIOR:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_interior_id))
		{
			//item has been deleted... make a new one
			generateInterior();
		}
		return m_interior;
	case ccGeoObject::UPPER_BOUNDARY:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_upper_id))
		{
			//item has been deleted... make a new one
			generateUpper();
		}
		return m_upper;
	case ccGeoObject::LOWER_BOUNDARY:
		//check region hasn't been deleted...
		if (!m_app->dbRootObject()->find(m_lower_id))
		{
			//item has been deleted... make a new one
			generateLower();
		}
		return  m_lower;
	default:
		return nullptr;
	}
}

//gets the topological relationship between this GeoObject and another
int ccGeoObject::getRelationTo(ccGeoObject* obj, ccTopologyRelation** out)
{
	ccTopologyRelation* r = getRelation(this, getUniqueID(), obj->getUniqueID()); //search for a relation belonging to us
	bool invert = false;
	if (!r) //not found - search for a relation belonging to obj
	{
		r = getRelation(obj, getUniqueID(), obj->getUniqueID());
		invert = true; //we need to invert backwards relationships (i.e. Obj OLDER THAN this, so this YOUNGER THAN Obj)
	}

	if (r) //a relation was found
	{
		*out = r; //set out pointer

		if (!invert)
		{
			return r->getType();
		}
		else
		{   //we need to invert backwards relationships (i.e. Obj OLDER THAN this, so this YOUNGER THAN Obj)
			return ccTopologyRelation::invertType(r->getType());
		}
	}
	else
	{
		*out = nullptr;
		return ccTopologyRelation::UNKNOWN;
	}
}

//recurse down the tree looking for the specified topology relationship
ccTopologyRelation* ccGeoObject::getRelation(ccHObject* obj, int id1, int id2)
{
	//is this object the relation we are looking for?
	if (ccTopologyRelation::isTopologyRelation(obj))
	{
		ccTopologyRelation* r = dynamic_cast<ccTopologyRelation*> (obj);
		if (r)
		{
			if ( (r->getOlderID() == id1 && r->getYoungerID() == id2) || 
				 (r->getOlderID() == id2 && r->getYoungerID() == id1) )
			{
				return r; //already has relationship between these objects
			}
		}
	}

	//search children
	for (int i = 0; i < obj->getChildrenNumber(); i++)
	{
		ccTopologyRelation* r = getRelation(obj->getChild(i), id1, id2);
		if (r)
		{
			return r; //cascade positive respones up
		}
	}

	return nullptr; //nothing found
}

//adds a topological relationship between this GeoObject and another
ccTopologyRelation* ccGeoObject::addRelationTo(ccGeoObject* obj2, int type, ccMainAppInterface* app)
{
	//does this relation already exist??
	ccTopologyRelation* out = nullptr;
	getRelationTo(obj2, &out);

	if (out)
	{
		//todo: ask if relation should be replaced?
		app->dispToConsole("Relation already exists!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}
	
	//all good - create and add a new one  (assume relation is in the younger form)
	ccGeoObject* younger = this;
	ccGeoObject* older = obj2;

	//if relation is in older form, invert it
	if (type == ccTopologyRelation::OLDER_THAN ||
		type == ccTopologyRelation::IMMEDIATELY_PRECEDES ||
		type == ccTopologyRelation::NOT_OLDER_THAN)
	{
		type = ccTopologyRelation::invertType(type); //invert type

		//flip start and end (i.e. we are older than obj2)
		younger = obj2;
		older = this; 
	}


	//build point cloud for TopologyRelation
	ccPointCloud* verts = new ccPointCloud("vertices");
	assert(verts);
	verts->setEnabled(false); //this is used for storage only!
	verts->setVisible(false); //this is used for storage only!

	//build TopologyRelation
	ccTopologyRelation* t = new ccTopologyRelation(verts, older->getUniqueID(), younger->getUniqueID(), type);
	t->constructGraphic(older, younger);
	
	//always store with younger member
	younger->getRegion(ccGeoObject::INTERIOR)->addChild(t);
	m_app->addToDB(this, false, false, false, true);

	return t;
}


void ccGeoObject::setActive(bool active)
{
	for (ccHObject* c : m_children)
	{
		recurseChildren(c,active);
	}
}

void ccGeoObject::recurseChildren(ccHObject* par, bool highlight)
{
	//set if par is a measurement
	ccMeasurement* m = dynamic_cast<ccMeasurement*>(par);
	if (m)
	{
		m->setHighlight(highlight);
		
		//draw labels (except for trace objects, when the child plane object will hold the useful info)
		if (!ccTrace::isTrace(par))
		{
			par->showNameIn3D(highlight);

			if (highlight) //show active objects...
			{
				par->setVisible(true);
			}
			else
			{
				//hide annoying graphics (we basically only want traces to be visible)
				if (ccPointPair::isPointPair(par) || ccFitPlane::isFitPlane(par))
				{
					par->setVisible(false);
				}
			}
		}
	}

	//recurse
	for (int i = 0; i < par->getChildrenNumber(); i++)
	{
		ccHObject* c = par->getChild(i);
		recurseChildren(c, highlight);
	}
}

void ccGeoObject::generateInterior()
{
	//check interior doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoInterior"))
		{
			m_interior = c;
			m_interior_id = c->getUniqueID();
			return;
		}
	}

	m_interior = new ccHObject("Interior");
	
	//give them associated property flags
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoInterior");
	m_interior->setMetaData(*map, true);

	//add these to the scene graph
	addChild(m_interior);
	m_interior_id = m_interior->getUniqueID();
}

void ccGeoObject::generateUpper()
{
	//check upper doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoUpperBoundary"))
		{
			m_upper = c;
			m_upper_id = c->getUniqueID();
			return;
		}
	}

	m_upper = new ccHObject("Upper Boundary");

	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoUpperBoundary");
	m_upper->setMetaData(*map, true);

	addChild(m_upper);
	m_upper_id = m_upper->getUniqueID();
}

void ccGeoObject::generateLower()
{
	//check upper doesn't already exist
	for (ccHObject* c : m_children)
	{
		if (c->hasMetaData("ccCompassType") & (c->getMetaData("ccCompassType").toString() == "GeoLowerBoundary"))
		{
			m_lower = c;
			m_lower_id = c->getUniqueID();
			return;
		}
	}

	m_lower = new ccHObject("Lower Boundary");

	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "GeoLowerBoundary");
	m_lower->setMetaData(*map, true);

	addChild(m_lower);
	m_lower_id = m_lower->getUniqueID();
}

bool ccGeoObject::isGeoObject(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoObject");
	}
	return false;
}

bool ccGeoObject::isGeoObjectUpper(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoUpperBoundary");
	}
	return false;
}

bool ccGeoObject::isGeoObjectLower(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoLowerBoundary");
	}
	return false;
}

bool ccGeoObject::isGeoObjectInterior(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("GeoInterior");
	}
	return false;
}

ccGeoObject* ccGeoObject::getGeoObjectParent(ccHObject* object)
{
	while (object != nullptr)
	{
		//is this a GeoObject?
		if (ccGeoObject::isGeoObject(object))
		{
			return dynamic_cast<ccGeoObject*> (object);
		}

		object = object->getParent();
	}

	return nullptr;
}