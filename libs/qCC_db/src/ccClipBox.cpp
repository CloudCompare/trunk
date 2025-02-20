//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
#include "ccIncludeGL.h"

#include "ccClipBox.h"

//Local
#include "ccCone.h"
#include "ccCylinder.h"
#include "ccHObjectCaster.h"
#include "ccSphere.h"
#include "ccTorus.h"

//system
#include <cassert>

#if defined(_OPENMP)
//OpenMP
#include <omp.h>
#endif

//Components geometry
static QSharedPointer<ccCylinder> c_arrowShaft   (nullptr);
static QSharedPointer<ccCone>     c_arrowHead    (nullptr);
static QSharedPointer<ccSphere>   c_centralSphere(nullptr);
static QSharedPointer<ccTorus>    c_torus        (nullptr);

void DrawUnitArrow(bool entityPickingMode, const CCVector3& start, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, start.x, start.y, start.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0.0, 0.0, 1.0);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = static_cast<PointCoordinateType>(180.0);

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees( acos(ps) );

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_arrowShaft)
		c_arrowShaft.reset(new ccCylinder(0.15f, 0.6f, nullptr, "ArrowShaft", 12, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities
	if (!c_arrowHead)
		c_arrowHead.reset(new ccCone(0.3f, 0, 0.4f, 0, 0, nullptr, "ArrowHead", 24, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	glFunc->glTranslatef(0, 0, 0.3f);
	c_arrowShaft->setTempColor(col);
	c_arrowShaft->showNormals(!entityPickingMode);
	c_arrowShaft->draw(context);
	glFunc->glTranslatef(0, 0, 0.3f + 0.2f);
	c_arrowHead->setTempColor(col);
	c_arrowHead->showNormals(!entityPickingMode);
	c_arrowHead->draw(context);

	glFunc->glPopMatrix();
}

static void DrawUnitTorus(bool entityPickingMode, const CCVector3& center, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0, 0, 1);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = 180;

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees( acos(ps) );

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_torus)
		c_torus.reset(new ccTorus(0.2f, 0.4f, 2.0*M_PI, false, 0, nullptr, "Torus", 12, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	glFunc->glTranslatef(0, 0, 0.3f);
	c_torus->setTempColor(col);
	c_torus->showNormals(!entityPickingMode);
	c_torus->draw(context);

	glFunc->glPopMatrix();
}

static void DrawUnitCross(bool entityPickingMode, const CCVector3& center, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	scale /= 2;
	DrawUnitArrow(entityPickingMode, center, CCVector3(-1, 0, 0), scale, col, context);
	DrawUnitArrow(entityPickingMode, center, CCVector3( 1, 0, 0), scale, col, context);
	DrawUnitArrow(entityPickingMode, center, CCVector3( 0,-1, 0), scale, col, context);
	DrawUnitArrow(entityPickingMode, center, CCVector3( 0, 1, 0), scale, col, context);
	DrawUnitArrow(entityPickingMode, center, CCVector3( 0, 0,-1), scale, col, context);
	DrawUnitArrow(entityPickingMode, center, CCVector3( 0, 0, 1), scale, col, context);
}

ccClipBox::ccClipBox(QString name/*= QString("clipping box")*/, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccHObject(name, uniqueID)
	, m_entityContainer("entities")
	, m_showBox(true)
	, m_activeComponent(NONE)
{
	setSelectionBehavior(SELECTION_IGNORED);

	// declare parts (proxies)
	m_parts.insert(X_MINUS_ARROW, new ccClipBoxPart(this, X_MINUS_ARROW));
	m_parts.insert(X_PLUS_ARROW, new ccClipBoxPart(this, X_PLUS_ARROW));
	m_parts.insert(Y_MINUS_ARROW, new ccClipBoxPart(this, Y_MINUS_ARROW));
	m_parts.insert(Y_PLUS_ARROW, new ccClipBoxPart(this, Y_PLUS_ARROW));
	m_parts.insert(Z_MINUS_ARROW, new ccClipBoxPart(this, Z_MINUS_ARROW));
	m_parts.insert(Z_PLUS_ARROW, new ccClipBoxPart(this, Z_PLUS_ARROW));
	m_parts.insert(CROSS, new ccClipBoxPart(this, CROSS));
	m_parts.insert(X_MINUS_TORUS, new ccClipBoxPart(this, X_MINUS_TORUS));
	m_parts.insert(Y_MINUS_TORUS, new ccClipBoxPart(this, Y_MINUS_TORUS));
	m_parts.insert(Z_MINUS_TORUS, new ccClipBoxPart(this, Z_MINUS_TORUS));
	m_parts.insert(X_PLUS_TORUS, new ccClipBoxPart(this, X_PLUS_TORUS));
	m_parts.insert(Y_PLUS_TORUS, new ccClipBoxPart(this, Y_PLUS_TORUS));
	m_parts.insert(Z_PLUS_TORUS, new ccClipBoxPart(this, Z_PLUS_TORUS));
}

ccClipBox::~ccClipBox()
{
	releaseAssociatedEntities();

	for (auto& part : m_parts)
	{
		delete part;
		part = nullptr;
	}
	m_parts.clear();
}

void ccClipBox::update()
{
	if (m_entityContainer.getChildrenNumber() == 0)
	{
		return;
	}

	//remove any existing clipping plane
	{
		for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
		{
			m_entityContainer.getChild(ci)->removeAllClipPlanes();
		}
	}

	//now add the 6 box clipping planes
	ccBBox extents;
	ccGLMatrix transformation;
	get(extents, transformation);

	CCVector3 C = transformation * extents.getCenter();
	CCVector3 halfDim = extents.getDiagVec() / 2;

	//for each dimension
	for (unsigned d = 0; d < 3; ++d)
	{
		CCVector3 N = transformation.getColumnAsVec3D(d);
		//positive side
		{
			ccClipPlane posPlane;
			posPlane.equation.x = N.x;
			posPlane.equation.y = N.y;
			posPlane.equation.z = N.z;

			//compute the 'constant' coefficient knowing that P belongs to the plane if (P - (C - half_dim * N)).N = 0
			posPlane.equation.w = -static_cast<double>(C.dot(N)) + halfDim.u[d];
			for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
			{
				m_entityContainer.getChild(ci)->addClipPlanes(posPlane);
			}
		}

		//negative side
		{
			ccClipPlane negPlane;
			negPlane.equation.x = -N.x;
			negPlane.equation.y = -N.y;
			negPlane.equation.z = -N.z;

			//compute the 'constant' coefficient knowing that P belongs to the plane if (P - (C + half_dim * N)).N = 0
			//negPlane.equation.w = -(static_cast<double>(C.dot(N)) + halfDim.u[d]);
			negPlane.equation.w = static_cast<double>(C.dot(N)) + halfDim.u[d];
			for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
			{
				m_entityContainer.getChild(ci)->addClipPlanes(negPlane);
			}
		}
	}
}

void ccClipBox::reset()
{
	m_box.clear();
	resetGLTransformation();

	if (m_entityContainer.getChildrenNumber())
	{
		m_box = m_entityContainer.getBB_recursive();
	}

	update();

	//send 'modified' signal
	Q_EMIT boxModified(&m_box);
}

void ccClipBox::set(const ccBBox& extents, const ccGLMatrix& transformation)
{
	m_box = extents;
	setGLTransformation(transformation);

	update();

	//send 'modified' signal
	Q_EMIT boxModified(&m_box);
}

void ccClipBox::get(ccBBox& extents, ccGLMatrix& transformation)
{
	extents = m_box;

	if (isGLTransEnabled())
	{
		transformation = m_glTrans;
	}
	else
	{
		transformation.toIdentity();
	}
}

void ccClipBox::releaseAssociatedEntities()
{
	for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
	{
		m_entityContainer.getChild(ci)->removeAllClipPlanes();
	}
	m_entityContainer.removeAllChildren();
}

bool ccClipBox::addAssociatedEntity(ccHObject* entity)
{
	m_entityContainer.addChild(entity, DP_NONE); //no dependency!

	//no need to reset the clipping box if the entity has not a valid bounding-box
	if (entity->getBB_recursive().isValid())
	{
		reset();
	}

	return true;
}

void ccClipBox::setActiveComponent(Components id)
{
	m_activeComponent = id;
}

static CCVector3d PointToVector(int x, int y, int screenWidth, int screenHeight)
{
	//convert mouse position to vector (screen-centered)
	CCVector3d v(	static_cast<double>(2 * std::max(std::min(x, screenWidth - 1), -screenWidth + 1) - screenWidth) / static_cast<double>(screenWidth),
					static_cast<double>(screenHeight - 2 * std::max(std::min(y, screenHeight - 1), -screenHeight + 1)) / static_cast<double>(screenHeight),
					0);

	//square 'radius'
	double d2 = v.x*v.x + v.y*v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = sqrt(1 - d2);
	}

	return v;
}

// 'move2D' was only possible with the sphere (now deprecated)
#if 0
bool ccClipBox::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	if (m_activeComponent != SPHERE || !m_box.isValid())
		return false;

	//convert mouse position to vector (screen-centered)
	CCVector3d currentOrientation = PointToVector(x, y, screenWidth, screenHeight);

	ccGLMatrixd rotMat = ccGLMatrixd::FromToRotation(m_lastOrientation, currentOrientation);

	CCVector3 C = m_box.getCenter();

	ccGLMatrixd transMat;
	transMat.setTranslation(-C);
	transMat = rotMat * transMat;
	transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

	//rotateGL(transMat);
	m_glTrans = ccGLMatrix(transMat.inverse().data()) * m_glTrans;
	enableGLTransformation(true);

	m_lastOrientation = currentOrientation;

	update();

	return true;
}
#endif

void ccClipBox::setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix)
{
	m_lastOrientation = PointToVector(x, y, screenWidth, screenHeight);
	m_viewMatrix = viewMatrix;
}

bool ccClipBox::move3D(const CCVector3d& uInput)
{
	if (m_activeComponent == NONE || !m_box.isValid())
		return false;

	CCVector3d u = uInput;

	//Arrows
	if (m_activeComponent >= X_MINUS_ARROW && m_activeComponent <= CROSS)
	{
		if (m_glTransEnabled)
			m_glTrans.inverse().applyRotation(u);

		switch(m_activeComponent)
		{
		case X_MINUS_ARROW:
			m_box.minCorner().x += static_cast<PointCoordinateType>(u.x);
			if (m_box.minCorner().x > m_box.maxCorner().x)
				m_box.minCorner().x = m_box.maxCorner().x;
			break;
		case X_PLUS_ARROW:
			m_box.maxCorner().x += static_cast<PointCoordinateType>(u.x);
			if (m_box.minCorner().x > m_box.maxCorner().x)
				m_box.maxCorner().x = m_box.minCorner().x;
			break;
		case Y_MINUS_ARROW:
			m_box.minCorner().y += static_cast<PointCoordinateType>(u.y);
			if (m_box.minCorner().y > m_box.maxCorner().y)
				m_box.minCorner().y = m_box.maxCorner().y;
			break;
		case Y_PLUS_ARROW:
			m_box.maxCorner().y += static_cast<PointCoordinateType>(u.y);
			if (m_box.minCorner().y > m_box.maxCorner().y)
				m_box.maxCorner().y = m_box.minCorner().y;
			break;
		case Z_MINUS_ARROW:
			m_box.minCorner().z += static_cast<PointCoordinateType>(u.z);
			if (m_box.minCorner().z > m_box.maxCorner().z)
				m_box.minCorner().z = m_box.maxCorner().z;
			break;
		case Z_PLUS_ARROW:
			m_box.maxCorner().z += static_cast<PointCoordinateType>(u.z);
			if (m_box.minCorner().z > m_box.maxCorner().z)
				m_box.maxCorner().z = m_box.minCorner().z;
			break;
		case CROSS:
			m_box += u.toPC();
			break;
		default:
			assert(false);
			return false;
		}
		
		//send 'modified' signal
		Q_EMIT boxModified(&m_box);
	}
	//else if (m_activeComponent == SPHERE)
	//{
	//	//handled by move2D!
	//	return false;
	//}
	else if (m_activeComponent >= X_MINUS_TORUS && m_activeComponent <= Z_PLUS_TORUS)
	{
		//we guess the rotation order by comparing the current screen 'normal'
		//and the vector prod of u and the current rotation axis
		CCVector3d Rb(0, 0, 0);
		switch(m_activeComponent)
		{
		case X_MINUS_TORUS:
			Rb.x = -1;
			break;
		case X_PLUS_TORUS:
			Rb.x = 1;
			break;
		case Y_MINUS_TORUS:
			Rb.y = -1;
			break;
		case Y_PLUS_TORUS:
			Rb.y = 1;
			break;
		case Z_MINUS_TORUS:
			Rb.z = -1;
			break;
		case Z_PLUS_TORUS:
			Rb.z = 1;
			break;
		default:
			assert(false);
			return false;
		}
		
		CCVector3d R = Rb;
		if (m_glTransEnabled)
		{
			m_glTrans.applyRotation(R);
		}

		CCVector3d RxU = R.cross(u);

		//look for the most parallel dimension
		double maxDot = m_viewMatrix.getColumnAsVec3D(0).dot(RxU);
		for (int i = 1; i < 3; ++i)
		{
			double dot = m_viewMatrix.getColumnAsVec3D(i).dot(RxU);
			if (std::abs(dot) > std::abs(maxDot))
			{
				maxDot = dot;
			}
		}

		//angle is proportional to absolute displacement
		double angle_rad = u.norm() / m_box.getDiagNorm() * M_PI;
		if (maxDot < 0.0)
			angle_rad = -angle_rad;

		ccGLMatrixd rotMat;
		rotMat.initFromParameters(angle_rad, Rb, CCVector3d(0, 0, 0));

		CCVector3 C = m_box.getCenter();
		ccGLMatrixd transMat;
		transMat.setTranslation(-C);
		transMat = rotMat * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

		m_glTrans = m_glTrans * ccGLMatrix(transMat.inverse().data());
		enableGLTransformation(true);
	}
	else
	{
		assert(false);
		return false;
	}

	update();

	return true;
}

void ccClipBox::setBox(const ccBBox& box)
{
	m_box = box;

	update();

	//send 'modified' signal
	Q_EMIT boxModified(&m_box);
}

void ccClipBox::shift(const CCVector3& v)
{
	m_box.minCorner() += v;
	m_box.maxCorner() += v;
		
	update();

	//send 'modified' signal
	Q_EMIT boxModified(&m_box);
}

void ccClipBox::flagPoints(
	ccGenericPointCloud *cloud,
	ccGenericPointCloud::VisibilityTableType *visTable,
	bool shrink,
	bool inside) const
{
	if (!cloud || !visTable)
	{
		//invalid input
		assert(false);
		return;
	}
	if (cloud->size() != visTable->size())
	{
		///size mismatch
		assert(false);
		return;
	}

	int count = static_cast<int>(cloud->size());

	if (m_glTransEnabled)
	{
		ccGLMatrix transMat = m_glTrans.inverse();

#if defined(_OPENMP)
		#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
		for (int i = 0; i < count; ++i)
		{
			if (!shrink || visTable->at(i) == CCCoreLib::POINT_VISIBLE)
			{
				CCVector3 P = *cloud->getPoint(static_cast<unsigned>(i));
				transMat.apply(P);
				visTable->at(i) = (m_box.contains(P) == inside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
			}
		}
	}
	else
	{
#if defined(_OPENMP)
		#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
		for (int i = 0; i < count; ++i)
		{
			if (!shrink || visTable->at(i) == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3* P = cloud->getPoint(static_cast<unsigned>(i));
				visTable->at(i) = (m_box.contains(*P) == inside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
			}
		}
	}
}

ccBBox ccClipBox::getOwnBB(bool withGLFeatures/*=false*/)
{
	ccBBox bbox = m_box;

	if (withGLFeatures)
	{
		PointCoordinateType scale = computeArrowsScale();
		bbox.minCorner() -= CCVector3(scale, scale, scale);
		bbox.maxCorner() += CCVector3(scale, scale, scale);
	}

	return bbox;
}

PointCoordinateType ccClipBox::computeArrowsScale() const
{
	PointCoordinateType scale = m_box.getDiagNorm() / 10;

	if (m_entityContainer.getChildrenNumber() != 0)
	{
		scale = std::max<PointCoordinateType>(scale, getBox().getDiagNorm() / 25);
	}

	return scale;
}

const ColorCompType c_lightComp = ccColor::MAX/2;
const ccColor::Rgb c_lightRed  (ccColor::MAX, c_lightComp , c_lightComp);
const ccColor::Rgb c_lightGreen(c_lightComp,  ccColor::MAX, c_lightComp);
const ccColor::Rgb c_lightBlue (c_lightComp,  c_lightComp , ccColor::MAX);

void ccClipBox::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	if (!m_box.isValid())
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;


	if (m_showBox)
	{
		//m_box.draw(m_selected ? context.bbDefaultCol : ccColor::magenta);
		m_box.draw(context, ccColor::yellow);
	}
	
	if (!m_selected)
	{
		//nothing to show
		return;
	}

	//color-based entity picking
	bool entityPickingMode = MACRO_FastEntityPicking(context);

	//draw the interactors
	{
		const CCVector3& minC = m_box.minCorner();
		const CCVector3& maxC = m_box.maxCorner();
		const CCVector3 center = m_box.getCenter();
	
		PointCoordinateType scale = computeArrowsScale();

		//custom arrow 'context'
		CC_DRAW_CONTEXT componentContext = context;
		componentContext.drawingFlags &= (~CC_ENTITY_PICKING); //we must remove the 'entity picking flag' so that the arrows don't push their own!
		componentContext.display = nullptr;

		//force the light on
		if (!entityPickingMode)
		{
			glFunc->glPushAttrib(GL_LIGHTING_BIT);
			glFunc->glEnable(GL_LIGHT0);
		}

		DrawUnitArrow(entityPickingMode, CCVector3(minC.x, center.y, center.z), CCVector3(-1.0, 0.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[X_MINUS_ARROW]) : ccColor::red), componentContext);
		DrawUnitArrow(entityPickingMode, CCVector3(maxC.x, center.y, center.z), CCVector3(1.0, 0.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[X_PLUS_ARROW]) : ccColor::red), componentContext);
		DrawUnitArrow(entityPickingMode, CCVector3(center.x, minC.y, center.z), CCVector3(0.0, -1.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Y_MINUS_ARROW]) : ccColor::green), componentContext);
		DrawUnitArrow(entityPickingMode, CCVector3(center.x, maxC.y, center.z), CCVector3(0.0, 1.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Y_PLUS_ARROW]) : ccColor::green), componentContext);
		DrawUnitArrow(entityPickingMode, CCVector3(center.x, center.y, minC.z), CCVector3(0.0, 0.0, -1.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Z_MINUS_ARROW]) : ccColor::blue), componentContext);
		DrawUnitArrow(entityPickingMode, CCVector3(center.x, center.y, maxC.z), CCVector3(0.0, 0.0, 1.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Z_PLUS_ARROW]) : ccColor::blue), componentContext);
		DrawUnitCross(entityPickingMode, minC - CCVector3(scale, scale, scale) / 2.0, scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[CROSS]) : ccColor::yellow), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(minC.x, center.y, center.z), CCVector3(-1.0, 0.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[X_MINUS_TORUS]) : c_lightRed), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(center.x, minC.y, center.z), CCVector3(0.0, -1.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Y_MINUS_TORUS]) : c_lightGreen), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(center.x, center.y, minC.z), CCVector3(0.0, 0.0, -1.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Z_MINUS_TORUS]) : c_lightBlue), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(maxC.x, center.y, center.z), CCVector3(1.0, 0.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[X_PLUS_TORUS]) : c_lightRed), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(center.x, maxC.y, center.z), CCVector3(0.0, 1.0, 0.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Y_PLUS_TORUS]) : c_lightGreen), componentContext);
		DrawUnitTorus(entityPickingMode, CCVector3(center.x, center.y, maxC.z), CCVector3(0.0, 0.0, 1.0), scale, (entityPickingMode ? context.entityPicking.registerEntity(m_parts[Z_PLUS_TORUS]) : c_lightBlue), componentContext);

		if (!entityPickingMode)
		{
			glFunc->glPopAttrib(); //GL_LIGHTING_BIT
		}
	}
}
