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

#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalSegmentationOptionsDlg.h"

//Local
#include "mainwindow.h"
#include "ccItemSelectionDlg.h"
#include "ccReservedIDs.h"

//CCCoreLib
#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <cc2DViewportObject.h>

//for the helper (apply)
#include <cc2DLabel.h>
#include <ccCameraSensor.h>
#include <ccGBLSensor.h>
#include <ccSubMesh.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QSettings>

//System
#include <assert.h>

ccGraphicalSegmentationTool::ccGraphicalSegmentationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalSegmentationDlg()
	, m_somethingHasChanged(false)
	, m_state(0)
	, m_segmentationPoly(nullptr)
	, m_polyVertices(nullptr)
	, m_rectangularSelection(false)
	, m_deleteHiddenParts(false)
{
	// Set QDialog background as transparent (DGM: doesn't work over an OpenGL context)
	//setAttribute(Qt::WA_NoSystemBackground);

	setupUi(this);

	connect(inButton,				&QToolButton::clicked, this, &ccGraphicalSegmentationTool::segmentIn);
	connect(outButton,				&QToolButton::clicked, this, &ccGraphicalSegmentationTool::segmentOut);
	connect(razButton,				&QToolButton::clicked, this, &ccGraphicalSegmentationTool::reset);
	connect(optionsButton,			&QToolButton::clicked, this, &ccGraphicalSegmentationTool::options);
	connect(validButton,			&QToolButton::clicked, this, &ccGraphicalSegmentationTool::apply);
	connect(validAndDeleteButton,	&QToolButton::clicked, this, &ccGraphicalSegmentationTool::applyAndDelete);
	connect(cancelButton,			&QToolButton::clicked, this, &ccGraphicalSegmentationTool::cancel);
	connect(pauseButton,			&QToolButton::toggled, this, &ccGraphicalSegmentationTool::pauseSegmentationMode);
	connect(addClassToolButton,	 &	QToolButton::clicked, this, &ccGraphicalSegmentationTool::setClassificationValue);

	//selection modes
	connect(actionSetPolylineSelection,			&QAction::triggered,	this,	&ccGraphicalSegmentationTool::doSetPolylineSelection);
	connect(actionSetRectangularSelection,		&QAction::triggered,	this,	&ccGraphicalSegmentationTool::doSetRectangularSelection);
	//import/export options
	connect(actionUseExistingPolyline,			&QAction::triggered,	this,	&ccGraphicalSegmentationTool::doActionUseExistingPolyline);
	connect(actionExportSegmentationPolyline,	&QAction::triggered,	this,	&ccGraphicalSegmentationTool::doExportSegmentationPolyline);

	//add shortcuts
	addOverriddenShortcut(Qt::Key_Space);	//space bar for the "pause" button
	addOverriddenShortcut(Qt::Key_Escape);	//escape key for the "cancel" button
	addOverriddenShortcut(Qt::Key_Return);	//return key for the "apply" button
	addOverriddenShortcut(Qt::Key_Delete);	//delete key for the "apply and delete" button
	addOverriddenShortcut(Qt::Key_Tab);		//tab key to switch between rectangular and polygonal selection modes
	addOverriddenShortcut(Qt::Key_I);		//'I' key for the "segment in" button
	addOverriddenShortcut(Qt::Key_O);		//'O' key for the "segment out" button
	addOverriddenShortcut(Qt::Key_C);		//'C' key for the "classify" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccGraphicalSegmentationTool::onShortcutTriggered);

	QMenu *selectionModeMenu = new QMenu(this);
	selectionModeMenu->addAction(actionSetPolylineSelection);
	selectionModeMenu->addAction(actionSetRectangularSelection);
	selectionModelButton->setDefaultAction(actionSetPolylineSelection);
	selectionModelButton->setMenu(selectionModeMenu);

	QMenu *importExportMenu = new QMenu(this);
	importExportMenu->addAction(actionUseExistingPolyline);
	importExportMenu->addAction(actionExportSegmentationPolyline);
	loadSaveToolButton->setMenu(importExportMenu);

	m_polyVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	m_segmentationPoly = new ccPolyline(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
	m_segmentationPoly->setForeground(true);
	m_segmentationPoly->setColor(ccColor::green);
	m_segmentationPoly->showColors(true);
	m_segmentationPoly->set2DMode(true);
	allowPolylineExport(false);
}

void ccGraphicalSegmentationTool::allowPolylineExport(bool state)
{
	if (state)
	{
		actionExportSegmentationPolyline->setEnabled(true);
	}
	else
	{
		loadSaveToolButton->setDefaultAction(actionUseExistingPolyline);
		actionExportSegmentationPolyline->setEnabled(false);
	}
}

ccGraphicalSegmentationTool::~ccGraphicalSegmentationTool()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = nullptr;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = nullptr;
}

void ccGraphicalSegmentationTool::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Space:
		// toggle pause mode
		pauseSegmentationMode(!pauseButton->isChecked());
		//pauseButton->toggle();
		return;

	case Qt::Key_I:
		segmentIn();
		return;

	case Qt::Key_O:
		segmentOut();
		return;

	case Qt::Key_C:
		setClassificationValue();
		return;

	case Qt::Key_Return:
		if (m_somethingHasChanged)
			apply();
		//validButton->click();
		return;

	case Qt::Key_Delete:
		if (m_somethingHasChanged)
			applyAndDelete();
		//validAndDeleteButton->click();
		return;

	case Qt::Key_Escape:
		cancel();
		//cancelButton->click();
		return;

	case Qt::Key_Tab:
		if (m_rectangularSelection)
			doSetPolylineSelection();
		else
			doSetRectangularSelection();
		return;

	default:
		//nothing to do
		break;
	}
}

bool ccGraphicalSegmentationTool::linkWith(ccGLWindow *win)
{
	assert(m_segmentationPoly);

	ccGLWindow *oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(nullptr);
		}
	}

	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked,	this, &ccGraphicalSegmentationTool::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked,	this, &ccGraphicalSegmentationTool::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved,			this, &ccGraphicalSegmentationTool::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonReleased,		this, &ccGraphicalSegmentationTool::closeRectangle);

		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(m_associatedWin);
		}
	}

	return true;
}

bool ccGraphicalSegmentationTool::start()
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}

	m_segmentationPoly->clear();
	m_polyVertices->clear();
	allowPolylineExport(false);

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	pauseSegmentationMode(false);

	m_somethingHasChanged = false;

	reset();

	return ccOverlayDialog::start();
}

void ccGraphicalSegmentationTool::prepareEntityForRemoval(ccHObject *entity, bool unallocateVisibilityArrays)
{
	if (!entity)
	{
		assert(false);
		return;
	}

	// restore the display state of the entity
	entity->popDisplayState();

	if (unallocateVisibilityArrays)
	{
		ccGenericPointCloud *asCloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (asCloud)
		{
			asCloud->unallocateVisibilityArray();
		}
	}

	// specific case: we may have automatically hidden the mesh or the polyline associated to a cloud
	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud *cloud = static_cast<ccGenericPointCloud *>(entity);

		ccGenericMesh *associatedMesh = nullptr;
		if (ccGenericMesh::IsCloudVerticesOfMesh(cloud, &associatedMesh) && associatedMesh)
		{
			associatedMesh->popDisplayState();
			return;
		}

		ccPolyline *associatedPolyline = nullptr;
		if (ccPolyline::IsCloudVerticesOfPolyline(cloud, &associatedPolyline) && associatedPolyline)
		{
			associatedPolyline->popDisplayState();
			return;
		}
	}
}

void ccGraphicalSegmentationTool::removeAllEntities(bool unallocateVisibilityArrays)
{
	for (QSet<ccHObject *>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccHObject *entity = *p;

		prepareEntityForRemoval(entity, unallocateVisibilityArrays);
	}

	m_toSegment.clear();
}

void ccGraphicalSegmentationTool::stop(bool accepted)
{
	assert(m_segmentationPoly);

	if (m_associatedWin)
	{
		m_associatedWin->displayNewMessage("Segmentation [OFF]",
											ccGLWindow::UPPER_CENTER_MESSAGE,
											false,
											2,
											ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);

		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		m_associatedWin->removeFromOwnDB(m_segmentationPoly);
	}

	ccOverlayDialog::stop(accepted);
}

void ccGraphicalSegmentationTool::reset()
{
	if (m_somethingHasChanged)
	{
		for (QSet<ccHObject *>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		{
			ccGenericPointCloud *asCloud = ccHObjectCaster::ToGenericPointCloud(*p);
			if (asCloud)
			{
				asCloud->resetVisibilityArray();
			}
		}

		m_somethingHasChanged = false;
	}
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
		m_associatedWin->releaseMouse();
	}
	razButton->setEnabled(false);
	validButton->setEnabled(false);
	validAndDeleteButton->setEnabled(false);
	loadSaveToolButton->setDefaultAction(actionUseExistingPolyline);
}

bool ccGraphicalSegmentationTool::addEntity(ccHObject *entity, bool silent/*=false*/)
{
	if (!entity)
	{
		assert(false);
		return false;
	}

	if (!entity->isDisplayedIn(m_associatedWin) && !silent)
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Entity [%1] is not visible in the active 3D view!").arg(entity->getName()));
	}

	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud *cloud = ccHObjectCaster::ToGenericPointCloud(entity);

		ccGenericMesh *associatedMesh = nullptr;
		if (ccGenericMesh::IsCloudVerticesOfMesh(cloud, &associatedMesh))
		{
			assert(nullptr != associatedMesh);
			if (m_toSegment.contains(associatedMesh))
			{
				if (!silent)
				{
					ccLog::Warning(QString("[Graphical Segmentation Tool] The mesh associated to cloud %1 is already selected").arg(cloud->getName()));
				}
				return false;
			}

			// hide the associated mesh, as it will also be (graphically) segmented
			associatedMesh->pushDisplayState();
			associatedMesh->setVisible(false);
		}

		ccPolyline *associatedPolyline = nullptr;
		if (ccPolyline::IsCloudVerticesOfPolyline(cloud, &associatedPolyline))
		{
			assert(nullptr != associatedPolyline);
			if (m_toSegment.contains(associatedPolyline))
			{
				if (!silent)
				{
					ccLog::Warning(QString("[Graphical Segmentation Tool] The polyline associated to cloud %1 is already selected").arg(cloud->getName()));
				}
				return false;
			}

			// hide the associated polyline, as it will also be (graphically) segmented
			associatedPolyline->pushDisplayState();
			associatedPolyline->setVisible(false);
		}

		cloud->resetVisibilityArray();
		m_toSegment.insert(cloud);
		cloud->pushDisplayState();
		cloud->setVisible(true);
		cloud->setEnabled(true);

		//DGM: not sure what was the idea behind the code below?
		//
		//automatically add cloud's children
		//for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
		//{
		//	ccHObject* child = entity->getChild(i);
		//	if (child != associatedMesh && child != associatedPolyline) // we don't add the associated mesh or polyline (if any)
		//	{
		//		result |= addEntity(entity->getChild(i), /*silent=*/true);
		//	}
		//}

		return true;
	}
	else if (entity->isKindOf(CC_TYPES::MESH))
	{
		if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			if (!silent)
			{
				ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment primitives yet! Sorry...");
			}
			return false;
		}
		if (entity->isKindOf(CC_TYPES::SUB_MESH))
		{
			if (!silent)
			{
				ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment sub-meshes! Select the parent mesh...");
			}
			return false;
		}

		ccGenericMesh *mesh = ccHObjectCaster::ToGenericMesh(entity);
		assert(mesh);

		//DGM: the code below is useless since we don't allow CC_TYPES::SUB_MESH entities (see above)
		//
		// first, we must check that there's no mesh and at least one of its sub-mesh mixed in the current selection!
		//for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		//{
		//	if ((*p)->isKindOf(CC_TYPES::MESH))
		//	{
		//		ccGenericMesh* otherMesh = ccHObjectCaster::ToGenericMesh(*p);
		//		if (otherMesh->getAssociatedCloud() == mesh->getAssociatedCloud())
		//		{
		//			if ((otherMesh->isA(CC_TYPES::SUB_MESH) && mesh->isA(CC_TYPES::MESH))
		//				|| (otherMesh->isA(CC_TYPES::MESH) && mesh->isA(CC_TYPES::SUB_MESH)))
		//			{
		//				if (!silent)
		//				{
		//					ccLog::Warning("[Graphical Segmentation Tool] Can't mix sub-meshes with their parent mesh!");
		//				}
		//				return false;
		//			}
		//		}
		//	}
		//}

		ccGenericPointCloud *vertices = mesh->getAssociatedCloud();
		if (!vertices)
		{
			assert(false);
			return false;
		}

		// Make sure the vertices of this mesh are not already in the 'to segment' list
		if (m_toSegment.contains(vertices))
		{
			//let's remove the vertices
			mesh->pushDisplayState(); // just in case the vertices were inserted before the mesh)
			vertices->popDisplayState();
			m_toSegment.remove(vertices);
		}

		vertices->resetVisibilityArray();
		m_toSegment.insert(mesh);
		mesh->pushDisplayState();
		mesh->setVisible(true);
		mesh->setEnabled(true);

		return true;
	}
	else if (entity->isKindOf(CC_TYPES::POLY_LINE))
	{
		ccPolyline *poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccGenericPointCloud *verticesCloud = dynamic_cast<ccGenericPointCloud *>(poly->getAssociatedCloud());
		if (!verticesCloud)
		{
			assert(false);
			return false;
		}

		// Make sure the vertices of this polyline are not already in the 'to segment' list
		if (verticesCloud && m_toSegment.contains(verticesCloud))
		{
			//let's remove the vertices
			poly->pushDisplayState(); // just in case the vertices were inserted before the polyline)
			verticesCloud->popDisplayState();
			m_toSegment.remove(verticesCloud);
		}

		verticesCloud->resetVisibilityArray();
		m_toSegment.insert(poly);
		poly->pushDisplayState();
		poly->setVisible(true);
		poly->setEnabled(true);

		return true;
	}
	else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//automatically add the entities contained in the group
		bool result = false;
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
			result |= addEntity(entity->getChild(i));

		return result;
	}
	else
	{
		if (!silent)
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment entity " + entity->getName());
		}
		return false;
	}
}

unsigned ccGraphicalSegmentationTool::getNumberOfValidEntities() const
{
	return static_cast<unsigned>(m_toSegment.size());
}

void ccGraphicalSegmentationTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	//process not started yet?
	if ((m_state & RUNNING) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned vertCount = m_polyVertices->size();

	//new point (expressed relatively to the screen center)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (vertCount != 4)
			m_polyVertices->resize(4);

		const CCVector3 *A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3 *B = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3 *C = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3 *D = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x, P.y, 0);
		*C = P;
		*D = CCVector3(P.x, A->y, 0);

		if (vertCount != 4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0, 4))
			{
				ccLog::Error("Out of memory!");
				allowPolylineExport(false);
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE)
	{
		if (vertCount < 2)
			return;
		//we replace last point by the current one
		CCVector3 *lastP = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
		*lastP = P;
	}

	m_associatedWin->redraw(true, false);
}

void ccGraphicalSegmentationTool::addPointToPolylineExt(int x, int y, bool allowClicksOutside)
{
	if ((m_state & STARTED) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (	!allowClicksOutside
			&&	(x < 0 || y < 0 || x >= m_associatedWin->qtWidth() || y >= m_associatedWin->qtHeight())
	   )
	{
		//ignore clicks outside of the 3D view
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned vertCount = m_polyVertices->size();

	//particular case: we close the rectangular selection by a 2nd click
	if (m_rectangularSelection && vertCount == 4 && (m_state & RUNNING))
		return;

	//new point
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	//CTRL key pressed at the same time?
	bool ctrlKeyPressed = m_rectangularSelection || ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0 || ctrlKeyPressed)
	{
		//reset state
		m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
		m_state |= STARTED;
		run();

		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			allowPolylineExport(false);
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			allowPolylineExport(false);
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			if (!m_polyVertices->reserve(vertCount+1))
			{
				ccLog::Error("Out of memory!");
				allowPolylineExport(false);
				return;
			}

			//we replace last point by the current one
			CCVector3 *lastP = const_cast<CCVector3 *>(m_polyVertices->getPointPersistentPtr(vertCount-1));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
		else //we must change mode
		{
			assert(false); //we shouldn't fall here?!
			stopRunning();
			addPointToPolylineExt(x, y, allowClicksOutside);
			return;
		}
	}

	//DGM: to increase the poll rate of the mouse movements in ccGLWindow::mouseMoveEvent
	//we have to completely grab the mouse focus!
	//(the only way to take back the control is to right-click now...)
	m_associatedWin->grabMouse();
	m_associatedWin->redraw(true, false);
}

void ccGraphicalSegmentationTool::closeRectangle()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & RECTANGLE) == 0 || (m_state & RUNNING) == 0)
		return;

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		//first point only? we keep the real time update mechanism
		if (m_rectangularSelection)
			return;
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		allowPolylineExport(false);
	}
	else
	{
		allowPolylineExport(true);
	}

	//stop
	stopRunning();

	if (m_associatedWin)
	{
		m_associatedWin->releaseMouse();
		m_associatedWin->redraw(true, false);
	}
}

void ccGraphicalSegmentationTool::closePolyLine(int, int)
{
	//only for polyline in RUNNING mode
	if ((m_state & POLYLINE) == 0 || (m_state & RUNNING) == 0)
		return;

	if (m_associatedWin)
	{
		m_associatedWin->releaseMouse();
	}

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	else
	{
		//remove last point!
		m_segmentationPoly->resize(vertCount-1); //can't fail --> smaller
		m_segmentationPoly->setClosed(true);
	}

	//stop
	stopRunning();

	//set the default import/export icon to 'export' mode
	loadSaveToolButton->setDefaultAction(actionExportSegmentationPolyline);
	allowPolylineExport(m_segmentationPoly->size() > 1);

	if (m_associatedWin)
	{
		m_associatedWin->redraw(true, false);
	}
}

void ccGraphicalSegmentationTool::segmentIn()
{
	segment(true);
}

void ccGraphicalSegmentationTool::segmentOut()
{
	segment(false);
}

void ccGraphicalSegmentationTool::segment(bool keepPointsInside, ScalarType classificationValue/*=CCCoreLib::NAN_VALUE*/)
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}

	// we must close the polyline if we are in RUNNING mode
	if ((m_state & POLYLINE) != 0 && (m_state & RUNNING) != 0)
	{
		QPoint mousePos = m_associatedWin->mapFromGlobal(QCursor::pos());
		ccLog::Warning(QString("Polyline was not closed - we'll close it with the current mouse cursor position: (%1 ; %2)").arg(mousePos.x()).arg(mousePos.y()));
		addPointToPolylineExt(mousePos.x(), mousePos.y(), true);
		closePolyLine(0, 0);
	}

	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//check if the polyline is totally inside the frustum or not
	bool polyInsideViewport = true;
	{
		int vertexCount = static_cast<int>(m_segmentationPoly->size());
		for (int i = 0; i < vertexCount; ++i)
		{
			const CCVector3* P2D = m_segmentationPoly->getPoint(i);

			if (P2D->x < -half_w || P2D->x > half_w
				|| P2D->y < -half_h || P2D->y > half_h)
			{
				polyInsideViewport = false;
				break;
			}
		}
	}
	ccLog::PrintDebug("Polyline is fully inside viewport: " + QString(polyInsideViewport ? "Yes" : "No"));

	bool classificationMode = CCCoreLib::ScalarField::ValidValue(classificationValue);

	// for each selected entity
	for (QSet<ccHObject *>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud *cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
		assert(!visibilityArray.empty());

		int cloudSize = static_cast<int>(cloud->size());

		// if a classification value is set as input, this means that we want to label the
		// set of points, and we don't want to segment it
		CCCoreLib::ScalarField* classifSF = nullptr;
		if (classificationMode)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(*p);
			if (!pc)
			{
				ccLog::Warning("Can't apply classification to cloud " + (*p)->getName());
				continue;
			}

			// check that the 'Classification' scalar field exists
			int sfIdx = pc->getScalarFieldIndexByName("Classification");
			if (sfIdx < 0)
			{
				// create the scalar field Classification if needed
				sfIdx = pc->addScalarField("Classification");
				if (sfIdx < 0)
				{
					ccLog::Error(tr("Not enough memory"));
					return;
				}
			}
			classifSF = pc->getScalarField(sfIdx);
			pc->showSF(true);
			pc->setCurrentDisplayedScalarField(sfIdx);
		}

		// we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
		#pragma omp parallel for
#endif
		for (int i = 0; i < cloudSize; ++i)
		{
			if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3 *P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustum = false;
				camera.project(*P3D, Q2D, &pointInFrustum);

				bool pointInside = false;
				if (pointInFrustum || !polyInsideViewport) //we can only skip the test if the point is outside the viewport/frustum AND the polyline is fully inside the viewport
				{
					CCVector2 P2D(	static_cast<PointCoordinateType>(Q2D.x - half_w),
									static_cast<PointCoordinateType>(Q2D.y - half_h));

					pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);
				}

				if (classifSF)
				{
					// classification mode
					if (pointInside)
					{
						classifSF->setValue(i, classificationValue);
					}
				}
				else
				{
					// standard segmentation mode
					visibilityArray[i] = (keepPointsInside != pointInside ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE);
				}
			}
		}

		if (classifSF)
		{
			classifSF->computeMinAndMax();
		}
	}

	if (classificationMode)
	{
		m_associatedWin->redraw(false);
	}
	else
	{
		m_somethingHasChanged = true;
		validButton->setEnabled(true);
		validAndDeleteButton->setEnabled(true);
		razButton->setEnabled(true);
		pauseSegmentationMode(true);
	}
}

void ccGraphicalSegmentationTool::run()
{
	m_state |= RUNNING;
	buttonsFrame->setEnabled(false); // we disable the buttons when running
}

void ccGraphicalSegmentationTool::stopRunning()
{
	m_state &= (~RUNNING);
	buttonsFrame->setEnabled(true); // we restore the buttons when running is stopped
}

void ccGraphicalSegmentationTool::pauseSegmentationMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
		return;

	if (state/*=activate pause mode*/)
	{
		stopRunning();
		m_state = PAUSED;
		if (m_polyVertices->size() != 0)
		{
			m_segmentationPoly->clear();
			m_polyVertices->clear();
			allowPolylineExport(false);
		}
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_associatedWin->displayNewMessage("Segmentation [PAUSED]", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Unpause to segment again", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}
	else
	{
		m_state = STARTED;
		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		if (m_rectangularSelection)
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: set opposite corners", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
		else
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (polygonal selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: add contour points / Right click: close", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

	m_associatedWin->redraw(!state);
}

void ccGraphicalSegmentationTool::setClassificationValue()
{
	static int s_classValue = 0;
	bool ok = false;
	int iValue = QInputDialog::getInt(m_associatedWin->asWidget(), QT_TR_NOOP("Classification"), QT_TR_NOOP("value"), s_classValue, -1000000, 1000000, 1, &ok);
	if (!ok)
	{
		return;
	}
	s_classValue = iValue;

	segment(true, static_cast<ScalarType>(s_classValue));
}

void ccGraphicalSegmentationTool::doSetPolylineSelection()
{
	if (!m_rectangularSelection)
		return;

	selectionModelButton->setDefaultAction(actionSetPolylineSelection);

	m_rectangularSelection = false;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalSegmentationTool::doSetRectangularSelection()
{
	if (m_rectangularSelection)
		return;

	selectionModelButton->setDefaultAction(actionSetRectangularSelection);

	m_rectangularSelection=true;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalSegmentationTool::doActionUseExistingPolyline()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	MainWindow *mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		ccHObject *root = mainWindow->dbRootObject();
		ccHObject::Container polylines;
		if (root)
		{
			root->filterChildren(polylines, true, CC_TYPES::POLY_LINE);
		}

		if (!polylines.empty())
		{
			int index = ccItemSelectionDlg::SelectEntity(polylines, 0, this);
			if (index < 0)
				return;
			assert(index >= 0 && index < static_cast<int>(polylines.size()));
			assert(polylines[index]->isA(CC_TYPES::POLY_LINE));
			ccPolyline *poly = static_cast<ccPolyline *>(polylines[index]);

			//look for an associated viewport
			ccHObject::Container viewports;
			if (poly->filterChildren(viewports, false, CC_TYPES::VIEWPORT_2D_OBJECT, true) == 1)
			{
				//shall we apply this viewport?
				if (QMessageBox::question(	m_associatedWin->asWidget(),
											"Associated viewport",
											"The selected polyline has an associated viewport: do you want to apply it?",
											QMessageBox::Yes,
											QMessageBox::No) == QMessageBox::Yes)
				{
					m_associatedWin->setViewportParameters(static_cast<cc2DViewportObject *>(viewports.front())->getParameters());
					m_associatedWin->redraw(false);
				}
			}

			CCCoreLib::GenericIndexedCloudPersist *vertices = poly->getAssociatedCloud();
			bool mode3D = !poly->is2DMode();

			//viewing parameters (for conversion from 3D to 2D)
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;

			//force polygonal selection mode
			doSetPolylineSelection();
			m_segmentationPoly->clear();
			m_polyVertices->clear();
			allowPolylineExport(false);

			//duplicate polyline 'a minima' (only points and indexes + closed state)
			if (	m_polyVertices->reserve(vertices->size() + (poly->isClosed() ? 0 : 1))
					&&	m_segmentationPoly->reserve(poly->size() + (poly->isClosed() ? 0 : 1)))
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					CCVector3 P = *vertices->getPoint(i);
					if (mode3D)
					{
						CCVector3d Q2D;
						camera.project(P, Q2D);

						P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
						P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
						P.z = 0;
					}
					m_polyVertices->addPoint(P);
				}
				for (unsigned j = 0; j < poly->size(); ++j)
				{
					m_segmentationPoly->addPointIndex(poly->getPointGlobalIndex(j));
				}

				m_segmentationPoly->setClosed(poly->isClosed());
				if (m_segmentationPoly->isClosed())
				{
					//stop (but we can't call pauseSegmentationMode as it would remove the current polyline)
					stopRunning();

					allowPolylineExport(m_segmentationPoly->size() > 1);
				}
				else if (vertices->size())
				{
					//we make as if the segmentation was in progress
					pauseSegmentationMode(false);
					unsigned lastIndex = vertices->size() - 1;
					m_polyVertices->addPoint(*m_polyVertices->getPoint(lastIndex));
					m_segmentationPoly->addPointIndex(lastIndex + 1);
					m_segmentationPoly->setClosed(true);
					m_state |= POLYLINE;
					run();
				}

				m_rectangularSelection = false;
				m_associatedWin->redraw(true, false);
			}
			else
			{
				ccLog::Error("Not enough memory!");
			}
		}
		else
		{
			ccLog::Error("No polyline in DB!");
		}
	}
}

static unsigned s_polylineExportCount = 0;
void ccGraphicalSegmentationTool::doExportSegmentationPolyline()
{
	MainWindow *mainWindow = MainWindow::TheInstance();
	if (mainWindow && m_segmentationPoly)
	{
		bool mode2D = false;
#ifdef ALLOW_2D_OR_3D_EXPORT
		QMessageBox messageBox(nullptr);
		messageBox.setWindowTitle("Choose export type");
		messageBox.setText("Export polyline in:\n - 2D (with coordinates relative to the screen)\n - 3D (with coordinates relative to the segmented entities)");
		QPushButton *button2D = new QPushButton("2D");
		QPushButton *button3D = new QPushButton("3D");
		messageBox.addButton(button2D,QMessageBox::AcceptRole);
		messageBox.addButton(button3D,QMessageBox::AcceptRole);
		messageBox.addButton(QMessageBox::Cancel);
		messageBox.setDefaultButton(button3D);
		messageBox.exec();
		if (messageBox.clickedButton() == messageBox.button(QMessageBox::Cancel))
		{
			//process cancelled by user
			return;
		}
		mode2D = (messageBox.clickedButton() == button2D);
#endif

		ccPolyline *poly = new ccPolyline(*m_segmentationPoly);

		//if the polyline is 2D and we export the polyline in 3D, we must project its vertices
		if (!mode2D)
		{
			//get current display parameters
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;

			//project the 2D polyline in 3D
			CCCoreLib::GenericIndexedCloudPersist *vertices = poly->getAssociatedCloud();
			ccPointCloud *verticesPC = dynamic_cast<ccPointCloud *>(vertices);
			if (verticesPC)
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					CCVector3 *Pscreen = const_cast<CCVector3 *>(verticesPC->getPoint(i));
					CCVector3d Pd(half_w + Pscreen->x, half_h + Pscreen->y, 0/*Pscreen->z*/);
					CCVector3d Q3D;
					camera.unproject(Pd, Q3D);
					*Pscreen = Q3D.toPC();
				}
				verticesPC->invalidateBoundingBox();
			}
			else
			{
				assert(false);
				ccLog::Warning("[Segmentation] Failed to convert 2D polyline to 3D! (internal inconsistency)");
				mode2D = false;
			}

			//export Global Shift & Scale info (if any)
			bool hasGlobalShift = false;
			CCVector3d globalShift(0, 0, 0);
			double globalScale = 1.0;
			{
				for (QSet<ccHObject *>::const_iterator it = m_toSegment.constBegin(); it != m_toSegment.constEnd(); ++it)
				{
					ccShiftedObject *shifted = ccHObjectCaster::ToShifted(*it);
					bool isShifted = (shifted && shifted->isShifted());
					if (isShifted)
					{
						globalShift = shifted->getGlobalShift();
						globalScale = shifted->getGlobalScale();
						hasGlobalShift = true;
						break;
					}
				}
			}

			if (hasGlobalShift && m_toSegment.size() != 1)
			{
				hasGlobalShift = (QMessageBox::question(MainWindow::TheInstance(), "Apply Global Shift", "At least one of the segmented entity has been shifted. Apply the same shift to the polyline?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
			}

			if (hasGlobalShift)
			{
				poly->setGlobalShift(globalShift);
				poly->setGlobalScale(globalScale);
			}
		}

		QString polyName = QString("Segmentation polyline #%1").arg(++s_polylineExportCount);
		poly->setName(polyName);
		poly->setEnabled(false); //we don't want it to appear while the segmentation mode is enabled! (anyway it's 2D only...)
		poly->set2DMode(mode2D);
		poly->setColor(ccColor::yellow); //we use a different color so as to differentiate them from the active polyline!

		//save associated viewport
		cc2DViewportObject *viewportObject = new cc2DViewportObject(polyName + QString(" viewport"));
		viewportObject->setParameters(m_associatedWin->getViewportParameters());
		viewportObject->setDisplay(m_associatedWin);
		poly->addChild(viewportObject);

		mainWindow->addToDB(poly, false, false, false);
		ccLog::Print(QString("[Segmentation] Polyline exported (%1 vertices)").arg(poly->size()));
	}
}

void ccGraphicalSegmentationTool::options()
{
	ccGraphicalSegmentationOptionsDlg optionsDlg("Segmentation Options", this);
	if (!optionsDlg.exec())
		return;
}



void ccGraphicalSegmentationTool::apply()
{
	m_deleteHiddenParts = false;
	stop(true);
}

void ccGraphicalSegmentationTool::applyAndDelete()
{
	m_deleteHiddenParts = true;
	stop(true);
}

void ccGraphicalSegmentationTool::cancel()
{
	reset();
	m_deleteHiddenParts = false;
	stop(false);
}

bool ccGraphicalSegmentationTool::applySegmentation(ccMainAppInterface *app, ccHObject::Container &newEntities)
{
	if (!app)
	{
		assert(false);
		return false;
	}

	bool cantModifyPolylinesWarningIssued = false;

	//additional vertices of which visibility array should be manually reset
	std::unordered_set<ccGenericPointCloud *> verticesToReset;

	for (QSet<ccHObject *>::iterator p = m_toSegment.begin(); p != m_toSegment.end(); )
	{
		ccHObject *entity = (*p);

		// check first if we can modify this entity directly or if there might be dire consequences...
		bool canModify = true;
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud *cloud = static_cast<ccGenericPointCloud *>(entity);
			if (cloud->size() == 0)
			{
				//ignore this cloud
				continue;
			}

			// check that the point cloud is not the vertices of a mesh or of a polyline
			if (ccGenericMesh::IsCloudVerticesOfMesh(cloud))
			{
				//we can't delete this cloud
				ccLog::Warning("Cloud " + cloud->getName() + " seems to be the vertices of a mesh. We won't be able to modify it");
				canModify = false;
			}
			else if (ccPolyline::IsCloudVerticesOfPolyline(cloud))
			{
				//we can't delete this cloud
				ccLog::Warning("Cloud " + cloud->getName() + " seems to be the vertices of a polyine. We won't be able to modify it");
				canModify = false;
			}
		}
		else if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh *mesh = static_cast<ccGenericMesh *>(entity);
			if (mesh->size() == 0 || mesh->getAssociatedCloud()->size() == 0)
			{
				//ignore this mesh
				continue;
			}
		}
		else if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline *poly = static_cast<ccPolyline *>(entity);
			if (poly->size() == 0 || poly->getAssociatedCloud()->size() == 0)
			{
				//ignore this polyline
				continue;
			}

			// can't modify polylines yet
			if (!cantModifyPolylinesWarningIssued)
			{
				ccLog::Warning("Can't modify polylines. A new polyline will be created.");
				cantModifyPolylinesWarningIssued = true;
			}
			canModify = false;
		}

		if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
		{
			//first, do the things that must absolutely be done BEFORE removing the entity from DB (even temporarily)
			ccPointCloud *cloud = ccHObjectCaster::ToPointCloud(entity);
			if (!cloud)
			{
				assert(false);
				continue;
			}

			ccMainAppInterface::ccHObjectContext objContext;
			if (canModify)
			{
				//specific case: remove dependent labels (do this before temporarily removing 'entity' from DB!)
				ccHObject::Container labels;

				if (app->dbRootObject())
				{
					app->dbRootObject()->filterChildren(labels, true, CC_TYPES::LABEL_2D);
				}
				for (ccHObject::Container::iterator it = labels.begin(); it != labels.end(); ++it)
				{
					if ((*it)->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
					{
						//we must search for all dependent labels and remove them!!!
						//TODO: couldn't we be more clever and update the label instead?
						cc2DLabel *label = static_cast<cc2DLabel *>(*it);
						bool removeLabel = false;
						for (unsigned i = 0; i < label->size(); ++i)
						{
							if (label->getPickedPoint(i).entity() == entity)
							{
								removeLabel = true;
								break;
							}
						}

						if (removeLabel && label->getParent())
						{
							ccLog::Warning(tr("[Segmentation] Label %1 depends on cloud %2 and will be removed").arg(label->getName(), cloud->getName()));
							ccHObject *labelParent = label->getParent();
							ccMainAppInterface::ccHObjectContext objContext = app->removeObjectTemporarilyFromDBTree(labelParent);
							labelParent->removeChild(label);
							label = nullptr;
							app->putObjectBackIntoDBTree(labelParent, objContext);
						}
					}
					else
					{
						assert(false);
					}
				} //for each label

				//we then temporarily detach the entity, as it may undergo
				//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection
				objContext = app->removeObjectTemporarilyFromDBTree(entity);

			} // if (canModify)

			//apply segmentation
			ccHObject *segmentationResult = nullptr;
			bool deleteOriginalEntity = (m_deleteHiddenParts && canModify);
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccGenericPointCloud *genCloud = ccHObjectCaster::ToGenericPointCloud(entity);
				ccGenericPointCloud *segmentedCloud = genCloud->createNewCloudFromVisibilitySelection(canModify && !m_deleteHiddenParts);
				if (segmentedCloud && segmentedCloud->size() == 0)
				{
					delete segmentedCloud;
					segmentedCloud = nullptr;
				}
				else
				{
					segmentationResult = segmentedCloud;
				}

				deleteOriginalEntity |= (genCloud->size() == 0);
			}
			else if (entity->isKindOf(CC_TYPES::MESH)/*|| entity->isA(CC_TYPES::PRIMITIVE)*/) //TODO
			{
				if (entity->isA(CC_TYPES::MESH))
				{
					segmentationResult = ccHObjectCaster::ToMesh(entity)->createNewMeshFromSelection(canModify && !m_deleteHiddenParts);
				}
				else if (entity->isA(CC_TYPES::SUB_MESH))
				{
					segmentationResult = ccHObjectCaster::ToSubMesh(entity)->createNewSubMeshFromSelection(canModify && !m_deleteHiddenParts);
				}

				deleteOriginalEntity |= (ccHObjectCaster::ToGenericMesh(entity)->size() == 0);
			}

			if (segmentationResult)
			{
				if (canModify)
				{
					//another specific case: remove sensors (on clouds)
					for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
					{
						ccHObject *child = entity->getChild(i);
						assert(child);
						if (child && child->isKindOf(CC_TYPES::SENSOR))
						{
							if (child->isA(CC_TYPES::GBL_SENSOR))
							{
								ccGBLSensor *sensor = ccHObjectCaster::ToGBLSensor(entity->getChild(i));
								//remove the associated depth buffer of the original sensor (derpecated)
								sensor->clearDepthBuffer();
								if (deleteOriginalEntity)
								{
									//either transfer
									entity->transferChild(sensor, *segmentationResult);
								}
								else
								{
									//or copy
									segmentationResult->addChild(new ccGBLSensor(*sensor));
								}
							}
							else if (child->isA(CC_TYPES::CAMERA_SENSOR))
							{
								ccCameraSensor *sensor = ccHObjectCaster::ToCameraSensor(entity->getChild(i));
								if (deleteOriginalEntity)
								{
									//either transfer
									entity->transferChild(sensor, *segmentationResult);
								}
								else
								{
									//or copy
									segmentationResult->addChild(new ccCameraSensor(*sensor));
								}
							}
							else
							{
								//unhandled sensor?!
								assert(false);
							}
						}
					} //for each child
				}
				else
				{
					verticesToReset.insert(cloud);
				}

				//we must take care of the remaining part
				if (!m_deleteHiddenParts)
				{
					if (!deleteOriginalEntity)
					{
						QSettings settings;
						settings.beginGroup("SegmentationToolOptions");
						QString remainingSuffix = settings.value("Remaining", ".remaining").toString();
						settings.endGroup();
						entity->setName(entity->getName() + remainingSuffix);
						if (canModify)
						{
							app->putObjectBackIntoDBTree(entity, objContext);
						}
					}
					else
					{
						//no need to put back the entity in DB if we delete it afterwards!
					}
				}
				else
				{
					//keep original name(s)
					segmentationResult->setName(entity->getName());
					if (entity->isKindOf(CC_TYPES::MESH) && segmentationResult->isKindOf(CC_TYPES::MESH))
					{
						ccGenericMesh *meshEntity = ccHObjectCaster::ToGenericMesh(entity);
						ccHObjectCaster::ToGenericMesh(segmentationResult)->getAssociatedCloud()->setName(meshEntity->getAssociatedCloud()->getName());

						//specific case: if the sub mesh is deleted afterwards (see below)
						//then its associated vertices won't be 'reset' by the segmentation tool!
						if (m_deleteHiddenParts && meshEntity->isA(CC_TYPES::SUB_MESH))
						{
							verticesToReset.insert(meshEntity->getAssociatedCloud());
						}
					}
				}

				if (segmentationResult->isA(CC_TYPES::SUB_MESH))
				{
					//for sub-meshes, we have no choice but to use its parent mesh!
					objContext.parent = static_cast<ccSubMesh *>(segmentationResult)->getAssociatedMesh();
				}
				else
				{
					//otherwise we look for first non-mesh or non-cloud parent
					while (objContext.parent && (objContext.parent->isKindOf(CC_TYPES::MESH) || objContext.parent->isKindOf(CC_TYPES::POINT_CLOUD)))
					{
						objContext.parent = objContext.parent->getParent();
					}
				}

				if (objContext.parent)
				{
					objContext.parent->addChild(segmentationResult); //FiXME: objContext.parentFlags?
				}

				segmentationResult->setDisplay_recursive(entity->getDisplay());
				segmentationResult->prepareDisplayForRefresh_recursive();

				app->addToDB(segmentationResult, false, true, false, false);

				newEntities.push_back(segmentationResult);
			}
			else if (!deleteOriginalEntity)
			{
				//ccConsole::Error(tr("An error occurred! (not enough memory?)"));
				if (canModify)
				{
					app->putObjectBackIntoDBTree(entity, objContext);
				}
			}

			if (deleteOriginalEntity)
			{
				prepareEntityForRemoval(entity, false);

				p = m_toSegment.erase(p);

				delete entity;
				entity = nullptr;
			}
			else
			{
				++p;
			}
		}
		else if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline *poly = static_cast<ccPolyline *>(entity);
			ccHObject *polyParent = poly->getParent();
			if (!polyParent)
			{
				polyParent = app->dbRootObject();
			}
			assert(polyParent);

			std::vector<ccPolyline *> polylines;
			if (poly->createNewPolylinesFromSelection(polylines))
			{
				for (ccPolyline *p : polylines)
				{
					p->setDisplay_recursive(poly->getDisplay());
					if (polyParent)
						polyParent->addChild(p);
					app->addToDB(p, false, true, false, false);
					newEntities.push_back(p);
				}
				poly->prepareDisplayForRefresh();
			}

			++p;
		}
		else
		{
			assert(false);
			++p;
		}
	}

	//specific actions
	{
		for (ccGenericPointCloud *cloud : verticesToReset)
		{
			cloud->resetVisibilityArray();
		}
	}

	removeAllEntities(!m_deleteHiddenParts);

	return true;
}

