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

#include "ccClippingBoxTool.h"

//Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccClippingBoxRepeatDlg.h"
#include "ccContourLinesGenerator.h"
#include "ccCropTool.h"
#include "ccGLWindowInterface.h"
#include "ccReservedIDs.h"
#include "mainwindow.h"

#include "db_tree/ccDBRoot.h"

//qCC_db
#include <ccClipBox.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccRasterGrid.h>

//Qt
#include <QMessageBox>

namespace
{
	//Last envelope or contour unique ID
	std::vector<unsigned> s_lastContourUniqueIDs;
	
	//Envelope extraction parameters (global)
	double s_maxEnvelopeEdgeLength = -1.0;

	//Meta-data key: origin entity UUID
	constexpr char s_originEntityUUID[] = "OriginEntityUUID";
	//Meta-data key: slice (unique) ID
	constexpr char s_sliceID[] = "SliceID";
}

// persistent map of the previous box used for each entity
struct ccClipBoxParams
{
	ccBBox box;
	ccGLMatrix trans;
};
static QMap< unsigned, ccClipBoxParams > s_lastBoxParams;

ccClippingBoxTool::ccClippingBoxTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::ClippingBoxDlg()
	, m_clipBox(nullptr)
{
	setupUi(this);

	connect(editBoxToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::editBox);
	connect(extractContourToolButton,		&QToolButton::clicked,				this, &ccClippingBoxTool::extractContour);
	connect(removeLastContourToolButton,	&QToolButton::clicked,				this, &ccClippingBoxTool::removeLastContour);
	connect(exportButton,					&QToolButton::clicked,				this, &ccClippingBoxTool::exportSlice);
	connect(exportMultButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::exportMultSlices);
	connect(resetButton,					&QToolButton::clicked,				this, &ccClippingBoxTool::reset);
	connect(restoreToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::restoreLastBox);
	connect(closeButton,					&QToolButton::clicked,				this, &ccClippingBoxTool::closeDialog);

	connect(showBoxToolButton,				&QToolButton::toggled,				this, &ccClippingBoxTool::toggleBox);
	connect(showInteractorsToolButton,		&QToolButton::toggled,				this, &ccClippingBoxTool::toggleInteractors);

	connect(thickXDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccClippingBoxTool::thicknessChanged);
	connect(thickYDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccClippingBoxTool::thicknessChanged);
	connect(thickZDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccClippingBoxTool::thicknessChanged);

	connect(minusXShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftXMinus);
	connect(plusXShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftXPlus);
	connect(minusYShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftYMinus);
	connect(plusYShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftYPlus);
	connect(minusZShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftZMinus);
	connect(plusZShiftToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::shiftZPlus);

	viewButtonsFrame->setEnabled(true);
	connect(viewUpToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::setTopView);
	connect(viewDownToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::setBottomView);
	connect(viewFrontToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::setFrontView);
	connect(viewBackToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::setBackView);
	connect(viewLeftToolButton,				&QToolButton::clicked,				this, &ccClippingBoxTool::setLeftView);
	connect(viewRightToolButton,			&QToolButton::clicked,				this, &ccClippingBoxTool::setRightView);

	s_maxEnvelopeEdgeLength = -1.0;
	//s_lastContourUniqueIDs.clear();
	removeLastContourToolButton->setEnabled(false);
}

ccClippingBoxTool::~ccClippingBoxTool()
{
	if (m_clipBox)
		delete m_clipBox;
	m_clipBox = nullptr;
}

void ccClippingBoxTool::editBox()
{
	if (!m_clipBox)
		return;

	ccBBox box;
	ccGLMatrix transformation;
	m_clipBox->get(box, transformation);

	//shift the box to its real center
	{
		CCVector3 C = box.getCenter();
		CCVector3 realC = transformation * C;
		box += (realC - C);

		ccGLMatrix transMat;
		transMat.setTranslation(-realC);
		transformation.clearTranslation();
		transMat = transformation * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + realC);
		transformation = transMat;
	}


	ccBoundingBoxEditorDlg bbeDlg(true, false, this); //show the box 'axes' (orientation)
	bbeDlg.setBaseBBox(box, false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Edit clipping box");
	
	//transformation.invert();
	bbeDlg.setBoxAxes(	transformation.getColumnAsVec3D(0),
						transformation.getColumnAsVec3D(1),
						transformation.getColumnAsVec3D(2) );

	if (!bbeDlg.exec())
		return;

	box = bbeDlg.getBox();

	//construct the local box orientation matrix
	{
		CCVector3d X;
		CCVector3d Y;
		CCVector3d Z;
		bbeDlg.getBoxAxes(X, Y, Z);
		//make sure the vectors define an orthogonal basis
		Z = X.cross(Y);
		Y = Z.cross(X);

		X.normalize();
		Y.normalize();
		Z.normalize();
		ccGLMatrixd rotMat;
		rotMat.setColumn(0, X);
		rotMat.setColumn(1, Y);
		rotMat.setColumn(2, Z);

		CCVector3 C = box.getCenter();
		ccGLMatrixd transMat;
		transMat.setTranslation(-C);
		transMat = rotMat * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

		m_clipBox->setGLTransformation(ccGLMatrix(transMat.data()));
	}

	m_clipBox->setBox(box);

	//onBoxModified(&box); //DGM: automatically called by 'm_clipBox'

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void ccClippingBoxTool::toggleInteractors(bool state)
{
	if (m_clipBox)
		m_clipBox->setSelected(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::toggleBox(bool state)
{
	if (m_clipBox)
		m_clipBox->showBox(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool ccClippingBoxTool::addAssociatedEntity(ccHObject* entity)
{
	if (!entity)
	{
		assert(false);
		return false;
	}
	
	//special case
	if (entity->isGroup())
	{
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
		{
			if (!addAssociatedEntity(entity->getChild(i)))
			{
				return false;
			}
		}
		return true;
	}

	if (!m_associatedWin || !m_clipBox)
	{
		ccLog::Error(QString("[Clipping box] No associated 3D view or no valid clipping box!"));
		return false;
	}
	
	//we don't handle entities associated to another context
	if (entity->getDisplay() != m_associatedWin)
	{
		ccLog::Warning(QString("[Clipping box] Can't use entity '%1' cause it's not displayed in the active 3D view!").arg(entity->getName()));
		return false;
	}

	bool firstEntity = (m_clipBox && m_clipBox->getContainer().getChildrenNumber() == 0);
	if (firstEntity)
	{
		restoreToolButton->setEnabled(false);
		contourGroupBox->setEnabled(false);
	}

	if (!m_clipBox->addAssociatedEntity(entity))
	{
		//error message already issued
		ccLog::Error("An error occurred (see Console)");
		return false;
	}

	if (s_lastBoxParams.contains(entity->getUniqueID()))
	{
		restoreToolButton->setEnabled(true);
	}

	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		contourGroupBox->setEnabled(true);
	}

	//force visibility
	entity->setVisible(true);
	entity->setEnabled(true);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	//set proper "steps" value for slice thickness editors
	{
		CCVector3 diag = m_clipBox->getBox().getDiagVec();
		thickXDoubleSpinBox->setSingleStep(diag.x / 100.0);
		thickYDoubleSpinBox->setSingleStep(diag.y / 100.0);
		thickZDoubleSpinBox->setSingleStep(diag.z / 100.0);
	}

	s_maxEnvelopeEdgeLength = -1.0;
	s_lastContourUniqueIDs.resize(0);
	removeLastContourToolButton->setEnabled(false);

	return true;
}

unsigned ccClippingBoxTool::getNumberOfAssociatedEntity() const
{
	return m_clipBox ? m_clipBox->getContainer().getChildrenNumber() : 0;
}

bool ccClippingBoxTool::linkWith(ccGLWindowInterface* win)
{
	if (m_associatedWin && m_clipBox)
	{
		// restore the the clipping planes
		m_associatedWin->setClippingPlanesEnabled(true);

		//remove clipping box from previous window
		m_associatedWin->removeFromOwnDB(m_clipBox);
		m_clipBox->disconnect(this);
		delete m_clipBox;
		m_clipBox = nullptr;
		m_associatedWin->redraw();
	}

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (win)
	{
		if (!m_clipBox)
		{
			m_clipBox = new ccClipBox(QString(), static_cast<unsigned>(ReservedIDs::CLIPPING_BOX));
			m_clipBox->setVisible(true);
			m_clipBox->setEnabled(true);
			m_clipBox->setSelected(showInteractorsToolButton->isChecked());
			connect(m_clipBox, &ccClipBox::boxModified, this, &ccClippingBoxTool::onBoxModified);
		}

		// the clipping box tool doesn't tak the clipping planes into account
		// so we'll disable them temporarily
		m_associatedWin->setClippingPlanesEnabled(false);

		m_associatedWin->addToOwnDB(m_clipBox);
	}
	
	return true;
}

bool ccClippingBoxTool::start()
{
	assert(!m_processing);
	assert(m_associatedWin);
	if (!m_associatedWin || !m_clipBox)
		return false;

	m_clipBox->reset();

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	//m_associatedWin->displayNewMessage(QString(), ccGLWindowInterface::UPPER_CENTER_MESSAGE); //clear the area
	//m_associatedWin->displayNewMessage("[Rotation/Translation mode]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
	m_associatedWin->redraw();

	return ccOverlayDialog::start();
}

void ccClippingBoxTool::stop(bool state)
{
	if (m_clipBox)
	{
		if (state && m_clipBox->getContainer().getChildrenNumber())
		{
			//save clip box parameters
			ccClipBoxParams params;
			m_clipBox->get(params.box, params.trans);
			for (unsigned ci = 0; ci != m_clipBox->getContainer().getChildrenNumber(); ++ci)
			{
				s_lastBoxParams[m_clipBox->getContainer().getChild(ci)->getUniqueID()] = params;
			}
		}

		m_clipBox->releaseAssociatedEntities();
	}

	if (m_associatedWin)
	{
		m_associatedWin->setUnclosable(false);
		//m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 2, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->redraw();
	}

	ccOverlayDialog::stop(state);
}

void ccClippingBoxTool::removeLastContour()
{
	if (s_lastContourUniqueIDs.empty())
		return;

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		for (size_t i = 0; i < s_lastContourUniqueIDs.size(); ++i)
		{
			ccHObject* obj = mainWindow->db()->find(s_lastContourUniqueIDs[i]);
			if (obj)
			{
				//obj->prepareDisplayForRefresh();
				mainWindow->removeFromDB(obj);
				ccGLWindowInterface* win = mainWindow->getActiveGLWindow();
				if (win)
					win->redraw();
			}
		}
	}

	s_lastContourUniqueIDs.resize(0);
	removeLastContourToolButton->setEnabled(false);
}

ccHObject* GetSlice(ccHObject* obj, ccClipBox* clipBox, bool silent, bool inside)
{
	assert(clipBox);
	if (!obj)
	{
		assert(false);
		return nullptr;
	}

	if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* inputCloud = ccHObjectCaster::ToGenericPointCloud(obj);

		ccGenericPointCloud::VisibilityTableType selectionTable;
		try
		{
			selectionTable.resize(inputCloud->size());
		}
		catch (const std::bad_alloc&)
		{
			if (!silent)
			{
				ccLog::Error("Not enough memory!");
			}
			return nullptr;
		}
		clipBox->flagPoints(inputCloud, &selectionTable, false, inside);

		ccGenericPointCloud* sliceCloud = inputCloud->createNewCloudFromVisibilitySelection(false, &selectionTable, nullptr, true);

		// specific case: all points were selected
		if (sliceCloud == inputCloud)
		{
			sliceCloud = inputCloud->clone();
		}

		if (!sliceCloud)
		{
			if (!silent)
				ccLog::Error("Not enough memory!");
		}
		else if (sliceCloud->size() == 0)
		{
			//ccLog::Warning("Empty slice"); // too many logs
			delete sliceCloud;
			sliceCloud = nullptr;
		}
		return sliceCloud;
	}
	else if (obj->isKindOf(CC_TYPES::MESH))
	{
		const ccGLMatrix* _transformation = nullptr;
		ccGLMatrix transformation;
		if (clipBox->isGLTransEnabled())
		{
			transformation = clipBox->getGLTransformation().inverse();
			_transformation = &transformation;
		}

		const ccBBox& cropBox = clipBox->getBox();
		ccHObject* mesh = ccCropTool::Crop(obj, cropBox, true, _transformation);
		if (!mesh)
		{
			if (!silent)
				ccLog::Error("Failed to segment the mesh!");
			return nullptr;
		}
		return mesh;
	}

	return nullptr;
}

void ccClippingBoxTool::exportSlice()
{
	if (!m_clipBox || !MainWindow::TheInstance())
		return;

	for (unsigned ci = 0; ci != m_clipBox->getContainer().getChildrenNumber(); ++ci)
	{
		ccHObject* obj = m_clipBox->getContainer().getChild(ci);
		if (!obj)
		{
			assert(false);
			continue;
		}

		ccHObject* result = GetSlice(obj, m_clipBox, false, !invertSelectionButton->isChecked());

		if (result)
		{
			result->setName(obj->getName() + QString(".section"));
			result->setDisplay(obj->getDisplay());
			result->prepareDisplayForRefresh();
			if (obj->getParent())
				obj->getParent()->addChild(result);
			MainWindow::TheInstance()->addToDB(result);
		}
	}
}

void ccClippingBoxTool::extractContour()
{
	extractSlicesAndContours(/*singleSliceMode=*/true);
}

void ccClippingBoxTool::exportMultSlices()
{
	extractSlicesAndContours(/*singleSliceMode=*/false);
}

static unsigned ComputeGridDimensions(	const ccBBox& localBox,
										const bool processDim[3],
										int indexMins[3],
										int indexMaxs[3],
										int gridDim[3],
										const CCVector3& gridOrigin,
										const CCVector3& cellSizePlusGap)
{
	//compute 'grid' extents in the local clipping box ref.
	for (int i = 0; i < 3; ++i)
	{
		indexMins[i] = 0;
		indexMaxs[i] = 0;
		gridDim[i]   = 1;
	}
	unsigned cellCount = 1;

	for (unsigned char d = 0; d < 3; ++d)
	{
		if (processDim[d])
		{
			if (CCCoreLib::LessThanEpsilon(cellSizePlusGap.u[d]))
			{
				ccLog::Error("Box size (plus gap) is null! Can't apply repetitive process!");
				return 0;
			}

			PointCoordinateType a = (localBox.minCorner().u[d] - gridOrigin.u[d]) / cellSizePlusGap.u[d]; //don't forget the user defined gap between 'cells'
			PointCoordinateType b = (localBox.maxCorner().u[d] - gridOrigin.u[d]) / cellSizePlusGap.u[d];

			indexMins[d] = static_cast<int>(floor(a + static_cast<PointCoordinateType>(1.0e-6)));
			indexMaxs[d] = static_cast<int>(ceil(b - static_cast<PointCoordinateType>(1.0e-6))) - 1;

			assert(indexMaxs[d] >= indexMins[d]);
			gridDim[d] = std::max(indexMaxs[d] - indexMins[d] + 1, 1);
			cellCount *= static_cast<unsigned>(gridDim[d]);
		}
	}

	return cellCount;
}

bool ccClippingBoxTool::ExtractSlicesAndContours
(
	const std::vector<ccGenericPointCloud*>& clouds,
	const std::vector<ccGenericMesh*>& meshes,
	ccClipBox& clipBox,
	bool singleSliceMode,
	bool repeatDimensions[3],
	std::vector<ccHObject*>& outputSlices,

	bool extractEnvelopes,
	PointCoordinateType maxEdgeLength,
	ccEnvelopeExtractor::EnvelopeType envelopeType,
	std::vector<ccPolyline*>& outputEnvelopes,

	bool extractLevelSet,
	double levelSetGridStep,
	int levelSetMinVertCount,
	std::vector<ccPolyline*>& levelSet,

	PointCoordinateType gap/*=0*/,
	bool multiPass/*=false*/,
	bool splitEnvelopes/*=false*/,
	bool projectOnBestFitPlane/*=false*/,
	bool visualDebugMode/*=false*/,
	bool generateRandomColors/*=false*/,
	ccProgressDialog* progressDialog/*=nullptr*/)
{
	//check input
	if (clouds.empty() && meshes.empty())
	{
		assert(false);
		return false;
	}

	//repeat dimensions
	int repeatDimensionsSum = static_cast<int>(repeatDimensions[0])
							+ static_cast<int>(repeatDimensions[1])
							+ static_cast<int>(repeatDimensions[2]);

	if (!singleSliceMode && repeatDimensionsSum == 0)
	{
		assert(false);
		ccLog::Error("No dimension selected to repeat the segmentation process?!");
		return false;
	}

	if (extractLevelSet && repeatDimensionsSum != 1)
	{
		assert(false);
		ccLog::Error("Only one repeat/flat dimension should be defined for level set extraction");
		extractLevelSet = false;
	}

	//compute the cloud bounding box in the local clipping box ref.
	ccGLMatrix localTrans;
	{
		if (clipBox.isGLTransEnabled())
			localTrans = clipBox.getGLTransformation().inverse();
		else
			localTrans.toIdentity();
	}

	CCVector3 gridOrigin = clipBox.getOwnBB().minCorner();
	CCVector3 cellSize = clipBox.getOwnBB().getDiagVec();
	CCVector3 cellSizePlusGap = cellSize + CCVector3(gap, gap, gap);

	//apply process
	try
	{
		bool error = false;
		bool warningsIssued = false;
		size_t cloudSliceCount = 0;

		if (singleSliceMode)
		{
			//single slice: easy
			outputSlices.reserve(clouds.size());
			for (size_t ci = 0; ci != clouds.size(); ++ci)
			{
				ccHObject* slice = GetSlice(clouds[ci], &clipBox, false, false);
				if (slice)
				{
					slice->setName(clouds[ci]->getName() + QString(".slice"));
					
					//set meta-data
					slice->setMetaData(s_originEntityUUID, clouds[ci]->getUniqueID());
					slice->setMetaData(s_sliceID, "slice");
					if (slice->isKindOf(CC_TYPES::POINT_CLOUD))
					{
						slice->setMetaData("slice.origin.dim(0)", gridOrigin.x);
						slice->setMetaData("slice.origin.dim(1)", gridOrigin.y);
						slice->setMetaData("slice.origin.dim(2)", gridOrigin.z);
					}

					outputSlices.push_back(slice);
				}
			}

			if (outputSlices.empty())
			{
				//error message already issued
				return false;
			}
			cloudSliceCount = outputSlices.size();
		}
		else //repeat mode
		{
			if (!clouds.empty()) //extract sections from clouds
			{
				//compute 'grid' extents in the local clipping box ref.
				ccBBox localBox;
				for (ccGenericPointCloud* cloud : clouds)
				{
					for (unsigned i = 0; i < cloud->size(); ++i)
					{
						CCVector3 P = *cloud->getPoint(i);
						localTrans.apply(P);
						localBox.add(P);
					}
				}

				int indexMins[3]{ 0, 0, 0 };
				int indexMaxs[3]{ 0, 0, 0 };
				int gridDim[3]{ 0, 0, 0 };
				unsigned cellCount = ComputeGridDimensions(localBox, repeatDimensions, indexMins, indexMaxs, gridDim, gridOrigin, cellSizePlusGap);

				//we'll potentially create up to one (ref.) cloud per input loud and per cell
				std::vector<CCCoreLib::ReferenceCloud*> refClouds;
				refClouds.resize(cellCount * clouds.size(), nullptr);

				if (progressDialog)
				{
					progressDialog->setWindowTitle(tr("Preparing extraction"));
					progressDialog->start();
					progressDialog->show();
					progressDialog->setAutoClose(false);
				}

				unsigned subCloudsCount = 0;

				//project points into grid
				for (size_t ci = 0; ci != clouds.size(); ++ci)
				{
					ccGenericPointCloud* cloud = clouds[ci];
					unsigned pointCount = cloud->size();

					QString infos = tr("Cloud '%1").arg(cloud->getName());
					infos += tr("Points: %L1").arg( pointCount );
					if (progressDialog)
					{
						progressDialog->setInfo(infos);
					}
					QApplication::processEvents();

					CCCoreLib::NormalizedProgress nProgress(progressDialog, pointCount);
					for (unsigned i = 0; i < pointCount; ++i)
					{
						CCVector3 P = *cloud->getPoint(i);
						localTrans.apply(P);

						//relative coordinates (between 0 and 1)
						P -= gridOrigin;
						P.x /= cellSizePlusGap.x;
						P.y /= cellSizePlusGap.y;
						P.z /= cellSizePlusGap.z;

						int xi = static_cast<int>(floor(P.x));
						xi = std::min(std::max(xi, indexMins[0]), indexMaxs[0]);
						int yi = static_cast<int>(floor(P.y));
						yi = std::min(std::max(yi, indexMins[1]), indexMaxs[1]);
						int zi = static_cast<int>(floor(P.z));
						zi = std::min(std::max(zi, indexMins[2]), indexMaxs[2]);

						if (gap == 0 ||
							(	(P.x - static_cast<PointCoordinateType>(xi))*cellSizePlusGap.x <= cellSize.x
							&&	(P.y - static_cast<PointCoordinateType>(yi))*cellSizePlusGap.y <= cellSize.y
							&&	(P.z - static_cast<PointCoordinateType>(zi))*cellSizePlusGap.z <= cellSize.z))
						{
							int cloudIndex = ((zi - indexMins[2]) * static_cast<int>(gridDim[1]) + (yi - indexMins[1])) * static_cast<int>(gridDim[0]) + (xi - indexMins[0]);
							assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex)* clouds.size() + ci < refClouds.size());

							CCCoreLib::ReferenceCloud*& destCloud = refClouds[cloudIndex * clouds.size() + ci];
							if (!destCloud)
							{
								destCloud = new CCCoreLib::ReferenceCloud(cloud);
								++subCloudsCount;
							}

							if (!destCloud->addPointIndex(i))
							{
								ccLog::Error("Not enough memory!");
								error = true;
								break;
							}
						}
					}

					if (!nProgress.oneStep())
					{
						// sadly, we can't cancel in the middle of this process!
					}
				} //project points into grid

				if (progressDialog)
				{
					progressDialog->setWindowTitle(QObject::tr("Section extraction"));
					progressDialog->setInfo(QObject::tr("Section(s): %L1").arg(subCloudsCount));
					progressDialog->setMaximum(static_cast<int>(subCloudsCount));
					progressDialog->setValue(0);
					QApplication::processEvents();
				}

				//reset count
				subCloudsCount = 0;

				//now create the real clouds
				for (int i = indexMins[0]; i <= indexMaxs[0]; ++i)
				{
					for (int j = indexMins[1]; j <= indexMaxs[1]; ++j)
					{
						for (int k = indexMins[2]; k <= indexMaxs[2]; ++k)
						{
							int cloudIndex = ((k - indexMins[2]) * static_cast<int>(gridDim[1]) + (j - indexMins[1])) * static_cast<int>(gridDim[0]) + (i - indexMins[0]);
							assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex)* clouds.size() < refClouds.size());

							for (size_t ci = 0; ci != clouds.size(); ++ci)
							{
								ccGenericPointCloud* cloud = clouds[ci];
								CCCoreLib::ReferenceCloud* destCloud = refClouds[cloudIndex * clouds.size() + ci];
								if (destCloud) //some slices can be empty!
								{
									//generate slice from previous selection
									int warnings = 0;
									ccPointCloud* sliceCloud = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud)->partialClone(destCloud, &warnings) : ccPointCloud::From(destCloud, cloud);
									warningsIssued |= (warnings != 0);

									if (sliceCloud)
									{
										if (generateRandomColors)
										{
											ccColor::Rgb col = ccColor::Generator::Random();
											if (!sliceCloud->setColor(col))
											{
												ccLog::Error("Not enough memory!");
												error = true;
												i = indexMaxs[0];
												j = indexMaxs[1];
												k = indexMaxs[2];
											}
											sliceCloud->showColors(true);
										}

										sliceCloud->setEnabled(true);
										sliceCloud->setVisible(true);
										sliceCloud->setDisplay(cloud->getDisplay());

										CCVector3 cellOrigin(	gridOrigin.x + i * cellSizePlusGap.x,
																gridOrigin.y + j * cellSizePlusGap.y,
																gridOrigin.z + k * cellSizePlusGap.z);
										QString slicePosStr = QString("(%1 ; %2 ; %3)").arg(cellOrigin.x).arg(cellOrigin.y).arg(cellOrigin.z);
										sliceCloud->setName(cloud->getName() + QString(".slice @ ") + slicePosStr);

										//set meta-data
										sliceCloud->setMetaData(s_originEntityUUID, cloud->getUniqueID());
										sliceCloud->setMetaData(s_sliceID, slicePosStr);
										sliceCloud->setMetaData("slice.origin.dim(0)", cellOrigin.x);
										sliceCloud->setMetaData("slice.origin.dim(1)", cellOrigin.y);
										sliceCloud->setMetaData("slice.origin.dim(2)", cellOrigin.z);

										//add slice to group
										outputSlices.push_back(sliceCloud);
										++subCloudsCount;

										if (progressDialog)
										{
											progressDialog->setValue(static_cast<int>(subCloudsCount));
										}
									}

									if (progressDialog && progressDialog->wasCanceled())
									{
										error = true;
										ccLog::Warning(QString("[ExtractSlicesAndContours] Process canceled by user"));
										//early stop
										i = indexMaxs[0];
										j = indexMaxs[1];
										k = indexMaxs[2];
										break;
									}
								}
							}
						}
					}
				} //now create the real clouds

				//release memory
				{
					for (size_t i = 0; i < refClouds.size(); ++i)
						if (refClouds[i])
							delete refClouds[i];
					refClouds.clear();
				}

				cloudSliceCount = outputSlices.size();

			} //extract sections from clouds

			if (!meshes.empty()) //extract sections from meshes
			{
				//compute 'grid' extents in the local clipping box ref.
				ccBBox localBox;
				for (ccGenericMesh* mesh : meshes)
				{
					ccGenericPointCloud* cloud = mesh->getAssociatedCloud();
					for (unsigned i = 0; i < cloud->size(); ++i)
					{
						CCVector3 P = *cloud->getPoint(i);
						localTrans.apply(P);
						localBox.add(P);
					}
				}

				int indexMins[3]{ 0, 0, 0 };
				int indexMaxs[3]{ 0, 0, 0 };
				int gridDim[3]{ 0, 0, 0 };
				unsigned cellCount = ComputeGridDimensions(localBox, repeatDimensions, indexMins, indexMaxs, gridDim, gridOrigin, cellSizePlusGap);

				const ccGLMatrix* _transformation = nullptr;
				ccGLMatrix transformation;
				if (clipBox.isGLTransEnabled())
				{
					transformation = clipBox.getGLTransformation().inverse();
					_transformation = &transformation;
				}

				if (progressDialog)
				{
					progressDialog->setWindowTitle("Section extraction");
					progressDialog->setInfo(QObject::tr("Up to (%1 x %2 x %3) = %4 section(s)").arg(gridDim[0]).arg(gridDim[1]).arg(gridDim[2]).arg(cellCount));
					progressDialog->setMaximum(static_cast<int>(cellCount * meshes.size()));
					progressDialog->show();
					QApplication::processEvents();
				}

				//now extract the slices
				for (int i = indexMins[0]; i <= indexMaxs[0]; ++i)
				{
					for (int j = indexMins[1]; j <= indexMaxs[1]; ++j)
					{
						for (int k = indexMins[2]; k <= indexMaxs[2]; ++k)
						{
							int sliceIndex = ((k - indexMins[2]) * static_cast<int>(gridDim[1]) + (j - indexMins[1])) * static_cast<int>(gridDim[0]) + (i - indexMins[0]);

							CCVector3 C = gridOrigin + CCVector3(i*cellSizePlusGap.x, j*cellSizePlusGap.y, k*cellSizePlusGap.z);
							ccBBox cropBox(C, C + cellSize, true);

							for (size_t mi = 0; mi != meshes.size(); ++mi)
							{
								ccGenericMesh* mesh = meshes[mi];
								ccHObject* croppedEnt = ccCropTool::Crop(mesh, cropBox, true, _transformation);
								if (croppedEnt)
								{
									if (generateRandomColors)
									{
										ccPointCloud* croppedVertices = ccHObjectCaster::ToPointCloud(mesh->getAssociatedCloud());
										if (croppedVertices)
										{
											ccColor::Rgb col = ccColor::Generator::Random();
											if (!croppedVertices->setColor(col))
											{
												ccLog::Error("Not enough memory!");
												error = true;
												i = indexMaxs[0];
												j = indexMaxs[1];
												k = indexMaxs[2];
											}
											croppedVertices->showColors(true);
											mesh->showColors(true);
										}
									}

									croppedEnt->setEnabled(true);
									croppedEnt->setVisible(true);
									croppedEnt->setDisplay(mesh->getDisplay());

									QString slicePosStr = QString("(%1 ; %2 ; %3)").arg(C.x).arg(C.y).arg(C.z);
									croppedEnt->setName(mesh->getName() + QString(".slice @ ") + slicePosStr);

									//set meta-data
									croppedEnt->setMetaData(s_originEntityUUID, mesh->getUniqueID());
									croppedEnt->setMetaData(s_sliceID, slicePosStr);
									croppedEnt->setMetaData("slice.origin.dim(0)", C.x);
									croppedEnt->setMetaData("slice.origin.dim(1)", C.y);
									croppedEnt->setMetaData("slice.origin.dim(2)", C.z);

									//add slice to group
									outputSlices.push_back(croppedEnt);
								}

								if (progressDialog)
								{
									if (progressDialog->wasCanceled())
									{
										error = true;
										ccLog::Warning(QString("[ExtractSlicesAndContours] Process canceled by user"));
										//early stop
										i = indexMaxs[0];
										j = indexMaxs[1];
										k = indexMaxs[2];
										break;
									}
									progressDialog->setValue(sliceIndex * static_cast<int>(meshes.size()) + static_cast<int>(mi));
								}
							}
						}
					}
				}
			} //extract sections from meshes

		} //repeat mode

		//extract level set (optionaly)
		if (!error && extractLevelSet && cloudSliceCount != 0)
		{
			for (int iteration = 0; iteration < 1; ++iteration) //fake loop for easy break
			{
				if (progressDialog)
				{
					progressDialog->setWindowTitle("Level set extraction");
					progressDialog->setInfo(QObject::tr("Level(s): %L1").arg(cloudSliceCount));
					progressDialog->setMaximum(static_cast<int>(cloudSliceCount));
					progressDialog->show();
					QApplication::processEvents();
				}

				int Z = 2;
				assert(repeatDimensionsSum == 1);
				{
					for (int i = 0; i < 3; ++i)
					{
						if (repeatDimensions[i])
						{
							Z = i;
							break;
						}
					}
				}
				int X = (Z == 2 ? 0 : Z + 1);
				int Y = (X == 2 ? 0 : X + 1);

				CCVector3 gridOrigin = clipBox.getOwnBB().minCorner();
				CCVector3 gridSize = clipBox.getOwnBB().getDiagVec();
				ccGLMatrix globalTrans = localTrans.inverse();

				assert(false == CCCoreLib::LessThanEpsilon(levelSetGridStep));
				unsigned gridWidth = 1 + static_cast<unsigned>(gridSize.u[X] / levelSetGridStep + 0.5);
				unsigned gridHeight = 1 + static_cast<unsigned>(gridSize.u[Y] / levelSetGridStep + 0.5);

				//add a margin to avoid issues in the level set generation
				gridWidth += 2;
				gridHeight += 2;
				gridOrigin.u[X] -= levelSetGridStep;
				gridOrigin.u[Y] -= levelSetGridStep;

				ccRasterGrid grid;
				if (!grid.init(gridWidth, gridHeight, levelSetGridStep, CCVector3d(0, 0, 0)))
				{
					ccLog::Error("Not enough memory!");
					error = true;
					break;
				}

				//process all the slices originating from point clouds
				assert(cloudSliceCount <= outputSlices.size());
				for (size_t i = 0; i < cloudSliceCount; ++i)
				{
					ccPointCloud* sliceCloud = ccHObjectCaster::ToPointCloud(outputSlices[i]);
					assert(sliceCloud);

					double sliceZ = sliceCloud->getMetaData(QString("slice.origin.dim(%1)").arg(Z)).toDouble();
					sliceZ += gridSize.u[Z] / 2;

					//grid.reset();
					for (ccRasterGrid::Row& row : grid.rows)
					{
						for (ccRasterCell& cell : row)
						{
							cell.h = 0.0;
							cell.nbPoints = 0;
						}
					}

					//project the slice in 2D
					for (unsigned pi = 0; pi != sliceCloud->size(); ++pi)
					{
						CCVector3 relativePos = *sliceCloud->getPoint(pi);
						localTrans.apply(relativePos);
						relativePos -= gridOrigin;

						int i = static_cast<int>(relativePos.u[X] / levelSetGridStep + 0.5);
						int j = static_cast<int>(relativePos.u[Y] / levelSetGridStep + 0.5);

						//we skip points that fall outside of the grid!
						if (	i < 0 || i >= static_cast<int>(gridWidth)
							||	j < 0 || j >= static_cast<int>(gridHeight))
						{
							//there shouldn't be any actually
							assert(false);
							continue;
						}

						ccRasterCell& cell = grid.rows[j][i];
						cell.h = 1.0;
						++cell.nbPoints;
					}

					grid.updateNonEmptyCellCount();	
					grid.updateCellStats();
					grid.setValid(true);

					//now extract the contour lines
					ccContourLinesGenerator::Parameters params;
					params.emptyCellsValue = std::numeric_limits<double>::quiet_NaN();
					params.minVertexCount = levelSetMinVertCount;
					params.parentWidget = progressDialog->parentWidget();
					params.startAltitude = 0.0;
					params.maxAltitude = 1.0;
					params.step = 1.0;

					std::vector<ccPolyline*> contours;
					if (ccContourLinesGenerator::GenerateContourLines(&grid, CCVector2d(gridOrigin.u[X], gridOrigin.u[Y]), params, contours))
					{
						for (size_t k = 0; k < contours.size(); ++k)
						{
							ccPolyline* poly = contours[k];
							CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
							for (unsigned pi = 0; pi < vertices->size(); ++pi)
							{
								//convert the vertices from the local coordinate system to the global one
								const CCVector3* Pconst = vertices->getPoint(pi);
								CCVector3 P;
								P.u[X] = Pconst->x;
								P.u[Y] = Pconst->y;
								P.u[Z] = sliceZ;
								*const_cast<CCVector3*>(Pconst) = globalTrans * P;
							}

							static char s_dimNames[3] = { 'X', 'Y', 'Z' };
							poly->setName(QString("Contour line %1=%2 (#%3)").arg(s_dimNames[Z]).arg(sliceZ).arg(k + 1));
							poly->copyGlobalShiftAndScale(*sliceCloud);
							poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(sliceZ)); //replace the 'altitude' meta-data by the right value

							//set meta-data
							poly->setMetaData(s_originEntityUUID, sliceCloud->getMetaData(s_originEntityUUID));
							poly->setMetaData(s_sliceID, sliceCloud->getMetaData(s_sliceID));
							poly->setMetaData("slice.origin.dim(0)", sliceCloud->getMetaData("slice.origin.dim(0)"));
							poly->setMetaData("slice.origin.dim(1)", sliceCloud->getMetaData("slice.origin.dim(1)"));
							poly->setMetaData("slice.origin.dim(2)", sliceCloud->getMetaData("slice.origin.dim(2)"));

							levelSet.push_back(poly);
						}
					}
					else
					{
						ccLog::Warning(tr("Failed to generate contour lines for cloud #%1").arg(i + 1));
					}

					if (progressDialog)
					{
						if (progressDialog->wasCanceled())
						{
							error = true;
							ccLog::Warning(tr("[ExtractSlicesAndContours] Process canceled by user"));
							break;
						}
						progressDialog->setValue(static_cast<int>(i) + 1);
					}
				}
			}
		}

		//extract envelopes as polylines (optionaly)
		if (!error && extractEnvelopes && cloudSliceCount != 0)
		{
			if (progressDialog)
			{
				progressDialog->setWindowTitle(tr("Envelope extraction"));
				progressDialog->setInfo(tr("Envelope(s): %L1").arg(cloudSliceCount));
				progressDialog->setMaximum(static_cast<int>(cloudSliceCount));
				if (!visualDebugMode)
				{
					progressDialog->show();
					QApplication::processEvents();
				}
			}

			//preferred dimension?
			PointCoordinateType* preferredNormDir = nullptr;
			PointCoordinateType* preferredUpDir = nullptr;
			if (repeatDimensionsSum == 1)
			{
				for (int i = 0; i < 3; ++i)
				{
					if (repeatDimensions[i])
					{
						ccGLMatrix invLocalTrans = localTrans.inverse();
						if (!projectOnBestFitPlane) //otherwise the normal will be automatically computed
							preferredNormDir = invLocalTrans.getColumn(i);
						preferredUpDir = invLocalTrans.getColumn(i < 2 ? 2 : 0);
						break;
					}
				}
			}

			assert(cloudSliceCount <= outputSlices.size());

			//process all the slices originating from point clouds
			for (size_t i = 0; i < cloudSliceCount; ++i)
			{
				ccPointCloud* sliceCloud = ccHObjectCaster::ToPointCloud(outputSlices[i]);
				assert(sliceCloud);

				std::vector<ccPolyline*> polys;
				if (ccEnvelopeExtractor::ExtractFlatEnvelope(sliceCloud,
					multiPass,
					maxEdgeLength,
					polys,
					envelopeType,
					splitEnvelopes,
					preferredNormDir,
					preferredUpDir,
					visualDebugMode))
				{
					if (!polys.empty())
					{
						for (size_t p = 0; p < polys.size(); ++p)
						{
							ccPolyline* poly = polys[p];
							poly->setColor(ccColor::green);
							poly->showColors(true);
							poly->setGlobalScale(sliceCloud->getGlobalScale());
							poly->setGlobalShift(sliceCloud->getGlobalShift());
							QString envelopeName = sliceCloud->getName();
							envelopeName.replace("slice", "envelope");
							if (polys.size() > 1)
							{
								envelopeName += QString(" (part %1)").arg(p + 1);
							}
							poly->setName(envelopeName);

							//set meta-data
							poly->setMetaData(s_originEntityUUID, sliceCloud->getMetaData(s_originEntityUUID));
							poly->setMetaData(s_sliceID, sliceCloud->getMetaData(s_sliceID));
							poly->setMetaData("slice.origin.dim(0)", sliceCloud->getMetaData("slice.origin.dim(0)"));
							poly->setMetaData("slice.origin.dim(1)", sliceCloud->getMetaData("slice.origin.dim(1)"));
							poly->setMetaData("slice.origin.dim(2)", sliceCloud->getMetaData("slice.origin.dim(2)"));

							outputEnvelopes.push_back(poly);
						}
					}
					else
					{
						ccLog::Warning(tr("%1: points are too far from each other! Increase the max edge length").arg(sliceCloud->getName()));
						warningsIssued = true;
					}
				}
				else
				{
					ccLog::Warning(tr("%1: envelope extraction failed!").arg(sliceCloud->getName()));
					warningsIssued = true;
				}

				if (progressDialog && !visualDebugMode)
				{
					if (progressDialog->wasCanceled())
					{
						error = true;
						ccLog::Warning(tr("[ExtractSlicesAndContours] Process canceled by user"));
						//early stop
						break;
					}
					progressDialog->setValue(static_cast<int>(i));
				}
			}

		} //extract envelope polylines

		//release memory
		if (error)
		{
			for (ccHObject* slice : outputSlices)
			{
				delete slice;
			}
			outputSlices.resize(0);

			for (ccPolyline* poly : outputEnvelopes)
			{
				delete poly;
			}
			return false;
		}
		else if (warningsIssued)
		{
			ccLog::Warning(tr("[ExtractSlicesAndContours] Warnings were issued during the process! (result may be incomplete)"));
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory!"));
		return false;
	}

	return true;
}

static ccHObject* FindOrCreateChildren(ccHObject* parent, QString name)
{
	assert(parent && !name.isEmpty());

	for (unsigned i = 0; i < parent->getChildrenNumber(); ++i)
	{
		ccHObject* child = parent->getChild(i);
		if (child->getName() == name)
		{
			return child;
		}
	}

	ccHObject* child = new ccHObject(name);
	parent->addChild(child);
	return child;
}


void ccClippingBoxTool::extractSlicesAndContours(bool singleSliceMode)
{
	if (!m_clipBox || m_clipBox->getContainer().getChildrenNumber() == 0)
	{
		assert(false);
		return;
	}

	std::vector<ccGenericPointCloud*> clouds;
	std::vector<ccGenericMesh*> meshes;
	try
	{
		for (unsigned ci = 0; ci != m_clipBox->getContainer().getChildrenNumber(); ++ci)
		{
			ccHObject* obj = m_clipBox->getContainer().getChild(ci);
			if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
				clouds.push_back(cloud);
			}
			else if (obj->isKindOf(CC_TYPES::MESH))
			{
				ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(obj);
				meshes.push_back(mesh);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return;
	}

	if (clouds.empty() && meshes.empty())
	{
		ccLog::Warning("Only works with point clouds or meshes!");
		return;
	}

	ccClippingBoxRepeatDlg repeatDlg(singleSliceMode, MainWindow::TheInstance());
	repeatDlg.extractEnvelopesGroupBox->setEnabled(!clouds.empty());
	repeatDlg.extractLevelSetGroupBox->setEnabled(!clouds.empty());

	//by default we set the 'flat/repeat' dimension to the smallest box dimension
	{
		CCVector3 diagVec = m_clipBox->getOwnBB().getDiagVec();
		unsigned char flatDim = 0;
		if (diagVec.y < diagVec.x)
			flatDim = 1;
		if (diagVec.z < diagVec.u[flatDim])
			flatDim = 2;
		if (singleSliceMode)
			repeatDlg.setFlatDim(flatDim);
		else
			repeatDlg.setRepeatDim(flatDim);
	}
	
	//random colors is only useful for mutliple slices mode
	repeatDlg.randomColorCheckBox->setEnabled(!singleSliceMode);

	//Semi persistent parameters
	static bool s_extractSliceCloudsOrMeshes = true;
	static bool s_generateRandomColors = false;

	static bool s_extractEnvelopes = false;
	static int s_envelopeTypeIndex = 2; //full
	static bool s_multiPassEnvelope = false;
	static bool s_envProjectPointsOnBestFitPlane = false;
	static bool s_splitEnvelopes = false;
	static bool s_envelopeDebugMode = false;

	static bool s_extractLevelSet = false;
	static double s_levelSetGridStep = 0.0;
	static int s_levelSetMinVertCount = 3;

	static double s_defaultGap = 0.0;
	static int s_groupByIndex = 0;

	//set default max edge length
	if (s_maxEnvelopeEdgeLength < 0)
		s_maxEnvelopeEdgeLength = m_clipBox->getBox().getDiagNorm() / 100.0;
	if (s_levelSetGridStep <= 0.0)
		s_levelSetGridStep = m_clipBox->getBox().getDiagNorm() / 300.0;

	//Extract slice cloud(s) or mesh(es)
	repeatDlg.extractSliceEntitiesGroupBox->setChecked(s_extractSliceCloudsOrMeshes);
	repeatDlg.randomColorCheckBox->setChecked(s_generateRandomColors);

	//Extract envelope(s)
	repeatDlg.extractEnvelopesGroupBox->setChecked(s_extractEnvelopes);
	repeatDlg.envelopeTypeComboBox->setCurrentIndex(s_envelopeTypeIndex);
	repeatDlg.maxEnvelopeEdgeLengthDoubleSpinBox->setValue(s_maxEnvelopeEdgeLength);
	repeatDlg.multiPassEnvelopeCheckBox->setChecked(s_multiPassEnvelope);
	repeatDlg.envProjectPointsOnBestFitCheckBox->setChecked(s_envProjectPointsOnBestFitPlane);
	repeatDlg.splitEnvelopeCheckBox->setChecked(s_splitEnvelopes);
	repeatDlg.envDebugModeCheckBox->setChecked(s_envelopeDebugMode);

	//Extract contours
	repeatDlg.extractLevelSetGroupBox->setChecked(s_extractLevelSet);
	repeatDlg.lsGridStepDoubleSpinBox->setValue(s_levelSetGridStep);
	repeatDlg.minLSVertexCountSpinBox->setValue(s_levelSetMinVertCount);

	//Other options
	repeatDlg.gapDoubleSpinBox->setValue(s_defaultGap);
	repeatDlg.groupByTypeComboBox->setCurrentIndex(s_groupByIndex);
	
	if (!repeatDlg.exec())
	{
		//cancelled by user
		return;
	}

	//repeat dimensions 
	bool processDim[3] = {	repeatDlg.xRepeatCheckBox->isChecked(),
							repeatDlg.yRepeatCheckBox->isChecked(),
							repeatDlg.zRepeatCheckBox->isChecked() };

	//whether to use random colors for (multiple) generated slices
	s_extractSliceCloudsOrMeshes = repeatDlg.extractSliceEntitiesGroupBox->isChecked();
	s_generateRandomColors = repeatDlg.randomColorCheckBox->isChecked();

	s_extractEnvelopes = repeatDlg.extractEnvelopesGroupBox->isChecked();
	s_envelopeTypeIndex = repeatDlg.envelopeTypeComboBox->currentIndex();
	s_maxEnvelopeEdgeLength = repeatDlg.maxEnvelopeEdgeLengthDoubleSpinBox->value();
	s_multiPassEnvelope = repeatDlg.multiPassEnvelopeCheckBox->isChecked();
	s_envProjectPointsOnBestFitPlane = repeatDlg.envProjectPointsOnBestFitCheckBox->isChecked();
	s_splitEnvelopes = repeatDlg.splitEnvelopeCheckBox->isChecked();
	s_envelopeDebugMode = repeatDlg.envDebugModeCheckBox->isChecked();

	s_extractLevelSet = repeatDlg.extractLevelSetGroupBox->isChecked();
	s_levelSetGridStep = repeatDlg.lsGridStepDoubleSpinBox->value();
	s_levelSetMinVertCount = repeatDlg.minLSVertexCountSpinBox->value();

	s_defaultGap = repeatDlg.gapDoubleSpinBox->value();
	s_groupByIndex = repeatDlg.groupByTypeComboBox->currentIndex();

	ccEnvelopeExtractor::EnvelopeType envelopeType = ccEnvelopeExtractor::EnvelopeType::FULL;
	switch (s_envelopeTypeIndex)
	{
	case 0:
		envelopeType = ccEnvelopeExtractor::EnvelopeType::LOWER;
		break;
	case 1:
		envelopeType = ccEnvelopeExtractor::EnvelopeType::UPPER;
		break;
	case 2:
		envelopeType = ccEnvelopeExtractor::EnvelopeType::FULL;
		break;
	default:
		assert(false);
		ccLog::Warning("Internal error: unhandled envelope type");
		break;
	}

	ccProgressDialog pDlg(false, this);
	std::vector<ccHObject*> outputSlices;
	std::vector<ccPolyline*> outputEnvelopes;
	std::vector<ccPolyline*> outputLevelSet;

	QElapsedTimer eTimer;
	eTimer.start();

	if (!ExtractSlicesAndContours(	clouds,
									meshes,
									*m_clipBox,
									singleSliceMode,
									processDim,
									outputSlices,

									s_extractEnvelopes,
									static_cast<PointCoordinateType>(s_maxEnvelopeEdgeLength),
									envelopeType,
									outputEnvelopes,

									s_extractLevelSet,
									s_levelSetGridStep,
									s_levelSetMinVertCount,
									outputLevelSet,

									static_cast<PointCoordinateType>(s_defaultGap),
									s_multiPassEnvelope,
									s_splitEnvelopes,
									s_envProjectPointsOnBestFitPlane,
									s_envelopeDebugMode,
									s_generateRandomColors,
									&pDlg
									))
	{
		//process failed (error message has already been issued)
		return;
	}

	ccLog::Print("[ccClippingBoxTool] Processed finished in %.2f s.", eTimer.elapsed() / 1.0e3);

	//possible outputs
	ccHObject* sliceGroup = nullptr;
	ccHObject* envelopeGroup = nullptr;
	ccHObject* levelSetGroup = nullptr;
	QMap<QString, ccHObject*> perSliceGroups;
	QMap<unsigned, ccHObject*> perEntityGroups;
	QMap<unsigned, QString> perEntityGroupNames;
	ccHObject* garbageGroup = new ccHObject("Extracted entities");

	enum OutputFormat { BY_TYPE = 0,
						BY_ENTITY = 1,
						BY_SLICE = 2,
						BY_ENTITY_THEN_SLICE = 3,
						BY_SLICE_THEN_ENTITY = 4,
	};

	if (	s_groupByIndex == OutputFormat::BY_ENTITY
		||	s_groupByIndex == OutputFormat::BY_ENTITY_THEN_SLICE
		||	s_groupByIndex == OutputFormat::BY_SLICE_THEN_ENTITY
		)
	{
		for (const ccGenericPointCloud* cloud : clouds)
		{
			QString name = cloud->getName() + ".slices";
			perEntityGroupNames[cloud->getUniqueID()] = name;
			if (s_groupByIndex != OutputFormat::BY_SLICE_THEN_ENTITY)
			{
				perEntityGroups[cloud->getUniqueID()] = new ccHObject(name);
			}
		}
		for (const ccGenericMesh* mesh : meshes)
		{
			QString name = mesh->getName() + ".slices";
			perEntityGroupNames[mesh->getUniqueID()] = name;
			if (s_groupByIndex != OutputFormat::BY_SLICE_THEN_ENTITY)
			{
				perEntityGroups[mesh->getUniqueID()] = new ccHObject(name);
			}
		}
	}

	if (s_groupByIndex == OutputFormat::BY_SLICE)
	{
		for (const ccHObject* slice : outputSlices)
		{
			QString sliceID = slice->getMetaData(s_sliceID).toString();
			perSliceGroups[sliceID] = new ccHObject(sliceID);
		}
	}

	auto dispatchEntity = [&](ccHObject* entity, ccHObject* sliceGroup)
	{
		if (entity == nullptr)
		{
			assert(false);
			return;
		}
		ccHObject* destGroup = nullptr;

		switch (s_groupByIndex)
		{
		case OutputFormat::BY_TYPE:
		{
			destGroup = sliceGroup;
		}
		break;

		case OutputFormat::BY_ENTITY:
		case OutputFormat::BY_ENTITY_THEN_SLICE:
		{
			//first level: entity
			unsigned entityUUID = entity->getMetaData(s_originEntityUUID).toUInt();
			if (perEntityGroups.contains(entityUUID))
			{
				destGroup = perEntityGroups[entityUUID];
			}
			else
			{
				assert(false);
				destGroup = garbageGroup;
			}

			if (s_groupByIndex == OutputFormat::BY_ENTITY_THEN_SLICE)
			{
				//second level: slice
				QString sliceID = entity->getMetaData(s_sliceID).toString();
				destGroup = FindOrCreateChildren(destGroup, sliceID);
			}
			
			destGroup->addChild(entity);
		}
		break;

		case OutputFormat::BY_SLICE:
		case OutputFormat::BY_SLICE_THEN_ENTITY:
		{
			//first level: slice
			QString sliceID = entity->getMetaData(s_sliceID).toString();
			if (perSliceGroups.contains(sliceID))
			{
				destGroup = perSliceGroups[sliceID];
			}
			else
			{
				assert(false);
				destGroup = garbageGroup;
			}
			
			if (s_groupByIndex == OutputFormat::BY_SLICE_THEN_ENTITY)
			{
				//second level: entity
				unsigned entityUUID = entity->getMetaData(s_originEntityUUID).toUInt();
				if (perEntityGroupNames.contains(entityUUID))
				{
					destGroup = FindOrCreateChildren(destGroup, perEntityGroupNames[entityUUID]);
				}
			}
		}
		break;

		default:
			assert(false);
			return;
		}

		if (destGroup)
		{
			destGroup->addChild(entity);
		}
		else
		{
			assert(false);
		}
	};

	//get the default output display
	ccGenericGLDisplay* defaultDisplay = nullptr;
	if (m_clipBox->getContainer().getFirstChild())
	{
		defaultDisplay = m_clipBox->getContainer().getFirstChild()->getDisplay();
	}

	//base name
	QString baseName;
	if (m_clipBox->getContainer().getChildrenNumber() == 1)
	{
		baseName = m_clipBox->getContainer().getFirstChild()->getName();
	}

	//slices (clouds or meshes)
	size_t sliceCount = outputSlices.size();
	if (sliceCount)
	{
		if (s_extractSliceCloudsOrMeshes)
		{
			if (s_groupByIndex == OutputFormat::BY_TYPE)
			{
				//we have to create the destination group
				sliceGroup = new ccHObject(baseName.isEmpty() ? QString("Slices") : baseName + QString(".slices"));
			}

			for (ccHObject* slice : outputSlices)
			{
				dispatchEntity(slice, sliceGroup);
			}
		}
		else
		{
			for (ccHObject* slice : outputSlices)
			{
				delete slice;
			}
			outputSlices.clear();
		}
	}

	if (!outputEnvelopes.empty() || !outputLevelSet.empty())
	{
		s_lastContourUniqueIDs.clear();
	}

	//envelopes
	if (!outputEnvelopes.empty())
	{
		if (s_groupByIndex == OutputFormat::BY_TYPE)
		{
			//we have to create the destination group
			envelopeGroup = new ccHObject(baseName.isEmpty() ? QString("Envelopes") : baseName + QString(".envelopes"));
		}

		for (ccPolyline* poly : outputEnvelopes)
		{
			dispatchEntity(poly, envelopeGroup);
			s_lastContourUniqueIDs.push_back(poly->getUniqueID());
		}

		if (envelopeGroup)
		{
			s_lastContourUniqueIDs.push_back(envelopeGroup->getUniqueID());
		}
	}

	//level set
	if (!outputLevelSet.empty())
	{
		if (s_groupByIndex == OutputFormat::BY_TYPE)
		{
			//we have to create the destination group
			levelSetGroup = new ccHObject(baseName.isEmpty() ? QString("Level set") : baseName + QString(".levelSet"));
		}

		for (ccPolyline* poly : outputLevelSet)
		{
			dispatchEntity(poly, levelSetGroup);
			s_lastContourUniqueIDs.push_back(poly->getUniqueID());
		}

		if (levelSetGroup)
		{
			s_lastContourUniqueIDs.push_back(levelSetGroup->getUniqueID());
		}
	}

	removeLastContourToolButton->setEnabled(!s_lastContourUniqueIDs.empty());

	//now take care of the 'output' groups
	{
		if (sliceGroup)
		{
			sliceGroup->setDisplay_recursive(defaultDisplay);
			MainWindow::TheInstance()->addToDB(sliceGroup);
		}
	
		if (envelopeGroup)
		{
			envelopeGroup->setDisplay_recursive(defaultDisplay);
			MainWindow::TheInstance()->addToDB(envelopeGroup);
		}

		if (levelSetGroup)
		{
			levelSetGroup->setDisplay_recursive(defaultDisplay);
			MainWindow::TheInstance()->addToDB(levelSetGroup);
		}

		for (ccHObject* group : perSliceGroups)
		{
			group->setDisplay_recursive(defaultDisplay);
			MainWindow::TheInstance()->addToDB(group);
		}

		for (ccHObject* group : perEntityGroups)
		{
			group->setDisplay_recursive(defaultDisplay);
			MainWindow::TheInstance()->addToDB(group);
		}

		//don't forget the 'garbage' group (just in case)
		if (garbageGroup)
		{
			if (garbageGroup->getChildrenNumber() != 0)
			{
				garbageGroup->setDisplay_recursive(defaultDisplay);
				MainWindow::TheInstance()->addToDB(garbageGroup);
			}
			else
			{
				delete garbageGroup;
				garbageGroup = nullptr;
			}
		}
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	if (sliceCount != 0)
	{
		QMessageBox::warning(nullptr, tr("Process finished"), tr("%1 slices have been generated.\n(you may have to close the tool and hide the initial cloud to see them...)").arg(sliceCount));
	}
	else if (s_extractSliceCloudsOrMeshes)
	{
		QMessageBox::warning(nullptr, tr("Process finished"), tr("The process has generated no output"));
	}
}

void ccClippingBoxTool::onBoxModified(const ccBBox* box)
{
	if (box && box->isValid())
	{
		CCVector3 dd = box->maxCorner() - box->minCorner();
		thickXDoubleSpinBox->setValue(dd.x);
		thickYDoubleSpinBox->setValue(dd.y);
		thickZDoubleSpinBox->setValue(dd.z);
		thicknessGroupBox->setEnabled(true);
	}
	else
	{
		thicknessGroupBox->setEnabled(false);
	}
}

void ccClippingBoxTool::thicknessChanged(double)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	CCVector3 th(static_cast<PointCoordinateType>(thickXDoubleSpinBox->value()),
				 static_cast<PointCoordinateType>(thickYDoubleSpinBox->value()),
				 static_cast<PointCoordinateType>(thickZDoubleSpinBox->value()));

	ccBBox box = m_clipBox->getBox();
	CCVector3 boxCenter = (box.maxCorner() + box.minCorner()) / 2;

	box.minCorner() = boxCenter - th/2;
	box.maxCorner() = boxCenter + th/2;

	m_clipBox->setBox(box);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::shiftBox(unsigned char dim, bool minus)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	assert(dim<3);

	PointCoordinateType width = (m_clipBox->getBox().maxCorner() - m_clipBox->getBox().minCorner()).u[dim];
	CCVector3 shiftVec(0, 0, 0);
	shiftVec.u[dim] = (minus ? -width : width);
	m_clipBox->shift(shiftVec);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::reset()
{
	if (m_clipBox)
		m_clipBox->reset();

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::restoreLastBox()
{
	if (!m_clipBox || m_clipBox->getContainer().getChildrenNumber() == 0)
	{
		assert(false);
		return;
	}
	
	unsigned uniqueID = m_clipBox->getContainer().getFirstChild()->getUniqueID();
	if (!s_lastBoxParams.contains(uniqueID))
	{
		assert(false);
		return;
	}

	const ccClipBoxParams& params = s_lastBoxParams[uniqueID];
	m_clipBox->set(params.box, params.trans);
}

void ccClippingBoxTool::closeDialog()
{
	stop(true);
}

void ccClippingBoxTool::setTopView()
{
	setView(CC_TOP_VIEW);
}

void ccClippingBoxTool::setBottomView()
{
	setView(CC_BOTTOM_VIEW);
}

void ccClippingBoxTool::setFrontView()
{
	setView(CC_FRONT_VIEW);
}

void ccClippingBoxTool::setBackView()
{
	setView(CC_BACK_VIEW);
}

void ccClippingBoxTool::setLeftView()
{
	setView(CC_LEFT_VIEW);
}

void ccClippingBoxTool::setRightView()
{
	setView(CC_RIGHT_VIEW);
}

void ccClippingBoxTool::setView(CC_VIEW_ORIENTATION orientation)
{
	if (!m_associatedWin)
		return;

	//m_associatedWin->blockSignals(true);
	m_associatedWin->setView(orientation,false);
	if (m_clipBox && m_clipBox->isGLTransEnabled())
	{
		ccViewportParameters params = m_associatedWin->getViewportParameters();
		const ccGLMatrix& glMat = m_clipBox->getGLTransformation();

		ccGLMatrixd rotMat(glMat.data()); rotMat.clearTranslation();

		params.viewMat = params.viewMat * rotMat.inverse();
		m_associatedWin->setViewportParameters(params);
	}
	//m_associatedWin->blockSignals(false);
	m_associatedWin->redraw();
}
