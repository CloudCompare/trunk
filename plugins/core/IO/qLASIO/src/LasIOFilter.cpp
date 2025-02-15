//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include "LasIOFilter.h"

#include "CopcLoader.h"
#include "LasMetadata.h"
#include "LasOpenDialog.h"
#include "LasSaveDialog.h"
#include "LasSaver.h"
#include "LasScalarFieldLoader.h"
#include "LasScalarFieldSaver.h"
#include "LasVlr.h"
#include "LasWaveformLoader.h"
#include "LasWaveformSaver.h"

// CC
#include <GenericProgressCallback.h>
#include <ccColorScalesManager.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <CCGeom.h>

// Qt
#include <QDate>
#include <QElapsedTimer>
#include <QFileInfo>

// LASzip
#include <laszip/laszip_api.h>

// System
#include <memory>
#include <utility>

static CCVector3d GetGlobalShift(FileIOFilter::LoadParameters& parameters,
                                 bool&                         preserveCoordinateShift,
                                 const CCVector3d&             lasOffset,
                                 const CCVector3d&             firstPoint)
{
	ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
	CCVector3d                 shift;
	bool                       useLasOffset = false;

	// set the lasOffset as default if none was provided
	if (lasOffset.norm2() != 0 && (!parameters._coordinatesShiftEnabled || !(*parameters._coordinatesShiftEnabled)))
	{
		if (csModeBackup != ccGlobalShiftManager::NO_DIALOG) // No dialog, practically means
		                                                     // that we don't want any shift!
		{
			useLasOffset = true;
			shift        = -lasOffset;
			if (csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
			{
				parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
			}
		}
	}

	if (!FileIOFilter::HandleGlobalShift(firstPoint,
	                                     shift,
	                                     preserveCoordinateShift,
	                                     parameters,
	                                     useLasOffset))
	{
		preserveCoordinateShift = false;
		shift                   = {0.0, 0.0, 0.0};
	}

	// restore previous parameters
	parameters.shiftHandlingMode = csModeBackup;
	return shift;
}

LasIOFilter::LasIOFilter()
    : FileIOFilter({"LAS IO Filter",
                    3.0f, // priority (same as the old PDAL-based plugin)
                    QStringList{"las", "laz"},
                    "las",
                    QStringList{"LAS file (*.las *.laz *.copc.laz)"},
                    QStringList{"LAS file (*.las *.laz)"},
                    Import | Export})
{
	m_openDialog.resetShouldSkipDialog();
}

CC_FILE_ERROR LasIOFilter::loadFile(const QString&  fileName,
                                    ccHObject&      container,
                                    LoadParameters& parameters)
{
	laszip_POINTER laszipReader;
	laszip_header* laszipHeader{nullptr};
	laszip_BOOL    isCompressed{false};
	laszip_CHAR*   errorMsg{nullptr};

	if (laszip_create(&laszipReader))
	{
		ccLog::Warning("[LAS] Failed to create reader");
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_open_reader(laszipReader, qPrintable(fileName), &isCompressed))
	{
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_get_header_pointer(laszipReader, &laszipHeader))
	{
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
		laszip_close_reader(laszipReader);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	laszip_U64 pointCount = TrueNumberOfPoints(laszipHeader);

	if (pointCount >= std::numeric_limits<unsigned>::max())
	{
		ccLog::Error("[LAS] Files with more that %u points are not supported", std::numeric_limits<unsigned>::max());
		return CC_FERR_NOT_IMPLEMENTED;
	}

	// intervalsToRead is initialized to only one interval
	// from 0 to the last point of the LAS file
	LasDetails::ChunkInterval                                      fullInterval(0, pointCount);
	std::vector<std::reference_wrapper<LasDetails::ChunkInterval>> chunksToRead;
	chunksToRead.emplace_back(fullInterval);

	// COPC handling
	m_openDialog.displayCopcTab(false);
	std::unique_ptr<copc::CopcLoader> copcLoader{nullptr};
	// Check that all COPC (pre)conditions are met before creating a loader
	if (copc::CopcLoader::IsPutativeCOPCFile(laszipHeader))
	{
		copcLoader = std::make_unique<copc::CopcLoader>(laszipHeader, fileName);
		// The Loader constructor could fail, check its valididy
		// If it fails to create a valid COPC reader we give the file another chance to be read as a "regular" LAZ file
		if (copcLoader->isValid())
		{
			m_openDialog.displayCopcTab(true);
			m_openDialog.setCopcInformations(copcLoader->levelPointCounts(), copcLoader->extent());
		}
		else
		{
			ccLog::Warning("[LAS] Something went wrong with the initial parsing of the COPC structure, fall back to regular LAZ reading");
			copcLoader.reset(nullptr);
		}
	}

	std::vector<LasScalarField>
	    availableScalarFields = LasScalarField::ForPointFormat(laszipHeader->point_data_format);

	std::vector<LasExtraScalarField> availableExtraScalarFields = LasExtraScalarField::ParseExtraScalarFields(*laszipHeader);

	std::unique_ptr<FileInfo> infoOfCurrentFile = std::make_unique<FileInfo>();
	infoOfCurrentFile->version.minorVersion     = laszipHeader->version_minor;
	infoOfCurrentFile->version.pointFormat      = laszipHeader->point_data_format;
	infoOfCurrentFile->extraScalarFields        = availableExtraScalarFields;

	bool fileContentIsDifferentFromPrevious = (m_infoOfLastOpened && (*m_infoOfLastOpened != *infoOfCurrentFile));

	m_openDialog.setInfo(laszipHeader->version_minor, laszipHeader->point_data_format, pointCount);
	m_openDialog.setAvailableScalarFields(availableScalarFields, availableExtraScalarFields);
	m_infoOfLastOpened = std::move(infoOfCurrentFile);

	// The idea is that when Loading a file (as opposed to tiling one)
	// we want to show the dialog when the file structure is different from the previous
	// (not same scalar fields, etc) even if the user asked to skip dialogs
	// as we can't choose for him.
	//
	// For tiling even if the file is different from the previous,
	// since we copy points without loading into CC we don't have to force the
	// dialog to be re-shown.
	if (m_openDialog.shouldSkipDialog() && m_openDialog.action() == LasOpenDialog::Action::Load && fileContentIsDifferentFromPrevious)
	{
		m_openDialog.resetShouldSkipDialog();
	}

	if (parameters.sessionStart)
	{
		// we do this AFTER restoring the previous context because it may still
		// be good that the previous configuration is restored even though the
		// user needs to confirm it
		m_openDialog.resetShouldSkipDialog();
	}

	if (parameters.alwaysDisplayLoadDialog && !m_openDialog.shouldSkipDialog())
	{
		m_openDialog.exec();
		if (m_openDialog.result() == QDialog::Rejected)
		{
			laszip_close_reader(laszipReader);
			laszip_clean(laszipReader);
			laszip_destroy(laszipReader);
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	// Tiling takes precedence over COPC
	if (m_openDialog.action() == LasOpenDialog::Action::Tile)
	{
		return TileLasReader(laszipReader, fileName, m_openDialog.tilingOptions());
	}

	// Update chunksToReads according to the COPCLoader if needed
	if (copcLoader)
	{
		const uint32_t copcUserDefinedMaxLevel = m_openDialog.copcMaxLevel();
		if (copcUserDefinedMaxLevel < copcLoader->maxLevel())
		{
			copcLoader->setMaxLevelConstraint(copcUserDefinedMaxLevel);
		}

		if (m_openDialog.hasUsableExtent())
		{
			const auto clippingExtent = m_openDialog.copcExtent();
			if (clippingExtent.isValid())
			{
				copcLoader->setClippingBoxConstraint(clippingExtent);
			}
		}
		// Update intervalsToRead and pointCount for current COPC query
		copcLoader->getChunkIntervalsSet(chunksToRead, pointCount);
	}

	std::array<LasExtraScalarField, 3> extraScalarFieldsToLoadAsNormals = m_openDialog.getExtraFieldsToBeLoadedAsNormals(availableExtraScalarFields);
	bool                               haveToLoadNormals                = std::any_of(extraScalarFieldsToLoadAsNormals.begin(),
                                         extraScalarFieldsToLoadAsNormals.end(),
                                         [](const LasExtraScalarField& e)
                                         {
                                             return e.type != LasExtraScalarField::DataType::Undocumented;
                                         });
	m_openDialog.filterOutNotChecked(availableScalarFields, availableExtraScalarFields);

	auto pointCloud = std::make_unique<ccPointCloud>(QFileInfo(fileName).fileName());
	if (!pointCloud->reserve(pointCount))
	{
		laszip_close_reader(laszipReader);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	if (haveToLoadNormals)
	{
		if (!pointCloud->reserveTheNormsTable())
		{
			laszip_close_reader(laszipReader);
			laszip_clean(laszipReader);
			laszip_destroy(laszipReader);
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}
	}

	laszip_F64    laszipCoordinates[3]{0};
	laszip_point* laszipPoint{nullptr};
	CCVector3     currentPoint{};
	bool          preserveGlobalShift{true};

	if (laszip_get_point_pointer(laszipReader, &laszipPoint))
	{
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
		laszip_close_reader(laszipReader);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	LasScalarFieldLoader loader(availableScalarFields,
	                            availableExtraScalarFields,
	                            *pointCloud);

	loader.setIgnoreFieldsWithDefaultValues(m_openDialog.shouldIgnoreFieldsWithDefaultValues());
	loader.setForce8bitRgbMode(m_openDialog.shouldForce8bitColors());
	loader.setDecomposeClassification(m_openDialog.shouldDecomposeClassification());
	std::unique_ptr<LasWaveformLoader> waveformLoader{nullptr};
	if (LasDetails::HasWaveform(laszipHeader->point_data_format))
	{
		waveformLoader = std::make_unique<LasWaveformLoader>(*laszipHeader,
		                                                     fileName,
		                                                     *pointCloud);
	}

	QElapsedTimer timer;
	timer.start();

	ccProgressDialog progressDialog(true, parameters.parentWidget);
	progressDialog.setMethodTitle("Loading LAS points");
	progressDialog.setInfo("Loading points");
	QScopedPointer<CCCoreLib::NormalizedProgress> normProgress;
	if (parameters.parentWidget)
	{
		normProgress.reset(new CCCoreLib::NormalizedProgress(&progressDialog, pointCount));
		progressDialog.start();
	}

	CC_FILE_ERROR error{CC_FERR_NO_ERROR};
	CCVector3d    globalShift(0, 0, 0);
	bool          isglobalShiftDefined = false;

	// Last Point ID of previous interval
	uint64_t nextPointIndex = 0;
	for (auto interval : chunksToRead)
	{
		// break if previous inner loop (i.e previous interval) leads to an error
		if (error != CC_FERR_NO_ERROR)
		{
			break;
		}

		LasDetails::ChunkInterval& intervalRef = interval.get();

		if (intervalRef.status == LasDetails::ChunkInterval::eFilterStatus::FAIL)
		{
			continue;
		}

		// keep track of the origin of the interval/chunk in the cloud.
		// this is needed for the LOD mechanism.
		intervalRef.pointOffsetInCCCloud = pointCloud->size();

		// For COPCLoader we allow to test if point is contained in a given extent
		bool testInExtent = intervalRef.status == LasDetails::ChunkInterval::eFilterStatus::INTERSECT_BB && copcLoader;

		// Minimize seeking for COPC.
		// It's not clear if it gives some performance improvements but it complexify the code.
		// since it enforces to keep track of multiples indices in order to generate the proper LOD
		// data structure.
		// The main bottleneck in LAZ reading is point decompression but high number of seeking
		// operation could have an impact on big files.
		// In a standard LAS/LAZ scenario this is noop since nextPointIndex = 0;
		if (nextPointIndex != intervalRef.pointOffsetInFile)
		{
			// Here int64_t is internally converted to uint32_t in LASzip, so it overflows if we have cloud with more
			// than approx. 4.2B. points laz-perf does not suffer from this limitation.
			// CC is also limited to unsigned in sizes.
			// https://github.com/LASzip/LASzip/issues/76
			// https://github.com/LASzip/LASzip/blob/103c4464611a39853d40aea9c3594b523a6c168b/src/laszip_dll.cpp#L4648
			laszip_seek_point(laszipReader, static_cast<int64_t>(intervalRef.pointOffsetInFile));
			nextPointIndex = intervalRef.pointOffsetInFile;
		}

		// Read the points int the interval
		for (unsigned i = 0; i < intervalRef.pointCount; ++i)
		{
			if (laszip_read_point(laszipReader))
			{
				error = CC_FERR_THIRD_PARTY_LIB_FAILURE; // error will be logged later
				break;
			}

			if (laszip_get_coordinates(laszipReader, laszipCoordinates))
			{
				error = CC_FERR_THIRD_PARTY_LIB_FAILURE; // error will be logged later
				break;
			}

			// increment nextPoint index
			++nextPointIndex;

			if (!isglobalShiftDefined)
			{
				CCVector3d firstPoint(laszipCoordinates);

				CCVector3d lasOffset(laszipHeader->x_offset,
				                     laszipHeader->y_offset,
				                     0.0 /*laszipHeader->z_offset*/); // it's never a good idea to shift along Z

				globalShift = GetGlobalShift(parameters,
				                             preserveGlobalShift,
				                             lasOffset,
				                             firstPoint);

				if (preserveGlobalShift)
				{
					pointCloud->setGlobalShift(globalShift);
				}

				if (copcLoader)
				{
					copcLoader->setGlobalShift(globalShift);
				}

				if (globalShift.norm2() != 0.0)
				{
					ccLog::Warning("[LAS] Cloud has been re-centered! Translation: "
					               "(%.2f ; %.2f ; %.2f)",
					               globalShift.x,
					               globalShift.y,
					               globalShift.z);
				}
				isglobalShiftDefined = true;
			}

			// Test if the point is within the allowed extent:
			// If the clippingBox intersects the current chunk interval, each point of the chunk must be tested individually.
			if (testInExtent)
			{
				if (!copcLoader->clippingExtent().contains(CCVector3d(laszipCoordinates[0], laszipCoordinates[1], laszipCoordinates[2])))
				{
					intervalRef.filteredPointCount++;
					continue;
				}
			}

			currentPoint.x = static_cast<PointCoordinateType>(laszipCoordinates[0] + globalShift.x);
			currentPoint.y = static_cast<PointCoordinateType>(laszipCoordinates[1] + globalShift.y);
			currentPoint.z = static_cast<PointCoordinateType>(laszipCoordinates[2] + globalShift.z);

			pointCloud->addPoint(currentPoint);

			error = loader.handleScalarFields(*pointCloud, *laszipPoint);
			if (error != CC_FERR_NO_ERROR)
			{
				break;
			}

			error = loader.handleExtraScalarFields(*laszipPoint);
			if (error != CC_FERR_NO_ERROR)
			{
				break;
			}

			if (LasDetails::HasRGB(laszipHeader->point_data_format))
			{
				error = loader.handleRGBValue(*pointCloud, *laszipPoint);
				if (error != CC_FERR_NO_ERROR)
				{
					break;
				}
			}

			if (waveformLoader)
			{
				waveformLoader->loadWaveform(*pointCloud, *laszipPoint);
			}

			if (haveToLoadNormals)
			{
				CCVector3 normal{};
				// Here, the array has 3 values, not because normals have 3 dimensions (x, y, z)
				// but because extra scalar field may have 3 dimensions.
				// Regardless of whether the extra scalar field has more than 1 dimensions
				// we only use the first one for each normal dimension.
				for (unsigned int normalIndex = 0; normalIndex < 3; ++normalIndex)
				{
					const LasExtraScalarField& extraField = extraScalarFieldsToLoadAsNormals[normalIndex];
					if (extraField.type == LasExtraScalarField::DataType::Undocumented)
					{
						continue;
					}
					ScalarType normalsValues[3]{0, 0, 0};
					error = loader.parseExtraScalarField(extraField, *laszipPoint, normalsValues);
					if (error != CC_FERR_NO_ERROR)
					{
						break;
					}
					normal[normalIndex] = normalsValues[0];
				}

				if (error != CC_FERR_NO_ERROR)
				{
					break;
				}
				pointCloud->addNorm(normal);
			}

			if (normProgress && !normProgress->oneStep())
			{
				error = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}
	}

	for (const LasScalarField& field : loader.standardFields())
	{
		if (field.sf == nullptr)
		{
			// It may be null if all values were the same
			continue;
		}
		field.sf->computeMinAndMax();
		field.sf->setSaturationStart(field.sf->getMin());
		field.sf->setSaturationStop(field.sf->getMax());
		field.sf->setMinDisplayed(field.sf->getMin());
		field.sf->setMaxDisplayed(field.sf->getMax());

		switch (field.id)
		{
		case LasScalarField::Intensity:
			field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
		case LasScalarField::ReturnNumber:
		case LasScalarField::NumberOfReturns:
		case LasScalarField::ScanDirectionFlag:
		case LasScalarField::EdgeOfFlightLine:
		case LasScalarField::Classification:
		case LasScalarField::SyntheticFlag:
		case LasScalarField::KeypointFlag:
		case LasScalarField::WithheldFlag:
		case LasScalarField::ScanAngleRank:
		case LasScalarField::UserData:
		case LasScalarField::PointSourceId:
		case LasScalarField::ExtendedScannerChannel:
		case LasScalarField::OverlapFlag:
		case LasScalarField::ExtendedClassification:
		case LasScalarField::ExtendedReturnNumber:
		case LasScalarField::ExtendedNumberOfReturns:
		case LasScalarField::NearInfrared:
		{
			auto    cMin  = static_cast<int64_t>(field.sf->getMin());
			auto    cMax  = static_cast<int64_t>(field.sf->getMax());
			int64_t steps = std::min<int64_t>(cMax - cMin + 1, 256);
			field.sf->setColorRampSteps(steps);
			break;
		}
		case LasScalarField::GpsTime:
			field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BGYR));
			break;
		case LasScalarField::ExtendedScanAngle:
			field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BGYR));
			break;
		}

		pointCloud->addScalarField(field.sf);
	}

	for (const LasExtraScalarField& field : loader.extraFields())
	{
		for (size_t i = 0; i < field.numElements(); ++i)
		{
			assert(field.scalarFields[i] != nullptr);
			field.scalarFields[i]->computeMinAndMax();
			field.scalarFields[i]->setSaturationStart(field.scalarFields[i]->getMin());
			field.scalarFields[i]->setSaturationStop(field.scalarFields[i]->getMax());
			field.scalarFields[i]->setMinDisplayed(field.scalarFields[i]->getMin());
			field.scalarFields[i]->setMaxDisplayed(field.scalarFields[i]->getMax());
			pointCloud->addScalarField(field.scalarFields[i]);
		}
	}

	int idx = pointCloud->getScalarFieldIndexByName(LasNames::Intensity);
	if (idx != -1)
	{
		pointCloud->setCurrentDisplayedScalarField(idx);
	}
	else if (pointCloud->getNumberOfScalarFields() > 0)
	{
		pointCloud->setCurrentDisplayedScalarField(0);
	}
	pointCloud->showColors(pointCloud->hasColors());
	pointCloud->showSF(!pointCloud->hasColors() && pointCloud->hasDisplayedScalarField());

	for (LasExtraScalarField& extraField : availableExtraScalarFields)
	{
		extraField.resetScalarFieldsPointers();
	}

	// With the copcLoader, we shrink the point cloud to take into account the point that could be filtered in the loading process
	// We have to do that before the LOD construction because shrinkTofit clears the LOD.
	if (copcLoader)
	{
		pointCloud->shrinkToFit();
	}

	LasMetadata::SaveMetadataInto(*laszipHeader, *pointCloud, availableExtraScalarFields);

	container.addChild(pointCloud.release());

	if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
	{
		ccLog::Warning("ERROR IS HERE");
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
	}

	laszip_close_reader(laszipReader);
	laszip_clean(laszipReader);
	laszip_destroy(laszipReader);

	timer.elapsed();
	qint64  elapsed = timer.elapsed();
	int32_t minutes = timer.elapsed() / (1000 * 60);
	elapsed -= minutes * (1000 * 60);
	int32_t seconds = elapsed / 1000;
	elapsed -= seconds * 1000;
	ccLog::Print(QString("[LAS] File loaded in %1m%2s%3ms").arg(minutes).arg(seconds).arg(elapsed));
	return error;
}

bool LasIOFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	multiple  = false;
	exclusive = true;
	return type == CC_TYPES::POINT_CLOUD;
}

CC_FILE_ERROR LasIOFilter::saveToFile(ccHObject* entity, const QString& filename, const FileIOFilter::SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	if (!entity->isA(CC_TYPES::POINT_CLOUD))
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	auto* pointCloud = static_cast<ccPointCloud*>(entity);

	LasSaveDialog saveDialog(pointCloud, parameters.parentWidget);

	CCVector3d bbMax, bbMin;
	if (!pointCloud->getOwnGlobalBB(bbMin, bbMax))
	{
		if (pointCloud->size() != 0)
		{
			// it can only be acceptable if the cloud is empty
			//(yes, some people expect to save empty clouds!)
			return CC_FERR_NO_SAVE;
		}
		else
		{
			bbMax = bbMin = CCVector3d(0.0, 0.0, 0.0);
		}
	}

	// Determine the best LAS offset (required for determing the best LAS scale)
	CCVector3d originalLASOffset(0, 0, 0);
	bool       hasLASOffset               = LasMetadata::LoadOffsetFrom(*pointCloud, originalLASOffset);
	bool       orignialLasOffsetCanBeUsed = hasLASOffset && !ccGlobalShiftManager::NeedShift(bbMax - originalLASOffset);

	CCVector3d globalShift          = pointCloud->getGlobalShift();
	bool       hasGlobalShift       = pointCloud->isShifted();
	bool       globalShiftCanBeUsed = hasGlobalShift && !ccGlobalShiftManager::NeedShift(bbMax + globalShift); //'global shift' is the opposite of LAS offset ;)

	bool noShiftCanBeUsed     = !ccGlobalShiftManager::NeedShift(bbMax);
	bool minBBCornerCanBeUsed = !ccGlobalShiftManager::NeedShift(bbMax - bbMin);

	bool globalShiftAndLASOffsetXYAreDifferent = (hasLASOffset != hasGlobalShift
	                                              || std::abs(originalLASOffset.x + globalShift.x) > 0.01 //'global shift' is the opposite of LAS offset ;)
	                                              || std::abs(originalLASOffset.y + globalShift.y) > 0.01);

	// List the available offset options
	QMap<LasSaveDialog::Offset, CCVector3d> availableOffsets;
	if (hasLASOffset)
	{
		availableOffsets[LasSaveDialog::ORIGN_LAS_OFFSET] = originalLASOffset;
	}
	if (hasGlobalShift)
	{
		availableOffsets[LasSaveDialog::GLOBAL_SHIFT] = -globalShift; //'global shift' is the opposite of LAS offset ;)
	}
	CCVector3d minBBCornerOffset(bbMin.x, bbMin.y, 0.0);
	// if (minBBCornerCanBeUsed) // we can still display it, even if it's not optimal
	{
		availableOffsets[LasSaveDialog::MIN_BB_CORNER] = minBBCornerOffset;
	}
	static CCVector3d s_customLASOffset(0, 0, 0);
	static bool       s_customLASOffsetWasUsedPreviously = false;
	{
		availableOffsets[LasSaveDialog::CUSTOM_LAS_OFFSET] = s_customLASOffset;
	}
	bool customLASOffsetCanBeUsed = s_customLASOffsetWasUsedPreviously && !ccGlobalShiftManager::NeedShift(bbMax - s_customLASOffset);

	// Now select the default offset (may be changed later by the user in the GUI version)
	LasSaveDialog::Offset defaultSelectedOffset = LasSaveDialog::ORIGN_LAS_OFFSET;
	if (customLASOffsetCanBeUsed)
	{
		// If the user has input a custom LAS offset, that's probably for good reasons
		defaultSelectedOffset = LasSaveDialog::CUSTOM_LAS_OFFSET;

		ccLog::Warning(QString("[LAS] The previously input custom LAS offset (%1 ; %2 ; %3) was selected by default").arg(s_customLASOffset.x).arg(s_customLASOffset.y).arg(s_customLASOffset.z));
	}
	else if (globalShiftCanBeUsed && globalShiftAndLASOffsetXYAreDifferent)
	{
		// If the global shift is different from the LAS offset, we prefer it
		defaultSelectedOffset = LasSaveDialog::GLOBAL_SHIFT;

		if (hasLASOffset)
		{
			ccLog::Warning("[LAS] The current Global Shift will be used as LAS offset by default (potentially different from the original LAS offset)");
		}
	}
	else if (parameters.parentWidget && !hasGlobalShift && noShiftCanBeUsed && globalShiftAndLASOffsetXYAreDifferent)
	{
		// In the GUI version, we can choose no shift by default, as the user can now change this behavior and restore the original LAS offset
		availableOffsets[LasSaveDialog::GLOBAL_SHIFT] = CCVector3d(0, 0, 0);
		defaultSelectedOffset                         = LasSaveDialog::GLOBAL_SHIFT;

		if (hasLASOffset)
		{
			ccLog::Warning("[LAS] LAS offset will be set to (0,0,0) by default (potentially different from the original LAS offset)");
		}
	}
	else if (orignialLasOffsetCanBeUsed)
	{
		// Else if we can use the original offset, let's use it
		defaultSelectedOffset = LasSaveDialog::ORIGN_LAS_OFFSET;

		if (hasGlobalShift || globalShiftAndLASOffsetXYAreDifferent)
		{
			ccLog::Warning(QString("[LAS] The original LAS offset (%1 ; %2 ; %3) will be used by default").arg(originalLASOffset.x).arg(originalLASOffset.y).arg(originalLASOffset.z));
		}
	}
	else
	{
		if (hasLASOffset)
		{
			ccLog::Warning(QString("[LAS] The original LAS offset (%1 ; %2 ; %3) doesn't seem to be optimal").arg(originalLASOffset.x).arg(originalLASOffset.y).arg(originalLASOffset.z));
		}

		if (hasGlobalShift && (globalShiftCanBeUsed || !minBBCornerCanBeUsed))
		{
			ccLog::Warning("[LAS] The current Global Shift will be used as LAS offset by default");
			defaultSelectedOffset = LasSaveDialog::GLOBAL_SHIFT;
		}
		else if (minBBCornerCanBeUsed)
		{
			ccLog::Warning("[LAS] The minimum bounding-box corner (X, Y) will be used as LAS offset by default");
			defaultSelectedOffset = LasSaveDialog::MIN_BB_CORNER;
		}
		else
		{
			// we still use the original offset as we don't have a better solution...
		}
	}
	saveDialog.setOffsets(availableOffsets, defaultSelectedOffset);
	// consistency check
	{
		LasSaveDialog::Offset dummyOffsetType;
		if ((saveDialog.chosenOffset(dummyOffsetType) - availableOffsets[defaultSelectedOffset]).norm() > 1.0e-6
		    || dummyOffsetType != defaultSelectedOffset)
		{
			ccLog::Error("Internal error: inconsistency detected between the save dialog and the code... please contact the admin");
		}
	}

	// compute the optimal scale
	CCVector3d optimalScale;
	{
		assert(availableOffsets.contains(defaultSelectedOffset));
		CCVector3d defaultLASOffset = availableOffsets[defaultSelectedOffset];

		// Maximum cloud 'extents' relatively to the 'offset' point
		CCVector3d diagPos = bbMax - defaultLASOffset;
		CCVector3d diagNeg = defaultLASOffset - bbMin;
		CCVector3d diag(std::max(diagPos.x, diagNeg.x),
		                std::max(diagPos.y, diagNeg.y),
		                std::max(diagPos.z, diagNeg.z));

		// Optimal scale (for accuracy) --> 1e-9 because the maximum 32bits integer is roughly +/-2e+9
		optimalScale = {1.0e-9 * std::max<double>(diag.x, 1.0),
		                1.0e-9 * std::max<double>(diag.y, 1.0),
		                1.0e-9 * std::max<double>(diag.z, 1.0)};
	}

	// See if we have a scale from an origin LAS file
	CCVector3d originalScale;
	bool       canUseOriginalScale = false;
	bool       hasScaleMetaData    = LasMetadata::LoadScaleFrom(*pointCloud, originalScale);
	if (hasScaleMetaData)
	{
		// We may not be able to use the previous LAS scale
		canUseOriginalScale = (std::abs(originalScale.x) >= optimalScale.x // DGM: some LAS files have negative scales?!
		                       && std::abs(originalScale.y) >= optimalScale.y
		                       && std::abs(originalScale.z) >= optimalScale.z);

		// If we can use the original scale, it will become the default scale
		saveDialog.setOriginalScale(originalScale, canUseOriginalScale, true);
	}

	// Uniformize the optimal scale to make it less disturbing to some lastools users ;)
	{
		double maxScale = std::max(optimalScale.x, std::max(optimalScale.y, optimalScale.z));
		double n        = ceil(log10(maxScale)); // ceil because n should be negative
		maxScale        = pow(10.0, n);
		optimalScale.x = optimalScale.y = optimalScale.z = maxScale;
	}
	saveDialog.setOptimalScale(optimalScale);

	// Find the best version for the file or try to use the one from original file
	LasDetails::LasVersion savedVersion;
	bool                   hasSavedVersion = LasMetadata::LoadLasVersionFrom(*pointCloud, savedVersion);
	LasDetails::LasVersion bestVersion     = LasDetails::SelectBestVersion(*pointCloud, hasSavedVersion ? savedVersion.minorVersion : 0);

	saveDialog.setVersionAndPointFormat(bestVersion);

	// Try to pre-fill in the UI any saved extra scalar fields
	LasVlr vlr;
	if (LasMetadata::LoadVlrs(*pointCloud, vlr))
	{
		LasExtraScalarField::MatchExtraBytesToScalarFields(vlr.extraScalarFields, *pointCloud);
	}

	saveDialog.setExtraScalarFields(vlr.extraScalarFields);

	if (parameters.alwaysDisplaySaveDialog)
	{
		saveDialog.exec();
		if (saveDialog.result() == QDialog::Rejected)
		{
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	LasSaver::Parameters params;
	{
		params.standardFields                      = saveDialog.fieldsToSave();
		params.extraFields                         = saveDialog.extraFieldsToSave();
		params.shouldSaveRGB                       = saveDialog.shouldSaveRGB();
		params.shouldSaveNormalsAsExtraScalarField = saveDialog.shouldSaveNormalsAsExtraScalarField();
		params.shouldSaveWaveform                  = saveDialog.shouldSaveWaveform();

		saveDialog.selectedVersion(params.versionMajor, params.versionMinor);
		params.pointFormat = saveDialog.selectedPointFormat();

		params.lasScale = saveDialog.chosenScale();

		LasSaveDialog::Offset offsetType;
		params.lasOffset = saveDialog.chosenOffset(offsetType);

		// Remember any custom offset input by the user
		if (offsetType == LasSaveDialog::CUSTOM_LAS_OFFSET)
		{
			s_customLASOffset                  = params.lasOffset;
			s_customLASOffsetWasUsedPreviously = true;
		}
	}

	// In case of command line call, add automatically all remaining scalar fields as extra scalar fields
	if (!parameters.alwaysDisplaySaveDialog)
	{
		uint sfCount = pointCloud->getNumberOfScalarFields();
		for (uint index = 0; index < sfCount; index++)
		{
			ccScalarField*     sf     = static_cast<ccScalarField*>(pointCloud->getScalarField(index));
			const std::string& sfName = sf->getName();
			bool               found  = false;
			for (auto& el : params.standardFields)
			{
				if (sfName.compare(el.name()) == 0)
				{
					found = true;
					break;
				}
			}
			if (!found)
			{
				for (auto& el : params.extraFields)
				{
					if (sfName.compare(el.scalarFields[0]->getName()) == 0)
					{
						found = true;
						break;
					}
				}
			}
			if (!found)
			{
				ccLog::Print("[LAS] scalar field " + QString::fromStdString(sfName) + " will be saved automatically in the extra fields of the output file");
				LasExtraScalarField field;
				const std::string   stdName = sfName;
				strncpy(field.name, stdName.c_str(), LasExtraScalarField::MAX_NAME_SIZE);

				if (stdName.size() > LasExtraScalarField::MAX_NAME_SIZE)
				{
					ccLog::Warning("[LAS] Extra Scalar field name '%s' is too long and will be truncated",
					               stdName.c_str());
				}

				field.type            = LasExtraScalarField::DataType::f32;
				field.scalarFields[0] = sf;

				params.extraFields.push_back(field);
			}
		}
	}

	LasSaver      saver(*pointCloud, params);
	CC_FILE_ERROR error = saver.open(filename);
	if (error != CC_FERR_NO_ERROR)
	{
		return error;
	}

	ccProgressDialog progressDialog(true, parameters.parentWidget);
	progressDialog.setMethodTitle("Saving LAS points");
	progressDialog.setInfo("Saving points");
	QScopedPointer<CCCoreLib::NormalizedProgress> normProgress;
	if (parameters.parentWidget)
	{
		normProgress.reset(new CCCoreLib::NormalizedProgress(&progressDialog, pointCloud->size()));
		progressDialog.start();
	}

	for (unsigned i = 0; i < pointCloud->size(); ++i)
	{
		error = saver.saveNextPoint();
		if (error != CC_FERR_NO_ERROR)
		{
			break;
		}

		if (normProgress && !normProgress->oneStep())
		{
			error = CC_FERR_CANCELED_BY_USER;
			break;
		}
	}

	if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
	{
		ccLog::Warning(QString("[LAS] laszip error :'%1'").arg(saver.getLastError()));
		return error;
	}

	if (saver.canSaveWaveforms())
	{
		const ccPointCloud::SharedFWFDataContainer& fwfData = pointCloud->fwfData();
		QFileInfo                                   info(filename);
		QString                                     wdpFilename = QString("%1/%2.wdp").arg(info.path(), info.baseName());
		QFile                                       fwfFile(wdpFilename);

		if (!fwfFile.open(QIODevice::WriteOnly))
		{
			ccLog::Error("[LAS] Failed to write waveform data");
			error = CC_FERR_WRITING;
		}
		else
		{
			{
				LasDetails::EvlrHeader header = LasDetails::EvlrHeader::Waveform();
				QDataStream            stream(&fwfFile);
				stream << header;
			}
			fwfFile.write(reinterpret_cast<const char*>(fwfData->data()), fwfData->size());
			ccLog::Print(QString("[LAS] Successfully saved FWF in external file '%1'").arg(wdpFilename));
		}
	}

	return error;
}
