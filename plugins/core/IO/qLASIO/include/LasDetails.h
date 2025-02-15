#pragma once

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

// CCCorelib
#include <BoundingBox.h>
#include <CCTypes.h>

// Qt
#include <QtGlobal>

// System
#include <array>
#include <cstdint>
#include <limits>

#include <vector>


class ccPointCloud;
class ccScalarField;

class QDataStream;

struct laszip_header;
struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;

constexpr size_t LAS_VLR_HEADER_SIZE = 54;
constexpr double SCAN_ANGLE_SCALE    = 0.006;

/// This namespace regroups constants for all the names we use
/// in CloudCompare's ScalarField system for the standard dimensions defined by the LAS Spec.
///
/// Notice that RGB and Waveforms are missing, that is normal as they
/// are not treated as scalar fields within CloudCompare.
namespace LasNames
{
	constexpr const char* Intensity         = "Intensity";
	constexpr const char* ReturnNumber      = "Return Number";
	constexpr const char* NumberOfReturns   = "Number Of Returns";
	constexpr const char* ScanDirectionFlag = "Scan Direction Flag";
	constexpr const char* EdgeOfFlightLine  = "EdgeOfFlightLine";
	constexpr const char* Classification    = "Classification";
	constexpr const char* SyntheticFlag     = "Synthetic Flag";
	constexpr const char* KeypointFlag      = "Keypoint Flag";
	constexpr const char* WithheldFlag      = "Withheld Flag";
	constexpr const char* ScanAngleRank     = "Scan Angle Rank";
	constexpr const char* UserData          = "User Data";
	constexpr const char* PointSourceId     = "Point Source ID";
	constexpr const char* GpsTime           = "Gps Time";

	// 1.4 point format 6 stuff
	constexpr const char* ScanAngle      = "Scan Angle";
	constexpr const char* ScannerChannel = "Scanner Channel";
	constexpr const char* OverlapFlag    = "Overlap Flag";
	constexpr const char* NearInfrared   = "Near Infrared";
} // namespace LasNames

namespace LasDetails
{
	/// Unscaled Extent for LAS or COPC
	///
	/// This is an alias for a double-precision bounding box (BB) that is meant to represent:
	/// - The unscaled extent of a LAS file (as defined in the header), or
	/// - The unscaled bounding box of a COPC file (derived from copc::Info -> center/halfsize variables).
	///
	/// Note: The COPC extent is cubic, corresponding to the COPC octree shape,
	/// rather than the bounding box of the actual points.
	using UnscaledExtent = CCCoreLib::BoundingBoxTpl<double>;

	/// LAZ Chunk but with point offset in file.
	///
	/// This is primarily used for COPC reading.
	/// Since the LASZip API does not allow chunk byte seeking, we use the point offset for seeking.
	/// This data structure can also encode the offset in the current CC point cloud,
	/// which is useful for the LOD data structure construction.
	/// Additionally, it includes a status field that indicates whether it should be loaded (PASS),
	/// checked during loading (INTERSECT_BB), or pruned (FAIL).
	struct ChunkInterval
	{
		enum class eFilterStatus
		{
			PASS         = 0,
			INTERSECT_BB = 1,
			FAIL         = 2,
		};

		ChunkInterval() = default;
		ChunkInterval(uint64_t _pointOffsetInFile, uint64_t _pointCount)
		    : pointOffsetInFile(_pointOffsetInFile)
		    , pointCount(_pointCount){};

		/// point offset in the LAZ file
		uint64_t pointOffsetInFile{0};
		/// point offset in the CC cloud (used for LOD construction)
		uint64_t pointOffsetInCCCloud{0};
		/// point count in the chunk
		uint64_t pointCount{0};
		/// Number of points that failed the filter pass (optional)
		/// total point for this chunk in the ccPointCloud
		/// should be pointCount - filteredPointCount
		uint64_t filteredPointCount{0};
		/// encode the status of the chunk
		/// basically chunks with FAIL status should not be loaded.
		eFilterStatus status{eFilterStatus::PASS};
	};

	/// the "true" number of points in a las file.
	/// the field to query in the laszip_header in order to have the number of point
	/// is format dependant.
	uint64_t TrueNumberOfPoints(const laszip_header* laszipHeader);

	// The position of the overlap flag in the classification flags
	// (valid for fmt >= 6)
	constexpr unsigned OVERLAP_FLAG_BIT_POS  = 3;
	constexpr unsigned OVERLAP_FLAG_BIT_MASK = 1 << OVERLAP_FLAG_BIT_POS;

	/// Header part of a LAS Extended VLR
	///
	/// In a LAS file, EVLRs are stored after the points.
	///
	/// We need this struct as Waveform data can be stored inside EVLRs.
	struct EvlrHeader
	{
		static constexpr size_t SIZE             = 60;
		static constexpr size_t USER_ID_SIZE     = 16;
		static constexpr size_t DESCRIPTION_SIZE = 32;

		char     userID[USER_ID_SIZE];
		uint16_t recordID{0};
		uint64_t recordLength{0};
		char     description[DESCRIPTION_SIZE];

		EvlrHeader() = default;

		static EvlrHeader Waveform();

		bool isWaveFormDataPackets() const;

		bool isCOPCEntry() const;

		friend QDataStream& operator>>(QDataStream& stream, EvlrHeader& hdr);
		friend QDataStream& operator<<(QDataStream& stream, const EvlrHeader& hdr);
	};

	/// Returns the size for the given point format id
	///
	/// Returns a size of 0 if the point format does not exist or is not handled
	///
	/// \param pointFormat the point format
	///
	uint16_t PointFormatSize(unsigned pointFormat);

	/// Returns the header size for the given minor version of the standard used
	uint16_t HeaderSize(unsigned versionMinor);

	/// Returns whether the point format supports GPS Time
	inline bool HasGpsTime(unsigned pointFormatId)
	{
		return pointFormatId == 1
		       || pointFormatId >= 3;
	}

	/// Returns whether the point format supports RGB
	inline bool HasRGB(unsigned pointFormatId)
	{
		return pointFormatId == 2
		       || pointFormatId == 3
		       || pointFormatId == 5
		       || pointFormatId == 7
		       || pointFormatId == 8
		       || pointFormatId == 10;
	}

	/// Returns whether the point format supports Waveforms
	inline bool HasWaveform(unsigned pointFormatId)
	{
		return pointFormatId == 4
		       || pointFormatId == 5
		       || pointFormatId >= 9;
	}

	/// Returns whether the point format supports Near Infrared
	inline bool HasNearInfrared(unsigned pointFormatId)
	{
		return pointFormatId == 8
		       || pointFormatId == 10;
	}

	/// Returns the number of bytes the VLRs amounts to.
	/// This includes the headers of each VLRs.
	unsigned SizeOfVlrs(const laszip_vlr_struct* vlrs, unsigned numVlrs);

	/// Returns whether the vlr is the vlr for/of LASzip compression.
	bool IsLaszipVlr(const laszip_vlr_struct&);

	/// Returns whether the vlr describes extra bytes.
	bool IsExtraBytesVlr(const laszip_vlr_struct&);

	/// Returns point the formats available for the given version.
	///
	/// If the version does not exists or is not supported a nullptr is returned.
	///
	/// \param version version string, must be "major.minor" e.g. "1.2"
	const std::vector<unsigned>& PointFormatsAvailableForVersion(const QString& version);

	const std::array<const char*, 3>& AvailableVersions();

	/// See `SelectBestVersion`
	struct LasVersion
	{
		int pointFormat  = 3;
		int minorVersion = 2;
	};

	/// This function looks into the point cloud
	/// and returns a LAS version + point format
	/// that best matches what the point cloud contains.
	/// If a previous minor version is provided, this method won't downgrade it
	LasVersion SelectBestVersion(const ccPointCloud& cloud, int previousMinorVersion = 0);

	/// Clones the content of the `src` vlr into the `dst` vlr.
	void CloneVlrInto(const laszip_vlr_struct& src, laszip_vlr_struct& dst);

} // namespace LasDetails
