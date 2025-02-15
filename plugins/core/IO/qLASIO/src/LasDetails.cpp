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

#include "LasDetails.h"

#include "LasScalarField.h"

// LASzip
#include <laszip/laszip_api.h>
// CC
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
// Qt
#include <QDataStream>
// System
#include <cstring>

static const std::vector<unsigned>      PointFormatForV1_2{0, 1, 2, 3};
static const std::vector<unsigned>      PointFormatForV1_3{0, 1, 2, 3, 4, 5};
static const std::vector<unsigned>      PointFormatForV1_4{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const std::array<const char*, 3> VersionsArray{"1.2", "1.3", "1.4"};

namespace LasDetails
{
	bool EvlrHeader::isWaveFormDataPackets() const
	{
		return recordID == 65'535 && strncmp(userID, "LASF_Spec", EvlrHeader::USER_ID_SIZE) == 0;
	}

	bool EvlrHeader::isCOPCEntry() const
	{
		return recordID == 1'000 && strncmp(userID, "copc", EvlrHeader::USER_ID_SIZE) == 0;
	}

	EvlrHeader EvlrHeader::Waveform()
	{
		EvlrHeader self;
		self.recordID = 65'535;
		strncpy(self.userID, "LASF_Spec", EvlrHeader::USER_ID_SIZE);
		strncpy(self.description, "Waveform Data Packets", EvlrHeader::DESCRIPTION_SIZE);
		self.recordLength = 0;
		return self;
	}

	uint64_t TrueNumberOfPoints(const laszip_header* laszipHeader)
	{
		laszip_U64 pointCount;
		if (laszipHeader->version_minor == 4)
		{
			pointCount = laszipHeader->extended_number_of_point_records;
		}
		else
		{
			pointCount = laszipHeader->number_of_point_records;
		}
		return pointCount;
	}

	QDataStream& operator>>(QDataStream& stream, EvlrHeader& hdr)
	{
		stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

		uint16_t reserved;
		stream >> reserved;
		stream.readRawData(hdr.userID, EvlrHeader::USER_ID_SIZE);
		stream >> hdr.recordID;
		quint64 recordLength_;
		stream >> recordLength_;
		hdr.recordLength = recordLength_;
		stream.readRawData(hdr.description, EvlrHeader::DESCRIPTION_SIZE);

		return stream;
	};

	QDataStream& operator<<(QDataStream& stream, const EvlrHeader& hdr)
	{
		uint16_t reserved{0};
		stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

		stream << reserved;
		stream.writeRawData(hdr.userID, EvlrHeader::USER_ID_SIZE);
		stream << hdr.recordID;
		stream << (quint64)hdr.recordLength;
		stream.writeRawData(hdr.description, EvlrHeader::DESCRIPTION_SIZE);

		return stream;
	}

	uint16_t PointFormatSize(unsigned pointFormat)
	{
		switch (pointFormat)
		{
		case 0:
			return 20;
		case 1:
			return 28;
		case 2:
			return 26;
		case 3:
			return 34;
		case 4:
			return 28 + 29;
		case 5:
			return 34 + 29;
		case 6:
			return 30;
		case 7:
			return 36;
		case 8:
			return 38;
		case 9:
			return 30 + 29;
		case 10:
			return 38 + 29;
		default:
			assert(false);
			return 0;
		}
	}

	uint16_t HeaderSize(unsigned versionMinor)
	{
		switch (versionMinor)
		{
		case 2:
			return 227;
		case 3:
			return 227 + 8;
		case 4:
			return 375;
		default:
			return 227;
		}
	}

	bool IsLaszipVlr(const laszip_vlr_struct& vlr)
	{
		if (strcmp(vlr.user_id, "Laszip encoded") == 0 && vlr.record_id == 22204)
		{
			return true;
		}
		return false;
	}

	bool IsExtraBytesVlr(const laszip_vlr_struct& vlr)
	{
		if (strcmp(vlr.user_id, "LASF_Spec") == 0 && vlr.record_id == 4)
		{
			return true;
		}
		return false;
	}

	unsigned SizeOfVlrs(const laszip_vlr_struct* vlrs, unsigned numVlrs)
	{
		constexpr laszip_U32 header_size = static_cast<laszip_U32>(LAS_VLR_HEADER_SIZE);
		return std::accumulate(vlrs,
		                       vlrs + numVlrs,
		                       0,
		                       [=](laszip_U32 size, const laszip_vlr_struct& vlr)
		                       { return vlr.record_length_after_header + header_size + size; });
	}

	const std::vector<unsigned>& PointFormatsAvailableForVersion(const QString& version)
	{
		if (version.size() == 3 && version.startsWith("1."))
		{
			if (version[2] == '2')
			{
				return PointFormatForV1_2;
			}
			if (version[2] == '3')
			{
				return PointFormatForV1_3;
			}
			if (version[2] == '4')
			{
				return PointFormatForV1_4;
			}
		}

		ccLog::Warning("Unknown LAS version: " + version);
		static std::vector<unsigned> InvalidPointFormat;
		return InvalidPointFormat;
	}

	const std::array<const char*, 3>& AvailableVersions()
	{
		return VersionsArray;
	}

	LasVersion SelectBestVersion(const ccPointCloud& cloud, int previousMinorVersion /*=0*/)
	{
		// These are exclusive to 'extended' point formats (>= 6)
		bool hasOverlapFlag    = cloud.getScalarFieldIndexByName(LasNames::OverlapFlag) != -1;
		bool hasNIR            = cloud.getScalarFieldIndexByName(LasNames::NearInfrared) != -1;
		bool hasScannerChannel = cloud.getScalarFieldIndexByName(LasNames::ScannerChannel) != -1;
		bool hasScanAngle      = cloud.getScalarFieldIndexByName(LasNames::ScanAngle) != -1;

		bool isExtendedRequired = hasOverlapFlag || hasNIR || hasScannerChannel || hasScanAngle;

		if (!isExtendedRequired)
		{
			// We may need extended point format because some fields need to store more value
			// than what is possible on non-extended point format.
			int classificationIdx = cloud.getScalarFieldIndexByName(LasNames::Classification);
			if (classificationIdx != -1)
			{
				const CCCoreLib::ScalarField* classification = cloud.getScalarField(classificationIdx);
				if (classification->getMax() > LasScalarField::ValueRange(LasScalarField::Classification).max)
				{
					isExtendedRequired = true;
				}
			}

			int returnNumberIdx = cloud.getScalarFieldIndexByName(LasNames::ReturnNumber);
			if (returnNumberIdx != -1)
			{
				const CCCoreLib::ScalarField* returnNumber = cloud.getScalarField(returnNumberIdx);
				if (returnNumber->getMax() > LasScalarField::ValueRange(LasScalarField::ReturnNumber).max)
				{
					isExtendedRequired = true;
				}
			}

			int numReturnsIdx = cloud.getScalarFieldIndexByName(LasNames::NumberOfReturns);
			if (numReturnsIdx != -1)
			{
				const CCCoreLib::ScalarField* numReturns = cloud.getScalarField(numReturnsIdx);
				if (numReturns->getMax() > LasScalarField::ValueRange(LasScalarField::NumberOfReturns).max)
				{
					isExtendedRequired = true;
				}
			}
		}

		bool hasRGB      = cloud.hasColors();
		bool hasWaveform = cloud.hasFWF();

		if (isExtendedRequired || previousMinorVersion >= 4)
		{
			int pointFormat = 6;
			if (hasWaveform)
			{
				if (hasRGB)
				{
					pointFormat = 10;
				}
				else
				{
					pointFormat = 9;
				}
			}
			else
			{
				if (hasRGB && hasNIR)
				{
					pointFormat = 8;
				}
				else if (hasRGB && !hasNIR)
				{
					pointFormat = 7;
				}
			}
			return {pointFormat, previousMinorVersion};
		}
		else
		{
			bool hasGpsTime   = cloud.getScalarFieldIndexByName(LasNames::GpsTime) != -1;
			int  pointFormat  = 0;
			int  minorVersion = previousMinorVersion == 3 ? 3 : 2;
			if (hasWaveform)
			{
				minorVersion = 3;
				pointFormat  = 4;
				if (hasRGB)
				{
					pointFormat = 5;
				}
			}
			else
			{
				if (hasGpsTime)
				{
					pointFormat += 1;
				}
				if (hasRGB)
				{
					pointFormat += 2;
				}
			}
			return {pointFormat, minorVersion};
		}
	}

	void CloneVlrInto(const laszip_vlr_struct& src, laszip_vlr_struct& dst)
	{
		dst      = src;
		dst.data = new laszip_U8[src.record_length_after_header];
		std::copy(src.data, src.data + src.record_length_after_header, dst.data);
	}

} // namespace LasDetails
