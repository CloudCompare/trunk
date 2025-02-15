#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
// #                            COPCLoader                                  #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: Hobu, Inc.                           	    #
// #                                                                        #
// ##########################################################################

// qLASIO
#include "LasDetails.h"

// CCCoreLib
#include <CCGeom.h>

// Qt
#include <QDataStream>

// System
#include <array>
#include <cstddef>
#include <cstdint>

/// COPC structs as defined in the specs
/// We kept COPC naming convention (snake case)
/// this is based on COPC v1 specs. See https://copc.io for documentation
namespace copc
{
	struct Info
	{
		/// Extract extent
		///
		/// return true if extent is valid false otherwise
		/// Note that COPC extent is cubic, it's the extent of the
		/// COPC octree, not the BB of the points.
		bool extractExtent(LasDetails::UnscaledExtent& extent) const
		{
			extent.clear();
			CCVector3d center(center_x, center_y, center_z);
			CCVector3d halfsizeVec(halfsize, halfsize, halfsize);
			extent.add(center + halfsizeVec);
			extent.add(center - halfsizeVec);
			// it should be valid ;) but we test its norm too
			// maybe we could also test it's a cube
			return extent.isValid() && extent.getDiagNormd() > 0;
		}

		/// Overload the stream extraction operation to read an Info object from a QDataStream.
		friend QDataStream& operator>>(QDataStream& stream, Info& copc_info)
		{
			// use quint64 for linux compat (macOS and Win are fine)
			quint64 root_hier_offset_, root_hier_size_, reserved_;
			stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);
			stream >> copc_info.center_x >> copc_info.center_y >> copc_info.center_z;
			stream >> copc_info.halfsize >> copc_info.spacing;
			stream >> root_hier_offset_ >> root_hier_size_;
			copc_info.root_hier_offset = root_hier_offset_;
			copc_info.root_hier_size   = root_hier_size_;
			stream >> copc_info.gpstime_minimum >> copc_info.gpstime_minimum;
			// Flush reserved
			std::for_each(std::begin(copc_info.reserved), std::end(copc_info.reserved), [&stream, &reserved_](auto& reserved)
			              { stream >> reserved_; });
			return stream;
		};

		static constexpr size_t SIZE = 160;

		/// Geometric parameters (root cell extent)
		double center_x{0.0};
		double center_y{0.0};
		double center_z{0.0};
		double halfsize{0.0};
		/// minimum distance between points at root (0) level.
		/// it is supposed to be halved at each level
		double spacing{0.0};

		/// Byte offeset in the LAS file
		/// the root node is not necessary the node comming at first
		/// in the COPC EVRL, so it's not the start of the COPC
		/// EVRL Reccord.
		uint64_t root_hier_offset{0};
		uint64_t root_hier_size{0};

		double gpstime_minimum{0.0};
		double gpstime_maximum{0.0};

		/// Should be all set to 0
		uint64_t reserved[11] = {0};
	};

	struct VoxelKey
	{
		/// Create an invalid Key
		static VoxelKey Invalid()
		{
			return {-1, 0, 0, 0};
		}

		/// Same as default constructor but used by convenience for its semantic meaning
		static VoxelKey Root()
		{
			return {0, 0, 0, 0};
		}

		/// Check if a key is valid
		bool isValid() const
		{
			// branching factor in one dim is 2 so
			// -> num cell is actually 2^level
			// -> max id in one dimention is actually 2^level - 1
			const int32_t numCellInOneDim = std::pow(2, level);
			return level >= 0 && x >= 0 && y >= 0 && z >= 0 && z < numCellInOneDim && y < numCellInOneDim && x < numCellInOneDim;
		}

		/// Construct the voxel Extent (aligned BB) from a given root extent
		bool extractExtent(const LasDetails::UnscaledExtent& rootExtent, LasDetails::UnscaledExtent& voxelExtent) const
		{
			if (!rootExtent.isValid())
			{
				return false;
			}
			// root is supposed to be a cube, cell width could be computed using only one dim
			const int32_t numCellAtLevel = std::pow(2, level);
			const double  cellSize       = (rootExtent.maxCorner().x - rootExtent.minCorner().x) / numCellAtLevel;

			// Like in PDAL, we clamp the voxelExtent to rootExtent extrem points coordinates in case
			// the voxelExtent is an extrem one too. it avoid as much as possible for rounding errors.
			const CCVector3d minCorner(
			    (x == 0 ? rootExtent.minCorner().x : rootExtent.minCorner().x + (cellSize * x)),
			    (y == 0 ? rootExtent.minCorner().y : rootExtent.minCorner().y + (cellSize * y)),
			    (z == 0 ? rootExtent.minCorner().z : rootExtent.minCorner().z + (cellSize * z)));

			const int32_t    maxIdAtLevel = numCellAtLevel - 1;
			const CCVector3d maxCorner(
			    (x == maxIdAtLevel ? rootExtent.maxCorner().x : minCorner.x + cellSize),
			    (y == maxIdAtLevel ? rootExtent.maxCorner().y : minCorner.y + cellSize),
			    (z == maxIdAtLevel ? rootExtent.maxCorner().z : minCorner.z + cellSize));
			voxelExtent = LasDetails::UnscaledExtent(minCorner, maxCorner, true);
			return true;
		}

		/// Generate the 8 putative children for this key
		std::array<VoxelKey, 8> childrenKeys() const
		{
			std::array<VoxelKey, 8> children{};
			for (int32_t i = 0; i < 8; i++)
			{
				// Bitwise operation are taken from PDAL
				// see https://github.com/PDAL/PDAL/blob/0c8e84521c040aa8a03ca848d32fb30e75a6e1ec/io/private/copc/Key.hpp#L105-L111
				children[i].level = level + 1;
				children[i].x     = (x << 1) | (i & 0x1);
				children[i].y     = (y << 1) | ((i >> 1) & 0x1);
				children[i].z     = (z << 1) | ((i >> 2) & 0x1);
			}
			return children;
		}

		/// return parent key of this node
		VoxelKey parentKey() const
		{
			if (!isValid() || level == 0)
			{
				return VoxelKey::Invalid();
			}
			return {level - 1, x >> 1, y >> 1, z >> 1};
		}

		/// overloaded equality operator
		bool operator==(const VoxelKey& other) const
		{
			return other.level == level && other.x == x && other.y == y && other.z == z;
		};

		/// Check if a given key is a valid parent of this key
		bool isParent(const VoxelKey& other) const
		{
			if (!other.isValid())
				return false;
			return other.parentKey() == *this;
		}

		friend QDataStream& operator>>(QDataStream& stream, VoxelKey& key)
		{
			stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);
			stream >> key.level >> key.x >> key.y >> key.z;
			return stream;
		};

		static constexpr size_t SIZE = 16;
		// octree depth
		int32_t level{0};
		int32_t x{0};
		int32_t y{0};
		int32_t z{0};
	};

	struct Entry
	{
		/// Check if the Entry is actually a hierarchy page
		bool isHierarchyPage() const
		{
			return point_count < 0;
		}

		/// Overload the stream extraction opentation to read an Info object from a QDataStream.
		friend QDataStream& operator>>(QDataStream& stream, Entry& entry)
		{
			stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);
			quint64 offset_;
			stream >> entry.key >> offset_ >> entry.byte_size >> entry.point_count;
			entry.offset = offset_;
			return stream;
		};

		static constexpr size_t SIZE = VoxelKey::SIZE + 16;
		VoxelKey                key;
		/// Absolute offset to the data chunk if the pointCount > 0.
		/// Absolute offset to a child hierarchy page if the pointCount is -1.
		/// 0 if the pointCount is 0.
		uint64_t offset{0};
		int32_t  byte_size{0};
		/// If > 0, represents the number of points in the data chunk.
		/// If -1, indicates the information for this octree node is found in another hierarchy page.
		/// If 0, no point data exists for this key, though may exist for child entries.
		int32_t point_count{0};
	};

	struct Page
	{
		Page(const uint64_t offset_, const size_t num_entries_)
		    : offset(offset_)
		    , num_entries(num_entries_)
		{
		}
		~Page() = default;

		uint64_t offset{0};
		size_t   num_entries{0};
	};
} // namespace copc

namespace std
{
	// Hash function taken from PDAL.
	// see https://github.com/PDAL/PDAL/blob/0c8e84521c040aa8a03ca848d32fb30e75a6e1ec/io/private/copc/Entry.hpp#L79-L89
	template <>
	struct hash<copc::VoxelKey>
	{
		std::size_t operator()(copc::VoxelKey const& key) const noexcept
		{
			std::hash<uint64_t> h;

			uint64_t k1 = (static_cast<uint64_t>(key.level) << 32) | key.x;
			uint64_t k2 = (static_cast<uint64_t>(key.y) << 32) | key.z;
			return h(k1) ^ (h(k2) << 1);
		}
	};
} // namespace std
