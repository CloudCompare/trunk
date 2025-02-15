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

#include "CopcLoader.h"
#include "CopcVlrs.h"

// CCCoreLib
#include <CCGeom.h>
#include <ParallelSort.h>
#include <ccLog.h>

// System
#include <algorithm>
#include <queue>

namespace copc
{
	/// static const used to check if a file is a COPC
	static const uint8_t         COPC_LAS_VERSION_MINOR{4};
	static const uint8_t         COPC_LAS_DATA_FORMAT_GT{5};
	static const uint8_t         COPC_LAS_DATA_FORMAT_LT{9};
	static const uint8_t         COPC_RECORD_ID{1};
	static constexpr const char* COPC_USER_ID = "copc";

	bool CopcLoader::IsPutativeCOPCFile(const laszip_header* laszipHeader)
	{
		return laszipHeader->version_minor >= COPC_LAS_VERSION_MINOR
		       && laszipHeader->point_data_format > COPC_LAS_DATA_FORMAT_GT
		       && laszipHeader->point_data_format < COPC_LAS_DATA_FORMAT_LT
		       && laszipHeader->number_of_variable_length_records > 0
		       && IsCOPCVlr(laszipHeader->vlrs[0]);
	}

	/// Check if a given VLR is a COPC one
	///
	/// Notes: COPC VLR has to be at id 0 of the vlr array.
	bool CopcLoader::IsCOPCVlr(const laszip_vlr_struct& vlr)
	{
		return (strcmp(COPC_USER_ID, vlr.user_id) == 0 && vlr.record_id == COPC_RECORD_ID);
	}

	CopcLoader::CopcLoader(const laszip_header* laszipHeader, const QString& fileName)
	{
		// Parse the info vlr
		const laszip_vlr copcInfoVlr = laszipHeader->vlrs[0];
		if (copcInfoVlr.record_length_after_header != Info::SIZE)
		{
			ccLog::Warning("[LAS] laszip error: invalid Copc Info");
			return;
		}

		// extract the COPC infos
		QDataStream copcInfoData(QByteArray::fromRawData(reinterpret_cast<const char*>(copcInfoVlr.data), Info::SIZE));
		copcInfoData >> m_copcInfo;

		if (!m_copcInfo.extractExtent(m_extent))
		{
			ccLog::Warning("[LAS] laszip error: invalid Copc extent");
			return;
		}
		m_ClippingConstraint = m_extent;

		QFile copcFile;
		copcFile.setFileName(fileName);

		if (!copcFile.open(QFile::ReadOnly))
		{
			ccLog::Warning("[LAS] laszip error: Unable to open the LAS file to parse COPC pages");
			return;
		}

		// Note: the root page is generally not the first entry of COPC EVRL data reccord,
		// This means we cannot obtain the EVLR header by seeking straight to `root_hier_offset` minus the EVLR header size (60).
		// To check the existence of the COPC EVLR hierarchy, we'll have to iterate thought all
		// EVLRs starting from laszip_header::start_of_first_extended_variable_length_record...
		// Here we do not check the existence of the EVLR header. we seek directly to the root_hier_offset
		// and validate each seeking operation by checking for EOF which should be robust enough.
		QDataStream        copcDataSource(&copcFile);
		std::queue<Page>   pageQueue;
		std::vector<Entry> entries;
		// Enque the root page
		pageQueue.emplace(m_copcInfo.root_hier_offset, m_copcInfo.root_hier_size / Entry::SIZE);

		size_t numPages = 0;
		while (!pageQueue.empty())
		{
			const Page& page = pageQueue.front();
			++numPages;

			copcFile.seek(page.offset);
			if (copcDataSource.status() == QDataStream::Status::ReadPastEnd)
			{
				ccLog::Warning("[LAS] Failed to read the the COPC pages (unexpected EOF)");
				return;
			}
			try
			{
				entries.reserve(entries.size() + page.num_entries);
			}
			catch (const std::bad_alloc&)
			{
				entries.clear();
				ccLog::Warning("[LAS] Failed to allocate meemory for the COPC datastructure");
				return;
			}
			QDataStream entryDataStream(&copcFile);
			for (size_t entry_id = 0; entry_id < page.num_entries; ++entry_id)
			{
				Entry entry;
				copcDataSource >> entry;

				// Check the key validity and early return if it fails
				if (!entry.key.isValid())
				{
					ccLog::Warning("[LAS] Failed to read the the COPC pages (invalid entry)");
					return;
				}

				m_maxLevel = std::max(m_maxLevel, entry.key.level);
				if (copcDataSource.status() == QDataStream::Status::ReadPastEnd)
				{
					ccLog::Warning("[LAS] Failed to read the the COPC pages (unexpected EOF)");
					return;
				}
				if (entry.isHierarchyPage())
				{
					pageQueue.emplace(entry.offset, entry.byte_size / Entry::SIZE);
				}
				else
				{
					m_numPoints += entry.point_count;
					entries.push_back(std::move(entry));
				}
			}
			pageQueue.pop();
		}

		ccLog::Print("[LAS] COPC file with %zu pages / %zu entries / %llu points", numPages, entries.size(), m_numPoints);

		try
		{
			// init our "octree" structure
			generateChunktableIntervalsHierarchy(entries);
		}
		catch (const std::bad_alloc&)
		{
			entries.clear();
			m_chunkIntervalsHierarchy.clear();
			ccLog::Warning("[LAS] Failed to allocate meemory for the COPC datastructure");
			return;
		}

		// Consistency check: Check the number of points in the COPC data structure
		// is equal to the number of points in the laszip header
		if (LasDetails::TrueNumberOfPoints(laszipHeader) != m_numPoints)
		{
			ccLog::Warning("[LAS] Failed to read the the COPC file (number of points in COPC structure does not match the one in LAZ header)");
			return;
		}

		// Consistency check: Does the octree have a root node?
		if (!hasRoot())
		{
			ccLog::Warning("[LAS] missing root node in the COPC file");
			return;
		}

		// No this is considered as a valid file
		m_isValid = true;

		// Fill m_levelPointCounts vector (used by UI to provide feedback)
		m_levelPointCounts.resize(m_maxLevel + 1);
		for (const auto& entry : entries)
		{
			m_levelPointCounts[entry.key.level] += entry.point_count;
		}

		// Consistency check: is the octree traversable
		// Otherwise, we only issue a warning. We will simply not handle the non-traversable part.
		if (!isTraversable())
		{
			ccLog::Warning("[LAS] COPC file contains unreachable nodes");
		}
	}

	void CopcLoader::generateChunktableIntervalsHierarchy(std::vector<Entry>& entries)
	{
		// Sort entries by offset to be able to get the first point of each chunk.
		ParallelSort(std::begin(entries), std::end(entries), [](const Entry& a, const Entry& b)
		             { return a.offset < b.offset; });
		uint64_t starting_point = 0;
		m_chunkIntervalsHierarchy.reserve(entries.size());
		std::for_each(std::cbegin(entries), std::cend(entries), [&starting_point, this](const Entry& entry)
		              {
			m_chunkIntervalsHierarchy.emplace(std::make_pair(entry.key, ChunkInterval(starting_point, static_cast<uint64_t>(entry.point_count))));
			starting_point += entry.point_count; });
	}

	ChunkInterval::eFilterStatus CopcLoader::checkConstraints(const VoxelKey& voxelkey)
	{
		// Level constraint takes precendence over BB constraint
		if (m_hasMaxLevelConstraint && voxelkey.level > m_maxLevelConstraint)
		{
			return ChunkInterval::eFilterStatus::FAIL;
		}
		if (m_hasClippingConstraint)
		{
			LasDetails::UnscaledExtent voxelExtent;
			bool                       valid = voxelkey.extractExtent(m_extent, voxelExtent);

			// this would always pass:
			// m_exent has been checked for consistency in the constructor
			assert(valid);

			// references for convenience
			const auto& aMin = m_ClippingConstraint.minCorner();
			const auto& aMax = m_ClippingConstraint.maxCorner();
			const auto& bMin = voxelExtent.minCorner();
			const auto& bMax = voxelExtent.maxCorner();

			// works because all extents are supposed to be axis aligned
			// test for inclusion
			if (aMax.x >= bMax.x
			    && aMax.y >= bMax.y
			    && aMax.z >= bMax.z
			    && aMin.x <= bMin.x
			    && aMin.y <= bMin.y
			    && aMin.z <= bMin.z)
			{
				return ChunkInterval::eFilterStatus::PASS;
			}
			// test for intersection
			else if (aMax.x >= bMin.x
			         && aMax.y >= bMin.y
			         && aMax.z >= bMin.z
			         && aMin.x <= bMax.x
			         && aMin.y <= bMax.y
			         && aMin.z <= bMax.z)
			{
				return ChunkInterval::eFilterStatus::INTERSECT_BB;
			}
			else
			{
				return ChunkInterval::eFilterStatus::FAIL;
			}
		}
		return ChunkInterval::eFilterStatus::PASS;
	}

	void CopcLoader::resetIntervalsStatus()
	{
		std::for_each(std::begin(m_chunkIntervalsHierarchy), std::end(m_chunkIntervalsHierarchy), [](auto& kv)
		              {
				kv.second.status = ChunkInterval::eFilterStatus::PASS;
				kv.second.filteredPointCount = 0; });
	}

	void CopcLoader::propagateFailureFlag(const VoxelKey& voxelkey)
	{
		if (voxelkey.level > m_maxLevel)
		{
			return;
		}
		auto maybeNode = m_chunkIntervalsHierarchy.find(voxelkey);
		if (maybeNode != std::end(m_chunkIntervalsHierarchy))
		{
			maybeNode->second.status = ChunkInterval::eFilterStatus::FAIL;
			for (const auto& childKey : voxelkey.childrenKeys())
			{
				propagateFailureFlag(childKey);
			}
		}
	}

	void CopcLoader::recurse(const VoxelKey& voxelkey, std::unordered_set<VoxelKey, std::hash<VoxelKey>>& visitedNodes) const
	{
		if (voxelkey.level <= m_maxLevel && m_chunkIntervalsHierarchy.count(voxelkey))
		{
			visitedNodes.insert(voxelkey);
			for (const auto& childKey : voxelkey.childrenKeys())
			{
				recurse(childKey, visitedNodes);
			}
		}
	}

	bool CopcLoader::isTraversable() const
	{
		std::unordered_set<VoxelKey> visitedNodes;
		recurse(VoxelKey::Root(), visitedNodes);
		return visitedNodes.size() == m_chunkIntervalsHierarchy.size();
	}

	uint64_t CopcLoader::flagIntervalNodes(const VoxelKey& voxelkey)
	{
		uint64_t pointCount = 0;
		auto     maybeNode  = m_chunkIntervalsHierarchy.find(voxelkey);
		if (maybeNode != std::end(m_chunkIntervalsHierarchy))
		{
			auto& node  = maybeNode->second;
			node.status = checkConstraints(voxelkey);
			if (node.status == ChunkInterval::eFilterStatus::FAIL && voxelkey.level < m_maxLevel)
			{
				propagateFailureFlag(voxelkey);
			}
			else
			{
				pointCount = node.pointCount;
				if (voxelkey.level < m_maxLevel)
				{
					const auto childrenKeys = voxelkey.childrenKeys();
					for (const auto& childKey : childrenKeys)
					{
						pointCount += flagIntervalNodes(childKey);
					}
				}
				// node with 0 point at a a given level are allowed but here
				// the entire traversal at this node does not provides any point,
				// so it's a "failure".
				if (pointCount == 0)
				{
					node.status = ChunkInterval::eFilterStatus::FAIL;
				}
			}
		}
		return pointCount;
	}

	void CopcLoader::getChunkIntervalsSet(std::vector<std::reference_wrapper<ChunkInterval>>& sortedChunkIntervalSet, uint64_t& estimatedPointCount)
	{
		resetIntervalsStatus();
		estimatedPointCount = 0;
		sortedChunkIntervalSet.clear();
		sortedChunkIntervalSet.reserve(m_chunkIntervalsHierarchy.size());

		// start by the root and flag all the hierachy according to constraints;
		estimatedPointCount = flagIntervalNodes(VoxelKey::Root());

		// fill sortedChunkIntervalSet with m_chunkIntervalsHierarchy values
		std::transform(std::begin(m_chunkIntervalsHierarchy), std::end(m_chunkIntervalsHierarchy), std::back_inserter(sortedChunkIntervalSet), [](auto& kv)
		               { return std::ref(kv.second); });

		// sort them by starting point to optimize seeking;
		ParallelSort(std::begin(sortedChunkIntervalSet), std::end(sortedChunkIntervalSet), [](const ChunkInterval& a, const ChunkInterval& b)
		             { return a.pointOffsetInFile < b.pointOffsetInFile; });
	}

} // namespace copc
