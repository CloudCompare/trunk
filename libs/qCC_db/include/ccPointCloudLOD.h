#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                    COPYRIGHT: CloudCompare project                     #
// #                                                                        #
// ##########################################################################

// qCC_db
#include "ccOctree.h"
#include "ccFrustum.h"
#include "ccGenericGLDisplay.h"
#include "ccVBOManager.h"

// Qt
#include <QMutex>

// system
#include <array>
#include <cassert>
#include <cstdint>
#include <memory>

class ccPointCloud;
class ccPointCloudLODThread;
class ccGenericPointCloudLODVisibilityFlagger;

//! Level descriptor
struct LODLevelDesc
{
	//! Default constructor
	LODLevelDesc()
	    : startIndex(0)
	    , count(0)
	{
	}
	//! Constructor from a start index and a count value
	LODLevelDesc(unsigned _startIndex, unsigned _count)
	    : startIndex(_startIndex)
	    , count(_count)
	{
	}
	//! Start index (refers to the 'indexes' table)
	unsigned startIndex;
	//! Index count for this level
	unsigned count;
};

//! L.O.D. indexes set
typedef std::vector<unsigned> LODIndexSet;

//! L.O.D. (Level of Detail) structure
class ccAbstractPointCloudLOD : public ccAbstractVBOManager
{
  public:
	//! Structure initialization state
	enum State
	{
		NOT_INITIALIZED,
		UNDER_CONSTRUCTION,
		INITIALIZED,
		BROKEN
	};

	//! Default constructor
	ccAbstractPointCloudLOD();

	//! Destructor
	virtual ~ccAbstractPointCloudLOD() = default;

	//! Octree 'tree' node
	struct Node
	{
		// Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!
		uint32_t               pointCount;          //  4 bytes
		float                  radius;              //  4 bytes
		CCVector3f             center;              // 12 bytes
		std::array<int32_t, 8> childIndexes;        // 32 bytes
		float                  score;               //  4 bytes
		uint32_t               firstCodeIndex;      //  4 bytes
		uint32_t               displayedPointCount; //  4 bytes
		ccVBO*                 vbo;                 //  4 bytes
		uint8_t                level;               //  1 byte
		uint8_t                childCount;          //  1 byte
		uint8_t                intersection;        //  1 byte

		// Total									// 67 bytes (68 with alignment)

		//! Default constructor
		Node(uint8_t _level = 0)
		    : pointCount(0)
		    , radius(0)
		    , center(0, 0, 0)
		    , childIndexes{-1, -1, -1, -1, -1, -1, -1, -1}
		    , firstCodeIndex(0)
		    , score(0)
		    , displayedPointCount(0)
		    , vbo(nullptr)
		    , level(_level)
		    , childCount(0)
		    , intersection(UNDEFINED)
		{
		}
	};

	//! Level data
	struct Level
	{
		std::vector<Node> data;
	};

	//! Returns the current state
	inline State getState() const
	{
		QMutexLocker locker(&m_mutex);
		State        state = m_state;
		return state;
	}

	//! Initializes the construction process if needed (could be asynchronous)
	virtual bool init(ccPointCloud* cloud) = 0;

	//! Clears the structure
	virtual void clear() = 0;

	//! Returns whether the structure is null (i.e. not under construction or initialized) or not
	inline bool isNull() const
	{
		return getState() == NOT_INITIALIZED;
	}

	//! Returns whether the structure is initialized or not
	inline bool isInitialized() const
	{
		return getState() == INITIALIZED;
	}

	//! Returns whether the structure is initialized or not
	inline bool isUnderConstruction() const
	{
		return getState() == UNDER_CONSTRUCTION;
	}

	//! Returns whether the structure is broken or not
	inline bool isBroken() const
	{
		return getState() == BROKEN;
	}

	//! Returns the maximum accessible level
	inline unsigned char maxLevel() const
	{
		QMutexLocker locker(&m_mutex);
		return (m_state == INITIALIZED ? static_cast<unsigned char>(std::max<size_t>(1, m_levels.size())) - 1 : 0);
	}

	//! Undefined visibility flag
	static const unsigned char UNDEFINED = 255;

	inline Node& node(int32_t index, unsigned char level)
	{
		assert(level < m_levels.size() && index >= 0 && index < m_levels[level].data.size());
		return m_levels[level].data[index];
	}

	inline const Node& node(int32_t index, unsigned char level) const
	{
		assert(level < m_levels.size() && index >= 0 && index < m_levels[level].data.size());
		return m_levels[level].data[index];
	}

	//! get the root node
	inline Node& root()
	{
		return node(0, 0);
	}

	inline const Node& root() const
	{
		return node(0, 0);
	}

	//! Test all cells visibility with a given frustum
	/** Automatically calls resetVisibility
	 **/
	virtual uint32_t flagVisibility(const ccGLCameraParameters& frustum, ccClipPlaneSet* clipPlanes = nullptr);

	//! Builds an index map with the remaining visible points
	virtual LODIndexSet& getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel) = 0;

	//! Returns the last index map
	inline const LODIndexSet& getLasIndexMap() const
	{
		return m_lastIndexMap;
	}

	//! Returns whether all points have been displayed or not
	inline bool allDisplayed() const
	{
		return m_currentState.displayedPoints >= m_currentState.visiblePoints;
	}

	//! Returns the memory used by the structure (in bytes)
	size_t memory() const;

  public: // inherited from ccAbrasctVBOManager
	virtual void releaseVBOs(const ccGenericGLDisplay* currentDisplay) override = 0;

	virtual bool updateVBOs(const ccPointCloud& pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override = 0;

	virtual bool renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override = 0;

  protected: // methods
	//! Constructor with provided Lod levels
	ccAbstractPointCloudLOD(const std::vector<ccAbstractPointCloudLOD::Level>& lodLayers);

	//! Factory to create the  visibilityFlagger (avoid templating)
	virtual std::unique_ptr<ccGenericPointCloudLODVisibilityFlagger> getVisibilityFlagger(ccAbstractPointCloudLOD& lod, const ccGLCameraParameters& camera, unsigned char maxLevel) = 0;

	//! Sets the current state
	inline void setState(State state)
	{
		QMutexLocker locker(&m_mutex);
		m_state = state;
	}

	//! Clears the internal (nodes) data
	virtual void clearData();

	//! Reserves a new cell at a given level
	/** \return the new cell index in the array corresponding to this level (see m_levels)
	 **/
	int32_t newCell(unsigned char level);

	//! Shrinks the internal data to its minimum size
	void shrink_to_fit();

	//! Resets the internal visibility flags
	/** All nodes are flagged as 'INSIDE' (= visible) and their 'visibleCount' attribute is set to 0.
	 **/
	void resetVisibility();

  protected: // members
	//! Per-level cells data
	std::vector<Level> m_levels;

	//! Parameters of the current render state
	struct RenderParams
	{
		RenderParams()
		    : visiblePoints(0)
		    , displayedPoints(0)
		    , unfinishedLevel(-1)
		    , unfinishedPoints(0)
		{
		}

		//! Number of visible points (for the last visibility test)
		uint32_t visiblePoints;
		//! Number of already displayed points
		uint32_t displayedPoints;
		//! Previously unfinished level
		int unfinishedLevel;
		//! Previously unfinished level
		unsigned unfinishedPoints;
	};

	//! Current rendering state
	RenderParams m_currentState;

	//! Index map
	LODIndexSet m_indexMap;

	//! Last index map
	LODIndexSet m_lastIndexMap;

	//! For concurrent access
	mutable QMutex m_mutex;

	//! State
	State m_state;
};

//! Base class for visibility flagging of Level of Detail (LOD) point cloud nodes.
/*!
 * This class determines which nodes are visible based on frustum culling and optional clipping planes.
 */
class ccGenericPointCloudLODVisibilityFlagger
{
  public:
	ccGenericPointCloudLODVisibilityFlagger(ccAbstractPointCloudLOD&    lod,
	                                        const ccGLCameraParameters& camera,
	                                        unsigned char               maxLevel);

	//! Sets custom clip planes for additional visibility constraints.
	void setClipPlanes(const ccClipPlaneSet& clipPlanes);
	//! Determines whether a node is intersected by the clipping planes.
	void clippingIntersection(ccAbstractPointCloudLOD::Node& node);
	//! Propagates a visibility flag to a node and its children.
	void propagateFlag(ccAbstractPointCloudLOD::Node& node, uint8_t flag);
	//! Flags the visibility status of a node based on frustum culling and clipping planes.
	virtual uint32_t flag(ccAbstractPointCloudLOD::Node& node);

  protected:
	ccAbstractPointCloudLOD&    m_lod;
	const ccGLCameraParameters& m_camera;
	Frustum                     m_frustum;
	unsigned char               m_maxLevel;
	ccClipPlaneSet              m_clipPlanes;
	bool                        m_hasClipPlanes;
};

//! A specialized visibility flagger that prioritizes nodes based on their projected screen footprint.
/*!
 * Extends the base LOD visibility flagger by introducing a score for nodes,
 * which prioritizes nodes based on their screen-space footprint (A la Potree)
 * Flagging differs as well, due to datastructure differences between CC LOD and NestedOctree LODs.
 */
class ccNestedOctreePointCloudLODVisibilityFlagger : public ccGenericPointCloudLODVisibilityFlagger
{
  public:
	ccNestedOctreePointCloudLODVisibilityFlagger(ccAbstractPointCloudLOD&    lod,
	                                             const ccGLCameraParameters& camera,
	                                             unsigned char               maxLevel,
	                                             float                       minPxFootprint);
	~ccNestedOctreePointCloudLODVisibilityFlagger() = default;

	//! Computes the projected screen-space footprint of a node.
	void computeNodeFootprint(ccAbstractPointCloudLOD::Node& node);
	//! Dedicated function for INSIDE flag propagation
	uint32_t propagateInsideFlag(ccAbstractPointCloudLOD::Node& node);
	//! override
	uint32_t flag(ccAbstractPointCloudLOD::Node& node) override;

  private:
	//! Rhreshold, below this value, the node is rendered considered "invisible".
	/* could be exposed to the properties tree */
	float m_minPxFootprint;
	//! Min level to render, even if the threshold requirement  is not met (see computeNodeFootprint)
	/* first two layers are very coarse, 1 is the default. Potree use a value of 2.
	It could be exposed to the propertiesTree too*/
	uint8_t m_minLevel;
};

//! The "original" CloudCompare LOD
class ccInternalPointCloudLOD : public ccAbstractPointCloudLOD
{
  public: // methods
	ccInternalPointCloudLOD();

	~ccInternalPointCloudLOD();

  public: // methods ccAbstractPointCloudLOD
	//! Initializes the construction process (asynchronous)
	bool init(ccPointCloud* cloud) override;

	void clear() override;

	LODIndexSet& getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel) override;

  public: // methods ccAbstractVBOManager
	void releaseVBOs(const ccGenericGLDisplay* currentDisplay) override
	{
		return;
	};

	bool updateVBOs(const ccPointCloud& pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override
	{
		return false;
	};

	bool renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override
	{
		return false;
	};

  protected: // methods
	         //! cleanData override
	void clearData() override;

	//! Return sthe flagger used by this LOD
	std::unique_ptr<ccGenericPointCloudLODVisibilityFlagger> getVisibilityFlagger(ccAbstractPointCloudLOD& lod, const ccGLCameraParameters& camera, unsigned char maxLevel) override
	{
		return std::make_unique<ccGenericPointCloudLODVisibilityFlagger>(lod, camera, maxLevel);
	}

	//! Reserves memory, used internally by the LOD construction thread
	bool initInternal(ccOctree::Shared octree);

	//! Adds a given number of points to the active index map (should be dispatched among the children cells)
	uint32_t addNPointsToIndexMap(Node& node, uint32_t count);

  protected: // members
	//! friend class
	friend ccPointCloudLODThread;

	//! Associated octree
	ccOctree::Shared m_octree;

	//! Computing thread
	ccPointCloudLODThread* m_thread;
};

//! The most common LOD datastructure (in the litterature and implementations)
/*!
This kind of structure is used by Potree and entwine (thus COPC, untwine...).
Each layer contains a subsambled version of the point cloud.
Cloud resolution increases as we go deeper into the octree levels.
It's additive, union of all points of all the cells (at all levels) = the point cloud

The creation of this kind of structure is not implemented in CC
but it should be easy in core in CC using CC octree and using a subsampling
strategy (either gridding or poisson sampling) bottom up.
Out of core creation would requiere more work in order to have efficiency
in computation and file I/O.

It assumes the point cloud is organized by chunks.
*/
class ccNestedOctreePointCloudLOD : public ccAbstractPointCloudLOD
{
  public: // methods
	      //! Default constucot
	ccNestedOctreePointCloudLOD() = default;

	//! Construct a LOD from externally defined lodLayers
	ccNestedOctreePointCloudLOD(const std::vector<ccAbstractPointCloudLOD::Level>& lodLayers);

	//! Default Destructor
	~ccNestedOctreePointCloudLOD() = default;

  public: // methods ccAbstractPointCloudLOD
	bool init(ccPointCloud* cloud) override;

	void clear() override;

	LODIndexSet& getIndexMap(unsigned char level, unsigned& maxCount, unsigned& remainingPointsAtThisLevel) override;

  public: // methods ccAbstractVBOManager
	void releaseVBOs(const ccGenericGLDisplay* currentDisplay) override;

	bool updateVBOs(const ccPointCloud& pc, const ccGenericGLDisplay* currentDisplay, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override;

	bool renderVBOs(const ccPointCloud& pc, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams) override;

  protected: // methods
	std::unique_ptr<ccGenericPointCloudLODVisibilityFlagger> getVisibilityFlagger(ccAbstractPointCloudLOD& lod, const ccGLCameraParameters& camera, unsigned char maxLevel) override
	{
		return std::make_unique<ccNestedOctreePointCloudLODVisibilityFlagger>(lod, camera, maxLevel, 75.0);
	}

  protected:
	template <class QOpenGLFunctions>
	bool renderVBOsRecursive(ccAbstractPointCloudLOD::Node& node, const CC_DRAW_CONTEXT& context, const glDrawParams& glParams, QOpenGLFunctions* glFunc);
};
