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

#ifndef CC_CLIP_BOX_HEADER
#define CC_CLIP_BOX_HEADER

//Local
#include "ccBBox.h"
#include "ccGenericPointCloud.h"
#include "ccHObject.h"
#include "ccInteractor.h"

//Qt
#include <QObject>

class ccClipBoxPart;

//! Clipping box
class QCC_DB_LIB_API ccClipBox : public QObject, public ccHObject, public ccInteractor
{
	Q_OBJECT

public:

	//! Default constructor
	/** \param name entity name (optional)
	    \param uniqueID unique ID (handle with care)
	**/
	ccClipBox(QString name = QString("Clipping box"), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Destructor
	~ccClipBox() override;

	//! Adds an associated entity
	/** Warning: resets the current clipping box
	**/
	bool addAssociatedEntity(ccHObject* associatedEntity);

	//! Releases all associated entities
	/** Warning: resets the current clipping box
	**/
	void releaseAssociatedEntities();

	//inherited from ccHObject
	ccBBox getOwnBB(bool withGLFeatures = false) override;

	//inherited from ccInteractor
	//bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;
	bool move3D(const CCVector3d& u) override;

	//! Sets last clicked point (on screen)
	void setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix);

	//! Components
	enum Components {	NONE = 0,
						X_MINUS_ARROW,
						X_PLUS_ARROW,
						Y_MINUS_ARROW,
						Y_PLUS_ARROW,
						Z_MINUS_ARROW,
						Z_PLUS_ARROW,
						CROSS,
						X_MINUS_TORUS,
						Y_MINUS_TORUS,
						Z_MINUS_TORUS,
						X_PLUS_TORUS,
						Y_PLUS_TORUS,
						Z_PLUS_TORUS
	};

	//! Sets currently active component
	/** \param id component ID (see Components)
	**/
	void setActiveComponent(Components id);

	//inherited from ccHObject
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CLIPPING_BOX; }

	//! Returns the box extents
	inline const ccBBox& getBox() const { return m_box; }

	//! Whether to show the box or not
	inline void showBox(bool state) { m_showBox = state; }

	//! Sets the box extents
	void setBox(const ccBBox& box);

	//! Shifts the current box
	void shift(const CCVector3& v);

	//! Flags the points of a given cloud depending on whether they are inside or outside of this clipping box
	/** \param cloud point cloud
		\param visTable visibility flags
		\param shrink Whether the box is shrinking (faster) or not
		\param inside If true, keep the point the box, if false, keep the point outside the box.
	**/
	void flagPoints(ccGenericPointCloud *cloud,
			ccGenericPointCloud::VisibilityTableType *visTable,
			bool shrink = false,
			bool inside = true) const;

	//! Resets box
	void reset();

	//! Manually sets the box parameters
	void set(const ccBBox& extents, const ccGLMatrix& transformation);

	//! Returns the box parameters
	void get(ccBBox& extents, ccGLMatrix& transformation);

	//! Associated entity container
	inline const ccHObject& getContainer() const { return m_entityContainer; }

Q_SIGNALS:

	//! Signal sent each time the box is modified
	void boxModified(const ccBBox* box);

protected: //methods

	//! Updates the associated entity clipping planes
	void update();

	//inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Computes arrows display scale
	PointCoordinateType computeArrowsScale() const;

protected: //members
	
	//! Associated entities container
	ccHObject m_entityContainer;

	//! Clipping box
	ccBBox m_box;

	//! Show box
	bool m_showBox;

	//! Active component
	Components m_activeComponent;

	//! Last "orientation" vector (corresponding to last clicked point)
	CCVector3d m_lastOrientation;

	//! View matrix
	ccGLMatrixd m_viewMatrix;

	//! Sub-parts (proxies)
	QMap<int, ccClipBoxPart*> m_parts;
};

//! Part of the clipping box (for picking)
class QCC_DB_LIB_API ccClipBoxPart : public ccHObject
{
public:

	//! Default constructor
	/** \param color picking color
		\param partID part ID
	**/
	ccClipBoxPart(ccClipBox* clipBox, ccClipBox::Components partID)
		: m_clipBox(clipBox)
		, m_partID(partID)
	{}

	//inherited from ccHObject
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CLIPPING_BOX_PART; }

	//! Returns the corresponding part ID
	inline ccClipBox::Components partID() const { return m_partID; }

	//! Returns the associated clipping box
	ccClipBox* clipBox() { return m_clipBox; }

protected:

	//! Associated clipping box
	ccClipBox* m_clipBox;
	//! Part number
	ccClipBox::Components m_partID;
};

#endif //CC_CLIP_BOX_HEADER
