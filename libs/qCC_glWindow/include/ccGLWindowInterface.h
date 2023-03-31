#pragma once

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

//qCC_db
#include <ccGenericGLDisplay.h>
#include <ccGLUtils.h>
#include <ccBBox.h>

//qCC
#include "ccGuiParameters.h"

//Qt
#include <QElapsedTimer>
#include <QOpenGLExtensions>
#include <QOpenGLTexture>
#include <QTimer>

//system
#include <list>
#include <unordered_set>

class QOpenGLDebugMessage;
class QOpenGLBuffer;
class QOpenGLContext;
class QLayout;

class ccColorRampShader;
class ccFrameBufferObject;
class ccGlFilter;
class ccHObject;
class ccInteractor;
class ccPolyline;
class ccShader;

struct HotZone;

//! ccGLWindow Signal emitter
class CCGLWINDOW_LIB_API ccGLWindowSignalEmitter : public QObject
{
	Q_OBJECT

public:
	//! Default constructor
	ccGLWindowSignalEmitter(QObject* parent) : QObject(parent) {}

Q_SIGNALS:

	//! Signal emitted when an entity is selected in the 3D view
	void entitySelectionChanged(ccHObject* entity);
	//! Signal emitted when multiple entities are selected in the 3D view
	void entitiesSelectionChanged(std::unordered_set<int> entIDs);

	//! Signal emitted when a point (or a triangle) is picked
	/** \param entity 'picked' entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
		\param P the picked point
		\param uvw barycentric coordinates of the point (if picked on a mesh)
	**/
	void itemPicked(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P, const CCVector3d& uvw);

	//! Signal emitted when an item is picked (FAST_PICKING mode only)
	/** \param entity entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
	**/
	void itemPickedFast(ccHObject* entity, int subEntityID, int x, int y);

	//! Signal emitted when fast picking is finished (FAST_PICKING mode only)
	void fastPickingFinished();

	/*** Camera link mode (interactive modifications of the view/camera are echoed to other windows) ***/

	//! Signal emitted when the window 'model view' matrix is interactively changed
	void viewMatRotated(const ccGLMatrixd& rotMat);
	//! Signal emitted when the mouse wheel is rotated
	void mouseWheelRotated(float wheelDelta_deg);

	//! Signal emitted when the perspective state changes (see setPerspectiveState)
	void perspectiveStateChanged();

	//! Signal emitted when the window 'base view' matrix is changed
	void baseViewMatChanged(const ccGLMatrixd& newViewMat);

	//! Signal emitted when the f.o.v. changes
	void fovChanged(float fov);

	//! Signal emitted when the near clipping depth has been changed
	void nearClippingDepthChanged(double depth);

	//! Signal emitted when the far clipping depth has been changed
	void farClippingDepthChanged(double depth);

	//! Signal emitted when the pivot point is changed
	void pivotPointChanged(const CCVector3d&);

	//! Signal emitted when the camera position is changed
	void cameraPosChanged(const CCVector3d&);

	//! Signal emitted when the selected object is translated by the user
	void translation(const CCVector3d& t);

	//! Signal emitted when the selected object is rotated by the user
	/** \param rotMat rotation applied to current viewport (4x4 OpenGL matrix)
	**/
	void rotation(const ccGLMatrixd& rotMat);

	//! Signal emitted when the left mouse button is clicked on the window
	/** See INTERACT_SIG_LB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void leftButtonClicked(int x, int y);

	//! Signal emitted when the right mouse button is clicked on the window
	/** See INTERACT_SIG_RB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void rightButtonClicked(int x, int y);

	//! Signal emitted when the mouse is moved
	/** See INTERACT_SIG_MOUSE_MOVED.
		The two first arguments correspond to the current cursor coordinates (x,y)
		relative to the window corner!
	**/
	void mouseMoved(int x, int y, Qt::MouseButtons buttons);

	//! Signal emitted when a mouse button is released (cursor on the window)
	/** See INTERACT_SIG_BUTTON_RELEASED.
	**/
	void buttonReleased();

	//! Signal emitted during 3D pass of OpenGL display process
	/** Any object connected to this slot can draw additional stuff in 3D.
		Depth buffering, lights and shaders are enabled by default.
	**/
	void drawing3D();

	//! Signal emitted when files are dropped on the window
	void filesDropped(const QStringList& filenames);

	//! Signal emitted when a new label is created
	void newLabel(ccHObject* obj);

	//! Signal emitted when the exclusive fullscreen is toggled
	void exclusiveFullScreenToggled(bool exclusive);

	//! Signal emitted when the middle mouse button is clicked on the window
	/** See INTERACT_SIG_MB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void middleButtonClicked(int x, int y);
};

//! OpenGL 3D view interface
class CCGLWINDOW_LIB_API ccGLWindowInterface : public ccGenericGLDisplay
{
public:

	//! Picking mode
	enum PICKING_MODE { NO_PICKING,
						ENTITY_PICKING,
						ENTITY_RECT_PICKING,
						FAST_PICKING,
						POINT_PICKING,
						TRIANGLE_PICKING,
						POINT_OR_TRIANGLE_PICKING,
						POINT_OR_TRIANGLE_OR_LABEL_PICKING,
						LABEL_PICKING,
						DEFAULT_PICKING,
	};

	//! Interaction flags (mostly with the mouse)
	enum INTERACTION_FLAG
	{
		//no interaction
		INTERACT_NONE = 0,

		//camera interactions
		INTERACT_ROTATE          =  1,
		INTERACT_PAN             =  2,
		INTERACT_CTRL_PAN        =  4,
		INTERACT_ZOOM_CAMERA     =  8,
		INTERACT_2D_ITEMS        = 16, //labels, etc.
		INTERACT_CLICKABLE_ITEMS = 32, //hot zone

		//options / modifiers
		INTERACT_TRANSFORM_ENTITIES = 64,
		
		//signals
		INTERACT_SIG_RB_CLICKED      =  128, //right button clicked
		INTERACT_SIG_LB_CLICKED      =  256, //left button clicked
		INTERACT_SIG_MOUSE_MOVED     =  512, //mouse moved (only if a button is clicked)
		INTERACT_SIG_BUTTON_RELEASED = 1024, //mouse button released
		INTERACT_SIG_MB_CLICKED      = 2048, //middle button clicked
		INTERACT_SEND_ALL_SIGNALS    = INTERACT_SIG_RB_CLICKED | INTERACT_SIG_LB_CLICKED | INTERACT_SIG_MB_CLICKED | INTERACT_SIG_MOUSE_MOVED | INTERACT_SIG_BUTTON_RELEASED,

		// default modes
		MODE_PAN_ONLY = INTERACT_PAN | INTERACT_ZOOM_CAMERA | INTERACT_2D_ITEMS | INTERACT_CLICKABLE_ITEMS,
		MODE_TRANSFORM_CAMERA = INTERACT_ROTATE | MODE_PAN_ONLY,
		MODE_TRANSFORM_ENTITIES = INTERACT_ROTATE | INTERACT_PAN | INTERACT_ZOOM_CAMERA | INTERACT_TRANSFORM_ENTITIES | INTERACT_CLICKABLE_ITEMS,
	};

	Q_DECLARE_FLAGS(INTERACTION_FLAGS, INTERACTION_FLAG)

	//! Default message positions on screen
	enum MessagePosition {  LOWER_LEFT_MESSAGE,
							UPPER_CENTER_MESSAGE,
							SCREEN_CENTER_MESSAGE,
	};

	//! Message type
	enum MessageType {  CUSTOM_MESSAGE = 0,
						SCREEN_SIZE_MESSAGE,
						PERSPECTIVE_STATE_MESSAGE,
						SUN_LIGHT_STATE_MESSAGE,
						CUSTOM_LIGHT_STATE_MESSAGE,
						MANUAL_TRANSFORMATION_MESSAGE,
						MANUAL_SEGMENTATION_MESSAGE,
						ROTAION_LOCK_MESSAGE,
						FULL_SCREEN_MESSAGE,
	};

	//! Pivot symbol visibility
	enum PivotVisibility {  PIVOT_HIDE,
							PIVOT_SHOW_ON_MOVE,
							PIVOT_ALWAYS_SHOW,
	};

	//! Default constructor
	ccGLWindowInterface(QObject* parent = nullptr, bool silentInitialization = false);

	//! Destructor
	virtual ~ccGLWindowInterface();

	// Qt-equivalent shortcuts
	virtual qreal getDevicePixelRatio() const = 0;
	virtual QFont getFont() const = 0;
	virtual QOpenGLContext* getOpenGLContext() const = 0;
	virtual void setWindowCursor(const QCursor&) = 0;
	virtual void doMakeCurrent() = 0;
	virtual QObject* asQObject() = 0;
	virtual const QObject* asQObject() const = 0;
	virtual QString getWindowTitle() const = 0;
	virtual void doGrabMouse() = 0;
	virtual void doReleaseMouse() = 0;
	virtual QPoint doMapFromGlobal(const QPoint &) const = 0;
	virtual void doShowMaximized() = 0;
	virtual void doResize(int w, int h) = 0;
	virtual void doResize(const QSize &) = 0;

	//! Sets 'scene graph' root
	void setSceneDB(ccHObject* root);

	//! Returns current 'scene graph' root
	inline ccHObject* getSceneDB() { return m_globalDBRoot; }

	//replacement for the missing methods of QGLWidget
	void renderText(int x, int y, const QString & str, uint16_t uniqueID = 0, const QFont & font = QFont());
	void renderText(double x, double y, double z, const QString & str, const QFont & font = QFont());

	//inherited from ccGenericGLDisplay
	void toBeRefreshed() override;
	void invalidateViewport() override { m_validProjectionMatrix = false; }
	void deprecate3DLayer() override { m_updateFBO = true; }
	void display3DLabel(const QString& str, const CCVector3& pos3D, const ccColor::Rgba* color = nullptr, const QFont& font = QFont()) override;
	void displayText(QString text, int x, int y, unsigned char align = ALIGN_DEFAULT, float bkgAlpha = 0.0f, const ccColor::Rgba* color = nullptr, const QFont* font = nullptr) override;
	QFont getTextDisplayFont() const override; //takes rendering zoom into account!
	QFont getLabelDisplayFont() const override; //takes rendering zoom into account!
	const ccViewportParameters& getViewportParameters() const override { return m_viewportParams; }
	QPointF toCenteredGLCoordinates(int x, int y) const override;
	QPointF toCornerGLCoordinates(int x, int y) const override;
	void setupProjectiveViewport(const ccGLMatrixd& cameraMatrix, float fov_deg = 0.0f, float ar = 1.0f, bool viewerBasedPerspective = true, bool bubbleViewMode = false) override;
	void aboutToBeRemoved(ccDrawableObject* entity) override;
	void getGLCameraParameters(ccGLCameraParameters& params) override;

	//! Displays a status message in the bottom-left corner
	/** WARNING: currently, 'append' is not supported for SCREEN_CENTER_MESSAGE
		\param message message (if message is empty and append is 'false', all messages will be cleared)
		\param pos message position on screen
		\param append whether to append the message or to replace existing one(s) (only messages of the same type are impacted)
		\param displayMaxDelay_sec minimum display duration
		\param type message type (if not custom, only one message of this type at a time is accepted)
	**/
	void displayNewMessage(	const QString& message,
							MessagePosition pos,
							bool append=false,
							int displayMaxDelay_sec = 2,
							MessageType type = CUSTOM_MESSAGE);

	//! Activates sun light
	void setSunLight(bool state);
	//! Toggles sun light
	void toggleSunLight();
	//! Returns whether sun light is enabled or not
	inline bool sunLightEnabled() const { return m_sunLightEnabled; }
	//! Activates custom light
	void setCustomLight(bool state);
	//! Toggles custom light
	void toggleCustomLight();
	//! Returns whether custom light is enabled or not
	inline bool customLightEnabled() const { return m_customLightEnabled; }

	//! Sets pivot visibility
	void setPivotVisibility(PivotVisibility vis);

	//! Returns pivot visibility
	inline PivotVisibility getPivotVisibility() const { return m_pivotVisibility; }

	//! Shows or hide the pivot symbol
	/** Warnings:
		- not to be mistaken with setPivotVisibility
		- only taken into account if pivot visibility is set to PIVOT_SHOW_ON_MOVE
	**/
	void showPivotSymbol(bool state);

	//! Sets pivot point
	/** Emits the 'pivotPointChanged' signal.
	**/
	void setPivotPoint(	const CCVector3d& P,
						bool autoUpdateCameraPos = false,
						bool verbose = false);

	//! Sets camera position
	/** Emits the 'cameraPosChanged' signal.
	**/
	void setCameraPos(const CCVector3d& P);

	//! Displaces camera
	/** Values are given in objects world along the current camera
		viewing directions (we use the right hand rule):
		* X: horizontal axis (right)
		* Y: vertical axis (up)
		* Z: depth axis (pointing out of the screen)
		\param v displacement vector
	**/
	void moveCamera(CCVector3d& v);

	//! Sets the OpenGL camera 'focal' distance to achieve a given width (in 'metric' units)
	/** The camera will be positionned relatively to the current pivot point
	**/
	void setCameraFocalToFitWidth(double width);

	//! Sets the focal distance
	void setFocalDistance(double focalDistance);

	//! Set perspective state/mode
	/** Persepctive mode can be:
		- object-centered (moving the mouse make the object rotate)
		- viewer-centered (moving the mouse make the camera move)
		\warning Disables bubble-view mode automatically

		\param state whether perspective mode is enabled or not
		\param objectCenteredView whether view is object- or viewer-centered (forced to true in ortho. mode)
	**/
	void setPerspectiveState(bool state, bool objectCenteredView);

	//! Toggles perspective mode
	/** If perspective is activated, the user must specify if it should be
		object or viewer based (see setPerspectiveState)
	**/
	void togglePerspective(bool objectCentered);

	//! Returns perspective mode
	bool getPerspectiveState(bool& objectCentered) const;

	//! Shortcut: returns whether object-based perspective mode is enabled
	bool objectPerspectiveEnabled() const;
	//! Shortcut: returns whether viewer-based perspective mode is enabled
	bool viewerPerspectiveEnabled() const;

	//! Sets bubble-view mode state
	/** Bubble-view is a kind of viewer-based perspective mode where
		the user can't displace the camera (apart from up-down or
		left-right rotations). The f.o.v. is also maximized.

		\warning Any call to a method that changes the perpsective will
		automatically disable this mode.
	**/
	void setBubbleViewMode(bool state);
	
	//! Returns whether bubble-view mode is enabled or no
	inline bool bubbleViewModeEnabled() const { return m_bubbleViewModeEnabled; }

	//! Set bubble-view f.o.v. (in degrees)
	void setBubbleViewFov(float fov_deg);

	//! Center and zoom on a given bounding box
	/** If no bounding box is defined, the current displayed 'scene graph'
		bounding box is used.
		\param boundingBox bounding box to zoom on
	**/
	void updateConstellationCenterAndZoom(const ccBBox* boundingBox = nullptr);

	//! Returns the visible objects bounding-box
	void getVisibleObjectsBB(ccBBox& box) const;

	//! Rotates the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	void rotateBaseViewMat(const ccGLMatrixd& rotMat);

	//! Returns the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	const ccGLMatrixd& getBaseViewMat() { return m_viewportParams.viewMat; }
	
	//! Sets the base view matrix
	/** Warning: 'base view' marix is either:
		- the rotation around the object in object-centered mode
		- the rotation around the camera center in viewer-centered mode
		(see setPerspectiveState).
	**/
	void setBaseViewMat(ccGLMatrixd& mat);

	//! Sets camera to a predefined view (top, bottom, etc.)
	void setView(CC_VIEW_ORIENTATION orientation, bool redraw = true);

	//! Sets camera to a custom view (forward and up directions must be specified)
	void setCustomView(const CCVector3d& forward, const CCVector3d& up, bool forceRedraw = true);

	//! Sets current interaction flags
	virtual void setInteractionMode(INTERACTION_FLAGS flags) = 0;
	//! Returns the current interaction flags
	inline virtual INTERACTION_FLAGS getInteractionMode() const { return m_interactionFlags; }

	//! Sets current picking mode
	/** Picking can be applied to entities (default), points, triangles, etc.)
		\param mode					picking mode
		\param defaultCursorShape	default cursor shape for default, entity or no picking modes
	**/
	void setPickingMode(PICKING_MODE mode = DEFAULT_PICKING, Qt::CursorShape defaultCursorShape = Qt::ArrowCursor);

	//! Returns current picking mode
	inline virtual PICKING_MODE getPickingMode() const { return m_pickingMode; }

	//! Locks picking mode
	/** \warning Bes sure to unlock it at some point ;)
	**/
	inline virtual void lockPickingMode(bool state) { m_pickingModeLocked = state; }

	//! Returns whether picking mode is locked or not
	inline virtual bool isPickingModeLocked() const { return m_pickingModeLocked; }

	//! Specify whether this 3D window can be closed by the user or not
	inline void setUnclosable(bool state) { m_unclosable = state; }

	//! Returns context information
	void getContext(CC_DRAW_CONTEXT& context);

	//! Minimum point size
	static constexpr float MIN_POINT_SIZE_F = 1.0f;
	//! Maximum point size
	static constexpr float MAX_POINT_SIZE_F = 16.0f;

	//! Sets point size
	/** \param size point size (between MIN_POINT_SIZE_F and MAX_POINT_SIZE_F)
		\param silent whether this function can log and/or display messages on the screen
	**/
	void setPointSize(float size, bool silent = false);
	
	//! Minimum line width
	static constexpr float MIN_LINE_WIDTH_F = 1.0f;
	//! Maximum line width
	static constexpr float MAX_LINE_WIDTH_F = 16.0f;

	//! Sets line width
	/** \param width lines width (between MIN_LINE_WIDTH_F and MAX_LINE_WIDTH_F)
		\param silent whether this function can log and/or display messages on the screen
	**/
	void setLineWidth(float width, bool silent = false);

	//! Returns current font size
	int getFontPointSize() const;
	//! Returns current font size for labels
	int getLabelFontPointSize() const;

	//! Returns window own DB
	inline ccHObject* getOwnDB() { return m_winDBRoot; }
	//! Adds an entity to window own DB
	/** By default no dependency link is established between the entity and the window (DB).
	**/
	void addToOwnDB(ccHObject* obj, bool noDependency = true);
	//! Removes an entity from window own DB
	void removeFromOwnDB(ccHObject* obj);

	//! Sets viewport parameters (all at once)
	void setViewportParameters(const ccViewportParameters& params);

	//! Sets current camera f.o.v. (field of view) in degrees
	/** FOV is only used in perspective mode.
	**/
	void setFov(float fov);
	//! Returns the current f.o.v. (field of view) in degrees
	float getFov() const;

	//! Sets current OpenGL camera aspect ratio (width/height)
	void setGLCameraAspectRatio(float ar);

	//! Whether to allow near and far clipping planes or not
	inline void setClippingPlanesEnabled(bool enabled) { m_clippingPlanesEnabled = enabled; }

	//! Whether to near and far clipping planes are enabled or not
	inline bool clippingPlanesEnabled() const { return m_clippingPlanesEnabled; }

	//! Sets near clipping plane depth (or disable it if NaN)
	/** \return whether the plane depth was actually changed
	**/
	bool setNearClippingPlaneDepth(double depth);

	//! Sets far clipping plane depth (or disable it if NaN)
	/** \return whether the plane depth was actually changed
	**/
	bool setFarClippingPlaneDepth(double depth);

	//! Invalidate current visualization state
	/** Forces view matrix update and 3D/FBO display.
	**/
	inline void invalidateVisualization() { m_validModelviewMatrix = false; }

	//! Renders screen to an image
	virtual QImage renderToImage(	float zoomFactor = 1.0f,
									bool dontScaleFeatures = false,
									bool renderOverlayItems = false,
									bool silent = false) = 0;

	//! Renders screen to a file
	bool renderToFile(	QString filename,
						float zoomFactor = 1.0f,
						bool dontScaleFeatures = false,
						bool renderOverlayItems = false);

	static void SetShaderPath(const QString &path);
	static QString GetShaderPath();

	void setShader(ccShader* shader);
	void setGlFilter(ccGlFilter* filter);
	inline ccGlFilter* getGlFilter() { return m_activeGLFilter; }
	inline const ccGlFilter* getGlFilter() const { return m_activeGLFilter; }

	inline bool areShadersEnabled() const { return m_shadersEnabled; }
	inline bool areGLFiltersEnabled() const { return m_glFiltersEnabled; }

	//! Returns the actual pixel size on screen (taking zoom or perspective parameters into account)
	/** In perspective mode, this value is approximate (especially if we are in 'viewer-based' viewing mode).
	**/
	double computeActualPixelSize() const;

	//! Returns whether the ColorRamp shader is supported or not
	inline bool hasColorRampShader() const { return m_colorRampShader != nullptr; }

	//! Returns whether rectangular picking is allowed or not
	inline bool isRectangularPickingAllowed() const { return m_allowRectangularEntityPicking; }

	//! Sets whether rectangular picking is allowed or not
	inline void setRectangularPickingAllowed(bool state) { m_allowRectangularEntityPicking = state; }

	//! Returns current parameters for this display (const version)
	/** Warning: may return overridden parameters!
	**/
	const ccGui::ParamStruct& getDisplayParameters() const;

	//! Sets current parameters for this display
	void setDisplayParameters(const ccGui::ParamStruct& params, bool thisWindowOnly = false);

	//! Whether display parameters are overidden for this window
	inline bool hasOverriddenDisplayParameters() const { return m_overriddenDisplayParametersEnabled; }

	//! Default picking radius value
	static const int DefaultPickRadius = 5;

	//! Sets picking radius
	inline void setPickingRadius(int radius) { m_pickRadius = radius; }
	//! Returns the current picking radius
	inline int getPickingRadius() const { return m_pickRadius; }

	//! Sets whether overlay entities (scale and trihedron) should be displayed or not
	inline void displayOverlayEntities(bool showScale, bool showTrihedron) { m_showScale = showScale; m_showTrihedron = showTrihedron; }

	//! Returns whether the scale bar is displayed or not
	inline bool scaleIsDisplayed() const { return m_showScale; }

	//! Returns whether the trihedron is displayed or not
	inline bool trihedronIsDisplayed() const { return m_showTrihedron; }

	//! Computes the trihedron size (in pixels)
	float computeTrihedronLength() const;

	//! GL filter banner margin (height = 2*margin + current font height)
	static constexpr int CC_GL_FILTER_BANNER_MARGIN = 5;

	//! Returns the height of the 'GL filter' banner
	int getGlFilterBannerHeight() const;

	//! Returns the extents of the vertical area available for displaying the color ramp
	void computeColorRampAreaLimits(int& yStart, int& yStop) const;

	//! Backprojects a 2D points on a 3D triangle
	/** \warning Uses the current display parameters!
		\param P2D point on the screen
		\param A3D first vertex of the 3D triangle
		\param B3D second vertex of the 3D triangle
		\param C3D third vertex of the 3D triangle
		\return backprojected point
	**/
	CCVector3 backprojectPointOnTriangle(	const CCVector2i& P2D,
											const CCVector3& A3D,
											const CCVector3& B3D,
											const CCVector3& C3D );

	//! Returns unique ID
	inline int getUniqueID() const { return m_uniqueID; }

	//! Returns the widget width (in pixels)
	virtual int qtWidth() const = 0;
	//! Returns the widget height (in pixels)
	virtual int qtHeight() const = 0;
	//! Returns the widget size (in pixels)
	virtual QSize qtSize() const = 0;

	//! Returns the OpenGL context width
	inline int glWidth() const { return m_glViewport.width(); }
	//! Returns the OpenGL context height
	inline int glHeight() const { return m_glViewport.height(); }
	//! Returns the OpenGL context size
	inline QSize glSize() const { return m_glViewport.size(); }

public: //LOD

	//! Returns whether LOD is enabled on this display or not
	inline bool isLODEnabled() const { return m_LODEnabled; }

	//! Enables or disables LOD on this display
	/** \return success
	**/
	bool setLODEnabled(bool state, bool autoDisable = false);

public: //fullscreen

	//! Toggles (exclusive) full-screen mode
	virtual void toggleExclusiveFullScreen(bool state) = 0;

	//! Returns whether the window is in exclusive full screen mode or not
	inline bool exclusiveFullScreen() const { return m_exclusiveFullscreen; }

public: //debug traces on screen

	//! Shows debug info on screen
	inline void enableDebugTrace(bool state) { m_showDebugTraces = state; }

	//! Toggles debug info on screen
	inline void toggleDebugTrace() { m_showDebugTraces = !m_showDebugTraces; }

public: //stereo mode

	//! Seterovision parameters
	struct CCGLWINDOW_LIB_API StereoParams
	{
		StereoParams();

		//! Glass/HMD type
		enum GlassType {	RED_BLUE = 1,
							BLUE_RED = 2,
							RED_CYAN = 3,
							CYAN_RED = 4,
							NVIDIA_VISION = 5,
							OCULUS = 6,
							GENERIC_STEREO_DISPLAY = 7
		};

		//! Whether stereo-mode is 'analgyph' or real stereo mode
		inline bool isAnaglyph() const { return glassType <= 4; }

		int screenWidth_mm;
		int screenDistance_mm;
		int eyeSeparation_mm;
		int stereoStrength;
		GlassType glassType;
	};

	//! Enables stereo display mode
	virtual bool enableStereoMode(const StereoParams& params) = 0;

	//! Disables stereo display mode
	virtual void disableStereoMode() = 0;

	//! Returns whether the stereo display mode is enabled or not
	inline bool stereoModeIsEnabled() const { return m_stereoModeEnabled; }
	
	//! Returns the current stereo mode parameters
	inline const StereoParams& getStereoParams() const { return m_stereoParams; }

	//! Sets whether to display the coordinates of the point below the cursor position
	inline void showCursorCoordinates(bool state) { m_showCursorCoordinates = state; }
	//! Whether the coordinates of the point below the cursor position are displayed or not
	inline bool cursorCoordinatesShown() const { return m_showCursorCoordinates; }

	//! Toggles the automatic setting of the pivot point at the center of the screen
	void setAutoPickPivotAtCenter(bool state);
	//! Whether the pivot point is automatically set at the center of the screen
	inline bool autoPickPivotAtCenter() const { return m_autoPickPivotAtCenter; }

	//! Lock the rotation axis
	void lockRotationAxis(bool state, const CCVector3d& axis);

	//! Returns whether the rotation axis is locaked or not
	inline bool isRotationAxisLocked() const { return m_rotationAxisLocked; }

public:

	//! Applies a 1:1 global zoom
	void zoomGlobal();

	//called when receiving mouse wheel is rotated
	void onWheelEvent(float wheelDelta_deg);

	//! Tests frame rate
	virtual void startFrameRateTest() = 0;

	//! Request an update of the display
	/** The request will be executed if not in auto refresh mode already
	**/
	virtual void requestUpdate() = 0;
	
	//! Returns the signal emitter (const version)
	const ccGLWindowSignalEmitter* signalEmitter() const { return m_signalEmitter; }
	//! Returns the signal emitter
	ccGLWindowSignalEmitter* signalEmitter() { return m_signalEmitter; }

	static void Create(	ccGLWindowInterface*& window,
						QWidget*& widget,
						bool stereoMode = false,
						bool silentInitialization = false);

	static ccGLWindowInterface* FromWidget(QWidget* widget);

	static bool SupportStereo();

protected: //rendering

	//Default OpenGL functions set
	using ccQOpenGLFunctions = QOpenGLFunctions_2_1;

	//! Returns the set of OpenGL functions
	virtual ccQOpenGLFunctions* functions() const = 0;

	//On some versions of Qt, QGLWidget::renderText seems to need glColorf instead of glColorub!
	// See https://bugreports.qt-project.org/browse/QTBUG-6217
	template<class QOpenGLFunctions> static void glColor3ubv_safe(QOpenGLFunctions* glFunc, const ccColor::Rgb& color)
	{
		assert(glFunc);
		glFunc->glColor3f(	color.r / 255.0f,
							color.g / 255.0f,
							color.b / 255.0f);
	}

	template<class QOpenGLFunctions> static void glColor4ubv_safe(QOpenGLFunctions* glFunc, const ccColor::Rgba& color)
	{
		assert(glFunc);
		glFunc->glColor4f(	color.r / 255.0f,
							color.g / 255.0f,
							color.b / 255.0f,
							color.a / 255.0f);
	}

	//! Binds an FBO or releases the current one (if input is nullptr)
	/** This method must be called instead of the FBO's own 'start' and 'stop' methods
		so as to properly handle the interactions with QOpenGLWidget's own FBO.
	**/
	bool bindFBO(ccFrameBufferObject* fbo);

	//! LOD state
	struct CCGLWINDOW_LIB_API LODState
	{
		LODState()
			: inProgress(false)
			, level(0)
			, startIndex(0)
			, progressIndicator(0)
		{}

		//! LOD display in progress
		bool inProgress;
		//! Currently rendered LOD level
		unsigned char level;
		//! Currently rendered LOD start index
		unsigned startIndex;
		//! Currently LOD progress indicator
		unsigned progressIndicator;
	};

protected: //other methods

	//these methods are now protected to prevent issues with Retina or other high DPI displays
	//(see glWidth(), glHeight(), qtWidth(), qtHeight(), qtSize(), glSize()
	virtual int width() const = 0;
	virtual int height() const = 0;
	virtual QSize size() const = 0;

	//! Returns the current (OpenGL) view matrix
	/** Warning: may be different from the 'view' matrix returned by getBaseViewMat.
		Will call automatically updateModelViewMatrix if necessary.
	**/
	const ccGLMatrixd& getModelViewMatrix();
	//! Returns the current (OpenGL) projection matrix
	/** Will call automatically updateProjectionMatrix if necessary.
	**/
	const ccGLMatrixd& getProjectionMatrix();

	//! Processes the clickable items
	/** \return true if an item has been clicked
	**/
	bool processClickableItems(int x, int y);

	//! Sets current font size
	/** Warning: only used internally.
		Change 'defaultFontSize' with setDisplayParameters instead!
	**/
	void setFontPointSize(int pixelSize);

	virtual GLuint defaultQtFBO() const = 0;

	//! Computes the model view matrix
	ccGLMatrixd computeModelViewMatrix() const;

	//! Optional output metrics (from computeProjectionMatrix)
	struct ProjectionMetrics;

	//! Computes the projection matrix
	/** \param[in]  withGLfeatures whether to take additional elements (pivot symbol, custom light, etc.) into account or not
		\param[out] metrics [optional] output other metrics (Znear and Zfar, etc.)
		\param[out] eyeOffset [optional] eye offset (for stereo display)
	**/
	ccGLMatrixd computeProjectionMatrix(	bool withGLfeatures, 
											ProjectionMetrics* metrics = nullptr, 
											double* eyeOffset = nullptr) const;
	void updateModelViewMatrix();
	void updateProjectionMatrix();
	void setStandardOrthoCenter();
	void setStandardOrthoCorner();

	//Lights controls (OpenGL scripts)
	void glEnableSunLight();
	void glDisableSunLight();
	void glEnableCustomLight();
	void glDisableCustomLight();
	void drawCustomLight();

	//! Picking parameters
	struct PickingParameters
	{
		//! Default constructor
		PickingParameters(PICKING_MODE _mode = NO_PICKING,
			int _centerX = 0,
			int _centerY = 0,
			int _pickWidth = 5,
			int _pickHeight = 5,
			bool _pickInSceneDB = true,
			bool _pickInLocalDB = true)
			: mode(_mode)
			, centerX(_centerX)
			, centerY(_centerY)
			, pickWidth(_pickWidth)
			, pickHeight(_pickHeight)
			, pickInSceneDB(_pickInSceneDB)
			, pickInLocalDB(_pickInLocalDB)
		{}

		PICKING_MODE mode;
		int centerX;
		int centerY;
		int pickWidth;
		int pickHeight;
		bool pickInSceneDB;
		bool pickInLocalDB;
	};

	//! Starts picking process
	/** \param params picking parameters
	**/
	void startPicking(PickingParameters& params);

	//! Performs the picking with OpenGL (color-based)
	void startOpenGLPicking(const PickingParameters& params);

	//! Starts OpenGL picking process
	void startCPUBasedPointPicking(const PickingParameters& params);

	//! Processes the picking process result and sends the corresponding signal
	void processPickingResult(	const PickingParameters& params,
								ccHObject* pickedEntity,
								int pickedItemIndex,
								const CCVector3* nearestPoint = nullptr,
								const CCVector3d* nearestPointBC = nullptr, //barycentric coordinates
								const std::unordered_set<int>* selectedIDs = nullptr);
	
	//! Updates currently active items list (m_activeItems)
	/** The items must be currently displayed in this context
		AND at least one of them must be under the mouse cursor.
	**/
	void updateActiveItemsList(int x, int y, bool extendToSelectedLabels = false);

	//! Currently active items
	/** Active items can be moved with mouse, etc.
	**/
	std::unordered_set<ccInteractor*> m_activeItems;

	//! Inits FBO (frame buffer object)
	virtual bool initFBO(int w, int h) = 0;
	//! Inits FBO (safe)
	bool initFBOSafe(ccFrameBufferObject* &fbo, int w, int h);
	//! Releases any active FBO
	void removeFBO();
	//! Releases any active FBO (safe)
	void removeFBOSafe(ccFrameBufferObject* &fbo);

	//! Inits active GL filter (advanced shader)
	bool initGLFilter(int w, int h, bool silent = false);
	//! Releases active GL filter
	void removeGLFilter();

	//! Converts a given (mouse) position in pixels to an orientation
	/** The orientation vector origin is the current pivot point!
	**/
	CCVector3d convertMousePositionToOrientation(int x, int y);

	//! Draws the 'hot zone' (+/- icons for point size), 'leave bubble-view' button, etc.
	void drawClickableItems(int xStart, int& yStart);

	//Graphical features controls
	void drawCross();
	void drawTrihedron();
	void drawScale(const ccColor::Rgbub& color);

	//! Disables current LOD rendering cycle
	void stopLODCycle();

	//! Schedules a full redraw
	/** Any previously scheduled redraw will be cancelled.
		\warning The redraw will be cancelled if redraw/update is called before.
		\param maxDelay_ms the maximum delay for the call to redraw (in ms)
	**/
	void scheduleFullRedraw(unsigned maxDelay_ms);

	//! Cancels any scheduled redraw
	/** See ccGLWindowInterface::scheduleFullRedraw.
	**/
	void cancelScheduledRedraw();

	//! Logs a GL error
	/** Logs a warning or error message corresponding to the input GL error.
		If error == GL_NO_ERROR, nothing is logged.

		\param error GL error code
		\param context name of the method/object that catched the error
	**/
	static void LogGLError(GLenum error, const char* context);

	//! Logs a GL error (shortcut)
	void logGLError(const char* context) const;

	//! Toggles auto-refresh mode
	void toggleAutoRefresh(bool state, int period_ms = 0);

	//! Returns the (relative) depth value at a given pixel position
	/** \return the (relative) depth or 1.0 if none is defined
	**/
	GLfloat getGLDepth(int x, int y, bool extendToNeighbors = false, bool usePBO = false);

	//! Returns the approximate 3D position of the clicked pixel
	bool getClick3DPos(int x, int y, CCVector3d& P3D, bool usePBO);

	//! Computes the default increment (for moving the perspective camera, the cutting planes, etc.)
	double computeDefaultIncrement() const;

protected: //definitions

	// Reserved texture indexes
	enum class RenderTextReservedIDs
	{
		NotReserved = 0,
		FullScreenLabel,
		BubbleViewLabel,
		PointSizeLabel,
		LineSizeLabel,
		GLFilterLabel,
		ScaleLabel,
		trihedronX,
		trihedronY,
		trihedronZ,
		StandardMessagePrefix = 1024
	};

	//! Percentage of the smallest screen dimension
	static constexpr double CC_DISPLAYED_PIVOT_RADIUS_PERCENT = 0.8;

protected: //members

	//! Unique ID
	int m_uniqueID;

	//! Initialization state
	bool m_initialized;

	//! Trihedron GL list
	GLuint m_trihedronGLList;

	//! Pivot center GL list
	GLuint m_pivotGLList;

	//! Viewport parameters (zoom, etc.)
	ccViewportParameters m_viewportParams;

    //! Last mouse position
	QPoint m_lastMousePos;

	//! Complete visualization matrix (GL style - double version)
	ccGLMatrixd m_viewMatd;
	//! Whether the model veiw matrix is valid (or need to be recomputed)
	bool m_validModelviewMatrix;
	//! Projection matrix (GL style - double version)
	ccGLMatrixd m_projMatd;
	//! Whether the projection matrix is valid (or need to be recomputed)
	bool m_validProjectionMatrix;
	//! Bounding-box of the currently visible objects (updated by computeProjectionMatrix)
	ccBBox m_visibleObjectsBBox;

	//! GL viewport
	QRect m_glViewport;

	//! Whether L.O.D. (level of detail) is enabled or not
	bool m_LODEnabled;
	//! Whether L.O.D. should be automatically disabled at the end of the rendering cycle
	bool m_LODAutoDisable;
	//! Whether the display should be refreshed on next call to 'refresh'
	bool m_shouldBeRefreshed;
	//! Whether the mouse (cursor) has moved after being pressed or not
	bool m_mouseMoved;
	//! Whether the mouse is currently pressed or not
	bool m_mouseButtonPressed;
	//! Whether this 3D window can be closed by the user or not
	bool m_unclosable;
	//! Current intercation flags
	INTERACTION_FLAGS m_interactionFlags;
	//! Current picking mode
	PICKING_MODE m_pickingMode;
	//! Whether picking mode is locked or not
	bool m_pickingModeLocked;

	//! Display capturing mode options
	struct CCGLWINDOW_LIB_API CaptureModeOptions
	{
		//! Default constructor
		CaptureModeOptions()
			: enabled(false)
			, zoomFactor(1.0f)
			, renderOverlayItems(false)
		{}

		bool enabled;
		float zoomFactor;
		bool renderOverlayItems;
	};

	//! Display capturing mode options
	CaptureModeOptions m_captureMode;

	//! Temporary Message to display in the lower-left corner
	struct CCGLWINDOW_LIB_API MessageToDisplay
	{
		MessageToDisplay()
			: messageValidity_sec(0)
			, position(LOWER_LEFT_MESSAGE)
			, type(CUSTOM_MESSAGE)
		{}
		
		//! Message
		QString message;
		//! Message end time (sec)
		qint64 messageValidity_sec;
		//! Message position on screen
		MessagePosition position;
		//! Message type
		MessageType type;
	};

	//! List of messages to display
	std::list<MessageToDisplay> m_messagesToDisplay;

	//! Last click time (msec)
	qint64 m_lastClickTime_ticks;

	//! Sun light position
	/** Relative to screen.
	**/
	float m_sunLightPos[4];
	//! Whether sun light is enabled or not
	bool m_sunLightEnabled;
	
	//! Custom light position
	/** Relative to object.
	**/
	float m_customLightPos[4];
	//! Whether custom light is enabled or not
	bool m_customLightEnabled;

	//! Clickable item
	struct CCGLWINDOW_LIB_API ClickableItem
	{
		enum Role {	NO_ROLE,
					INCREASE_POINT_SIZE,
					DECREASE_POINT_SIZE,
					INCREASE_LINE_WIDTH,
					DECREASE_LINE_WIDTH,
					LEAVE_BUBBLE_VIEW_MODE,
					LEAVE_FULLSCREEN_MODE,
		};

		ClickableItem(): role(NO_ROLE) {}
		ClickableItem(Role _role, QRect _area) : role(_role), area(_area) {}

		Role role;
		QRect area;
	};
	
	//! Currently displayed clickable items
	std::vector<ClickableItem> m_clickableItems;

	//! Whether clickable items are visible (= mouse over) or not
	bool m_clickableItemsVisible;

	//! Currently active shader
	ccShader* m_activeShader;
	//! Whether shaders are enabled or not
	bool m_shadersEnabled;

	//! Currently active FBO (frame buffer object)
	ccFrameBufferObject* m_activeFbo;
	//! First default FBO (frame buffer object)
	ccFrameBufferObject* m_fbo;
	//! Second default FBO (frame buffer object) - used for stereo rendering
	ccFrameBufferObject* m_fbo2;
	//! Picking FBO (frame buffer object)
	ccFrameBufferObject* m_pickingFbo;
	//! Whether to always use FBO or only for GL filters
	bool m_alwaysUseFBO;
	//! Whether FBO should be updated (or simply displayed as a texture = faster!)
	bool m_updateFBO;

	// Color ramp shader
	ccColorRampShader* m_colorRampShader;
	// Custom rendering shader (OpenGL 3.3+)
	ccShader* m_customRenderingShader;

	//! Active GL filter
	ccGlFilter* m_activeGLFilter;
	//! Whether GL filters are enabled or not
	bool m_glFiltersEnabled;

	//! Window own DB
	ccHObject* m_winDBRoot;

	//! CC main DB
	ccHObject* m_globalDBRoot;

	//! Default font
	QFont m_font;

	//! Pivot symbol visibility
	PivotVisibility m_pivotVisibility;

	//! Whether pivot symbol should be shown or not
	bool m_pivotSymbolShown;

	//! Whether rectangular picking is allowed or not
	bool m_allowRectangularEntityPicking;

	//! Rectangular picking polyline
	ccPolyline* m_rectPickingPoly;

	//! Overridden display parameter 
	ccGui::ParamStruct m_overriddenDisplayParameters;

	//! Whether display parameters are overidden for this window
	bool m_overriddenDisplayParametersEnabled;

	//! Whether to display the scale bar
	bool m_showScale;

	//! Whether to display the trihedron
	bool m_showTrihedron;

	//! Whether initialization should be silent or not (i.e. no message to console)
	bool m_silentInitialization;

	//! Bubble-view mode state
	bool m_bubbleViewModeEnabled;
	//! Bubble-view mode f.o.v. (degrees)
	float m_bubbleViewFov_deg;
	//! Pre-bubble-view camera parameters (backup)
	ccViewportParameters m_preBubbleViewParameters;

	//! Current LOD state
	LODState m_currentLODState;

	//! LOD refresh signal sent
	bool m_LODPendingRefresh;
	//! LOD refresh signal should be ignored
	bool m_LODPendingIgnore;

	//! Internal timer
	QElapsedTimer m_timer;

	//! Touch event in progress
	bool m_touchInProgress;
	//! Touch gesture initial distance
	qreal m_touchBaseDist;

	//! Scheduler timer
	QTimer m_scheduleTimer;
	//! Scheduled full redraw (no LOD)
	qint64 m_scheduledFullRedrawTime;

	//! Seterovision mode parameters
	StereoParams m_stereoParams;

	//! Whether seterovision mode is enabled or not
	bool m_stereoModeEnabled;

	//! Former parent object (for exclusive full-screen display)
	QWidget* m_formerParent;

	//! Wether exclusive full screen is enabled or not
	bool m_exclusiveFullscreen;
	
	//! Former geometry (for exclusive full-screen display)
	QByteArray m_formerGeometry;

	//! Debug traces visibility
	bool m_showDebugTraces;

	//! Picking radius (pixels)
	int m_pickRadius;

	//! FBO support
	QOpenGLExtension_ARB_framebuffer_object	m_glExtFunc;

	//! Whether FBO support is on
	bool m_glExtFuncSupported;

	//! Auto-refresh mode
	bool m_autoRefresh;
	//! Auto-refresh timer
	QTimer m_autoRefreshTimer;

	//! Precomputed stuff for the 'hot zone'
	struct HotZone
	{
		//display font
		QFont font;
		//text height
		int textHeight;
		//text shift
		int yTextBottomLineShift;
		//default color
		ccColor::Rgb color;

		//bubble-view label rect.
		QString bbv_label;
		//bubble-view label rect.
		QRect bbv_labelRect;
		//bubble-view row width
		int bbv_totalWidth;

		//fullscreen label rect.
		QString fs_label;
		//fullscreen label rect.
		QRect fs_labelRect;
		//fullscreen row width
		int fs_totalWidth;

		//point size label
		QString psi_label;
		//point size label rect.
		QRect psi_labelRect;
		//point size row width
		int psi_totalWidth;

		//line size label
		QString lsi_label;
		//line size label rect.
		QRect lsi_labelRect;
		//line size row width
		int lsi_totalWidth;

		int margin;
		int iconSize;
		QPoint topCorner;

		explicit HotZone(ccGLWindowInterface* win);

		QRect rect(bool clickableItemsVisible, bool bubbleViewModeEnabled, bool fullScreenEnabled) const;
	};

	//! Hot zone
	HotZone* m_hotZone;

	//! Whether to display the coordinates of the point below the cursor position
	bool m_showCursorCoordinates;

	//! Whether the pivot point is automatically picked at the center of the screen (when possible)
	bool m_autoPickPivotAtCenter;

	//! Deferred picking
	QTimer m_deferredPickingTimer;

	//! Ignore next mouse release event
	bool m_ignoreMouseReleaseEvent;

	//! Wheter the rotation axis is locked or not
	bool m_rotationAxisLocked;
	//! Locked rotation axis
	CCVector3d m_lockedRotationAxis;

	//! Shared texture type
	using SharedTexture = QSharedPointer< QOpenGLTexture>;

	//! Reserved textures (for renderText)
	QMap<uint16_t, SharedTexture> m_uniqueTextures;

	//! Texture pool (for renderText)
	std::vector<SharedTexture> m_texturePool;

	//! Last texture pool index
	size_t m_texturePoolLastIndex;

	//! Fast pixel reading mechanism with PBO
	struct PBOPicking
	{
		//! Whether the picking PBO seems supported or not
		bool supported = true;

		//! PBO object
		QOpenGLBuffer* glBuffer = nullptr;

		//! Last read operation timestamp
		qint64 lastReadTime_ms = 0;

		//! Elapsed timer
		QElapsedTimer timer;

		bool init();
		void release();
	};

	//! Fast pixel reading mechanism with PBO
	PBOPicking m_pickingPBO;

	//! Whether to near and far clipping planes are enabled or not
	bool m_clippingPlanesEnabled;

	//! Default mouse cursor
	Qt::CursorShape m_defaultCursorShape;

	//! Signal emitter
	ccGLWindowSignalEmitter* m_signalEmitter;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ccGLWindowInterface::INTERACTION_FLAGS);
