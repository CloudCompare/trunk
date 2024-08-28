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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_PLY_OPEN_DIALOG
#define CC_PLY_OPEN_DIALOG

// GUIs generated by Qt Designer
#include <ui_openPlyFileDlg.h>

// Qt
#include <QStringList>

// system
#include <vector>

class QComboBox;
struct PlyLoadingContext;

//! Dialog for configuration of PLY files opening sequence
class PlyOpenDlg : public QDialog
    , public Ui::PlyOpenDlg
{
	Q_OBJECT

  public:
	explicit PlyOpenDlg(QWidget* parent = nullptr);

	void setDefaultComboItems(const QStringList& stdPropsText);
	void setListComboItems(const QStringList& listPropsText);
	void setSingleComboItems(const QStringList& singlePropsText);

	//! Standard comboboxes
	std::vector<QComboBox*> m_standardCombos;
	//! List-related comboboxes (faces, etc.)
	std::vector<QComboBox*> m_listCombos;
	//! Single-related comboboxes (texture index, etc.)
	std::vector<QComboBox*> m_singleCombos;
	//! SF comboboxes
	std::vector<QComboBox*> m_sfCombos;

	//! Returns whether the current configuration is valid or not
	bool isValid(bool displayErrors = true) const;

	//! Restores the previously saved configuration (if any)
	/** \param hasAPreviousContext returns whether a previous context is stored
	    \return whether the previous context can be restored or not
	**/
	bool restorePreviousContext(bool& hasAPreviousContext);

	//! Resets the "apply all" flag (if set)
	static void ResetApplyAll();

	//! Returns whether the dialog can be 'skipped'
	/** i.e. 'Apply all' button has been previously clicked
	    and the configuration is valid.
	**/
	bool canBeSkipped() const;

  public:
	void addSFComboBox(int selectedIndex = 0);

  protected:
	void apply();
	void applyAll();

  Q_SIGNALS:
	void fullyAccepted();

  protected:
	//! Saves current configuration (for internal use)
	void saveContext(PlyLoadingContext* context);
	//! Restore a previously saved configuration (for internal use)
	bool restoreContext(PlyLoadingContext* context, int& unassignedProps, int& mismatchProps);

	//! Standard comboboxes elements
	QStringList m_stdPropsText;
	//! List-related comboboxes elements
	QStringList m_listPropsText;
	//! Single-related comboboxes elements
	QStringList m_singlePropsText;
};

#endif
