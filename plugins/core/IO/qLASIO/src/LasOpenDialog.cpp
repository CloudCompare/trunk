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

#include "LasOpenDialog.h"

// Qt
#include <QFileDialog>
#include <QLocale>
#include <QSettings>
#include <QStringListModel>

// System
#include <algorithm>

constexpr int TILING_TAB_INDEX = 1;
constexpr int COPC_TAB_INDEX = 2;

static QListWidgetItem* CreateItem(const char* name, bool checked = true)
{
	auto item = new QListWidgetItem(name);
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
	return item;
}

static bool IsCheckedIn(const QString& name, const QListWidget* list)
{
	if (!list)
	{
		assert(false);
		return false;
	}

	for (int i = 0; i < list->count(); ++i)
	{
		if (list->item(i)->text() == name)
		{
			return list->item(i)->checkState() == Qt::Checked;
		}
	}
	return false;
}

// TODO use std::remove_if
template <typename T, typename Pred>
static void RemoveFalse(std::vector<T>& vec, Pred predicate)
{
	auto firstFalse = std::partition(vec.begin(), vec.end(), predicate);

	if (firstFalse != vec.end())
	{
		vec.erase(firstFalse, vec.end());
	}
}

LasOpenDialog::LasOpenDialog(QWidget* parent)
    : QDialog(parent)
{
	setupUi(this);

	connect(applyButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(applyAllButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
	connect(applyAllButton, &QPushButton::clicked, this, &LasOpenDialog::onApplyAll);
	connect(selectAllToolButton, &QPushButton::clicked, [&]
	        { doSelectAll(true); });
	connect(unselectAllToolButton, &QPushButton::clicked, this, [&]
	        { doSelectAll(false); });
	connect(tilingBrowseToolButton, &QPushButton::clicked, this, &LasOpenDialog::onBrowseTilingOutputDir);
	connect(actionTab, &QTabWidget::currentChanged, this, &LasOpenDialog::onCurrentTabChanged);
	connect(selectAllESFToolButton, &QPushButton::clicked, [&]
	        { doSelectAllESF(true); });
	connect(unselectAllESFToolButton, &QPushButton::clicked, this, [&]
	        { doSelectAllESF(false); });
	connect(xNormalComboBox,
	        (void(QComboBox::*)(const QString&))(&QComboBox::currentIndexChanged),
	        this,
	        &LasOpenDialog::onNormalComboBoxChanged);
	connect(yNormalComboBox,
	        (void(QComboBox::*)(const QString&))(&QComboBox::currentIndexChanged),
	        this,
	        &LasOpenDialog::onNormalComboBoxChanged);
	connect(zNormalComboBox,
	        (void(QComboBox::*)(const QString&))(&QComboBox::currentIndexChanged),
	        this,
	        &LasOpenDialog::onNormalComboBoxChanged);
	connect(decomposeClassificationCheckBox, &QCheckBox::toggled, this, &LasOpenDialog::onDecomposeClassificationToggled);

	{}
	const auto extentSpinBoxes = copcExtentGroupBox->findChildren<QDoubleSpinBox *>();
	for(auto extentSpinBox : extentSpinBoxes)
	{
		connect(extentSpinBox, (void(QDoubleSpinBox::*)(double))(&QDoubleSpinBox::valueChanged),  this, &LasOpenDialog::checkExtentConsistency);
	}
	// reload the last tiling output path
	{
		QSettings settings;
		settings.beginGroup("LasIO");
		QString tilingPath   = settings.value("TilingPath", QCoreApplication::applicationDirPath()).toString();
		int     tiling0Count = settings.value("Tiling0", 1).toInt();
		int     tiling1Count = settings.value("Tiling1", 1).toInt();
		int     tilingDim    = settings.value("TilingDim", 0).toInt();
		settings.endGroup();

		tilingDimensioncomboBox->setCurrentIndex(tilingDim);
		tilingSpinBox0->setValue(tiling0Count);
		tilingSpinBox1->setValue(tiling1Count);
		tilingOutputPathLineEdit->setText(tilingPath);
	}
}

void LasOpenDialog::doSelectAll(bool doSelect)
{
	if (!availableScalarFields)
	{
		assert(false);
		return;
	}

	for (int i = 0; i < availableScalarFields->count(); ++i)
	{
		availableScalarFields->item(i)->setCheckState(doSelect ? Qt::Checked : Qt::Unchecked);
	}
}

void LasOpenDialog::doSelectAllESF(bool doSelect)
{
	if (!availableExtraScalarFields)
	{
		assert(false);
		return;
	}

	for (int i = 0; i < availableExtraScalarFields->count(); ++i)
	{
		availableExtraScalarFields->item(i)->setCheckState(doSelect ? Qt::Checked : Qt::Unchecked);
	}
}

void LasOpenDialog::setInfo(int versionMinor, int pointFormatId, qulonglong numPoints)
{
	versionLabelValue->setText(QString("1.%1").arg(QString::number(versionMinor)));
	pointFormatLabelValue->setText(QString::number(pointFormatId));
	numPointsLabelValue->setText(QLocale(QLocale::English).toString(numPoints));

	force8bitColorsCheckBox->setEnabled(LasDetails::HasRGB(pointFormatId));

	decomposeClassificationCheckBox->setHidden(pointFormatId >= 6);
}

void LasOpenDialog::setAvailableScalarFields(const std::vector<LasScalarField>&      scalarFields,
                                             const std::vector<LasExtraScalarField>& extraScalarFields)
{
	// remember previous state
	QMap<QString, bool> previousScalarFields;
	{
		for (int i = 0; i < availableScalarFields->count(); ++i)
		{
			auto item                          = availableScalarFields->item(i);
			previousScalarFields[item->text()] = (item->checkState() == Qt::Checked);
		}
	}

	availableScalarFields->clear();
	availableExtraScalarFields->clear();

	if (!scalarFields.empty())
	{
		scalarFieldFrame->show();
		for (const LasScalarField& lasScalarField : scalarFields)
		{
			bool checked = true;
			if (previousScalarFields.contains(lasScalarField.name()))
			{
				checked = previousScalarFields[lasScalarField.name()];
			}
			availableScalarFields->addItem(CreateItem(lasScalarField.name(), checked));
		}
		decomposeClassificationFields(decomposeClassificationCheckBox->isChecked(), false);
	}
	else
	{
		scalarFieldFrame->hide();
	}

	if (!extraScalarFields.empty())
	{
		extraScalarFieldsFrame->show();
		QStringList availableExtraScalarFieldsName;
		availableExtraScalarFieldsName << QString(); // Use empty string to denote selecting none
		for (const LasExtraScalarField& lasExtraScalarField : extraScalarFields)
		{
			availableExtraScalarFields->addItem(CreateItem(lasExtraScalarField.name));
			availableExtraScalarFieldsName.append(lasExtraScalarField.name);
		}
		int height = availableExtraScalarFields->frameWidth() + (availableExtraScalarFields->sizeHintForRow(0) + availableExtraScalarFields->frameWidth()) * availableExtraScalarFields->count();
		availableExtraScalarFields->setMaximumHeight(height);

		auto* model = new QStringListModel;
		model->setStringList(availableExtraScalarFieldsName);
		xNormalComboBox->setModel(model);
		yNormalComboBox->setModel(model);
		zNormalComboBox->setModel(model);

		// Pre-select some normals if name matches
		for (const LasExtraScalarField& lasExtraScalarField : extraScalarFields)
		{
			if (strncmp(lasExtraScalarField.name, "NormalX", LasExtraScalarField::MAX_NAME_SIZE) == 0)
			{
				xNormalComboBox->setCurrentText("NormalX");
			}
			else if (strncmp(lasExtraScalarField.name, "NormalY", LasExtraScalarField::MAX_NAME_SIZE) == 0)
			{
				yNormalComboBox->setCurrentText("NormalY");
			}
			else if (strncmp(lasExtraScalarField.name, "NormalZ", LasExtraScalarField::MAX_NAME_SIZE) == 0)
			{
				zNormalComboBox->setCurrentText("NormalZ");
			}
		}
	}
	else
	{
		extraScalarFieldsFrame->hide();
	}
}

void LasOpenDialog::displayCopcTab(bool visibilityState)
{
	actionTab->setTabVisible(COPC_TAB_INDEX, visibilityState);
}

void LasOpenDialog::setCopcInformations(const std::vector<uint64_t>& pointCountPerLevel, const LasDetails::UnscaledExtent& copcBB)
{
	// reset previous combo box
	copcDepthComboBox->clear();
	uint64_t cumulative_point_count = 0;
	for (size_t level_id = 0; level_id < pointCountPerLevel.size(); ++level_id)
	{
		cumulative_point_count += pointCountPerLevel[level_id];
		auto label = QString("level %1 (%2 points)").arg(QString::number(level_id), QString::number(cumulative_point_count));
		copcDepthComboBox->insertItem(0, label, QVariant(static_cast<uint32_t>(level_id)));
	}
	copcDepthComboBox->setCurrentIndex(0);

	// set extent
	copcExtentSpinMinX->setValue(copcBB.minCorner().x);
	copcExtentSpinMinY->setValue(copcBB.minCorner().y);
	copcExtentSpinMinZ->setValue(copcBB.minCorner().z);
	copcExtentSpinMaxX->setValue(copcBB.maxCorner().x);
	copcExtentSpinMaxY->setValue(copcBB.maxCorner().y);
	copcExtentSpinMaxZ->setValue(copcBB.maxCorner().z);
}

LasDetails::UnscaledExtent LasOpenDialog::copcExtent() const
{
	LasDetails::UnscaledExtent extent;
	CCVector3d minCorner(copcExtentSpinMinX->value(), copcExtentSpinMinY->value(), copcExtentSpinMinZ->value());
	CCVector3d maxCorner(copcExtentSpinMaxX->value(), copcExtentSpinMaxY->value(), copcExtentSpinMaxZ->value());
	extent.add(minCorner);
	extent.add(maxCorner);
	return extent;
}

uint32_t LasOpenDialog::copcMaxLevel() const
{
	return copcDepthComboBox->currentData().toUInt();
}

bool LasOpenDialog::hasUsableExtent() const
{
	return copcExtentGroupBox->isChecked() && m_validExtent;
}

void LasOpenDialog::checkExtentConsistency(double value)
{
	m_validExtent = copcExtentSpinMaxX->value() - copcExtentSpinMinX->value() > 0 &&
	copcExtentSpinMaxY->value() - copcExtentSpinMinY->value() > 0 &&
	copcExtentSpinMaxZ->value() - copcExtentSpinMinZ->value() > 0;
	copcLabelWarningExtent->setVisible(!m_validExtent);
}

void LasOpenDialog::filterOutNotChecked(std::vector<LasScalarField>&      scalarFields,
                                        std::vector<LasExtraScalarField>& extraScalarFields)
{
	const auto isFieldSelected = [this](const auto& field)
	{ return isChecked(field); };

	RemoveFalse(scalarFields, isFieldSelected);
	RemoveFalse(extraScalarFields, isFieldSelected);
}

std::array<LasExtraScalarField, 3> LasOpenDialog::getExtraFieldsToBeLoadedAsNormals(const std::vector<LasExtraScalarField>& extraScalarFields) const
{
	std::array<LasExtraScalarField, 3> array;

	if (!extraScalarFields.empty())
	{
		const std::array<const QComboBox*, 3> boxes{xNormalComboBox, yNormalComboBox, zNormalComboBox};

		for (size_t i = 0; i < 3; ++i)
		{
			const QComboBox* comboBox = boxes[i];
			if (comboBox && comboBox->currentIndex() > 0)
			{
				const std::string name = comboBox->currentText().toStdString();
				const auto        it   = std::find_if(
                    extraScalarFields.begin(),
                    extraScalarFields.end(),
                    [&name](const LasExtraScalarField& e)
                    { return e.name == name; });

				// safeguard
				if (it != extraScalarFields.end())
				{
					array[i] = *it;
				}
				else
				{
					assert(false);
				}
			}
		}
	}

	return array;
}

bool LasOpenDialog::shouldIgnoreFieldsWithDefaultValues() const
{
	return ignoreFieldsWithDefaultValuesCheckBox->isChecked();
}

bool LasOpenDialog::shouldForce8bitColors() const
{
	return force8bitColorsCheckBox->isChecked();
}

bool LasOpenDialog::shouldDecomposeClassification() const
{
	return decomposeClassificationCheckBox->isChecked();
}

bool LasOpenDialog::isChecked(const LasExtraScalarField& lasExtraScalarField) const
{
	return IsCheckedIn(lasExtraScalarField.name, availableExtraScalarFields);
}

bool LasOpenDialog::isChecked(const LasScalarField& lasScalarField) const
{
	return IsCheckedIn(lasScalarField.name(), availableScalarFields);
}

bool LasOpenDialog::shouldSkipDialog() const
{
	return m_shouldSkipDialog;
}

void LasOpenDialog::resetShouldSkipDialog()
{
	m_shouldSkipDialog = false;
}

void LasOpenDialog::onApplyAll()
{
	m_shouldSkipDialog = true;
	accept();
}

void LasOpenDialog::onNormalComboBoxChanged(const QString& name)
{
	for (int i = 0; i < availableExtraScalarFields->count(); ++i)
	{
		QListWidgetItem* item = availableExtraScalarFields->item(i);
		if (item->text() == name)
		{
			item->setCheckState(Qt::CheckState::Unchecked);
			break;
		}
	}
}

LasOpenDialog::Action LasOpenDialog::action() const
{
	if (actionTab->currentIndex() == TILING_TAB_INDEX)
	{
		return Action::Tile;
	}
	else
	{
		return Action::Load;
	}
}

LasTilingOptions LasOpenDialog::tilingOptions() const
{
	int index = tilingDimensioncomboBox->currentIndex();
	if (index > 2)
	{
		index = 0;
	}

	const auto dimensions = static_cast<LasTilingDimensions>(index);

	// Do these maxs for safety, but the UI should not allow
	// users to enter values below 1
	int numTiles0 = std::max(tilingSpinBox0->value(), 1);
	int numTiles1 = std::max(tilingSpinBox1->value(), 1);

	// Save the parameters for later
	QSettings settings;
	settings.beginGroup("LasIO");
	settings.setValue("Tiling0", numTiles0);
	settings.setValue("Tiling1", numTiles1);
	settings.setValue("TilingDim", index);
	settings.endGroup();

	return LasTilingOptions{
	    tilingOutputPathLineEdit->text(),
	    dimensions,
	    static_cast<unsigned>(numTiles0),
	    static_cast<unsigned>(numTiles1),
	};
}

void LasOpenDialog::onBrowseTilingOutputDir()
{
	const QString outputDir = QFileDialog::getExistingDirectory(this, "Select output directory for tiles");
	if (outputDir.isEmpty())
	{
		return;
	}

	// update output path
	tilingOutputPathLineEdit->setText(outputDir);

	QSettings settings;
	settings.beginGroup("LasIO");
	settings.setValue("TilingPath", outputDir);
	settings.endGroup();
}

void LasOpenDialog::onCurrentTabChanged(int index)
{
	const static QString TILE_TEXT     = QStringLiteral("Tile");
	const static QString TILE_ALL_TEXT = QStringLiteral("Tile All");

	const static QString APPLY_TEXT     = QStringLiteral("Apply");
	const static QString APPLY_ALL_TEXT = QStringLiteral("Apply All");

	if (index == TILING_TAB_INDEX)
	{
		applyButton->setText(TILE_TEXT);
		applyAllButton->setText(TILE_ALL_TEXT);
	}
	else
	{
		applyButton->setText(APPLY_TEXT);
		applyAllButton->setText(APPLY_ALL_TEXT);
	}
}

void LasOpenDialog::decomposeClassificationFields(bool decompose, bool autoUpdateCheckSate)
{
	static constexpr std::array<const char*, 3> FlagNames{LasNames::SyntheticFlag, LasNames::KeypointFlag, LasNames::WithheldFlag};

	for (const char* flagName : FlagNames)
	{
		for (int i = 0; i < availableScalarFields->count(); ++i)
		{
			QListWidgetItem* item = availableScalarFields->item(i);
			if (item->text() == flagName)
			{
				item->setHidden(!decompose);
				if (autoUpdateCheckSate)
				{
					item->setCheckState(decompose ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
				}
				break;
			}
		}
	}
}

void LasOpenDialog::onDecomposeClassificationToggled(bool checked)
{
	decomposeClassificationFields(checked, true);
}
