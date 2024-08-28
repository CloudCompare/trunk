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

#include "STLFilter.h"

// Qt
#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QStringList>
#include <QTextStream>

// qCC_db
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccMesh.h>
#include <ccNormalVectors.h>
#include <ccOctree.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

// System
#include <cstring>

STLFilter::STLFilter()
    : FileIOFilter({"_STL Filter",
                    10.0f, // priority
                    QStringList{"stl"},
                    "stl",
                    QStringList{"STL mesh (*.stl)"},
                    QStringList{"STL mesh (*.stl)"},
                    Import | Export})
{
}

bool STLFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::MESH)
	{
		multiple  = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR STLFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;

	if (!entity->isKindOf(CC_TYPES::MESH))
		return CC_FERR_BAD_ENTITY_TYPE;

	ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
	if (!mesh || mesh->size() == 0)
	{
		ccLog::Warning(QString("[STL] No facet in mesh '%1'!")
		                   .arg(mesh ? mesh->getName() : QStringLiteral("unnamed")));
		return CC_FERR_NO_ERROR;
	}

	// ask for output format
	bool binaryMode = true;
	if (parameters.alwaysDisplaySaveDialog)
	{
		QMessageBox  msgBox(QMessageBox::Question, "Choose output format", "Save in BINARY or ASCII format?");
		QPushButton* binaryButton = msgBox.addButton("BINARY", QMessageBox::AcceptRole);
		msgBox.addButton("ASCII", QMessageBox::AcceptRole);
		msgBox.exec();
		binaryMode = (msgBox.clickedButton() == binaryButton);
	}

	// try to open file for saving
	QFile theFile(filename);
	if (!theFile.open(QFile::WriteOnly))
	{
		return CC_FERR_WRITING;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	if (binaryMode)
	{
		result = saveToBINFile(mesh, theFile);
	}
	else // if (msgBox.clickedButton() == asciiButton)
	{
		result = saveToASCIIFile(mesh, theFile);
	}

	return result;
}

CC_FILE_ERROR STLFilter::saveToBINFile(ccGenericMesh* mesh, QFile& theFile, QWidget* parentWidget /*=nullptr*/)
{
	assert(theFile.isOpen() && mesh && mesh->size() != 0);
	unsigned faceCount = mesh->size();

	// progress
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parentWidget));
		pDlg->setMethodTitle(QObject::tr("Saving mesh [%1]").arg(mesh->getName()));
		pDlg->setInfo(QObject::tr("Number of facets: %1").arg(faceCount));
		pDlg->start();
		QApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nprogress(pDlg.data(), faceCount);

	// header
	{
		char header[80];
		memset(header, 0, 80);
		strcpy(header, "Binary STL file generated by CloudCompare!");
		if (theFile.write(header, 80) < 80)
			return CC_FERR_WRITING;
	}

	// UINT32 Number of triangles
	{
		uint32_t tmpInt32 = static_cast<uint32_t>(faceCount);
		if (theFile.write((const char*)&tmpInt32, 4) < 4)
			return CC_FERR_WRITING;
	}

	ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
	assert(vertices);

	// Can't save global shift information....
	if (vertices->isShifted())
	{
		ccLog::Warning("[STL] Global shift information can't be restored in STL Binary format! (too low precision)");
	}

	mesh->placeIteratorAtBeginning();
	for (unsigned i = 0; i < faceCount; ++i)
	{
		CCCoreLib::VerticesIndexes* tsi = mesh->getNextTriangleVertIndexes();

		const CCVector3* A = vertices->getPointPersistentPtr(tsi->i1);
		const CCVector3* B = vertices->getPointPersistentPtr(tsi->i2);
		const CCVector3* C = vertices->getPointPersistentPtr(tsi->i3);
		// compute face normal (right hand rule)
		CCVector3 N = (*B - *A).cross(*C - *A);

		// REAL32[3] Normal vector
		CCVector3f buffer = N.toFloat(); // convert to an explicit float array (as PointCoordinateType may be a double!)
		assert(sizeof(float) == 4);
		if (theFile.write((const char*)buffer.u, 12) < 12)
			return CC_FERR_WRITING;

		// REAL32[3] Vertex 1,2 & 3
		buffer = A->toFloat(); // convert to an explicit float array (as PointCoordinateType may be a double!)
		if (theFile.write((const char*)buffer.u, 12) < 12)
			return CC_FERR_WRITING;
		buffer = B->toFloat(); // convert to an explicit float array (as PointCoordinateType may be a double!)
		if (theFile.write((const char*)buffer.u, 12) < 12)
			return CC_FERR_WRITING;
		buffer = C->toFloat(); // convert to an explicit float array (as PointCoordinateType may be a double!)
		if (theFile.write((const char*)buffer.u, 12) < 12)
			return CC_FERR_WRITING;

		// UINT16 Attribute byte count (not used)
		{
			char byteCount[2] = {0, 0};
			if (theFile.write(byteCount, 2) < 2)
				return CC_FERR_WRITING;
		}

		// progress
		if (pDlg && !nprogress.oneStep())
		{
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	if (pDlg)
	{
		pDlg->stop();
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR STLFilter::saveToASCIIFile(ccGenericMesh* mesh, QFile& theFile, QWidget* parentWidget /*=nullptr*/)
{
	assert(theFile.isOpen() && mesh && mesh->size() != 0);
	unsigned faceCount = mesh->size();

	// progress
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parentWidget));
		pDlg->setMethodTitle(QObject::tr("Saving mesh [%1]").arg(mesh->getName()));
		pDlg->setInfo(QObject::tr("Number of facets: %1").arg(faceCount));
		pDlg->start();
		QApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nprogress(pDlg.data(), faceCount);

	QTextStream stream(&theFile);
	stream << "solid " << mesh->getName() << endl;
	if (theFile.error() != QFile::NoError) // empty names are acceptable!
	{
		return CC_FERR_WRITING;
	}

	// vertices
	ccGenericPointCloud* vertices = mesh->getAssociatedCloud();

	mesh->placeIteratorAtBeginning();
	for (unsigned i = 0; i < faceCount; ++i)
	{
		CCCoreLib::VerticesIndexes* tsi = mesh->getNextTriangleVertIndexes();

		const CCVector3* A = vertices->getPointPersistentPtr(tsi->i1);
		const CCVector3* B = vertices->getPointPersistentPtr(tsi->i2);
		const CCVector3* C = vertices->getPointPersistentPtr(tsi->i3);
		// compute face normal (right hand rule)
		CCVector3 N = (*B - *A).cross(*C - *A);

		//%e = scientific notation
		stream << "facet normal " << N.x << ' ' << N.y << ' ' << N.z << endl;
		stream << "outer loop" << endl;

		CCVector3d Aglobal = vertices->toGlobal3d<PointCoordinateType>(*A);
		stream << "vertex " << Aglobal.x << ' ' << Aglobal.y << ' ' << Aglobal.z << endl;
		CCVector3d Bglobal = vertices->toGlobal3d<PointCoordinateType>(*B);
		stream << "vertex " << Bglobal.x << ' ' << Bglobal.y << ' ' << Bglobal.z << endl;
		CCVector3d Cglobal = vertices->toGlobal3d<PointCoordinateType>(*C);
		stream << "vertex " << Cglobal.x << ' ' << Cglobal.y << ' ' << Cglobal.z << endl;
		stream << "endloop" << endl;
		stream << "endfacet" << endl;

		if (theFile.error() != QFile::NoError)
		{
			return CC_FERR_WRITING;
		}

		// progress
		if (pDlg && !nprogress.oneStep())
		{
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	stream << "endsolid " << mesh->getName() << endl;
	if (theFile.error() != QFile::NoError)
	{
		return CC_FERR_WRITING;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR STLFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	ccLog::Print(QString("[STL] Loading '%1'").arg(filename));

	// try to open the file
	QFile fp(filename);
	if (!fp.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	// ASCII OR BINARY?
	QString name("mesh");
	bool    ascii = true;
	{
		// buffer
		char   header[80] = {0};
		qint64 sz         = fp.read(header, 80);
		if (sz < 80)
		{
			// either ASCII or BINARY STL FILES are always > 80 bytes
			return sz == 0 ? CC_FERR_READING : CC_FERR_MALFORMED_FILE;
		}
		// normally, binary files shouldn't start by 'solid'
		if (!QString(header).trimmed().toUpper().startsWith("SOLID"))
		{
			ascii = false;
		}
		else //... but sadly some BINARY files does start by SOLID?!!!! (wtf)
		{
			// go back to the beginning of the file
			fp.seek(0);

			QTextStream stream(&fp);
			// skip first line
			stream.readLine();
			// we look if the second line (if any) starts by 'facet'
			QString line = stream.readLine();
			ascii        = true;
			if (line.isEmpty()
			    || fp.error() != QFile::NoError
			    || !QString(line).trimmed().toUpper().startsWith("FACET"))
			{
				ascii = false;
			}
		}
		// go back to the beginning of the file
		fp.seek(0);
	}
	ccLog::Print("[STL] Detected format: %s", ascii ? "ASCII" : "BINARY");

	// vertices
	ccPointCloud* vertices = new ccPointCloud("vertices");
	// mesh
	ccMesh* mesh = new ccMesh(vertices);
	mesh->setName(name);
	// add normals
	mesh->setTriNormsTable(new NormsIndexesTableType());

	CC_FILE_ERROR error = CC_FERR_NO_ERROR;
	if (ascii)
		error = loadASCIIFile(fp, mesh, vertices, parameters);
	else
		error = loadBinaryFile(fp, mesh, vertices, parameters);

	if (error != CC_FERR_NO_ERROR)
	{
		return CC_FERR_MALFORMED_FILE;
	}

	unsigned vertCount = vertices->size();
	unsigned faceCount = mesh->size();
	ccLog::Print("[STL] %i points, %i face(s)", vertCount, faceCount);

	// do some cleaning
	{
		vertices->shrinkToFit();
		mesh->shrinkToFit();
		NormsIndexesTableType* normals = mesh->getTriNormsTable();
		if (normals)
		{
			normals->shrink_to_fit();
		}
	}

	// remove duplicated vertices
	mesh->mergeDuplicatedVertices(ccMesh::DefaultMergeDuplicateVerticesLevel, parameters.parentWidget);
	vertices = nullptr; // warning, after this point, 'vertices' is not valid anymore

	ccGenericPointCloud* meshVertices = mesh->getAssociatedCloud();
	if (mesh->size() != 0 && meshVertices) // their might not remain anymore triangle after 'mergeDuplicatedVertices'
	{
		NormsIndexesTableType* normals = mesh->getTriNormsTable();
		if (normals)
		{
			// normals->link();
			// mesh->addChild(normals); //automatically done by setTriNormsTable
			mesh->showNormals(true);
		}
		else
		{
			// DGM: normals can be per-vertex or per-triangle so it's better to let the user do it himself later
			// Moreover it's not always good idea if the user doesn't want normals (especially in ccViewer!)
			// if (mesh->computeNormals())
			//	mesh->showNormals(true);
			// else
			//	ccLog::Warning("[STL] Failed to compute per-vertex normals...");
			ccLog::Warning("[STL] Mesh has no normal! You can manually compute them (select it then call \"Edit > Normals > Compute\")");
		}

		meshVertices->setEnabled(false);
		// meshVertices->setLocked(true); //DGM: no need to lock it as it is only used by one mesh!
		mesh->addChild(meshVertices);

		container.addChild(mesh);
	}
	else
	{
		delete mesh;
		mesh = nullptr;
		return CC_FERR_NO_LOAD;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR STLFilter::loadASCIIFile(QFile&          fp,
                                       ccMesh*         mesh,
                                       ccPointCloud*   vertices,
                                       LoadParameters& parameters)
{
	assert(fp.isOpen() && mesh && vertices);

	// text stream
	QTextStream stream(&fp);

	// 1st line: 'solid name'
	QString name("mesh");
	{
		QString currentLine = stream.readLine();
		if (currentLine.isEmpty() || fp.error() != QFile::NoError)
		{
			return CC_FERR_READING;
		}
		QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
		if (tokens.empty() || tokens[0].toUpper() != "SOLID")
		{
			ccLog::Warning("[STL] File should begin by 'solid [name]'!");
			return CC_FERR_MALFORMED_FILE;
		}
		// Extract name
		if (tokens.size() > 1)
		{
			tokens.removeAt(0);
			name = tokens.join(" ");
		}
	}
	mesh->setName(name);

	// progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("(ASCII) STL file"));
		pDlg->setInfo(QObject::tr("Loading in progress..."));
		pDlg->setRange(0, 0);
		pDlg->start();
		QApplication::processEvents();
	}

	// current vertex shift
	CCVector3d Pshift(0, 0, 0);

	unsigned               pointCount                    = 0;
	unsigned               faceCount                     = 0;
	static const unsigned  s_defaultMemAllocCount        = 65536;
	bool                   normalWarningAlreadyDisplayed = false;
	NormsIndexesTableType* normals                       = mesh->getTriNormsTable();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	unsigned lineCount = 1;
	while (true)
	{
		CCVector3 N;
		bool      normalIsOk = false;

		// 1st line of a 'facet': "facet normal ni nj nk" / or 'endsolid' (i.e. end of file)
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty())
			{
				break;
			}
			else if (fp.error() != QFile::NoError)
			{
				result = CC_FERR_READING;
				break;
			}
			++lineCount;

			QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
			if (tokens.empty() || tokens[0].toUpper() != "FACET")
			{
				if (tokens[0].toUpper() != "ENDSOLID")
				{
					ccLog::Warning("[STL] Error on line #%i: line should start by 'facet'!", lineCount);
					return CC_FERR_MALFORMED_FILE;
				}
				break;
			}

			if (normals && tokens.size() >= 5)
			{
				// let's try to read normal
				if (tokens[1].toUpper() == "NORMAL")
				{
					N.x = static_cast<PointCoordinateType>(tokens[2].toDouble(&normalIsOk));
					if (normalIsOk)
					{
						N.y = static_cast<PointCoordinateType>(tokens[3].toDouble(&normalIsOk));
						if (normalIsOk)
						{
							N.z = static_cast<PointCoordinateType>(tokens[4].toDouble(&normalIsOk));
						}
					}
					if (!normalIsOk && !normalWarningAlreadyDisplayed)
					{
						ccLog::Warning("[STL] Error on line #%i: failed to read 'normal' values!", lineCount);
						normalWarningAlreadyDisplayed = true;
					}
				}
				else if (!normalWarningAlreadyDisplayed)
				{
					ccLog::Warning("[STL] Error on line #%i: expecting 'normal' after 'facet'!", lineCount);
					normalWarningAlreadyDisplayed = true;
				}
			}
			else if (tokens.size() > 1 && !normalWarningAlreadyDisplayed)
			{
				ccLog::Warning("[STL] Error on line #%i: incomplete 'normal' description!", lineCount);
				normalWarningAlreadyDisplayed = true;
			}
		}

		// 2nd line: 'outer loop'
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty()
			    || fp.error() != QFile::NoError
			    || !QString(currentLine).trimmed().toUpper().startsWith("OUTER LOOP"))
			{
				ccLog::Warning("[STL] Error: expecting 'outer loop' on line #%i", lineCount + 1);
				result = CC_FERR_READING;
				break;
			}
			++lineCount;
		}

		// 3rd to 5th lines: 'vertex vix viy viz'
		unsigned vertIndexes[3]{0, 0, 0};
		// unsigned pointCountBefore = pointCount;
		for (unsigned i = 0; i < 3; ++i)
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty()
			    || fp.error() != QFile::NoError
			    || !QString(currentLine).trimmed().toUpper().startsWith("VERTEX"))
			{
				ccLog::Warning("[STL] Error: expecting a line starting by 'vertex' on line #%i", lineCount + 1);
				result = CC_FERR_MALFORMED_FILE;
				break;
			}
			++lineCount;

			QStringList tokens = QString(currentLine).simplified().split(QChar(' '), QString::SkipEmptyParts);
			if (tokens.size() < 4)
			{
				ccLog::Warning("[STL] Error on line #%i: incomplete 'vertex' description!", lineCount);
				result = CC_FERR_MALFORMED_FILE;
				break;
			}

			// read vertex
			CCVector3d Pd(0, 0, 0);
			{
				bool vertexIsOk = false;
				Pd.x            = tokens[1].toDouble(&vertexIsOk);
				if (vertexIsOk)
				{
					Pd.y = tokens[2].toDouble(&vertexIsOk);
					if (vertexIsOk)
						Pd.z = tokens[3].toDouble(&vertexIsOk);
				}
				if (!vertexIsOk)
				{
					ccLog::Warning("[STL] Error on line #%i: failed to read 'vertex' coordinates!", lineCount);
					result = CC_FERR_MALFORMED_FILE;
					break;
				}
			}

			// first point: check for 'big' coordinates
			if (pointCount == 0)
			{
				bool preserveCoordinateShift = true;
				if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
				{
					if (preserveCoordinateShift)
					{
						vertices->setGlobalShift(Pshift);
					}
					ccLog::Warning("[STLFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			CCVector3 P = (Pd + Pshift).toPC();

			// cloud is already full?
			if (vertices->capacity() == pointCount && !vertices->reserve(pointCount + s_defaultMemAllocCount))
			{
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}

			// insert new point
			vertIndexes[i] = pointCount++;
			vertices->addPoint(P);
		}

		if (CC_FERR_NO_ERROR != result)
		{
			break;
		}

		// we have successfully read the 3 vertices
		// let's add a new triangle
		{
			// mesh is full?
			if (mesh->capacity() == faceCount)
			{
				if (!mesh->reserve(faceCount + s_defaultMemAllocCount))
				{
					result = CC_FERR_NOT_ENOUGH_MEMORY;
					break;
				}

				if (normals)
				{
					bool success = normals->reserveSafe(mesh->capacity());
					if (success && faceCount == 0) // specific case: allocate per triangle normal indexes the first time!
					{
						success = mesh->reservePerTriangleNormalIndexes();
					}

					if (!success)
					{
						ccLog::Warning("[STL] Not enough memory: can't store normals!");
						mesh->removePerTriangleNormalIndexes();
						mesh->setTriNormsTable(nullptr);
						normals->release();
						normals = nullptr;
					}
				}
			}

			mesh->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
			++faceCount;
		}

		// and a new normal?
		if (normals)
		{
			int index = -1;
			if (normalIsOk)
			{
				// compress normal
				index                     = static_cast<int>(normals->currentSize());
				CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);
				normals->addElement(nIndex);
			}
			mesh->addTriangleNormalIndexes(index, index, index);
		}

		// 6th line: 'endloop'
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty()
			    || fp.error() != QFile::NoError
			    || !QString(currentLine).trimmed().toUpper().startsWith("ENDLOOP"))
			{
				ccLog::Warning("[STL] Error: expecting 'endnloop' on line #%i", lineCount + 1);
				result = CC_FERR_MALFORMED_FILE;
				break;
			}
			++lineCount;
		}

		// 7th and last line: 'endfacet'
		{
			QString currentLine = stream.readLine();
			if (currentLine.isEmpty()
			    || fp.error() != QFile::NoError
			    || !QString(currentLine).trimmed().toUpper().startsWith("ENDFACET"))
			{
				ccLog::Warning("[STL] Error: expecting 'endfacet' on line #%i", lineCount + 1);
				result = CC_FERR_MALFORMED_FILE;
				break;
			}
			++lineCount;
		}

		// progress
		if (pDlg && (faceCount % 1024) == 0)
		{
			if (pDlg->wasCanceled())
				break;
			pDlg->setValue(static_cast<int>(faceCount >> 10));
		}
	}

	if (normalWarningAlreadyDisplayed)
	{
		ccLog::Warning("[STL] Failed to read some 'normal' values!");
	}

	if (pDlg)
	{
		pDlg->close();
	}

	return result;
}

CC_FILE_ERROR STLFilter::loadBinaryFile(QFile&          fp,
                                        ccMesh*         mesh,
                                        ccPointCloud*   vertices,
                                        LoadParameters& parameters)
{
	assert(fp.isOpen() && mesh && vertices);

	unsigned pointCount = 0;
	unsigned faceCount  = 0;

	// UINT8[80] Header (we skip it)
	fp.seek(80);
	mesh->setName("Mesh"); // hard to guess solid name with binary files!

	// UINT32 Number of triangles
	{
		unsigned tmpInt32;
		if (fp.read((char*)&tmpInt32, 4) < 4)
			return CC_FERR_READING;
		faceCount = tmpInt32;
	}

	if (!mesh->reserve(faceCount))
		return CC_FERR_NOT_ENOUGH_MEMORY;
	if (!vertices->reserve(3 * faceCount))
		return CC_FERR_NOT_ENOUGH_MEMORY;
	NormsIndexesTableType* normals = mesh->getTriNormsTable();
	if (normals && (!normals->reserveSafe(faceCount) || !mesh->reservePerTriangleNormalIndexes()))
	{
		ccLog::Warning("[STL] Not enough memory: can't store normals!");
		mesh->removePerTriangleNormalIndexes();
		mesh->setTriNormsTable(nullptr);
	}

	// progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
		pDlg->setMethodTitle(QObject::tr("Loading binary STL file"));
		pDlg->setInfo(QObject::tr("Loading %1 faces").arg(faceCount));
		pDlg->start();
		QApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nProgress(pDlg.data(), faceCount);

	// current vertex shift
	CCVector3d Pshift(0, 0, 0);

	for (unsigned f = 0; f < faceCount; ++f)
	{
		// REAL32[3] Normal vector
		assert(sizeof(float) == 4);
		CCVector3 N;
		if (fp.read((char*)N.u, 12) < 12)
			return CC_FERR_READING;

		// 3 vertices
		unsigned vertIndexes[3];
		for (unsigned i = 0; i < 3; ++i)
		{
			// REAL32[3] Vertex 1,2 & 3
			float Pf[3];
			if (fp.read((char*)Pf, 12) < 0)
				return CC_FERR_READING;

			// first point: check for 'big' coordinates
			CCVector3d Pd(Pf[0], Pf[1], Pf[2]);
			if (pointCount == 0)
			{
				bool preserveCoordinateShift = true;
				if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
				{
					if (preserveCoordinateShift)
					{
						vertices->setGlobalShift(Pshift);
					}
					ccLog::Warning("[STLFilter::loadFile] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			CCVector3 P = (Pd + Pshift).toPC();

			// insert new point
			vertIndexes[i] = pointCount++;
			vertices->addPoint(P);
		}

		// UINT16 Attribute byte count (not used)
		{
			char a[2];
			if (fp.read(a, 2) < 0)
				return CC_FERR_READING;
		}

		// we have successfully read the 3 vertices
		// let's add a new triangle
		{
			mesh->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
		}

		// and a new normal?
		if (normals)
		{
			// compress normal
			int                index  = static_cast<int>(normals->currentSize());
			CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);
			normals->addElement(nIndex);
			mesh->addTriangleNormalIndexes(index, index, index);
		}

		// progress
		if (pDlg && !nProgress.oneStep())
		{
			break;
		}
	}

	if (pDlg)
	{
		pDlg->stop();
	}

	return CC_FERR_NO_ERROR;
}
