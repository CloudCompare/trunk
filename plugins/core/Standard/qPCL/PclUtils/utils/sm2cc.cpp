//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "sm2cc.h"

//Local
#include "PCLConv.h"
#include "my_point_types.h"

//PCL
#include <pcl/common/io.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//system
#include <list>

//PCL V1.6 or older
#ifdef PCL_VER_1_6_OR_OLDER

#include <sensor_msgs/PointField.h>
typedef sensor_msgs::PointField PCLScalarField;

#else //Version 1.7 or newer

#include <pcl/PCLPointField.h>
typedef pcl::PCLPointField PCLScalarField;

#endif

//system
#include <assert.h>

// Custom PCL point types
template<typename T> struct PointXYZTpl
{
  union EIGEN_ALIGN16
  {
    T data[3];
    struct
	{
      T x;
      T y;
      T z;
    };
  };
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTpl<std::int32_t>,
									(std::int32_t, x, x)
									(std::int32_t, y, y)
									(std::int32_t, z, z) )

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTpl<std::int16_t>,
									(std::int16_t, x, x)
									(std::int16_t, y, y)
									(std::int16_t, z, z) )

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTpl<double>,
									(double, x, x)
									(double, y, y)
									(double, z, z) )

static size_t GetNumberOfPoints(const PCLCloud& pclCloud)
{
	return static_cast<size_t>(pclCloud.width) * pclCloud.height;
}

static bool ExistField(const PCLCloud& pclCloud, std::string fieldName)
{
	for (size_t index = 0; index < pclCloud.fields.size(); ++index)
		if (pclCloud.fields[index].name == fieldName)
			return true;

	return false;
}

template<class T> void PCLCloudToCCCloud(	const PCLCloud& pclCloud,
											ccPointCloud& ccCloud,
											ccGLMatrixd* _transform = nullptr,
											FileIOFilter::LoadParameters* _loadParameters = nullptr )
{
	size_t pointCount = GetNumberOfPoints(pclCloud);

	pcl::PointCloud<T> pcl_cloud;
	FROM_PCL_CLOUD(pclCloud, pcl_cloud);

	CCVector3d Pshift(0, 0, 0);

	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3d P(	pcl_cloud.at(i).x,
						pcl_cloud.at(i).y,
						pcl_cloud.at(i).z );

		if (_transform)
		{
			P = (*_transform) * P;
		}

		if (_loadParameters)
		{
			if (i == 0) // first point
			{
				bool preserveCoordinateShift = true;
				if (FileIOFilter::HandleGlobalShift(P, Pshift, preserveCoordinateShift, *_loadParameters))
				{
					if (preserveCoordinateShift)
					{
						ccCloud.setGlobalShift(Pshift);
					}
					ccLog::Warning("[PCL-to-CC] PCL cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);

					if (_transform)
					{
						// we update the input transform so as to shift the points already 'at the source'
						_transform->setTranslation(_transform->getTranslationAsVec3D() + Pshift);

						// and the first point has to be shifted only this time
						P += Pshift;
					}
				}
			}

			if (nullptr == _transform)
			{
				// we have to apply the shift locally
				P += Pshift;
			}
		}

		ccCloud.addPoint(P.toPC());
	}
}

bool pcl2cc::CopyXYZ(	const PCLCloud& pclCloud,
						ccPointCloud& ccCloud,
						uint8_t coordinateType,
						ccGLMatrixd* _transform/*=nullptr*/,
						FileIOFilter::LoadParameters* _loadParameters/*=nullptr*/)
{
	size_t pointCount = GetNumberOfPoints(pclCloud);
	if (pointCount == 0)
	{
		assert(false);
		return false;
	}

	if (!ccCloud.reserve(static_cast<unsigned>(pointCount)))
	{
		return false;
	}

	//add xyz to the input cloud taking xyz infos from the sm cloud
	switch (coordinateType)
	{
		case pcl::PCLPointField::INT16:
			PCLCloudToCCCloud<PointXYZTpl<std::int16_t>>(pclCloud, ccCloud, _transform, _loadParameters);
			break;
		case pcl::PCLPointField::INT32:
			PCLCloudToCCCloud<PointXYZTpl<std::int32_t>>(pclCloud, ccCloud, _transform, _loadParameters);
			break;
		case pcl::PCLPointField::FLOAT32:
			PCLCloudToCCCloud<pcl::PointXYZ>(pclCloud, ccCloud, _transform, _loadParameters);
			break;
		case pcl::PCLPointField::FLOAT64:
			PCLCloudToCCCloud< PointXYZTpl<double>>(pclCloud, ccCloud, _transform, _loadParameters);
			break;
		default:
			ccLog::Warning("[PCL] Unsupported coordinate type " + QString::number(coordinateType));
			return false;
	};

	return true;
}

bool pcl2cc::CopyNormals(const PCLCloud& pclCloud, ccPointCloud& ccCloud)
{
	pcl::PointCloud<OnlyNormals>::Ptr pcl_cloud_normals (new pcl::PointCloud<OnlyNormals>);
	FROM_PCL_CLOUD(pclCloud, *pcl_cloud_normals);

	if (!ccCloud.reserveTheNormsTable())
		return false;

	size_t pointCount = GetNumberOfPoints(pclCloud);

	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 N(	static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_x),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_y),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_z) );

		ccCloud.addNorm(N);
	}

	ccCloud.showNormals(true);

	return true;
}

bool pcl2cc::CopyRGB(const PCLCloud& pclCloud, ccPointCloud& ccCloud)
{
	pcl::PointCloud<OnlyRGB>::Ptr pcl_cloud_rgb (new pcl::PointCloud<OnlyRGB>);
	FROM_PCL_CLOUD(pclCloud, *pcl_cloud_rgb);
	size_t pointCount = GetNumberOfPoints(pclCloud);
	if (pointCount == 0)
		return true;
	if (!ccCloud.reserveTheRGBTable())
		return false;


	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		ccColor::Rgb C(	static_cast<ColorCompType>(pcl_cloud_rgb->points[i].r),
						static_cast<ColorCompType>(pcl_cloud_rgb->points[i].g),
						static_cast<ColorCompType>(pcl_cloud_rgb->points[i].b) );
		ccCloud.addColor(C);
	}

	ccCloud.showColors(true);

	return true;
}

bool pcl2cc::CopyScalarField(	const PCLCloud& pclCloud,
								const std::string& sfName,
								ccPointCloud& ccCloud,
								bool overwriteIfExist/*=true*/)
{
	//if the input field already exists...
	int id = ccCloud.getScalarFieldIndexByName(sfName.c_str());
	if (id >= 0)
	{
		if (overwriteIfExist)
			//we simply delete it
			ccCloud.deleteScalarField(id);
		else
			//we keep it as is
			return false;
	}

	unsigned pointCount = ccCloud.size();

	//create new scalar field
	ccScalarField* newSF = new ccScalarField(sfName.c_str());
	if (!newSF->reserveSafe(pointCount))
	{
		newSF->release();
		return false;
	}

	//get PCL field
	int fieldIndex = pcl::getFieldIndex(pclCloud, sfName);
	if (fieldIndex < 0)
	{
		newSF->release();
		return false;
	}
	const PCLScalarField& pclField = pclCloud.fields[fieldIndex];
	//temporary change the name of the given field to something else -> S5c4laR should be a pretty uncommon name,
	const_cast<PCLScalarField&>(pclField).name = "S5c4laR";

	switch (pclField.datatype)
	{
	case PCLScalarField::FLOAT32:
	{
		pcl::PointCloud<FloatScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::FLOAT64:
	{
		pcl::PointCloud<DoubleScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT8:
	{
		pcl::PointCloud<Int8Scalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT8:
	{
		pcl::PointCloud<UInt8Scalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT16:
	{
		pcl::PointCloud<ShortScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT16:
	{
		pcl::PointCloud<UShortScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT32:
	{
		pcl::PointCloud<UIntScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT32:
	{
		pcl::PointCloud<IntScalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT64:
	{
		pcl::PointCloud<UInt64Scalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT64:
	{
		pcl::PointCloud<Int64Scalar> pclScalar;
		FROM_PCL_CLOUD(pclCloud, pclScalar);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pclScalar.points[i].S5c4laR);
			newSF->addElement(scalar);
		}
	}
	break;

	default:
		ccLog::Warning(QString("[PCL] Field with an unmanaged type (= %1)").arg(pclField.datatype));
		newSF->release();
		return false;
	}

	newSF->computeMinAndMax();
	int sfIdex = ccCloud.addScalarField(newSF);
	ccCloud.setCurrentDisplayedScalarField(sfIdex);
	ccCloud.showSF(true);

	//restore old name for the scalar field
	const_cast<PCLScalarField&>(pclField).name = sfName;

	return true;
}

ccPointCloud* pcl2cc::Convert(	const PCLCloud& pclCloud,
								ccGLMatrixd* _transform/*=nullptr*/,
								FileIOFilter::LoadParameters* _loadParameters/*=nullptr*/)
{
	//retrieve the valid fields
	std::list<std::string> fields;
	bool hasIntegerCoordinates = false;
	uint8_t coordinateType = 0;
	for (const auto& field : pclCloud.fields)
	{
		if (field.name != "_") //PCL padding fields
		{
			fields.push_back(field.name);
		}

		if (coordinateType == 0
			&&	(field.name == "x" || field.name == "y" || field.name == "z"))
		{
			coordinateType = field.datatype;
		}
	}

	//begin with checks and conversions
	//be sure we have x, y, and z fields
	if (!ExistField(pclCloud, "x") || !ExistField(pclCloud, "y") || !ExistField(pclCloud, "z"))
	{
		return nullptr;
	}

	//create cloud
	ccPointCloud* ccCloud = new ccPointCloud();
	size_t expectedPointCount = GetNumberOfPoints(pclCloud);
	if (expectedPointCount != 0)
	{
		//push points inside
		if (!CopyXYZ(pclCloud, *ccCloud, coordinateType, _transform, _loadParameters))
		{
			delete ccCloud;
			return nullptr;
		}
	}

	//remove x,y,z fields from the vector of field names
	fields.remove("x");
	fields.remove("y");
	fields.remove("z");

	//do we have normals?
	if (ExistField(pclCloud, "normal_x") || ExistField(pclCloud, "normal_y") || ExistField(pclCloud, "normal_z"))
	{
		CopyNormals(pclCloud, *ccCloud);

		//remove the corresponding fields
		fields.remove("normal_x");
		fields.remove("normal_y");
		fields.remove("normal_z");
	}

	//The same for colors
	if (ExistField(pclCloud, "rgb"))
	{
		CopyRGB(pclCloud, *ccCloud);

		//remove the corresponding field
		fields.remove("rgb");
	}
	//The same for colors
	else if (ExistField(pclCloud, "rgba"))
	{
		CopyRGB(pclCloud, *ccCloud);

		//remove the corresponding field
		fields.remove("rgba");
	}

	//All the remaining fields will be stored as scalar fields
	for (const std::string& name : fields)
	{
		CopyScalarField(pclCloud, name, *ccCloud);
	}

	return ccCloud;
}
