//#include <boost/python.hpp> // Uncomment to use standalonce boost library

/*
 * Just in case we want to build against statndard boost library
 */
#ifndef BOOST_PYTHON_MODULE 
#include <hboost/python.hpp>
#include <hboost/python/overloads.hpp>
#define BOOST_PYTHON_MODULE HBOOST_PYTHON_MODULE
#define BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS HBOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS
namespace boost = hboost;
#endif // BOOST_PYTHON_MODULE

#include "../../piston_lib/base_curves_deformer.h"
#include "../../piston_lib/fast_curves_deformer.h"
#include "../../piston_lib/curves_deformer_factory.h"

char const* greet() {
	return "Parovoz Piston python library!";
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BaseCurvesDeformer_deform_overloads, Piston::BaseCurvesDeformer::deform, 0, 1)

BOOST_PYTHON_MODULE(_piston) {
  using namespace boost::python;
  using namespace Piston;

	class_<CurvesDeformerFactory, boost::noncopyable>("DeformerFactory",  no_init)
		.def("getInstance", &CurvesDeformerFactory::getInstance, return_value_policy<reference_existing_object>())
		.def("getFastDeformer", &CurvesDeformerFactory::getFastDeformer)
	;

	class_<BaseCurvesDeformer, BaseCurvesDeformer::SharedPtr, boost::noncopyable>("BaseCurvesDeformer",  no_init)
		.def("setMeshGeoPrim", &BaseCurvesDeformer::setMeshGeoPrim)
		.def("setCurvesGeoPrim", &BaseCurvesDeformer::setCurvesGeoPrim)
		.def("setMeshRestPositionAttrName", &BaseCurvesDeformer::setMeshRestPositionAttrName)
		.def("getMeshRestPositionAttrName", &BaseCurvesDeformer::getMeshRestPositionAttrName, return_value_policy<copy_const_reference>())
		.def("setСurvesSkinPrimAttrName", &BaseCurvesDeformer::setСurvesSkinPrimAttrName)
		.def("getСurvesSkinPrimAttrName", &BaseCurvesDeformer::getСurvesSkinPrimAttrName, return_value_policy<copy_const_reference>())
		.def("deform", &BaseCurvesDeformer::deform, BaseCurvesDeformer_deform_overloads(args("time_code")))
		.def("toString", &BaseCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<FastCurvesDeformer, FastCurvesDeformer::SharedPtr, bases<BaseCurvesDeformer>, boost::noncopyable>("FastCurvesDeformer", no_init)
		.def("create", &FastCurvesDeformer::create)
		.staticmethod("create")
		.def("toString", &FastCurvesDeformer::toString, return_value_policy<copy_const_reference>())
		.def("setDeformMode", &FastCurvesDeformer::setDeformMode)
		.def("getDeformMode", &FastCurvesDeformer::getDeformMode)
	;

	enum_<FastCurvesDeformer::DeformMode>("DeformMode")
    .value("FACET", FastCurvesDeformer::DeformMode::FACET)
    .value("SMOOTH", FastCurvesDeformer::DeformMode::SMOOTH)
    .value("ACCURATE", FastCurvesDeformer::DeformMode::ACCURATE)
  ;
	
	def("greet", greet);
}
