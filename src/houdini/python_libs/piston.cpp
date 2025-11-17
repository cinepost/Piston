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
#include "../../piston_lib/wrap_curves_deformer.h"
#include "../../piston_lib/curves_deformer_factory.h"
#include "../../piston_lib/deformer_stats.h"
#include "../../piston_lib/simple_profiler.h"
#include "../../piston_lib/tests.h"

#include <vector>


char const* greet() {
	return "Parovoz Piston python library!";
}

using BSON = std::vector<uint8_t>;

struct BSON_to_hex_string {
	static PyObject *convert(BSON const& bson) {
#ifdef _DEBUG
		bool truncate = false;
#else
		bool truncate = true;
#endif
		return boost::python::incref(boost::python::object(Piston::bson_to_hex_string(bson, truncate)).ptr());
	}
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BaseCurvesDeformer_deform_overloads, Piston::BaseCurvesDeformer::deform, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BaseCurvesDeformer_deform_dbg_overloads, Piston::BaseCurvesDeformer::deform_dbg, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BaseCurvesDeformer_writeJsonDataToPrim_overloads, Piston::BaseCurvesDeformer::writeJsonDataToPrim, 0, 1)


BOOST_PYTHON_MODULE(_piston) {
 	using namespace boost::python;
	using namespace Piston;

	class_<SimpleProfiler, boost::noncopyable>("Profiler",  no_init)
		.def("printReport", &SimpleProfiler::printReport)
		.staticmethod("printReport")
		.def("clear", &SimpleProfiler::clear)
		.staticmethod("clear")
	;

	class_<CurvesDeformerFactory, boost::noncopyable>("DeformerFactory",  no_init)
		.def("getInstance", &CurvesDeformerFactory::getInstance, return_value_policy<reference_existing_object>())
		.def("getFastDeformer", &CurvesDeformerFactory::getFastDeformer)
		.def("getWrapDeformer", &CurvesDeformerFactory::getWrapDeformer)
	;

	class_<BaseCurvesDeformer, BaseCurvesDeformer::SharedPtr, boost::noncopyable>("BaseCurvesDeformer",  no_init)
		.def("setMeshGeoPrim", &BaseCurvesDeformer::setMeshGeoPrim)
		.def("setCurvesGeoPrim", &BaseCurvesDeformer::setCurvesGeoPrim)
		.def("setMeshRestPositionAttrName", &BaseCurvesDeformer::setMeshRestPositionAttrName)
		.def("getMeshRestPositionAttrName", &BaseCurvesDeformer::getMeshRestPositionAttrName, return_value_policy<copy_const_reference>())
		.def("setСurvesSkinPrimAttrName", &BaseCurvesDeformer::setСurvesSkinPrimAttrName)
		.def("getСurvesSkinPrimAttrName", &BaseCurvesDeformer::getСurvesSkinPrimAttrName, return_value_policy<copy_const_reference>())
		.def("setVelocityAttrName", &BaseCurvesDeformer::setVelocityAttrName)
		.def("getVelocityAttrName", &BaseCurvesDeformer::getVelocityAttrName, return_value_policy<copy_const_reference>())

		.def("deform", &BaseCurvesDeformer::deform, BaseCurvesDeformer_deform_overloads(args("time_code")))
		.def("deform_dbg", &BaseCurvesDeformer::deform_dbg, BaseCurvesDeformer_deform_overloads(args("time_code")))

		.def("setReadJsonDataFromPrim", &BaseCurvesDeformer::setReadJsonDataFromPrim)
		.def("writeJsonDataToPrim", &BaseCurvesDeformer::writeJsonDataToPrim, BaseCurvesDeformer_writeJsonDataToPrim_overloads(args("time_code")))

		.def("toString", &BaseCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<FastCurvesDeformer, FastCurvesDeformer::SharedPtr, bases<BaseCurvesDeformer>, boost::noncopyable>("FastCurvesDeformer", no_init)
		.def("create", &FastCurvesDeformer::create)
		.staticmethod("create")
		.def("toString", &FastCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<WrapCurvesDeformer, WrapCurvesDeformer::SharedPtr, bases<BaseCurvesDeformer>, boost::noncopyable>("WrapCurvesDeformer", no_init)
		.def("create", &WrapCurvesDeformer::create)
		.staticmethod("create")
		.def("setBindMode", &WrapCurvesDeformer::setBindMode)
		.def("getBindMode", &WrapCurvesDeformer::getBindMode, return_value_policy<copy_const_reference>())
		.def("toString", &WrapCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	enum_<WrapCurvesDeformer::BindMode>("__BindMode")
    	.value("SPACE", WrapCurvesDeformer::BindMode::SPACE)
    	.value("DIST", WrapCurvesDeformer::BindMode::DIST)
    	.export_values()
  ;

	class_<DeformerStats, boost::noncopyable>("DeformerStats", no_init)
		.def("toString", &DeformerStats::toString)
	;

	to_python_converter<BSON , BSON_to_hex_string>();

	def("runTests", &Tests::runTests);	
	def("greet", greet);
}
