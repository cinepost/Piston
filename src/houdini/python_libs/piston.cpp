//#include <boost/python.hpp> // Uncomment to use standalonce boost library

/*
 * Just in case we want to build against statndard boost library
 */
#ifndef BOOST_PYTHON_MODULE 
#include <hboost/python.hpp>
#include <hboost/python/overloads.hpp>
#include <hboost/python/suite/indexing/map_indexing_suite.hpp>
#define BOOST_PYTHON_MODULE HBOOST_PYTHON_MODULE
#define BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS HBOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS
namespace boost = hboost;
#endif // BOOST_PYTHON_MODULE

#include "../../piston_lib/common.h"
#include "../../piston_lib/base_curves_deformer.h"
#include "../../piston_lib/base_mesh_curves_deformer.h"
#include "../../piston_lib/fast_curves_deformer.h"
#include "../../piston_lib/wrap_curves_deformer.h"
#include "../../piston_lib/guide_curves_deformer.h"
#include "../../piston_lib/deformer_factory.h"
#include "../../piston_lib/logging.h"
#include "../../piston_lib/deformer_stats.h"
#include "../../piston_lib/simple_profiler.h"
#include "../../piston_lib/tests.h"

#include <pxr/base/tf/pyPtrHelpers.h>

#include <vector>


static void _setLogLevel(Piston::LogLevel level) {
	Piston::Logger::getInstance().setLogLevel(level);
}

static void _clearPistonDataFromStage(pxr::UsdStageRefPtr pStage) {
	if(Piston::clearPistonDataFromStage(pStage)) {
		Piston::Logger::getInstance().getStream(Piston::LogLevel::INFO) << "Piston data deleted from stage " << Piston::getStageName(pStage);
	}
}

static void _clearPistonDataFromPrim(pxr::UsdStageRefPtr pStage, const pxr::SdfPath& prim_path) {
	if(Piston::clearPistonDataFromPrim(pStage, prim_path)) {
		Piston::Logger::getInstance().getStream(Piston::LogLevel::INFO) << "Piston data deleted from prim " << prim_path;
	}
}

struct BSON_to_Python {
	static PyObject *convert(const Piston::BSON& bson) {
		dbg_printf("BSON_to_Python::convert()\n");
		static const std::string sTestString = "Encoded bson data.";
		return boost::python::incref(boost::python::object(sTestString).ptr());
	}
};

// convert std::map to a list of tuples for .items() behavior
boost::python::list get_deformers_map_items(Piston::CurvesDeformerFactory::DeformersMap& deformers) {
    boost::python::list items;
    for (auto const& [key, ptr] : deformers) {
        items.append(boost::python::make_tuple(key, ptr));
    }
    return items;
}

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

	class_<CurvesDeformerFactory::Key>("DeformerKey")
		.def_readonly("type", &CurvesDeformerFactory::Key::type)
		.def_readwrite("name", &CurvesDeformerFactory::Key::name)
		.def("__repr__", &CurvesDeformerFactory::Key::repr)
	;

	class_<CurvesDeformerFactory, boost::noncopyable>("DeformerFactory",  no_init)
		.def("getInstance", &CurvesDeformerFactory::getInstance, return_value_policy<reference_existing_object>(), "@DocString(CurvesDeformerFactory::getInstance)")
		.def("deleteDeformer", &CurvesDeformerFactory::deleteDeformer)
		.def("getFastDeformer", &CurvesDeformerFactory::getFastDeformer)
		.def("getWrapDeformer", &CurvesDeformerFactory::getWrapDeformer)
		.def("getGuidesDeformer", &CurvesDeformerFactory::getGuidesDeformer)
		.def("setPointsCacheUsageState", &CurvesDeformerFactory::setPointsCacheUsageState)
		.def("getPointsCacheUsageState", &CurvesDeformerFactory::getPointsCacheUsageState)
		.def("__iter__", range(&CurvesDeformerFactory::begin, &CurvesDeformerFactory::end))
		.def("deformers", &CurvesDeformerFactory::deformers, return_value_policy<reference_existing_object>())
		.def("clear", &CurvesDeformerFactory::clear)
	;

	class_<CurvesDeformerFactory::DeformersMap>("DeformersMap")
        .def(map_indexing_suite<CurvesDeformerFactory::DeformersMap, true>()) // true as the second template argument for NoProxy
        .def("items", &get_deformers_map_items) 
        .def("__iter__", iterator<CurvesDeformerFactory::DeformersMap>()); 
    ;

	class_<BaseCurvesDeformer, BaseCurvesDeformer::SharedPtr, boost::noncopyable>("BaseCurvesDeformer",  no_init)
		//.def("setDeformerGeoPrim", &BaseCurvesDeformer::setDeformerGeoPrim, "@DocString(setDeformerGeoPrim)")

		.def("setDeformerGeoPrim", static_cast<void (BaseCurvesDeformer::*)(const pxr::UsdPrim&)>(&BaseCurvesDeformer::setDeformerGeoPrim), "@DocString(setDeformerGeoPrim)")
		.def("setDeformerGeoPrim", static_cast<void (BaseCurvesDeformer::*)(const BaseCurvesDeformer::SharedPtr&)>(&BaseCurvesDeformer::setDeformerGeoPrim))

		.def("setCurvesGeoPrim", &BaseCurvesDeformer::setCurvesGeoPrim, "@DocString(setCurvesGeoPrim)")

		.def("setDeformerRestAttrName", &BaseCurvesDeformer::setDeformerRestAttrName)
		.def("getDeformerRestAttrName", &BaseCurvesDeformer::getDeformerRestAttrName, return_value_policy<copy_const_reference>())
		.def("setCurvesRestAttrName", &BaseCurvesDeformer::setCurvesRestAttrName)
		.def("getCurvesRestAttrName", &BaseCurvesDeformer::getCurvesRestAttrName, return_value_policy<copy_const_reference>())

		.def("setVelocityAttrName", &BaseCurvesDeformer::setVelocityAttrName)
		.def("getVelocityAttrName", &BaseCurvesDeformer::getVelocityAttrName, return_value_policy<copy_const_reference>())

		.def("setSkinPrimAttrName", &BaseCurvesDeformer::setSkinPrimAttrName)
		.def("getSkinPrimAttrName", &BaseCurvesDeformer::getSkinPrimAttrName, return_value_policy<copy_const_reference>())

		.def("setMotionBlurState", &BaseCurvesDeformer::setMotionBlurState)
		.def("getMotionBlurState", &BaseCurvesDeformer::getMotionBlurState)

		.def("setDataPrimPath", &BaseCurvesDeformer::setDataPrimPath)
		.def("getDataPrimPath", &BaseCurvesDeformer::getDataPrimPath, return_value_policy<copy_const_reference>())

		.def("setPointsCacheUsageState", &BaseCurvesDeformer::setPointsCacheUsageState)
		.def("getPointsCacheUsageState", &BaseCurvesDeformer::getPointsCacheUsageState)

		.def("deform", &BaseCurvesDeformer::deform, BaseCurvesDeformer_deform_overloads(args("time_code")))
		.def("deform_dbg", &BaseCurvesDeformer::deform_dbg, BaseCurvesDeformer_deform_dbg_overloads(args("time_code")))

		.def("setReadJsonDataFromPrim", &BaseCurvesDeformer::setReadJsonDataFromPrim)
		.def("writeJsonDataToPrim", &BaseCurvesDeformer::writeJsonDataToPrim, BaseCurvesDeformer_writeJsonDataToPrim_overloads(args("time_code")))

		.def("showDebugGeometry", &BaseCurvesDeformer::showDebugGeometry)

		.def("__repr__", &BaseCurvesDeformer::repr)
		.def("toString", &BaseCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<BaseMeshCurvesDeformer, BaseMeshCurvesDeformer::SharedPtr, bases<BaseCurvesDeformer>, boost::noncopyable>("BaseMeshCurvesDeformer",  no_init)
		.def("setReadJsonDataFromPrim", &BaseMeshCurvesDeformer::setReadJsonDataFromPrim)
		.def("toString", &BaseMeshCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<FastCurvesDeformer, FastCurvesDeformer::SharedPtr, bases<BaseMeshCurvesDeformer>, boost::noncopyable>("FastCurvesDeformer", no_init)
		.def("create", &FastCurvesDeformer::create)
		.staticmethod("create")
		.def("toString", &FastCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<WrapCurvesDeformer, WrapCurvesDeformer::SharedPtr, bases<BaseMeshCurvesDeformer>, boost::noncopyable>("WrapCurvesDeformer", no_init)
		.def("create", &WrapCurvesDeformer::create)
		.staticmethod("create")
		.def("setBindMode", &WrapCurvesDeformer::setBindMode)
		.def("getBindMode", &WrapCurvesDeformer::getBindMode)
		.def("toString", &WrapCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<GuideCurvesDeformer, GuideCurvesDeformer::SharedPtr, bases<BaseCurvesDeformer>, boost::noncopyable>("GuideCurvesDeformer", no_init)
		.def("create", &GuideCurvesDeformer::create)
		.staticmethod("create")
		.def("setBindRootsToSkinSurface", &GuideCurvesDeformer::setBindRootsToSkinSurface)
		.def("getBindRootsToSkinSurface", &GuideCurvesDeformer::getBindRootsToSkinSurface)
		.def("setBindMode", &GuideCurvesDeformer::setBindMode)
		.def("getBindMode", &GuideCurvesDeformer::getBindMode)
		.def("setGuideIDPrimAttrName", &GuideCurvesDeformer::setGuideIDPrimAttrName)
		.def("getGuideIDPrimAttrName", &GuideCurvesDeformer::getGuideIDPrimAttrName, return_value_policy<copy_const_reference>())
		.def("setGuidesSkinGeoPrimAttrName", &GuideCurvesDeformer::setGuidesSkinGeoPrimAttrName)
		.def("getGuidesSkinGeoPrimAttrName", &GuideCurvesDeformer::getGuidesSkinGeoPrimAttrName, return_value_policy<copy_const_reference>())
		.def("setGuidesSkinGeoPrimRestAttrName", &GuideCurvesDeformer::setGuidesSkinGeoPrimRestAttrName)
		.def("getGuidesSkinGeoPrimRestAttrName", &GuideCurvesDeformer::getGuidesSkinGeoPrimRestAttrName, return_value_policy<copy_const_reference>())
		.def("setGuidesSkinGeoPrim", &GuideCurvesDeformer::setGuidesSkinGeoPrim)
		.def("setFastPointBind", &GuideCurvesDeformer::setFastPointBind)
		.def("isFastPointBind", &GuideCurvesDeformer::isFastPointBind)
		.def("toString", &GuideCurvesDeformer::toString, return_value_policy<copy_const_reference>())
	;

	enum_<BaseCurvesDeformer::Type>("DeformerType")
		.value("FAST", BaseCurvesDeformer::Type::FAST)
		.value("WRAP", BaseCurvesDeformer::Type::WRAP)
		.value("GUIDES", BaseCurvesDeformer::Type::GUIDES)
		.export_values()
	;

	enum_<WrapCurvesDeformer::BindMode>("__WrapDeformer_BindMode")
		.value("SPACE", WrapCurvesDeformer::BindMode::SPACE)
		.value("DIST", WrapCurvesDeformer::BindMode::DIST)
		.export_values()
	;

	enum_<GuideCurvesDeformer::BindMode>("__GuideDeformer_BindMode")
		.value("NTB", GuideCurvesDeformer::BindMode::NTB)
		.value("ANGLE", GuideCurvesDeformer::BindMode::ANGLE)
		.value("SPACE", GuideCurvesDeformer::BindMode::SPACE)
		.export_values()
	;

	enum_<LogLevel>("LogLevel")
		.value("TRACE", LogLevel::TRACE)
		.value("DEBUG", LogLevel::DEBUG)
		.value("INFO", LogLevel::INFO)
		.value("WARNING", LogLevel::WARNING)
		.value("ERROR", LogLevel::ERROR)
		.value("FATAL", LogLevel::FATAL)
		.export_values()
	;

	class_<DeformerStats, boost::noncopyable>("DeformerStats", no_init)
		.def("toString", &DeformerStats::toString)
	;

#if !BSON_USES_PXR_VTARRAY
	to_python_converter<Piston::BSON , BSON_to_Python>();
#endif

	def("clearPistonDataFromStage", _clearPistonDataFromStage);
	def("clearPistonDataFromPrim", _clearPistonDataFromPrim);
	def("getLogger", &Logger::getInstance, return_value_policy<reference_existing_object>());
	def("setLogLevel", _setLogLevel);
	def("runTests", &Tests::runTests);	

	implicitly_convertible<BaseMeshCurvesDeformer::SharedPtr, BaseCurvesDeformer::SharedPtr>();
	implicitly_convertible<FastCurvesDeformer::SharedPtr, BaseMeshCurvesDeformer::SharedPtr>();
	implicitly_convertible<WrapCurvesDeformer::SharedPtr, BaseMeshCurvesDeformer::SharedPtr>();
	implicitly_convertible<GuideCurvesDeformer::SharedPtr, BaseCurvesDeformer::SharedPtr>();
}
