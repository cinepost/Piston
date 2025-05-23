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

#include "../../piston_lib/base_hair_deformer.h"
#include "../../piston_lib/fast_hair_deformer.h"
#include "../../piston_lib/hair_deformer_factory.h"

char const* greet() {
	return "Parovoz Piston python library!";
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BaseHairDeformer_deform_overloads, Piston::BaseHairDeformer::deform, 0, 1)

BOOST_PYTHON_MODULE(_piston) {
  using namespace boost::python;
  using namespace Piston;

	class_<HairDeformerFactory, boost::noncopyable>("HairDeformerFactory",  no_init)
		.def("getInstance", &HairDeformerFactory::getInstance, return_value_policy<reference_existing_object>())
		.def("getFastDeformer", &HairDeformerFactory::getFastDeformer)
	;

	class_<BaseHairDeformer, BaseHairDeformer::SharedPtr, boost::noncopyable>("BaseHairDeformer",  no_init)
		.def("setMeshGeoPrim", &BaseHairDeformer::setMeshGeoPrim)
		.def("setHairGeoPrim", &BaseHairDeformer::setHairGeoPrim)
		.def("setMeshRestPositionAttrName", &BaseHairDeformer::setMeshRestPositionAttrName)
		.def("getMeshRestPositionAttrName", &BaseHairDeformer::getMeshRestPositionAttrName, return_value_policy<copy_const_reference>())
		.def("deform", &BaseHairDeformer::deform, BaseHairDeformer_deform_overloads(args("time_code")))
		.def("toString", &BaseHairDeformer::toString, return_value_policy<copy_const_reference>())
	;

	class_<FastHairDeformer, FastHairDeformer::SharedPtr, bases<BaseHairDeformer>>("FastHairDeformer",  no_init)
		.def("create", &FastHairDeformer::create)
		.staticmethod("create")
		.def("toString", &FastHairDeformer::toString, return_value_policy<copy_const_reference>())
	;

	def("greet", greet);
}
