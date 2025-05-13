//#include <boost/python.hpp> // Uncomment to use standalonce boost library

/*
 * Just in case we want to build against statndard boost library
 */
#ifndef BOOST_PYTHON_MODULE 
#include <hboost/python.hpp>
#define BOOST_PYTHON_MODULE HBOOST_PYTHON_MODULE
namespace boost = hboost;
#endif // BOOST_PYTHON_MODULE

#include "../../piston_lib/base_hair_deformer.h"
#include "../../piston_lib/fast_hair_deformer.h"
#include "../../piston_lib/hair_deformer_factory.h"

char const* greet() {
	return "Parovoz Piston python library!";
}

BOOST_PYTHON_MODULE(_piston) {
  using namespace boost::python;

  class_<HairDeformerFactory, boost::noncopyable>("HairDeformerFactory",  no_init)
  	.def("getInstance", &HairDeformerFactory::getInstance, return_value_policy<reference_existing_object>())
  	.def("getFastDeformer", &HairDeformerFactory::getFastDeformer)
  ;

	class_<BaseHairDeformer, BaseHairDeformer::SharedPtr>("BaseHairDeformer",  no_init)
		.def("setRestGeoPrim", &BaseHairDeformer::setRestGeoPrim)
		.def("greet", &BaseHairDeformer::greet, return_value_policy<copy_const_reference>())
	;

	class_<FastHairDeformer, FastHairDeformer::SharedPtr>("FastHairDeformer",  no_init)
		.def("create", &FastHairDeformer::create)
		.staticmethod("create")
		.def("greet", &FastHairDeformer::greet, return_value_policy<copy_const_reference>())
	;

	def("greet", greet);
}
