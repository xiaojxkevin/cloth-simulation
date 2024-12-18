/// @ref core
/// @file glm/ext/vector_float3_precision.hpp

#ifndef LIBS_GLM_GLM_EXT_VECTOR_FLOAT3_PRECISION_HPP_
#define LIBS_GLM_GLM_EXT_VECTOR_FLOAT3_PRECISION_HPP_
#include "../detail/type_vec3.hpp"

namespace glm
{
	/// @addtogroup core_vector_precision
	/// @{

	/// 3 components vector of high single-qualifier floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<3, float, highp>		highp_vec3;

	/// 3 components vector of medium single-qualifier floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<3, float, mediump>		mediump_vec3;

	/// 3 components vector of low single-qualifier floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<3, float, lowp>			lowp_vec3;

	/// @}
}//namespace glm
#endif  // LIBS_GLM_GLM_EXT_VECTOR_FLOAT3_PRECISION_HPP_