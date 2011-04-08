#include <iostream>
#include <lzmath.h>
#include <share/structs.h>
#include <mesh/custommesh.h>
#include <mesh/lwomesh.h>
#include <scene/scene.h>
#include <util/luabind.h>

using namespace luabind;
using namespace lzmath;

extern void SetNumRayTraceJobs(int n);

void bind(lua_State *L)
{
	module(L)
    [
		class_<Scene>("Scene")
			.def("SetLight", &Scene::SetLight)
			.def("AddLWOMesh", &Scene::AddLWOMesh)
			.def("AddCustomMesh", &Scene::AddCustomMesh)
			.def("KDTreeMaxLeafSize", &Scene::KDTreeMaxLeafSize)
			.def("KDTreeExtraDepth", &Scene::KDTreeExtraDepth)
			.def("FrustumSetDim", &Scene::FrustumSetDim)
			.def("FrustumSetNumSteps", &Scene::FrustumSetNumSteps)
			.def("FrustumSetStepSize", &Scene::FrustumSetStepSize)
			.def("KDSetNumSplits", &Scene::KDSetNumSplits)
			.def("KDSetNumAxises", &Scene::KDSetNumAxises)
			.def("ShadowRays", &Scene::ShadowRays)
			.def("SecondRays", &Scene::SecondRays),
		class_<Transform>("Transform"),

		def("Translate", &lzmath::Translate),
		def("RotateX", &lzmath::RotateX),
		def("RotateY", &lzmath::RotateY),
		def("RotateZ", &lzmath::RotateZ),
		def("Mult", &lzmath::Mult),

		class_<LWOMesh>("LWOMesh")
			.def(constructor<const char*>())
			.def("Load", &LWOMesh::Load)
			.def("Apply", &LWOMesh::Apply),

		class_<CustomMesh>("CustomMesh")
			.def(constructor<int>())
			.def("BeginTriangle", &CustomMesh::BeginTriangle)
			.def("Vertex", &CustomMesh::Vertex)
			.def("Color", &CustomMesh::Color)
			.def("EndTriangle", &CustomMesh::EndTriangle)
			.def("Reflective", &CustomMesh::Reflective)
			.def("Apply", &CustomMesh::Apply),

		def("GetScene", &GetScene),
		def("SetNumRayTraceJobs", &SetNumRayTraceJobs)

    ];

	

}


lua_State* InitLua()
{
	lua_State* L = lua_open();
	lua_baselibopen(L);
	lua_mathlibopen(L);
	luabind::open(L);

	bind(L);

	return L;
}




