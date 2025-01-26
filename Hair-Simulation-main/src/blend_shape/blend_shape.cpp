#include "blend_shape.hpp"

#include "../environment.hpp"

using namespace cgp;

void blend_shape_structure::update()
{
#ifdef SOLUTION
	vertices = faces_storage[0].position;
	for (size_t k = 0; k < faces_storage.size() - 1; ++k)
		vertices += weights[k] * face_difference[k];
	
	face.vbo_position.update(vertices);
	face.vbo_normal.update(normal_per_vertex(vertices, faces_storage[0].connectivity));
#else
	// To Do:
	//  Compute here the new face considering the current blend shape weights
	//  Note:
	//    - This function is called every time one of the slider is modified. You should avoid creating new mesh_drawable: Try to only update existing ones.
	//  Help: 
	//    - weights[k_face] - referes to the blend shape weights for the pose k_face
	//    - face.vbo_position.update( [buffer of new positions] ) - allows to update the displayed position 
	//    - face.vbo_normal.update( normal_per_vertex([vertices], [connectivity]) ) - can be used to automatically compute new normals
	//    - If you use numarray to store the vertices, you can use the operator + (or +=) between two numarray of same size to add their values. You can also multiply a numarray with a scalar value.

	std::cout << "Called update_blend_shape with weights values: " << weights << std::endl; // line to remove once you have seen its action
#endif
}

void blend_shape_structure::initialize()
{
	// Load faces
	std::cout << " Load faces ... " << std::endl;
	std::cout << "\t Load faces 00 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/cuir_hair.obj"));
	std::cout << "\t Load faces 01 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_01.obj"));
	face.initialize_data_on_gpu(faces_storage[0]);


	size_t N_face = faces_storage.size();
	weights.resize(N_face);

#ifdef SOLUTION
	// Compute face difference
	size_t N_vertex = faces_storage[0].position.size();
	face_difference.resize(N_face - 1);
	for (size_t k_face = 0; k_face < N_face - 1; ++k_face)
	{
		face_difference[k_face].resize(N_vertex);
		for (size_t k = 0; k < N_vertex; ++k)
		{
			vec3 const& p = faces_storage[k_face + 1].position[k];
			vec3 const& p0 = faces_storage[0].position[k];
			face_difference[k_face][k] = p - p0;
		}
	}

	// Allocate deformed vertices
	vertices = faces_storage[0].position;
#else
	// To Do:
	//  You can add here any precomputation you need to do. For instance, precomputing the difference between a pose and the reference one.
	//  Help: 
	//    faces_storage[0].position[k] - refers to the k-th vertex position of the reference pose
	//    faces_storage[k_face].position[k] - refers to the k-th vertex of the pose k_face (for k_face>0)
#endif
}