#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"

// Parameters of the colliding sphere (center, radius)
struct sphere_parameter {
	cgp::vec3 center;
	float radius;
};

// Parameter attached to a fixed vertex (ku,kv) coordinates + 3D position
struct position_contraint {
	int ku;
	int kv;
	cgp::vec3 position;
};


struct constraint_structure
{
	float ground_z = 0.0f;                                   // Height of the flood
	numarray<sphere_parameter> sphere = { {{0.0f, 3.2f, 0.9f}, 1.5f}, {{0.0f, 3.3f, 0.4f}, 1.6f} }; // Colliding sphere
	
	std::map<size_t, position_contraint> fixed_sample; // Storage of all fixed position of the cloth

	// Add a new fixed position
	void add_fixed_position(int ku, int kv, cloth_structure const& cloth);
	// Remove a fixed position
	void remove_fixed_position(int ku, int kv);

};