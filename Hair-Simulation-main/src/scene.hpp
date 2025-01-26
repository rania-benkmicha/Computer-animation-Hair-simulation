#pragma once
/**
	Objective: Implement the Blend Shape deformation on the character face
	- Observe the relation between the change of weights by the sliders (blend_shapes_sliders) and the call of the function update_blend_shape
	- Add the necessary precomputation in the function initialize, and the update of the deformed face in update_blend_shape.
*/


#include "cgp/cgp.hpp"

#include "environment.hpp"

#include "blend_shape/blend_shape.hpp"

#include "cloth/cloth.hpp"
#include "simulation/simulation.hpp"

using cgp::mesh_drawable;



struct gui_parameters {
	bool display_frame     = true;
	bool display_wireframe = false;
	bool display_face = true;
	bool display_body = true;
	int N_sample_edge = 20;  // number of samples of the cloth (the total number of vertices is N_sample_edge^2)
};



// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment; // Standard environment controler
	input_devices inputs; // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                       // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;

	// Display of the obstacles and constraints
	mesh_drawable obstacle_sphere;
	mesh_drawable obstacle_sphere1;

	mesh_drawable body;         // The static body of the character
	blend_shape_structure blend_shape;

	// Cloth related structures
	numarray<cloth_structure> cloth;                     // The values of the position, velocity, forces, etc, stored as a 2D grid
	numarray<cloth_structure_drawable> cloth_drawable;   // Helper structure to display the cloth as a mesh
	simulation_parameters parameters;          // Stores the parameters of the simulation (stiffness, mass, damping, time step, etc)
	numarray<constraint_structure> constraint;           // Handle the parameters of the constraints (fixed vertices, floor and sphere)

	// Helper variables
	bool simulation_running = true;   // Boolean indicating if the simulation should be computed
	cgp::opengl_texture_image_structure cloth_texture;             // Storage of the texture ID used for the cloth


	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display_frame();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop


	void initialize_cloth(int N_sample, numarray<mesh> face_storage); // Recompute the cloth from scratch


	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

};





