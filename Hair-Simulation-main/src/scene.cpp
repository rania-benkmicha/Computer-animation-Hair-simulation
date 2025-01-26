#include "scene.hpp"


using namespace cgp;

#define step 5

void scene_structure::initialize()
{
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	camera_control.look_at({ 8.0f, 4.0f, 4.0f }, {0,0,0}, {0,0,1});
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	//obstacle sphere
	obstacle_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
	obstacle_sphere1.initialize_data_on_gpu(mesh_primitive_sphere());


	// Initialize the shapes of the scene
	// ***************************************** //
	std::cout << " Load body ... " << std::endl;
	body.initialize_data_on_gpu(mesh_load_file_obj(project::path+"assets/head.obj"));

	blend_shape.initialize(); // Load the faces

	cloth_texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/hair.png");
	initialize_cloth(gui.N_sample_edge, blend_shape.faces_storage);
	
}

void scene_structure::initialize_cloth(int N_sample, numarray<mesh> faces_storage)
{
	numarray<vec3> vertices =  faces_storage[0].position;
	size_t N_vert = vertices.size();
	
	cloth.resize(N_vert);
	cloth_drawable.resize(N_vert);
	constraint.resize(N_vert);
	#pragma omp parallel for
	for( int k=0; k < N_vert; k+=step){
		//std::cout<<"vertex "<<vertices[k]<<std::endl;
		cloth[k].initialize(N_sample, vertices[k]);
		cloth_drawable[k].initialize(N_sample, vertices[k]);
		cloth_drawable[k].drawable.texture = cloth_texture;
		cloth_drawable[k].drawable.material.texture_settings.two_sided = true;

		constraint[k].fixed_sample.clear();
		for(int i=0; i<N_sample; i++){
			constraint[k].add_fixed_position(0,i,cloth[k]);
		}
	}
	
	obstacle_sphere.model.translation = constraint[0].sphere[0].center;
	obstacle_sphere.model.scaling = constraint[0].sphere[0].radius;
	obstacle_sphere.material.color = { 1,0,0 }; 
	obstacle_sphere1.model.translation = constraint[0].sphere[1].center;
	obstacle_sphere1.model.scaling = constraint[0].sphere[1].radius;
	obstacle_sphere1.material.color = { 1,0,0 };
	//constraint.add_fixed_position(0, 0, cloth);
	//constraint.add_fixed_position(0, N_sample - 1, cloth);
}

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	if (gui.display_frame)
		draw(global_frame, environment);

	// Handle elements of the scene
	// ***************************************** //
	timer.update(); // update the timer to the current elapsed time


	// Display the elements
	if (gui.display_face)
		draw(blend_shape.face, environment);
	if (gui.display_wireframe)
		draw_wireframe(blend_shape.face, environment, { 0,0,1 });
	if (gui.display_body)
		draw(body, environment);

	//######################################################## ADD
	//draw(obstacle_sphere, environment);
	//draw(obstacle_sphere1, environment);

	
	// Simulation of the cloth
	// ***************************************** //
	numarray<vec3> vertices =  blend_shape.faces_storage[0].position;
	size_t N_vert = vertices.size();
	#pragma omp parallel for
	for( int k=0; k < N_vert; k+=step){

#ifdef SOLUTION
	int const N_step = 5;
#else
	int const N_step = 1; // Adapt here the number of intermediate simulation steps (ex. 5 intermediate steps per frame)
#endif
	for (int k_step = 0; simulation_running == true && k_step < N_step; ++k_step)
	{
		// Update the forces on each particle
		simulation_compute_force(cloth[k], parameters);

		// One step of numerical integration
		simulation_numerical_integration(cloth[k], parameters, parameters.dt);

		// Apply the positional (and velocity) constraints
		simulation_apply_constraints(cloth[k], constraint[k], constraint[0].sphere);

		// Check if the simulation has not diverged - otherwise stop it
		bool const simulation_diverged = simulation_detect_divergence(cloth[k]);
		if (simulation_diverged) {
			std::cout << "\n *** Simulation has diverged ***" << std::endl;
			std::cout << " > The simulation is stoped" << std::endl;
			simulation_running = false;
		}
	}


	// Cloth display
	// ***************************************** //

	// Prepare to display the updated cloth
	cloth[k].update_normal();        // compute the new normals
	cloth_drawable[k].update(cloth[k]); // update the positions on the GPU

	// Display the cloth
	draw(cloth_drawable[k], environment);
	//if (gui.display_wireframe)
	//	draw_wireframe(cloth_drawable[k], environment);
		
	}
}


void scene_structure::display_gui()
{
	bool reset = false;
	
	ImGui::Checkbox("Frame", &gui.display_frame);
	
	ImGui::Checkbox("Face", &gui.display_face); ImGui::SameLine();
	ImGui::Checkbox("Wireframe", &gui.display_wireframe);
	ImGui::Checkbox("Body", &gui.display_body);

	//ImGui::Checkbox("Texture Cloth", &cloth_drawable.drawable.material.texture_settings.active);


	// Slider of the GUI
	bool const slider_01 = ImGui::SliderFloat("w1", &blend_shape.weights[0], 0.0f, 1.0f);
	bool const slider_02 = ImGui::SliderFloat("w2", &blend_shape.weights[1], 0.0f, 1.0f);
	// return values slider_0x are true if the slider is modified, otherwise they are false.

	// If one of the slider of the GUI is modified, then call the function update_blend_shape
	if (slider_01 || slider_02 )
		blend_shape.update();

	//######################################################
	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Simulation parameters");
	ImGui::SliderFloat("Time step", &parameters.dt, 0.0001f, 0.02f, "%.4f", 2.0f);
	ImGui::SliderFloat("Stiffness", &parameters.K, 0.0f, 50.0f, "%.3f", 0.0f);
	ImGui::SliderFloat("Wind magnitude", &parameters.wind.magnitude, 0, 60, "%.3f", 2.0f);
	ImGui::SliderFloat("Damping", &parameters.mu, 1.0f, 30.0f);
	ImGui::SliderFloat("Mass", &parameters.mass_total, 0.0f, 5.0f, "%.3f", 2.0f);

	ImGui::Spacing(); ImGui::Spacing();

	reset |= ImGui::SliderInt("Cloth samples", &gui.N_sample_edge, 4, 80);

	ImGui::Spacing(); ImGui::Spacing();
	reset |= ImGui::Button("Restart");
	if (reset) {
		initialize_cloth(gui.N_sample_edge, blend_shape.faces_storage);
		simulation_running = true;
	}

}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
	
}