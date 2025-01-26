#include "simulation.hpp"

using namespace cgp;


#ifdef SOLUTION
static vec3 spring_force(const vec3& p_i, const vec3& p_j, float L0, float K)
{
    vec3 const p = p_i - p_j;
    float const L = norm(p);
    vec3 const u = p / L;

    vec3 const F = -K * (L - L0) * u;
    return F;
}
#endif


// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters)
{
    // Direct access to the variables
    //  Note: A grid_2D is a structure you can access using its 2d-local index coordinates as grid_2d(k1,k2)
    //   The index corresponding to grid_2d(k1,k2) is k1 + N1*k2, with N1 the first dimension of the grid.
    //   
    grid_2D<vec3>& force = cloth.force;  // Storage for the forces exerted on each vertex

    grid_2D<vec3> const& position = cloth.position;  // Storage for the positions of the vertices
    grid_2D<vec3> const& velocity = cloth.velocity;  // Storage for the normals of the vertices
    grid_2D<vec3> const& normal = cloth.normal;      // Storage for the velocity of the vertices
    

    size_t const N_total = cloth.position.size();       // total number of vertices
    size_t const N = cloth.N_samples();                 // number of vertices in one dimension of the grid

    // Retrieve simulation parameter
    //  The default value of the simulation parameters are defined in simulation.hpp
    float const K = parameters.K;              // spring stifness
    float const m = parameters.mass_total / N_total; // mass of a particle
    float const mu = parameters.mu;            // damping/friction coefficient
    float const	L0 = 1.0f / (N - 1.0f);        // rest length between two direct neighboring particle

#ifdef SOLUTION
#define N_neighbor 4
    static const int offset_u[N_neighbor] = { -1,1,0,0 };
    static const int offset_v[N_neighbor] = { 0,0,-1,1 };
    static const float alpha[N_neighbor] = { 1,1,1,1 };

//#define N_neighbor 8
//    static const int offset_u[N_neighbor] = { -1,1,0,0, -1,-1,1,1 };
//    static const int offset_v[N_neighbor] = { 0,0,-1,1, -1,1,-1,1 };
//    static const float alpha[N_neighbor] = { 1,1,1,1, sqrtf(2),sqrtf(2),sqrtf(2),sqrtf(2) };

//#define N_neighbor 12
//    static const int offset_u[N_neighbor] = { -1,1,0,0, -1,-1,1,1, 2,-2,0,0 };
//    static const int offset_v[N_neighbor] = { 0,0,-1,1, -1,1,-1,1, 0,0,2,-2 };
//    static const float alpha[N_neighbor] = { 1,1,1,1, sqrtf(2),sqrtf(2),sqrtf(2),sqrtf(2), 2,2,2,2 };

//#define N_neighbor 24
//    static const int offset_u[24] = { 2,2,2,2,2, 1,1,1,1,1, 0,0,0,0, -1,-1,-1,-1,-1, -2,-2,-2,-2,-2 };
 //   static const int offset_v[24] = { 2,1,0,-1,-2, 2,1,0,-1,-2, 2,1,-1,-2, 2,1,0,-1,-2, 2,1,0,-1,-2 };
 //   static const float alpha[24] = { sqrtf(8.0f), sqrtf(5),2,sqrtf(5),sqrtf(8), sqrtf(5),sqrtf(2),1,sqrtf(2),sqrtf(5), 2,1,1,2, sqrtf(5),sqrtf(2),1,sqrtf(2),sqrtf(5), sqrtf(8),sqrtf(5),2,sqrtf(5),sqrtf(8) };

    const vec3 g = { 0,-9.81f, 0 };

// Use #prgam omp parallel for - for parallel loops
#pragma omp parallel for
    for (int k = 0; k < N_total; ++k)
    {
        vec3& f = force.at(k);
        vec3 const& n = cloth.normal.at(k);

        // gravity
        f = m * g; 

        // damping
        f += -mu * m * velocity.at(k);

        //wind
        float const coeff = dot(parameters.wind.direction, n);
        f += parameters.wind.magnitude * coeff * n * L0 * L0;
    }

    // Spring
#pragma omp parallel for
    for (int kv = 0; kv < N; ++kv){
        for (int ku = 0; ku < N; ++ku){
            int const offset = ku + N * kv;

            vec3& f = force.at(offset);
            vec3 const& p = position.at(offset);

            for (int kn = 0; kn < N_neighbor; ++kn) {
                int const ku_n = ku + offset_u[kn];
                int const kv_n = kv + offset_v[kn];
                if (ku_n >= 0 && ku_n < N && kv_n >= 0 && kv_n < N)
                {
                    float const a = alpha[kn];
                    int const offset_neighbor = ku_n + N * kv_n;
                    vec3 const& pn = position.at(offset_neighbor);

                    f += spring_force(p, pn, a * L0, K / a);
                }
            }
        }
    }

#else

    // Gravity
    const vec3 g = { 0,0,-9.81f };
    for (int ku = 0; ku < N; ++ku)
        for (int kv = 0; kv < N; ++kv)
            force(ku, kv) = m * g;

    // Drag (= friction)
    for (int ku = 0; ku < N; ++ku)
        for (int kv = 0; kv < N; ++kv)
            force(ku, kv) += -mu * m * velocity(ku, kv);


    // TO DO: Add spring forces ...
    for (int ku = 0; ku < N; ++ku) {
        for (int kv = 0; kv < N; ++kv) {
            // ...
            // force(ku,kv) = ... fill here the force exerted by all the springs attached to the vertex at coordinates (ku,kv).
            // 
            // Notes:
            //   - The vertex positions can be accessed as position(ku,kv)
            //   - The neighbors are at position(ku+1,kv), position(ku-1,kv), position(ku,kv+1), etc. when ku+offset is still in the grid dimension.
            //   - You may want to loop over all the neighbors of a vertex to add each contributing force to this vertex
            //   - To void repetitions and limit the need of debuging, it may be a good idea to define a generic function that computes the spring force between two positions given the parameters K and L0
            //   - If the simulation is a bit too slow, you can speed it up in adapting the parameter N_step in scene.cpp that loops over several simulation step between two displays.
        }
    }

#endif
}

void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
    int const N = cloth.N_samples();
    int const N_total = cloth.position.size();
    float const m = parameters.mass_total/ static_cast<float>(N_total);

#ifdef SOLUTION
#pragma omp parallel for
    for (int k = 0; k < N_total; ++k)
    {
        vec3 const& f = cloth.force.data.at_unsafe(k);
        vec3& p = cloth.position.data.at_unsafe(k);
        vec3& v = cloth.velocity.data.at_unsafe(k);

        // Standard semi-implicit numerical integration
        v = v + dt * f / m;
        p = p + dt * v;
    }
#else
    for (int ku = 0; ku < N; ++ku) {
        for (int kv = 0; kv < N; ++kv) {
            vec3& v = cloth.velocity(ku, kv);
            vec3& p = cloth.position(ku, kv);
            vec3 const& f = cloth.force(ku, kv);

            // Standard semi-implicit numerical integration
            v = v + dt * f / m;
            p = p + dt * v;
        }
    }
#endif
    
}

void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint, numarray<sphere_parameter> sphere)
{
    // Fixed positions of the cloth
    for (auto const& it : constraint.fixed_sample) {
        position_contraint c = it.second;
        cloth.position(c.ku, c.kv) = c.position; // set the position to the fixed one
    }

#ifdef SOLUTION
 const int N = cloth.position.size();
    const float epsilon = 1e-2f;
#pragma omp parallel for
    for (int k = 0; k < N; ++k)
    {
        vec3& p = cloth.position.data.at_unsafe(k);
        vec3& v = cloth.velocity.data.at_unsafe(k);

        /*// Ground constraint
        {
            if (p.z <= constraint.ground_z + epsilon)
            {
                p.z = constraint.ground_z + epsilon;
                v.z = 0.0f;
            }
        }*/

        // Sphere constraint
        {
            vec3 const& p0 = sphere[0].center;
            float const r = sphere[0].radius;
            if (norm(p - p0) < (r + epsilon))
            {
                const vec3 u = normalize(p - p0);
                p = (r + epsilon) * u + p0;
                v = v - dot(v, u) * u;
            }
        } 
        {
            vec3 const& p0 = sphere[1].center;
            float const r = sphere[1].radius;
            if (norm(p - p0) < (r + epsilon))
            {
                const vec3 u = normalize(p - p0);
                p = (r + epsilon) * u + p0;
                v = v - dot(v, u) * u;
            }
        }
    }

#else
    // To do: apply external constraints
    // For all vertex:
    //   If vertex is below floor level ...
    //   If vertex is inside collision sphere ...
#endif
}



bool simulation_detect_divergence(cloth_structure const& cloth)
{
    bool simulation_diverged = false;
    const size_t N = cloth.position.size();
    for (size_t k = 0; simulation_diverged == false && k < N; ++k)
    {
        const float f = norm(cloth.force.data.at_unsafe(k));
        const vec3& p = cloth.position.data.at_unsafe(k);

        if (std::isnan(f)) // detect NaN in force
        {
            std::cout << "\n **** NaN detected in forces" << std::endl;
            simulation_diverged = true;
        }

        if (f > 600.0f) // detect strong force magnitude
        {
            std::cout << "\n **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
            simulation_diverged = true;
        }

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
        {
            std::cout << "\n **** NaN detected in positions" << std::endl;
            simulation_diverged = true;
        }
    }

    return simulation_diverged;
}

