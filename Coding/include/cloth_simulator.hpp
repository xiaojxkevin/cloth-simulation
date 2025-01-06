#pragma once

#include "cloth.hpp"
#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/Dense"
#include "Eigen/unsupported/Eigen/KroneckerProduct"

class RectClothSimulator {
private:
    struct MassParticle {
        glm::vec3 position;
        std::vector<unsigned int> connectedSpringStartIndices;
        std::vector<unsigned int> connectedSpringEndIndices;
        
        /// project properties
        glm::vec3 v; // velocity
        glm::vec3 a; // acceleration
        float mass; // mass
        bool isFixed; // True if the particle is fixed

        bool cutted;
        
    };

    struct Spring
    {
        unsigned int fromMassIndex;
        unsigned int toMassIndex;
        
        /// project properties
        float rest_length; // rest length of the spring
        float stiff; // stiffness

    };

    RectCloth* cloth;
    std::vector<MassParticle> particles;
    std::vector<Spring> springs;

    // Simulation parameters
    glm::vec3 gravity;
    float airResistanceCoefficient; // Per-particle

    /// project
    MassParticle* scratchPoint = nullptr;
    float scratchDistance = INFINITY;
    float damping_factor = 0.993f;
    int max_iter;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mass_matrix;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> L_matrix;
    Eigen::LDLT<Eigen::MatrixXf> cholesky_decompsition;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J_matrix;
    Eigen::Matrix<float, Eigen::Dynamic, 1> qn;
    Eigen::Matrix<float, Eigen::Dynamic, 1> qn_1;

public:
    RectClothSimulator(
            RectCloth* cloth,
            float totalMass,
            float stiffnessReference,
            float airResistanceCoefficient,
            const glm::vec3& gravity);
    ~RectClothSimulator() = default;

    /// project
    void step_fast();
    void updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update, bool cut);

private:
    void createMassParticles(float totalMass);
    void createSprings(float stiffnessReference);
    void updateCloth();

    /// project
    void initMatrices();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getExternalForceVector();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> solve();
    void applyConstraints();
    
};