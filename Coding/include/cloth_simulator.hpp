#pragma once

#include "cloth.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

class RectClothSimulator {
private:
    struct MassParticle {
        glm::vec3 position;
        std::vector<unsigned int> connectedSpringStartIndices;
        std::vector<unsigned int> connectedSpringEndIndices;

        // TODO: define other particle properties here

        glm::vec3 v;
        glm::vec3 a;
        float mass;

        bool lock;
        
    };

    struct Spring
    {
        unsigned int fromMassIndex;
        unsigned int toMassIndex;

        // TODO: define other spring properties here

        float rest_length;
        float stiff;

    };

    RectCloth* cloth;
    std::vector<MassParticle> particles;
    std::vector<Spring> springs;

    // Simulation parameters
    glm::vec3 gravity;
    float airResistanceCoefficient; // Per-particle


    //new
    MassParticle* scratchPoint = nullptr;
    float scratchDistance = INFINITY;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mass_matrix;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> L_matrix;
    Eigen::LDLT<Eigen::MatrixXf> cholesky_decompsition;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> J_matrix;
    Eigen::Matrix<float, Eigen::Dynamic, 1> qn;
    Eigen::Matrix<float, Eigen::Dynamic, 1> qn_1;

    int max_iter;

    

public:
    RectClothSimulator(
            RectCloth* cloth,
            float totalMass,
            float stiffnessReference,
            float airResistanceCoefficient,
            const glm::vec3& gravity);
    ~RectClothSimulator() = default;

    void step(float timeStep);


    //new
    void step_fast();
    void updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update, bool lock);






private:
    void createMassParticles(float totalMass);
    void createSprings(float stiffnessReference);
    void updateCloth();


    //new
    void setequation();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> kroneckerProduct(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& A, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& B);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getExternalForceVector();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> solve();

    void applyConstraints();

    
};