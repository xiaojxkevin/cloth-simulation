#include "cloth_simulator.hpp"
#include <GLFW/glfw3.h>
#include "iostream"
#include <omp.h>

const float timestep = 0.002f;

RectClothSimulator::
RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3& gravity) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
    setequation();
    max_iter = 1;
}

void RectClothSimulator::setequation() {
    std::cout << "particle number: " << particles.size() << std::endl;
    std::cout << "spring number: " << springs.size() << std::endl;

    //mass matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cur_mass_matrix(3 * particles.size(), 3 * particles.size());
    cur_mass_matrix.setZero();
    for (int i = 0; i < particles.size(); ++i) {
        cur_mass_matrix.block<3, 3>(3 * i, 3 * i) = particles[i].mass * Eigen::Matrix3f::Identity();
    }
    mass_matrix = cur_mass_matrix;


    //L matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> spring_index_matrix = Eigen::MatrixXf::Zero(particles.size(), particles.size());
    for (int i = 0; i < springs.size(); i++) {
        Spring cur = springs[i];
        spring_index_matrix(cur.fromMassIndex, cur.fromMassIndex) += cur.stiff;
        spring_index_matrix(cur.fromMassIndex, cur.toMassIndex) -= cur.stiff;
        spring_index_matrix(cur.toMassIndex, cur.fromMassIndex) -= cur.stiff;
        spring_index_matrix(cur.toMassIndex, cur.toMassIndex) += cur.stiff;
    }
    Eigen::Matrix3f id_3;
    id_3.setIdentity();
    L_matrix = kroneckerProduct(spring_index_matrix, id_3);

    //apply decomposition
    cholesky_decompsition = (mass_matrix + timestep * timestep * L_matrix).ldlt();


    //J matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> force_matrix = Eigen::MatrixXf::Zero(particles.size(), springs.size());
    for (int i = 0; i < springs.size(); i++) {
        Spring cur = springs[i];
        force_matrix(cur.fromMassIndex, i) += cur.stiff;
        force_matrix(cur.toMassIndex, i) -= cur.stiff;
    }
    J_matrix = kroneckerProduct(force_matrix, id_3);


    //particle situation
    qn = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    for (int i = 0; i < particles.size(); i++) {
        qn(3 * i) = particles[i].position[0];
        qn(3 * i + 1) = particles[i].position[1];
        qn(3 * i + 2) = particles[i].position[2];
    }
    qn_1 = qn;

}



void RectClothSimulator::step_fast() {
    applyConstraints();


    auto qn_add_1 = solve();
    qn_1 = qn;
    for(int i = 0 ; i < particles.size();i++){
        if(particles[i].lock){
            qn_add_1.block<3, 1>(3 * i, 0) = qn.block<3, 1>(3 * i, 0);
        }
    }
    qn = qn_add_1;
    for (int i = 0; i < particles.size(); ++i) {
        particles[i].position = glm::vec3(qn(3 * i, 0), qn(3 * i + 1, 0), qn(3 * i + 2, 0));
        particles[i].v = glm::vec3(qn(3 * i, 0) - qn_1(3 * i, 0),   qn(3 * i + 1, 0) - qn_1(3 * i + 1, 0),   qn(3 * i + 2, 0) - qn_1(3 * i + 2, 0)); //这个甚至都不用
    }

    updateCloth();
}





void RectClothSimulator::
createMassParticles(float totalMass) {
    // Create mass particles based on given cloth.
    particles.resize(cloth->nw * cloth->nh);
    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            MassParticle particle;
            particle.position = cloth->getPosition(iw, ih);

            // TODO: Initialize other mass properties.
            //  Use 'cloth->...' to access cloth properties.

            particle.mass = totalMass / particles.size();
            particle.v = glm::vec3(0.0f);
            particle.a = glm::vec3(0.0f);

            particle.lock = false;
            

            particles[cloth->idxFromCoord(iw, ih)] = particle;
        }
    }
}

void RectClothSimulator::
createSprings(float stiffnessReference) {
    // First clear all springs
    springs.clear();

    // TODO: Create springs connecting mass particles.
    //  You may find 'cloth->idxFromCoord(...)' useful.
    //  You can store springs into the member variable 'springs' which is a std::vector.
    //  You may want to modify mass particles too.

    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            unsigned int currentParticleIndex = cloth->idxFromCoord(iw, ih);

            std::vector<glm::ivec2> valid_position;
            glm::ivec2 right = { iw + 1 ,ih };
            if (right[0] < cloth->nw) {
                valid_position.push_back(right);
            }
            glm::ivec2 left = { iw - 1, ih };
            if (left[0] >= 0) {
                valid_position.push_back(left);
            }
            glm::ivec2 up = { iw, ih - 1 };
            if (up[1] >= 0) {
                valid_position.push_back(up);
            }
            glm::ivec2 down = { iw , ih + 1 };
            if (down[1] < cloth->nh) {
                valid_position.push_back(down);
            }
            glm::ivec2 right_up = { iw + 1, ih - 1 };
            if (right_up[0] < cloth->nw && right_up[1] >= 0) {
                valid_position.push_back(right_up);
            }
            glm::ivec2 right_down = { iw + 1, ih + 1 };
            if (right_down[0] < cloth->nw && right_down[1] < cloth->nh) {
                valid_position.push_back(right_down);
            }
            glm::ivec2 left_up = { iw - 1, ih - 1 };
            if (left_up[0] >= 0 && left_up[1] >= 0) {
                valid_position.push_back(left_up);
            }
            glm::ivec2 left_down = { iw - 1, ih + 1 };
            if (left_down[0] >= 0 && left_down[1] < cloth->nh) {
                valid_position.push_back(left_down);
            }

            for (int k = 0; k < valid_position.size(); k++) {
                Spring cur;
                cur.fromMassIndex = currentParticleIndex;
                cur.toMassIndex = cloth->idxFromCoord(valid_position[k][0], valid_position[k][1]);
                cur.rest_length = glm::distance(particles[currentParticleIndex].position, particles[cur.toMassIndex].position);
                cur.stiff = stiffnessReference;

                particles[cur.toMassIndex].connectedSpringEndIndices.push_back(springs.size());
                particles[currentParticleIndex].connectedSpringStartIndices.push_back(springs.size());
                springs.push_back(cur);
            }

        }
    }
}

void RectClothSimulator::
step(float timeStep) {
    // TODO: Simulate one step based on given time step.
    //  Step 1: Update particle positions
    //  Step 2: Update springs
    //  Step 3: Apply constraints
    //  Hint: See cloth_simulator.hpp to check for member variables you need.
    //  Hint: You may use 'cloth->getInitialPosition(...)' for constraints.


    for (int ih = 0; ih < cloth->nh; ih++) {
        for (int iw = 0; iw < cloth->nw; iw++) {
            int currentParticleIndex = cloth->idxFromCoord(iw, ih);
            //glm::vec3 wind = { 8.0f * particles[currentParticleIndex].mass, 0.0f,0.0f };   //向右
            //particles[currentParticleIndex].a = gravity + wind / particles[currentParticleIndex].mass;
            particles[currentParticleIndex].a = gravity;
            glm::vec3 air_res = airResistanceCoefficient * particles[currentParticleIndex].v / particles[currentParticleIndex].mass;
            particles[currentParticleIndex].a -= air_res;
        }
    }

    for (int i = 0; i < springs.size(); i++)
    {
        int toMassIndex = springs[i].toMassIndex;
        int fromMassIndex = springs[i].fromMassIndex;
        float springLength = glm::length(particles[toMassIndex].position - particles[fromMassIndex].position);
        glm::vec3 tension = springs[i].stiff * (springLength - springs[i].rest_length) * glm::normalize(particles[toMassIndex].position - particles[fromMassIndex].position);
        particles[fromMassIndex].a += tension / particles[fromMassIndex].mass;
        particles[toMassIndex].a -= tension / particles[toMassIndex].mass;
    }

    for (int ih = 0; ih < cloth->nh; ih++) {
        for (int iw = 0; iw < cloth->nw; iw++) {
            int currentParticleIndex = cloth->idxFromCoord(iw, ih);
            if (currentParticleIndex == 0 || currentParticleIndex == cloth->nw - 1)
            {
                continue;
            }

            if(particles[currentParticleIndex].lock == true){
                continue;
            }

            particles[currentParticleIndex].v += particles[currentParticleIndex].a * timeStep;
            particles[currentParticleIndex].position = particles[currentParticleIndex].position + timeStep * particles[currentParticleIndex].v;
        }
    }

    // Finally update cloth data
    updateCloth();
}




void RectClothSimulator::applyConstraints() {
    int left_up_index = cloth->idxFromCoord(0, 0);
    int right_up_index = cloth->idxFromCoord(cloth->nw - 1, 0);
    qn.block<3, 1>(3 * left_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(0, 0).x, cloth->getInitialPosition(0, 0).y, cloth->getInitialPosition(0, 0).z);
    qn.block<3, 1>(3 * right_up_index, 0) = Eigen::Vector3f(cloth->getInitialPosition(cloth->nw - 1, 0).x, cloth->getInitialPosition(cloth->nw - 1, 0).y, cloth->getInitialPosition(cloth->nw - 1, 0).z);
    
}


Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::kroneckerProduct(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& A, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& B) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> result(A.rows() * B.rows(), A.cols() * B.cols());

    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            result.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
        }
    }
    return result;
}






Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::getExternalForceVector() {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> externalForceVector = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    for (int i = 0; i < particles.size(); i++) {
        MassParticle cur = particles[i];
        glm::vec3 grav = cur.mass * gravity;
        externalForceVector(3 * i) = grav[0];
        externalForceVector(3 * i + 1) = grav[1];
        externalForceVector(3 * i + 2) = grav[2];
    }
    return timestep * timestep * -externalForceVector;
}



Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::solve() {

    Eigen::Matrix<float, Eigen::Dynamic, 1> y = 2 * qn - qn_1;
    auto stratTime_0 = omp_get_wtime();

    Eigen::Matrix<float, Eigen::Dynamic, 1> b = -mass_matrix * y + getExternalForceVector();
    Eigen::Matrix<float, Eigen::Dynamic, 1> x = y;
    Eigen::Matrix<float, Eigen::Dynamic, 1> d = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 * springs.size(), 1);

    for (int i = 0; i < max_iter; i++) {

        for (int j = 0; j < springs.size(); ++j) {
            int a = springs[j].fromMassIndex, b = springs[j].toMassIndex;
            Eigen::Vector3f p12 = x.block<3, 1>(3 * a, 0) - x.block<3, 1>(3 * b, 0);
            d.block<3, 1>(3 * j, 0) = springs[j].rest_length * p12 / p12.norm();
        }

        x = cholesky_decompsition.solve(timestep * timestep * J_matrix * d - b);
    }
    auto endTime_0 = omp_get_wtime();
    std::cout << "solve time: " << (endTime_0 - stratTime_0) * 1000 << "ms" << std::endl;

    return x;
}



void RectClothSimulator::
updateCloth() {
    for (unsigned int i = 0u; i < cloth->nw * cloth->nh; i++)
    {
        cloth->setPosition(i, particles[i].position);
    }
}


float last_lock = 0;
float cur_lock = 0;

void RectClothSimulator::updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update,  bool lock) {
    if (update && scratchPoint != nullptr) {

        auto newPosition = ori + scratchDistance * dir;
        Eigen::Vector3f newPositionEigen(newPosition.x, newPosition.y, newPosition.z);
        int scratch_index = scratchPoint - &particles[0];
        qn.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;
        qn_1.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;


        if(lock){
            scratchPoint->lock = true; 
        }





    }
    if (update && scratchPoint == nullptr) {
        float dist = INFINITY;
        for (MassParticle& particle : particles) {
            glm::vec3 diffVec = particle.position - ori;
            glm::vec3 distanceVec = diffVec - glm::dot(diffVec, dir) * dir;
            float curDist = glm::length(distanceVec);
            if (curDist < dist) {
                dist = curDist;
                scratchPoint = &particle;
                scratchDistance = glm::dot(diffVec, dir);
            }
        }
    }
    if (!update && scratchPoint != nullptr) {
        scratchPoint = nullptr;
        scratchDistance = INFINITY;
    }

    

}
