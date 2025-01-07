#include "cloth_simulator.hpp"
#include <GLFW/glfw3.h>
#include "iostream"
#include <omp.h>

extern const float timeStep;
/// sphere info
//#define COLLISION
//static glm::vec3 spherePos = { 0.5f, -3.0f, 0.0f };
//static float sphereRadiance = 0.5f;

RectClothSimulator::
RectClothSimulator(
    RectCloth* cloth,
    float totalMass,
    float stiffnessReference,
    float airResistanceCoefficient,
    const glm::vec3& gravity, glm::vec3 spherePos, float sphereRadiance, bool collsion) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient),
    gravity(gravity), spherePos(spherePos), sphereRadiance(sphereRadiance), collsion(collsion), max_iter(3) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
    initMatrices();
}

void RectClothSimulator::
createMassParticles(float totalMass) {
    // Create mass particles based on given cloth.
    particles.resize(cloth->nw * cloth->nh);
#pragma omp parallel for num_threads(4)
    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            MassParticle particle{ .position = cloth->getPosition(iw, ih),
                                .v = glm::vec3(0.0f),
                                .a = glm::vec3(0.0f),
                                .mass = totalMass / static_cast<float>(particles.size()),
                                .isFixed = false,
                                .cutted = false
            };
            particles[cloth->idxFromCoord(iw, ih)] = particle;
        }
    }
}

void RectClothSimulator::
createSprings(float stiffnessReference) {
    // First clear all springs
    springs.clear();
#pragma omp parallel for num_threads(4)
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
                Spring cur{ .fromMassIndex = currentParticleIndex,
                        .toMassIndex = cloth->idxFromCoord(valid_position[k][0], valid_position[k][1]),
                        .rest_length = glm::distance(particles[currentParticleIndex].position, particles[cur.toMassIndex].position),
                        .stiff = stiffnessReference };

                particles[cur.toMassIndex].connectedSpringEndIndices.push_back(springs.size());
                particles[currentParticleIndex].connectedSpringStartIndices.push_back(springs.size());
                springs.push_back(cur);
            }
        }
    }
}

/// @brief project
void RectClothSimulator::
initMatrices() {
    std::cout << "particle number: " << particles.size() << std::endl;
    std::cout << "spring number: " << springs.size() << std::endl;

    /// mass matrix M
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        cur_mass_matrix(3 * particles.size(), 3 * particles.size());
    cur_mass_matrix.setZero();
    for (int i = 0; i < particles.size(); ++i) {
        cur_mass_matrix.block<3, 3>(3 * i, 3 * i) = particles[i].mass * Eigen::Matrix3f::Identity();
    }
    this->mass_matrix = std::move(cur_mass_matrix);

    /// L matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        spring_index_matrix = Eigen::MatrixXf::Zero(particles.size(), particles.size());
    for (int i = 0; i < springs.size(); i++) {
        const Spring& cur = springs[i];
        spring_index_matrix(cur.fromMassIndex, cur.fromMassIndex) += cur.stiff;
        spring_index_matrix(cur.fromMassIndex, cur.toMassIndex) -= cur.stiff;
        spring_index_matrix(cur.toMassIndex, cur.fromMassIndex) -= cur.stiff;
        spring_index_matrix(cur.toMassIndex, cur.toMassIndex) += cur.stiff;
    }
    Eigen::Matrix3f id_3; id_3.setIdentity();
    this->L_matrix = std::move(Eigen::kroneckerProduct(spring_index_matrix, id_3));

    /// cholesky decomposition
    cholesky_decompsition = (mass_matrix + timeStep * timeStep * L_matrix).ldlt();

    /// J matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        force_matrix = Eigen::MatrixXf::Zero(particles.size(), springs.size());
    for (int i = 0; i < springs.size(); i++) {
        const Spring& cur = springs[i];
        force_matrix(cur.fromMassIndex, i) += cur.stiff;
        force_matrix(cur.toMassIndex, i) -= cur.stiff;
    }
    this->J_matrix = std::move(Eigen::kroneckerProduct(force_matrix, id_3));

    /// define positions of particles
    this->qn = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    for (int i = 0; i < particles.size(); i++) {
        qn(3 * i) = particles[i].position[0];
        qn(3 * i + 1) = particles[i].position[1];
        qn(3 * i + 2) = particles[i].position[2];
    }
    /// init qn_1 as the same as qn
    this->qn_1 = this->qn;
}

void RectClothSimulator::step_fast() {
    /// compute the next state
    auto qn_next = solve();

    /// assign current state with previous state
    qn = std::move(qn_next);

//#ifdef COLLISION
    if (collsion) {
        for (int i = 0; i < particles.size(); i++) {
            /// collision test
            glm::vec3 p{ qn(3 * i, 0) - spherePos.x,
                qn(3 * i + 1, 0) - spherePos.y,
                qn(3 * i + 2, 0) - spherePos.z };
            float dis = glm::length(p);
            if (dis < sphereRadiance)
            {
                glm::vec3 dVec = glm::normalize(p);
                for (int j = 0; j != 3; ++j)
                    qn(3 * i + j, 0) = spherePos[j] + sphereRadiance * dVec[j];
                //qn_1.block<3, 1>(3 * i, 0) = qn.block<3, 1>(3 * i, 0);
            }
        }
    }
    else {
        this->applyConstraints();
    }

//#else
    /// Apply constraints
    
//#endif

    /// update info for particles
    for (int i = 0; i < particles.size(); ++i) {
        particles[i].position = glm::vec3(qn(3 * i, 0), qn(3 * i + 1, 0), qn(3 * i + 2, 0));
        particles[i].v = glm::vec3(qn(3 * i, 0) - qn_1(3 * i, 0),
            qn(3 * i + 1, 0) - qn_1(3 * i + 1, 0),
            qn(3 * i + 2, 0) - qn_1(3 * i + 2, 0)) / timeStep;
    }

    updateCloth();
}

void RectClothSimulator::
applyConstraints() {
    int left_up_index = cloth->idxFromCoord(0, 0);
    int right_up_index = cloth->idxFromCoord(cloth->nw - 1, 0);
    qn.block<3, 1>(3 * left_up_index, 0) =
        std::move(Eigen::Vector3f(cloth->getInitialPosition(0, 0).x,
            cloth->getInitialPosition(0, 0).y,
            cloth->getInitialPosition(0, 0).z));
    this->qn.block<3, 1>(3 * right_up_index, 0) =
        std::move(Eigen::Vector3f(cloth->getInitialPosition(cloth->nw - 1, 0).x,
            cloth->getInitialPosition(cloth->nw - 1, 0).y,
            cloth->getInitialPosition(cloth->nw - 1, 0).z));

}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::
solve() {
    Eigen::Matrix<float, Eigen::Dynamic, 1> y = (1.0f + damping_factor) * qn - damping_factor * qn_1;
    Eigen::Matrix<float, Eigen::Dynamic, 1> b = std::move(-mass_matrix * y + this->getExternalForceVector());
    Eigen::Matrix<float, Eigen::Dynamic, 1> d = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 * springs.size(), 1);
    Eigen::Matrix<float, Eigen::Dynamic, 1> x = std::move(y);

    qn_1 = std::move(qn);

    for (int i = 0; i < max_iter; i++) {
        /// local step
    #pragma omp parallel for num_threads(4)
        for (int j = 0; j < springs.size(); ++j) {
            int from = springs[j].fromMassIndex, to = springs[j].toMassIndex;
            Eigen::Vector3f p12 = x.block<3, 1>(3 * from, 0) - x.block<3, 1>(3 * to, 0);
            d.block<3, 1>(3 * j, 0) = p12 / p12.norm() * springs[j].rest_length;
        }

        /// global step
        x = cholesky_decompsition.solve(timeStep * timeStep * J_matrix * d - b);
    }
    return x;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RectClothSimulator::
getExternalForceVector() {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        externalForceVector = Eigen::MatrixXf::Zero(3 * particles.size(), 1);
    for (int i = 0; i < particles.size(); i++) {
        const MassParticle& cur = particles[i];
        glm::vec3 force(0.0f);
        force += cur.mass * gravity;
        force -= airResistanceCoefficient * cur.v;

        /// collision test
        // float dis = glm::length(cur.position - spherePos);
        // if (dis - sphereRadiance <= 1e-4f)
        // {
        //     glm::vec3 dVec = glm::normalize(cur.position - spherePos);
        //     if (glm::dot(cur.v, dVec) < -0.0f) 
        //     {
        //         /// add a force on the particle
        //         /// F*t = m * \delta v -> F = (2 * m * v) / t
        //         force -= 2.0f * glm::dot(cur.v, dVec) * dVec * cur.mass / timeStep 
        //             + cur.mass * gravity;
        //         std::cout << cur.v.x<<
        //              ' ' << cur.v.y << 
        //              ' ' << cur.v.z << '\n';
        //     }
        // }

        externalForceVector(3 * i) = force[0];
        externalForceVector(3 * i + 1) = force[1];
        externalForceVector(3 * i + 2) = force[2];
    }
    return timeStep * timeStep * -externalForceVector;
}


void RectClothSimulator::
updateCloth() {
    for (unsigned int i = 0u; i < cloth->nw * cloth->nh; i++)
    {
        //cloth->setPosition(i, particles[i].position);
        if (particles[i].cutted) {
            //cloth->setPosition(i, glm::vec3(666,777,888));
            cloth->setPosition(i, particles[i].position);
            cloth->setcut(i);
        }
        else {
            cloth->setPosition(i, particles[i].position);
        }
    }
}


void RectClothSimulator::
updateScratchPoint(glm::vec3 ori, glm::vec3 dir, bool update, bool cut) {

    /// to update the existed point
    if (update && scratchPoint != nullptr) {
        auto newPosition = ori + scratchDistance * dir;
        Eigen::Vector3f newPositionEigen(newPosition.x, newPosition.y, newPosition.z);
        int scratch_index = scratchPoint - &particles[0];
        qn.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;
        qn_1.block<3, 1>(3 * scratch_index, 0) = newPositionEigen;
    }

    //cut
    if (cut && scratchPoint != nullptr) {

        const unsigned int nh = cloth->nh;
        const unsigned int nw = cloth->nw;
        unsigned int total = nh * nw;
        /*auto newPosition = ori + scratchDistance * dir;
        Eigen::Vector3f newPositionEigen(newPosition.x, newPosition.y, newPosition.z);*/
        int scratch_index = scratchPoint - &particles[0];
        particles[scratch_index].cutted = true;

        for (int i = 0; i < particles[scratch_index].connectedSpringEndIndices.size(); i++) {
            int spring_idx = particles[scratch_index].connectedSpringEndIndices[i];
            springs[spring_idx].stiff = 0;
            spring_idx = particles[scratch_index].connectedSpringStartIndices[i];
            springs[spring_idx].stiff = 0;
        }

        //cut shared springs
        //cloth->idxFromCoord(iw, ih)

        if (scratch_index >= nw && scratch_index % nw > 0) {//leftup
            if (cloth->cutted[scratch_index - nw - 1]) {
                //from left to up
                for (int i = 0; i < particles[scratch_index - 1].connectedSpringStartIndices.size(); i++) {
                    int spring_index = particles[scratch_index - 1].connectedSpringStartIndices[i];
                    if (springs[spring_index].toMassIndex == scratch_index - nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
                //from up to left
                for (int i = 0; i < particles[scratch_index - 1].connectedSpringEndIndices.size(); i++) {
                    int spring_index = particles[scratch_index - 1].connectedSpringEndIndices[i];
                    if (springs[spring_index].fromMassIndex == scratch_index - nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
            }
        }
        if (scratch_index >= nw && scratch_index % nw < nw - 1) { //right up
            if (cloth->cutted[scratch_index - nw + 1]) {
                for (int i = 0; i < particles[scratch_index + 1].connectedSpringStartIndices.size(); i++) {
                    int spring_index = particles[scratch_index + 1].connectedSpringStartIndices[i];
                    if (springs[spring_index].toMassIndex == scratch_index - nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
                for (int i = 0; i < particles[scratch_index + 1].connectedSpringEndIndices.size(); i++) {
                    int spring_index = particles[scratch_index + 1].connectedSpringEndIndices[i];
                    if (springs[spring_index].fromMassIndex == scratch_index - nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
            }
        }
        if (scratch_index + nw < total && scratch_index % nw > 0) {//left down
            if (cloth->cutted[scratch_index + nw - 1]) {
                for (int i = 0; i < particles[scratch_index - 1].connectedSpringStartIndices.size(); i++) {
                    int spring_index = particles[scratch_index - 1].connectedSpringStartIndices[i];
                    if (springs[spring_index].toMassIndex == scratch_index + nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
                for (int i = 0; i < particles[scratch_index - 1].connectedSpringEndIndices.size(); i++) {
                    int spring_index = particles[scratch_index - 1].connectedSpringEndIndices[i];
                    if (springs[spring_index].fromMassIndex == scratch_index + nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
            }
        }
        if (scratch_index + nw < total && scratch_index % nw < nw - 1) {//right down
            if (cloth->cutted[scratch_index + nw + 1]) {
                for (int i = 0; i < particles[scratch_index + 1].connectedSpringStartIndices.size(); i++) {
                    int spring_index = particles[scratch_index + 1].connectedSpringStartIndices[i];
                    if (springs[spring_index].toMassIndex == scratch_index + nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
                for (int i = 0; i < particles[scratch_index + 1].connectedSpringEndIndices.size(); i++) {
                    int spring_index = particles[scratch_index + 1].connectedSpringEndIndices[i];
                    if (springs[spring_index].fromMassIndex == scratch_index + nw) {
                        springs[spring_index].stiff = 0;
                    }
                }
            }
        }



        /// L matrix
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
            spring_index_matrix = Eigen::MatrixXf::Zero(particles.size(), particles.size());
    #pragma omp parallel for num_threads(4)
        for (int i = 0; i < springs.size(); i++) {
            const Spring& cur = springs[i];
            spring_index_matrix(cur.fromMassIndex, cur.fromMassIndex) += cur.stiff;
            spring_index_matrix(cur.fromMassIndex, cur.toMassIndex) -= cur.stiff;
            spring_index_matrix(cur.toMassIndex, cur.fromMassIndex) -= cur.stiff;
            spring_index_matrix(cur.toMassIndex, cur.toMassIndex) += cur.stiff;
        }
        Eigen::Matrix3f id_3; id_3.setIdentity();
        this->L_matrix = std::move(Eigen::kroneckerProduct(spring_index_matrix, id_3));

        /// cholesky decomposition
        cholesky_decompsition = (mass_matrix + timeStep * timeStep * L_matrix).ldlt();

        /// J matrix
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
            force_matrix = Eigen::MatrixXf::Zero(particles.size(), springs.size());
    #pragma omp parallel for num_threads(4)
        for (int i = 0; i < springs.size(); i++) {
            const Spring& cur = springs[i];
            force_matrix(cur.fromMassIndex, i) += cur.stiff;
            force_matrix(cur.toMassIndex, i) -= cur.stiff;
        }
        this->J_matrix = std::move(Eigen::kroneckerProduct(force_matrix, id_3));
    }




    /// to update a point that does not existed
    if ((update || cut) && scratchPoint == nullptr) {
        float dist = 0.1f;
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

    /// NO updates, reset
    if ((!update && !cut) && scratchPoint != nullptr) {
        scratchPoint = nullptr;
        scratchDistance = INFINITY;
    }
}
