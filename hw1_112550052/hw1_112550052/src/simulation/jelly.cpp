#include "jelly.h"

#include "Eigen/Dense"

#include "../util/helper.h"
namespace simulation {
constexpr float g_cdK = 1400.0f;
constexpr float g_cdD = 50.0f;

Jelly::Jelly()
    : particleNumPerEdge(10),
      jellyLength(2.0),
      initialPosition(Eigen::Vector3f(0.0, 0.0, 0.0)),
      springCoefStruct(g_cdK),
      springCoefShear(g_cdK),
      springCoefBending(g_cdK),
      damperCoefStruct(g_cdD),
      damperCoefShear(g_cdD),
      damperCoefBending(g_cdD) {
    particleNumPerFace = particleNumPerEdge * particleNumPerEdge;
    initializeParticle();
    initializeSpring();
}

Jelly::Jelly(const Eigen::Vector3f &a_kInitPos, const float jellyLength, const int numAtEdge, const float dSpringCoef,
           const float dDamperCoef)
    : particleNumPerEdge(numAtEdge),
      jellyLength(jellyLength),
      initialPosition(a_kInitPos),
      springCoefStruct(dSpringCoef),
      springCoefShear(dSpringCoef),
      springCoefBending(dSpringCoef),
      damperCoefStruct(dDamperCoef),
      damperCoefShear(dDamperCoef),
      damperCoefBending(dDamperCoef) {
    particleNumPerFace = numAtEdge * numAtEdge;
    initializeParticle();
    initializeSpring();
}

int Jelly::getParticleNum() const { return static_cast<int>(particles.size()); }

int Jelly::getSpringNum() const { return static_cast<int>(springs.size()); }

int Jelly::getNumAtEdge() const { return particleNumPerEdge; }

unsigned int Jelly::getPointMap(const int a_ciSide, const int a_ciI, const int a_ciJ) {
    int r = -1;

    switch (a_ciSide) {
        case 1:  // [a_ciI][a_ciJ][0] bottom face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ;
            break;
        case 6:  // [a_ciI][a_ciJ][9] top face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ + particleNumPerEdge - 1;
            break;
        case 2:  // [a_ciI][0][a_ciJ] front face
            r = particleNumPerFace * a_ciI + a_ciJ;
            break;
        case 5:  // [a_ciI][9][a_ciJ] back face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * (particleNumPerEdge - 1) + a_ciJ;
            break;
        case 3:  // [0][a_ciI][a_ciJ] left face
            r = particleNumPerEdge * a_ciI + a_ciJ;
            break;
        case 4:  // [9][a_ciI][a_ciJ] ra_ciIght face
            r = particleNumPerFace * (particleNumPerEdge - 1) + particleNumPerEdge * a_ciI + a_ciJ;
            break;
    }

    return r;
}

Particle &Jelly::getParticle(int particleIdx) { return particles[particleIdx]; }

std::vector<Particle> *Jelly::getParticlePointer() { return &particles; }

Spring &Jelly::getSpring(int springIdx) { return springs[springIdx]; }

void Jelly::setSpringCoef(const float springCoef, const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        springCoefStruct = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::STRUCT);
    } else if (springType == Spring::SpringType::SHEAR) {
        springCoefShear = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::SHEAR);
    } else if (springType == Spring::SpringType::BENDING) {
        springCoefBending = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::BENDING);
    }
}

void Jelly::setDamperCoef(const float damperCoef, const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        damperCoefStruct = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::STRUCT);
    } else if (springType == Spring::SpringType::SHEAR) {
        damperCoefShear = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::SHEAR);
    } else if (springType == Spring::SpringType::BENDING) {
        damperCoefBending = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::BENDING);
    }
}

void Jelly::resetJelly(const Eigen::Vector3f &offset, const float &rotate) {
    float dTheta = util::radians(rotate);  //  change angle from degree to
                                           //  radian

    for (unsigned int uiI = 0; uiI < particles.size(); uiI++) {
        int i = uiI / particleNumPerFace;
        int j = (uiI / particleNumPerEdge) % particleNumPerEdge;
        int k = uiI % particleNumPerEdge;
        float offset_x = (float)((i - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));
        float offset_y = (float)((j - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));
        float offset_z = (float)((k - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));

        Eigen::Vector3f RotateVec(offset_x, offset_y,
                                  offset_z);  //  vector from center of cube to the particle

        Eigen::AngleAxis<float> rotation(dTheta, Eigen::Vector3f(1.0f, 0.0f, 1.0f).normalized());

        RotateVec = rotation * RotateVec;

        particles[uiI].setPosition(initialPosition + offset + RotateVec);
        particles[uiI].setForce(Eigen::Vector3f::Zero());
        particles[uiI].setVelocity(Eigen::Vector3f::Zero());
    }
}

void Jelly::addForceField(const Eigen::Vector3f &force) {
    for (unsigned int uiI = 0; uiI < particles.size(); uiI++) {
        particles[uiI].setAcceleration(force);
    }
}

void Jelly::computeInternalForce() {
    // TODO#2-3: Compute the internal force (including spring force and damper force) for each spring.
    //   1. Read the start-particle and end-particle index from spring.
    //   2. Use `getPosition()` to get particle i's position and use `getVelocity()` get particle i's velocity.
    //   3. Call `computeSpringForce` and `computeDamperForce` to compute spring force and damper force.
    //   4. Compute net internal force and call `addForce` to apply the force onto particles.
    // Note:
    //   1. Direction of the force.
    // Hint:
    //   1. Use a.norm() to get length of a.
    //   2. Use a.normalize() to normalize a inplace.
    //          a.normalized() will create a new vector.
    //   3. Use a.dot(b) to get dot product of a and b.

    for (unsigned int springIdx = 0; springIdx < springs.size(); ++springIdx) {
        Spring& link = springs[springIdx];
        const int id0 = link.getSpringStartID();
        const int id1 = link.getSpringEndID();
        Particle& p0 = particles[id0];
        Particle& p1 = particles[id1];

        const Eigen::Vector3f x0 = p0.getPosition();
        const Eigen::Vector3f x1 = p1.getPosition();
        const Eigen::Vector3f v0 = p0.getVelocity();
        const Eigen::Vector3f v1 = p1.getVelocity();

        const Eigen::Vector3f elastic = computeSpringForce(x0, x1, link.getSpringCoef(), link.getSpringRestLength());
        const Eigen::Vector3f damping = computeDamperForce(x0, x1, v0, v1, link.getDamperCoef());
        const Eigen::Vector3f netOnStart = elastic + damping;

        p0.addForce(netOnStart);
        p1.addForce(-netOnStart);
    }
}

Eigen::Vector3f Jelly::computeSpringForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                         const float springCoef, const float restLength) {
    // TODO#2-1: Compute spring force given the two positions of the spring.
    //   1. Review "particles.pptx" from p.9 - p.13
    //   2. The sample below just set spring force to zero
    const Eigen::Vector3f edge = positionB - positionA;
    const float length = edge.norm();
    constexpr float kTiny = 1e-8f;
    if (length < kTiny) {
        return Eigen::Vector3f::Zero();
    }
    const Eigen::Vector3f n = edge / length;
    const float stretch = length - restLength;
    return (springCoef * stretch) * n;
}

Eigen::Vector3f Jelly::computeDamperForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                         const Eigen::Vector3f &velocityA, const Eigen::Vector3f &velocityB,
                                         const float damperCoef) {
    // TODO#2-2: Compute damper force given the two positions and the two velocities of the spring.
    //   1. Review "particles.pptx" from p.9 - p.13
    //   2. The sample below just set damper force to zero
    const Eigen::Vector3f span = positionB - positionA;
    const float spanLen = span.norm();
    constexpr float kTiny = 1e-8f;
    if (spanLen < kTiny) {
        return Eigen::Vector3f::Zero();
    }
    const Eigen::Vector3f axis = span / spanLen;
    const Eigen::Vector3f relVel = velocityB - velocityA;
    // Damping opposes the rate of change of spring length (relative speed along the spring direction).
    return (damperCoef * relVel.dot(axis)) * axis;
}

void Jelly::initializeParticle() {
    for (int i = 0; i < particleNumPerEdge; i++) {
        for (int j = 0; j < particleNumPerEdge; j++) {
            for (int k = 0; k < particleNumPerEdge; k++) {
                Particle Particle;
                float offset_x = (float)((i - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));
                float offset_y = (float)((j - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));
                float offset_z = (float)((k - particleNumPerEdge / 2) * jellyLength / (particleNumPerEdge - 1));
                Particle.setPosition(Eigen::Vector3f(initialPosition(0) + offset_x, initialPosition(1) + offset_y,
                                                     initialPosition(2) + offset_z));
                particles.push_back(Particle);

            }
        }
    }
}

void Jelly::initializeSpring() {
    // Note:
    //   1. The particles index can be computed in a similar way as below:
    //   ===============================================
    //   0 1 2 3 ... particlesPerEdge
    //   particlesPerEdge + 1 ....
    //   ... ... particlesPerEdge * particlesPerEdge - 1
    //   ===============================================
    // Here is a simple example which connects the structrual springs along z-axis.

    auto pushLink = [this](int lhs, int rhs, Spring::SpringType type, float k, float d) {
        const float rest = (particles[lhs].getPosition() - particles[rhs].getPosition()).norm();
        springs.push_back(Spring(lhs, rhs, rest, k, d, type));
    };

    for (int ix = 0; ix < particleNumPerEdge; ++ix) {
        for (int iy = 0; iy < particleNumPerEdge; ++iy) {
            for (int iz = 0; iz < particleNumPerEdge; ++iz) {
                const int base = ix * particleNumPerFace + iy * particleNumPerEdge + iz;

                // STRUCT: axis-aligned neighbors
                if (ix + 1 < particleNumPerEdge) {
                    pushLink(base, base + particleNumPerFace, Spring::SpringType::STRUCT, springCoefStruct, damperCoefStruct);
                }
                if (iy + 1 < particleNumPerEdge) {
                    pushLink(base, base + particleNumPerEdge, Spring::SpringType::STRUCT, springCoefStruct, damperCoefStruct);
                }
                if (iz + 1 < particleNumPerEdge) {
                    pushLink(base, base + 1, Spring::SpringType::STRUCT, springCoefStruct, damperCoefStruct);
                }

                // BENDING: two steps away on axes
                if (ix + 2 < particleNumPerEdge) {
                    pushLink(base, base + 2 * particleNumPerFace, Spring::SpringType::BENDING, springCoefBending, damperCoefBending);
                }
                if (iy + 2 < particleNumPerEdge) {
                    pushLink(base, base + 2 * particleNumPerEdge, Spring::SpringType::BENDING, springCoefBending, damperCoefBending);
                }
                if (iz + 2 < particleNumPerEdge) {
                    pushLink(base, base + 2, Spring::SpringType::BENDING, springCoefBending, damperCoefBending);
                }

                // SHEAR: face diagonals and body diagonals
                // Face diagonals (i+1 with j±1, k fixed)
                if (ix + 1 < particleNumPerEdge) {
                    if (iy + 1 < particleNumPerEdge) {
                        pushLink(base, (ix + 1) * particleNumPerFace + (iy + 1) * particleNumPerEdge + iz,
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                    if (iy - 1 >= 0) {
                        pushLink(base, (ix + 1) * particleNumPerFace + (iy - 1) * particleNumPerEdge + iz,
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                    // Face diagonals (i+1 with k±1, j fixed)
                    if (iz + 1 < particleNumPerEdge) {
                        pushLink(base, (ix + 1) * particleNumPerFace + iy * particleNumPerEdge + (iz + 1),
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                    if (iz - 1 >= 0) {
                        pushLink(base, (ix + 1) * particleNumPerFace + iy * particleNumPerEdge + (iz - 1),
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                }
                // Face diagonals (j+1 with k±1, i fixed)
                if (iy + 1 < particleNumPerEdge) {
                    if (iz + 1 < particleNumPerEdge) {
                        pushLink(base, ix * particleNumPerFace + (iy + 1) * particleNumPerEdge + (iz + 1),
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                    if (iz - 1 >= 0) {
                        pushLink(base, ix * particleNumPerFace + (iy + 1) * particleNumPerEdge + (iz - 1),
                                 Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                    }
                }
                // Body diagonals (i+1, j±1, k±1)
                if (ix + 1 < particleNumPerEdge) {
                    for (int offsetY : {-1, 1}) {
                        for (int offsetZ : {-1, 1}) {
                            const int ny = iy + offsetY;
                            const int nz = iz + offsetZ;
                            if (ny < 0 || ny >= particleNumPerEdge || nz < 0 || nz >= particleNumPerEdge) {
                                continue;
                            }
                            pushLink(base, (ix + 1) * particleNumPerFace + ny * particleNumPerEdge + nz,
                                     Spring::SpringType::SHEAR, springCoefShear, damperCoefShear);
                        }
                    }
                }
            }
        }
    }
}

void Jelly::updateSpringCoef(const float a_cdSpringCoef, const Spring::SpringType a_cSpringType) {
    for (unsigned int uiI = 0; uiI < springs.size(); uiI++) {
        if (springs[uiI].getType() == a_cSpringType) {
            springs[uiI].setSpringCoef(a_cdSpringCoef);
        }
    }
}

void Jelly::updateDamperCoef(const float a_cdDamperCoef, const Spring::SpringType a_cSpringType) {
    for (unsigned int uiI = 0; uiI < springs.size(); uiI++) {
        if (springs[uiI].getType() == a_cSpringType) {
            springs[uiI].setDamperCoef(a_cdDamperCoef);
        }
    }
}
}  //  namespace simulation
