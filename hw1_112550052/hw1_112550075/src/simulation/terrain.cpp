#include "terrain.h"

#include <cmath>
#include <stdexcept>

#include "../util/helper.h"
#include <iostream>

namespace simulation {
// Factory
std::unique_ptr<Terrain> TerrainFactory::CreateTerrain(TerrainType type) {
    switch (type) {
        case simulation::TerrainType::Plane:
            return std::make_unique<PlaneTerrain>();
        case simulation::TerrainType::Elevator:
            return std::make_unique<ElevatorTerrain>();

        default:
            throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
            break;
    }
    return nullptr;
}
// Terrain

Eigen::Matrix4f Terrain::getModelMatrix() { return modelMatrix; }

float Terrain::getMass() const { return mass; }

Eigen::Vector3f Terrain::getPosition() const { return position; }

Eigen::Vector3f Terrain::getVelocity() const { return velocity; }

Eigen::Vector3f Terrain::getAcceleration() const { return force / mass; }

Eigen::Vector3f Terrain::getForce() const { return force; }

void Terrain::setMass(const float _mass) { mass = _mass; }

void Terrain::setPosition(const Eigen::Vector3f& _position) {
    modelMatrix = util::translate(_position - position) * modelMatrix;
    position = _position;
}

void Terrain::setVelocity(const Eigen::Vector3f& _velocity) { velocity = _velocity; }

void Terrain::setAcceleration(const Eigen::Vector3f& _acceleration) { force = _acceleration * mass; }

void Terrain::setForce(const Eigen::Vector3f& _force) { force = _force; }

void Terrain::addPosition(const Eigen::Vector3f& _position) {
    position += _position;
    modelMatrix = util::translate(_position) * modelMatrix;
}

void Terrain::addVelocity(const Eigen::Vector3f& _velocity) { velocity += _velocity; }

void Terrain::addAcceleration(const Eigen::Vector3f& _acceleration) { force += _acceleration * mass; }

void Terrain::addForce(const Eigen::Vector3f& _force) { force += _force; }

// Note:
// You should update each particles' velocity (base on the equation in
// slide) and force (contact force : resist + friction) in handleCollision function

// PlaneTerrain //

PlaneTerrain::PlaneTerrain() { reset(); }

void PlaneTerrain::reset() {
    modelMatrix = util::translate(0.0f, position[1], 0.0f) * util::rotateDegree(0, 0, -20) * util::scale(30, 1, 30);
}


TerrainType PlaneTerrain::getType() { return TerrainType::Plane; }

void PlaneTerrain::handleCollision(const float delta_T, Jelly& jelly) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.5f;
    // TODO#3-1: Handle collision when a particle collide with the plane terrain.
    //   If collision happens:
    //      1. Directly update particles' velocity
    //      2. Apply contact force to particles when needed
    // Note:
    //   1. There are `jelly.getParticleNum()` particles.
    //   2. See TODOs in `Jelly::computeInternalForce` and functions in `particle.h` if you don't know how to access
    //   data.
    //   3. The plane spans 30x30 units in the XZ plane and is rotated -20 degrees around the Z-axis.
    // Hint:
    //   1. Review "particles.pptx" from p.14 - p.19
    //   2. Use a.norm() to get length of a.
    //   3. Use a.normalize() to normalize a inplace.
    //          a.normalized() will create a new vector.
    //   4. Use a.dot(b) to get dot product of a and b.
    const Eigen::Vector3f n = normal.normalized();
    const Eigen::Vector3f planeCenter = position;
    const float halfExtent = 15.0f;

    const float angle = util::PI<float>() / 9.0f;
    const Eigen::Vector3f tangentX(std::cos(angle), -std::sin(angle), 0.0f);
    const Eigen::Vector3f tangentZ(0.0f, 0.0f, 1.0f);

    for (int particleIdx = 0; particleIdx < jelly.getParticleNum(); particleIdx++) {
        Particle& particle = jelly.getParticle(particleIdx);
        const Eigen::Vector3f p = particle.getPosition();
        Eigen::Vector3f v = particle.getVelocity();

        const Eigen::Vector3f relativePos = p - planeCenter;
        const float localX = relativePos.dot(tangentX);
        const float localZ = relativePos.dot(tangentZ);
        if (std::abs(localX) > halfExtent || std::abs(localZ) > halfExtent) {
            continue;
        }

        const float signedDistance = relativePos.dot(n);
        if (signedDistance > eEPSILON) {
            continue;
        }

        const float normalSpeed = v.dot(n);
        if (normalSpeed < 0.0f) {
            v -= (1.0f + coefResist) * normalSpeed * n;
            particle.setVelocity(v);
        }

        const float penetration = eEPSILON - signedDistance;
        const float resistMagnitude = particle.getMass() * coefResist * penetration / (delta_T * delta_T);
        const Eigen::Vector3f resistForce = resistMagnitude * n;
        particle.addForce(resistForce);

        const Eigen::Vector3f tangentVelocity = v - v.dot(n) * n;
        const float tangentSpeed = tangentVelocity.norm();
        if (tangentSpeed > 1e-6f) {
            const Eigen::Vector3f frictionForce = -coefFriction * resistMagnitude * tangentVelocity / tangentSpeed;
            particle.addForce(frictionForce);
        }
    }
}

ElevatorTerrain::ElevatorTerrain() {
    reset();
}

void ElevatorTerrain::reset() {
    modelMatrix = util::translate(0.0f, 1.0f, 0.0f) * util::rotateDegree(0, 0, 0) * util::scale(5, 1, 5);
    position = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

TerrainType ElevatorTerrain::getType() { return TerrainType::Elevator; }

void ElevatorTerrain::handleCollision(const float delta_T, Jelly& jelly) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.5f;

    addVelocity(delta_T * getAcceleration());
    addPosition(delta_T * getVelocity());
    setForce(Eigen::Vector3f::Zero());

    const Eigen::Vector3f n = normal.normalized();
    const Eigen::Vector3f planeCenter = position;
    const float halfExtent = 2.5f;
    const Eigen::Vector3f tangentX(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f tangentZ(0.0f, 0.0f, 1.0f);

    for (int particleIdx = 0; particleIdx < jelly.getParticleNum(); particleIdx++) {
        Particle& particle = jelly.getParticle(particleIdx);
        const Eigen::Vector3f p = particle.getPosition();
        Eigen::Vector3f v = particle.getVelocity();

        const Eigen::Vector3f relativePos = p - planeCenter;
        const float localX = relativePos.dot(tangentX);
        const float localZ = relativePos.dot(tangentZ);
        if (std::abs(localX) > halfExtent || std::abs(localZ) > halfExtent) {
            continue;
        }

        const float signedDistance = relativePos.dot(n);
        if (signedDistance > eEPSILON) {
            continue;
        }

        Eigen::Vector3f relativeVel = v - velocity;
        const float normalSpeed = relativeVel.dot(n);
        if (normalSpeed < 0.0f) {
            relativeVel -= (1.0f + coefResist) * normalSpeed * n;
            v = velocity + relativeVel;
            particle.setVelocity(v);
        }

        const float penetration = eEPSILON - signedDistance;
        const float resistMagnitude = particle.getMass() * coefResist * penetration / (delta_T * delta_T);
        const Eigen::Vector3f resistForce = resistMagnitude * n;
        particle.addForce(resistForce);
        addForce(-resistForce);

        const Eigen::Vector3f tangentVelocity = relativeVel - relativeVel.dot(n) * n;
        const float tangentSpeed = tangentVelocity.norm();
        if (tangentSpeed > 1e-6f) {
            const Eigen::Vector3f frictionForce = -coefFriction * resistMagnitude * tangentVelocity / tangentSpeed;
            particle.addForce(frictionForce);
            addForce(-frictionForce);
        }
    }
}

}  // namespace simulation
