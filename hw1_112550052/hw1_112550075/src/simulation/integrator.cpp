#include "integrator.h"
#include <iostream>
#include <cmath>
#include <vector>
namespace simulation {
// Factory
std::unique_ptr<Integrator> IntegratorFactory::CreateIntegrator(IntegratorType type) {
    switch (type) {
        case simulation::IntegratorType::ExplicitEuler:
            return std::make_unique<ExplicitEulerIntegrator>();
        case simulation::IntegratorType::ImplicitEuler:
            return std::make_unique<ImplicitEulerIntegrator>();
        case simulation::IntegratorType::MidpointEuler:
            return std::make_unique<MidpointEulerIntegrator>();
        case simulation::IntegratorType::RungeKuttaFourth:
            return std::make_unique<RungeKuttaFourthIntegrator>();
        case simulation::IntegratorType::PositionBasedDynamic:
            return std::make_unique<PositionBasedDynamicIntegrator>();
        default:
            throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
            break;
    }
    return nullptr;
}

//
// ExplicitEulerIntegrator
//

IntegratorType ExplicitEulerIntegrator::getType() { return IntegratorType::ExplicitEuler; }


void ExplicitEulerIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO#4-1: Integrate position and velocity
    //   1. Integrate position using current velocity.
    //   2. Integrate velocity using current acceleration.
    //   3. Clear force
    // Note:
    //   1. You should do this first because it is very simple. Then you can check whether your collision is correct or
    //   not.
    //   2. See functions in `particle.h` if you don't know how to access data.

    //   3. Review "ODE_basics.pptx" from p.15 - p.16
    float dt = particleSystem.deltaTime;
    for (int jellyIdx = 0; jellyIdx < particleSystem.getJellyCount(); jellyIdx++) {
        Jelly* jelly = particleSystem.getJellyPointer(jellyIdx);
        const int particleNum = jelly->getParticleNum();

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            particle.addPosition(dt * particle.getVelocity());
            particle.addVelocity(dt * particle.getAcceleration());
            particle.setForce(Eigen::Vector3f::Zero());
        }
    }
}

//
// ImplicitEulerIntegrator
//

IntegratorType ImplicitEulerIntegrator::getType() { return IntegratorType::ImplicitEuler; }

void ImplicitEulerIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO#4-2: Integrate position and velocity
    //   1. Backup original particles' data.
    //   2. Integrate position and velocity using explicit euler to get Xn+1.
    //   3. Compute refined Xn+1 and Vn+1 using (1.) and (2.).
    // Note:
    //   1. Use `MassSpringSystem::computeJellyForce`
    //      with modified position and velocity to get Xn+1.
    //   2. See functions in `particle.h` if you don't know how to access data.
    //   3. Review "ODE_implicit.pptx" from p.18 - p.19

    float dt = particleSystem.deltaTime;
    constexpr int kImplicitIterations = 6;
    constexpr float kRelaxation = 0.35f;

    for (int jellyIdx = 0; jellyIdx < particleSystem.getJellyCount(); jellyIdx++) {
        Jelly* jelly = particleSystem.getJellyPointer(jellyIdx);
        const int particleNum = jelly->getParticleNum();

        std::vector<Eigen::Vector3f> x0(particleNum);
        std::vector<Eigen::Vector3f> v0(particleNum);
        std::vector<Eigen::Vector3f> xIter(particleNum);
        std::vector<Eigen::Vector3f> vIter(particleNum);
        std::vector<Eigen::Vector3f> a0(particleNum);

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            x0[particleIdx] = particle.getPosition();
            v0[particleIdx] = particle.getVelocity();
            a0[particleIdx] = particle.getAcceleration();

            // Initial guess from explicit Euler.
            vIter[particleIdx] = v0[particleIdx] + dt * a0[particleIdx];
            xIter[particleIdx] = x0[particleIdx] + dt * vIter[particleIdx];
        }

        for (int iter = 0; iter < kImplicitIterations; iter++) {
            for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
                Particle& particle = jelly->getParticle(particleIdx);
                particle.setPosition(xIter[particleIdx]);
                particle.setVelocity(vIter[particleIdx]);
                particle.setForce(Eigen::Vector3f::Zero());
            }

            // Backward-Euler fixed-point update: evaluate force at current predicted state.
            particleSystem.computeJellyForce(*jelly);

            for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
                Particle& particle = jelly->getParticle(particleIdx);
                const Eigen::Vector3f vTarget = v0[particleIdx] + dt * particle.getAcceleration();
                const Eigen::Vector3f vNew = (1.0f - kRelaxation) * vIter[particleIdx] + kRelaxation * vTarget;
                const Eigen::Vector3f xNew = x0[particleIdx] + dt * vNew;
                vIter[particleIdx] = vNew;
                xIter[particleIdx] = xNew;
            }
        }

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            particle.setPosition(xIter[particleIdx]);
            particle.setVelocity(vIter[particleIdx]);
            particle.setForce(Eigen::Vector3f::Zero());
        }
    }
}

//
// MidpointEulerIntegrator
//

IntegratorType MidpointEulerIntegrator::getType() { return IntegratorType::MidpointEuler; }

void MidpointEulerIntegrator::integrate(MassSpringSystem& particleSystem) {}

//
// RungeKuttaFourthIntegrator
//

IntegratorType RungeKuttaFourthIntegrator::getType() { return IntegratorType::RungeKuttaFourth; }

void RungeKuttaFourthIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO#Bonus: Integrate velocity and acceleration
    //   1. Backup original particles' data.
    //   2. Compute k1, k2, k3, k4
    //   3. Compute refined Xn+1 using (1.) and (2.).
    // Note:
    //   1. Use `MassSpringSystem::computeJellyForce`
    //      with modified position and velocity to get Xn+1.
    //   2. See functions in `particle.h` if you don't know how to access data.
    //   3. StateStep struct is just a hint, you can use whatever you want.
    //   4. Review "ODE_basics.pptx" from p.21

    const float dt = particleSystem.deltaTime;
    for (int jellyIdx = 0; jellyIdx < particleSystem.getJellyCount(); jellyIdx++) {
        Jelly* jelly = particleSystem.getJellyPointer(jellyIdx);
        const int particleNum = jelly->getParticleNum();

        std::vector<Eigen::Vector3f> x0(particleNum), v0(particleNum);
        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            x0[particleIdx] = jelly->getParticle(particleIdx).getPosition();
            v0[particleIdx] = jelly->getParticle(particleIdx).getVelocity();
        }

        auto evaluate = [&](const std::vector<Eigen::Vector3f>& xState, const std::vector<Eigen::Vector3f>& vState,
                            std::vector<Eigen::Vector3f>& dxdt, std::vector<Eigen::Vector3f>& dvdt) {
            for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
                Particle& particle = jelly->getParticle(particleIdx);
                particle.setPosition(xState[particleIdx]);
                particle.setVelocity(vState[particleIdx]);
                particle.setForce(Eigen::Vector3f::Zero());
            }

            particleSystem.computeJellyForce(*jelly);

            for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
                Particle& particle = jelly->getParticle(particleIdx);
                dxdt[particleIdx] = particle.getVelocity();
                dvdt[particleIdx] = particle.getAcceleration();
            }
        };

        std::vector<Eigen::Vector3f> k1x(particleNum), k1v(particleNum);
        std::vector<Eigen::Vector3f> k2x(particleNum), k2v(particleNum);
        std::vector<Eigen::Vector3f> k3x(particleNum), k3v(particleNum);
        std::vector<Eigen::Vector3f> k4x(particleNum), k4v(particleNum);
        std::vector<Eigen::Vector3f> xTmp(particleNum), vTmp(particleNum);

        evaluate(x0, v0, k1x, k1v);
        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            xTmp[particleIdx] = x0[particleIdx] + 0.5f * dt * k1x[particleIdx];
            vTmp[particleIdx] = v0[particleIdx] + 0.5f * dt * k1v[particleIdx];
        }

        evaluate(xTmp, vTmp, k2x, k2v);
        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            xTmp[particleIdx] = x0[particleIdx] + 0.5f * dt * k2x[particleIdx];
            vTmp[particleIdx] = v0[particleIdx] + 0.5f * dt * k2v[particleIdx];
        }

        evaluate(xTmp, vTmp, k3x, k3v);
        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            xTmp[particleIdx] = x0[particleIdx] + dt * k3x[particleIdx];
            vTmp[particleIdx] = v0[particleIdx] + dt * k3v[particleIdx];
        }

        evaluate(xTmp, vTmp, k4x, k4v);

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            const Eigen::Vector3f xNext = x0[particleIdx] +
                                          (dt / 6.0f) *
                                              (k1x[particleIdx] + 2.0f * k2x[particleIdx] + 2.0f * k3x[particleIdx] +
                                               k4x[particleIdx]);
            const Eigen::Vector3f vNext = v0[particleIdx] +
                                          (dt / 6.0f) *
                                              (k1v[particleIdx] + 2.0f * k2v[particleIdx] + 2.0f * k3v[particleIdx] +
                                               k4v[particleIdx]);
            particle.setPosition(xNext);
            particle.setVelocity(vNext);
            particle.setForce(Eigen::Vector3f::Zero());
        }
    }
}

IntegratorType PositionBasedDynamicIntegrator::getType() { return IntegratorType::PositionBasedDynamic; }

void PositionBasedDynamicIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO#4-3: Integrate position and velocity
    //   1. Prediction Step
    //   2. Constraint Projection
    //   3. Position and Velocity Update with XSPH Viscosity

    const float dt = particleSystem.deltaTime;
    const int solverIterations = 4;
    const float xsphFactor = 0.1f;

    for (int jellyIdx = 0; jellyIdx < particleSystem.getJellyCount(); jellyIdx++) {
        Jelly* jelly = particleSystem.getJellyPointer(jellyIdx);
        const int particleNum = jelly->getParticleNum();
        const int nEdge = jelly->getNumAtEdge();
        const int nFace = nEdge * nEdge;
        const float cellSize = jelly->getLength() / static_cast<float>(nEdge - 1);

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            const Eigen::Vector3f velocity = particle.getVelocity() + dt * particle.getAcceleration();
            const Eigen::Vector3f predictedPosition = particle.getPosition() + dt * velocity;
            particle.setVelocity(velocity);
            particle.setPredictedPosition(predictedPosition);
        }

        for (int iter = 0; iter < solverIterations; iter++) {
            Eigen::Vector3f planeCenter = Eigen::Vector3f::Zero();
            Eigen::Vector3f n = Eigen::Vector3f::Zero();
            Eigen::Vector3f tangentX = Eigen::Vector3f::UnitX();
            Eigen::Vector3f tangentZ = Eigen::Vector3f::UnitZ();
            float halfExtent = 15.0f;

            if (particleSystem.sceneIdx == 0) {
                const float angle = static_cast<float>(20.0 * util::PI<double>() / 180.0);
                n = Eigen::Vector3f(std::sin(angle), std::cos(angle), 0.0f).normalized();
                tangentX = Eigen::Vector3f(std::cos(angle), -std::sin(angle), 0.0f);
                tangentZ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
                halfExtent = 15.0f;
            } else {
                n = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
                if (particleSystem.elevatorTerrain) {
                    planeCenter = particleSystem.elevatorTerrain->getPosition();
                }
                halfExtent = 2.5f;
            }

            for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
                Particle& particle = jelly->getParticle(particleIdx);
                Eigen::Vector3f predicted = particle.getPredictedPosition();
                const Eigen::Vector3f relative = predicted - planeCenter;
                const float localX = relative.dot(tangentX);
                const float localZ = relative.dot(tangentZ);
                if (std::abs(localX) > halfExtent || std::abs(localZ) > halfExtent) {
                    continue;
                }

                const float signedDistance = relative.dot(n);
                if (signedDistance < 0.0f) {
                    predicted += (-signedDistance + 0.01f) * n;
                    particle.setPredictedPosition(predicted);
                }
            }

            for (int i = 0; i < nEdge; i++) {
                for (int j = 0; j < nEdge; j++) {
                    for (int k = 0; k < nEdge; k++) {
                        const int idxA = i * nFace + j * nEdge + k;
                        for (int di = -1; di <= 1; di++) {
                            for (int dj = -1; dj <= 1; dj++) {
                                for (int dk = -1; dk <= 1; dk++) {
                                    if (di == 0 && dj == 0 && dk == 0) {
                                        continue;
                                    }
                                    if (di < 0 || (di == 0 && dj < 0) || (di == 0 && dj == 0 && dk < 0)) {
                                        continue;
                                    }

                                    const int ni = i + di;
                                    const int nj = j + dj;
                                    const int nk = k + dk;
                                    if (ni < 0 || ni >= nEdge || nj < 0 || nj >= nEdge || nk < 0 || nk >= nEdge) {
                                        continue;
                                    }

                                    const int idxB = ni * nFace + nj * nEdge + nk;
                                    Particle& particleA = jelly->getParticle(idxA);
                                    Particle& particleB = jelly->getParticle(idxB);
                                    Eigen::Vector3f pA = particleA.getPredictedPosition();
                                    Eigen::Vector3f pB = particleB.getPredictedPosition();

                                    Eigen::Vector3f delta = pB - pA;
                                    const float dist = delta.norm();
                                    if (dist < 1e-6f) {
                                        continue;
                                    }

                                    const float restDist = cellSize * std::sqrt(static_cast<float>(di * di + dj * dj + dk * dk));
                                    const float minDistance = 0.8f * restDist;
                                    const float maxDistance = 1.2f * restDist;
                                    const Eigen::Vector3f dir = delta / dist;

                                    if (dist < minDistance) {
                                        const Eigen::Vector3f corr = 0.5f * (minDistance - dist) * dir;
                                        pA -= corr;
                                        pB += corr;
                                        particleA.setPredictedPosition(pA);
                                        particleB.setPredictedPosition(pB);
                                    } else if (dist > maxDistance) {
                                        const Eigen::Vector3f corr = 0.5f * (dist - maxDistance) * dir;
                                        pA += corr;
                                        pB -= corr;
                                        particleA.setPredictedPosition(pA);
                                        particleB.setPredictedPosition(pB);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        std::vector<Eigen::Vector3f> xsphVelocities(particleNum, Eigen::Vector3f::Zero());
        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            Particle& particle = jelly->getParticle(particleIdx);
            const Eigen::Vector3f xOld = particle.getPosition();
            const Eigen::Vector3f xNew = particle.getPredictedPosition();
            const Eigen::Vector3f vNew = (xNew - xOld) / dt;
            particle.setVelocity(vNew);
            particle.setPosition(xNew);
            particle.setForce(Eigen::Vector3f::Zero());
        }

        for (int i = 0; i < nEdge; i++) {
            for (int j = 0; j < nEdge; j++) {
                for (int k = 0; k < nEdge; k++) {
                    const int idx = i * nFace + j * nEdge + k;
                    const Eigen::Vector3f vP = jelly->getParticle(idx).getVelocity();

                    float weightSum = 0.0f;
                    Eigen::Vector3f velocityCorrection = Eigen::Vector3f::Zero();

                    for (int di = -1; di <= 1; di++) {
                        for (int dj = -1; dj <= 1; dj++) {
                            for (int dk = -1; dk <= 1; dk++) {
                                if (di == 0 && dj == 0 && dk == 0) {
                                    continue;
                                }

                                const int ni = i + di;
                                const int nj = j + dj;
                                const int nk = k + dk;
                                if (ni < 0 || ni >= nEdge || nj < 0 || nj >= nEdge || nk < 0 || nk >= nEdge) {
                                    continue;
                                }

                                const int nIdx = ni * nFace + nj * nEdge + nk;
                                const Eigen::Vector3f xP = jelly->getParticle(idx).getPosition();
                                const Eigen::Vector3f xQ = jelly->getParticle(nIdx).getPosition();
                                const float dist = (xQ - xP).norm();
                                if (dist <= 1e-6f) {
                                    continue;
                                }

                                const float w = 1.0f / dist;
                                weightSum += w;
                                velocityCorrection += w * (jelly->getParticle(nIdx).getVelocity() - vP);
                            }
                        }
                    }

                    xsphVelocities[idx] = vP;
                    if (weightSum > 1e-6f) {
                        xsphVelocities[idx] += xsphFactor * velocityCorrection / weightSum;
                    }
                }
            }
        }

        for (int particleIdx = 0; particleIdx < particleNum; particleIdx++) {
            jelly->getParticle(particleIdx).setVelocity(xsphVelocities[particleIdx]);
        }
    }
}

}  // namespace simulation
