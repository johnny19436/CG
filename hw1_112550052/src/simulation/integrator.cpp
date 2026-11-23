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
    const float step = particleSystem.deltaTime;
    for (int j = 0; j < particleSystem.jellyCount; ++j) {
        Jelly& body = particleSystem.jellies[j];
        const int pCount = body.getParticleNum();
        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            const Eigen::Vector3f velNow = pt.getVelocity();
            const Eigen::Vector3f accNow = pt.getAcceleration();
            pt.addPosition(velNow * step);
            pt.addVelocity(accNow * step);
            pt.setForce(Eigen::Vector3f::Zero());
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

    const float step = particleSystem.deltaTime;
    for (int j = 0; j < particleSystem.jellyCount; ++j) {
        Jelly& body = particleSystem.jellies[j];
        const int pCount = body.getParticleNum();

        std::vector<Eigen::Vector3f> pos0(pCount);
        std::vector<Eigen::Vector3f> vel0(pCount);
        std::vector<Eigen::Vector3f> pos1(pCount);
        std::vector<Eigen::Vector3f> vel1(pCount);

        for (int id = 0; id < pCount; ++id) {
            const Particle& pt = body.getParticle(id);
            pos0[id] = pt.getPosition();
            vel0[id] = pt.getVelocity();
        }

        for (int id = 0; id < pCount; ++id) {
            pos1[id] = pos0[id] + vel0[id] * step;
            vel1[id] = vel0[id];
            Particle& pt = body.getParticle(id);
            pt.setPosition(pos1[id]);
            pt.setVelocity(vel1[id]);
        }

        body.addForceField(particleSystem.gravity);
        body.computeInternalForce();

        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            const Eigen::Vector3f accPred = pt.getAcceleration();
            vel1[id] = vel0[id] + accPred * step;
            pos1[id] = pos0[id] + vel1[id] * step;
            pt.setPosition(pos1[id]);
            pt.setVelocity(vel1[id]);
        }

        if (particleSystem.sceneIdx == 0) {
            particleSystem.planeTerrain->handleCollision(step, body);
        } else {
            particleSystem.elevatorTerrain->handleCollision(step, body);
        }

        for (int id = 0; id < pCount; ++id) {
            body.getParticle(id).setForce(Eigen::Vector3f::Zero());
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

    const float step = particleSystem.deltaTime;
    for (int j = 0; j < particleSystem.jellyCount; ++j) {
        Jelly& body = particleSystem.jellies[j];
        const int pCount = body.getParticleNum();

        std::vector<Eigen::Vector3f> xBase(pCount), vBase(pCount);
        std::vector<Eigen::Vector3f> dx1(pCount), dv1(pCount), dx2(pCount), dv2(pCount),
            dx3(pCount), dv3(pCount), dx4(pCount), dv4(pCount);

        auto stageEval = [&](const std::vector<Eigen::Vector3f>& xIn, const std::vector<Eigen::Vector3f>& vIn,
                             std::vector<Eigen::Vector3f>& dxOut, std::vector<Eigen::Vector3f>& dvOut) {
            for (int id = 0; id < pCount; ++id) {
                Particle& pt = body.getParticle(id);
                pt.setPosition(xIn[id]);
                pt.setVelocity(vIn[id]);
            }
            body.addForceField(particleSystem.gravity);
            body.computeInternalForce();
            for (int id = 0; id < pCount; ++id) {
                Particle& pt = body.getParticle(id);
                dxOut[id] = vIn[id];
                dvOut[id] = pt.getAcceleration();
            }
        };

        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            xBase[id] = pt.getPosition();
            vBase[id] = pt.getVelocity();
        }

        stageEval(xBase, vBase, dx1, dv1);
        {
            std::vector<Eigen::Vector3f> xTmp(pCount), vTmp(pCount);
            for (int id = 0; id < pCount; ++id) {
                xTmp[id] = xBase[id] + 0.5f * step * dx1[id];
                vTmp[id] = vBase[id] + 0.5f * step * dv1[id];
            }
            stageEval(xTmp, vTmp, dx2, dv2);
        }
        {
            std::vector<Eigen::Vector3f> xTmp(pCount), vTmp(pCount);
            for (int id = 0; id < pCount; ++id) {
                xTmp[id] = xBase[id] + 0.5f * step * dx2[id];
                vTmp[id] = vBase[id] + 0.5f * step * dv2[id];
            }
            stageEval(xTmp, vTmp, dx3, dv3);
        }
        {
            std::vector<Eigen::Vector3f> xTmp(pCount), vTmp(pCount);
            for (int id = 0; id < pCount; ++id) {
                xTmp[id] = xBase[id] + step * dx3[id];
                vTmp[id] = vBase[id] + step * dv3[id];
            }
            stageEval(xTmp, vTmp, dx4, dv4);
        }

        for (int id = 0; id < pCount; ++id) {
            const Eigen::Vector3f posNew = xBase[id] + (step / 6.0f) * (dx1[id] + 2.0f * dx2[id] + 2.0f * dx3[id] + dx4[id]);
            const Eigen::Vector3f velNew = vBase[id] + (step / 6.0f) * (dv1[id] + 2.0f * dv2[id] + 2.0f * dv3[id] + dv4[id]);
            Particle& pt = body.getParticle(id);
            pt.setPosition(posNew);
            pt.setVelocity(velNew);
        }

        if (particleSystem.sceneIdx == 0) {
            particleSystem.planeTerrain->handleCollision(step, body);
        } else {
            particleSystem.elevatorTerrain->handleCollision(step, body);
        }

        for (int id = 0; id < pCount; ++id) {
            body.getParticle(id).setForce(Eigen::Vector3f::Zero());
        }
    }

}

IntegratorType PositionBasedDynamicIntegrator::getType() { return IntegratorType::PositionBasedDynamic; }

void PositionBasedDynamicIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO#4-3: Integrate position and velocity
    //   1. Prediction Step
    //   2. Constraint Projection
    //   3. Position and Velocity Update with XSPH Viscosity

    const float step = particleSystem.deltaTime;
    const int projectionPasses = 4;
    const float visc = 0.1f;
    constexpr float kMin = 1e-6f;

    for (int j = 0; j < particleSystem.jellyCount; ++j) {
        Jelly& body = particleSystem.jellies[j];
        const int pCount = body.getParticleNum();
        std::vector<Eigen::Vector3f> xPrev(pCount);
        std::vector<std::vector<int>> nbr(pCount);

        const int linkCount = body.getSpringNum();
        for (int s = 0; s < linkCount; ++s) {
            Spring& link = body.getSpring(s);
            const int lhs = link.getSpringStartID();
            const int rhs = link.getSpringEndID();
            nbr[lhs].push_back(rhs);
            nbr[rhs].push_back(lhs);
        }

        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            xPrev[id] = pt.getPosition();
            const Eigen::Vector3f velPred = pt.getVelocity() + pt.getAcceleration() * step;
            pt.setVelocity(velPred);
            pt.setPredictedPosition(pt.getPosition() + velPred * step);
        }

        for (int pass = 0; pass < projectionPasses; ++pass) {
            if (particleSystem.sceneIdx == 0) {
                const Eigen::Vector3f q = particleSystem.planeTerrain->getPosition();
                Eigen::Vector3f n(std::sin(M_PI / 9.0), std::cos(M_PI / 9.0), 0.0);
                n.normalize();
                for (int id = 0; id < pCount; ++id) {
                    Particle& pt = body.getParticle(id);
                    Eigen::Vector3f xp = pt.getPredictedPosition();
                    const float sd = n.dot(xp - q);
                    if (sd < 0.0f) {
                        xp -= sd * n;
                        pt.setPredictedPosition(xp);
                    }
                }
            } else {
                const float yFloor = particleSystem.elevatorTerrain->getPosition().y() + 0.5f;
                for (int id = 0; id < pCount; ++id) {
                    Particle& pt = body.getParticle(id);
                    Eigen::Vector3f xp = pt.getPredictedPosition();
                    if (xp.y() < yFloor) {
                        xp.y() = yFloor;
                        pt.setPredictedPosition(xp);
                    }
                }
            }

            for (int s = 0; s < linkCount; ++s) {
                Spring& link = body.getSpring(s);
                const int a = link.getSpringStartID();
                const int b = link.getSpringEndID();
                Particle& pa = body.getParticle(a);
                Particle& pb = body.getParticle(b);
                Eigen::Vector3f xa = pa.getPredictedPosition();
                Eigen::Vector3f xb = pb.getPredictedPosition();
                const Eigen::Vector3f delta = xb - xa;
                const float dist = delta.norm();
                if (dist < kMin) {
                    continue;
                }
                const Eigen::Vector3f axis = delta / dist;
                const float rest = link.getSpringRestLength();
                const float dMin = 0.9f * rest;
                const float dMax = 1.1f * rest;
                if (dist < dMin) {
                    const Eigen::Vector3f shift = 0.5f * (dMin - dist) * axis;
                    pa.setPredictedPosition(xa - shift);
                    pb.setPredictedPosition(xb + shift);
                } else if (dist > dMax) {
                    const Eigen::Vector3f shift = 0.5f * (dist - dMax) * axis;
                    pa.setPredictedPosition(xa + shift);
                    pb.setPredictedPosition(xb - shift);
                }
            }
        }

        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            const Eigen::Vector3f xNow = pt.getPredictedPosition();
            pt.setVelocity((xNow - xPrev[id]) / step);
            pt.setPosition(xNow);
        }

        std::vector<Eigen::Vector3f> vFiltered(pCount, Eigen::Vector3f::Zero());
        for (int id = 0; id < pCount; ++id) {
            Particle& pi = body.getParticle(id);
            float wSum = 0.0f;
            Eigen::Vector3f dv = Eigen::Vector3f::Zero();
            for (int q : nbr[id]) {
                Particle& pj = body.getParticle(q);
                const float r = (pj.getPosition() - pi.getPosition()).norm();
                if (r < kMin) {
                    continue;
                }
                const float w = 1.0f / r;
                wSum += w;
                dv += w * (pj.getVelocity() - pi.getVelocity());
            }
            vFiltered[id] = (wSum > kMin) ? (pi.getVelocity() + visc * dv / wSum) : pi.getVelocity();
        }

        for (int id = 0; id < pCount; ++id) {
            Particle& pt = body.getParticle(id);
            pt.setVelocity(vFiltered[id]);
            pt.setForce(Eigen::Vector3f::Zero());
        }
    }
}

}  // namespace simulation
