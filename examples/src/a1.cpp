//
// Created by cha on 23. 7. 15.
//

// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rbdl/rbdl.h"
#include "rbdl/rbdl_utils.h"
#include "rbdl/addons/urdfreader/urdfreader.h"

RigidBodyDynamics::Model* modelLF = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR) + "canine/canineV4/urdf/canineV4_2_FL.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), modelLF, true);

int main(int argc, char* argv[]) {
    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    auto ground = world.addGround();
    ground->setAppearance("steel");
    auto aliengo = world.addArticulatedSystem(std::string(URDF_RSC_DIR) + "a1/urdf/a1.urdf");

    /// aliengo joint PD controller
    Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
    jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(aliengo->getDOF()), jointDgain(aliengo->getDOF());
    jointPgain.tail(12).setConstant(100.0);
    jointDgain.tail(12).setConstant(1.0);

    aliengo->setGeneralizedCoordinate(jointNominalConfig);
    aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
    aliengo->setPdGains(jointPgain, jointDgain);
    aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
    aliengo->setName("aliengo");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(aliengo);
    server.launchServer();

    for (int i=0; i<2000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    std::cout<<"total mass "<<aliengo->getCompositeMass()[0]<<std::endl;

    server.killServer();
}
