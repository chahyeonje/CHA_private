// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rbdl/rbdl.h"
#include "rbdl/rbdl_utils.h"
#include "rbdl/addons/urdfreader/urdfreader.h"


int main(int argc, char* argv[]) {
    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    world.addGround();
    auto kinova = world.addArticulatedSystem(std::string(URDF_RSC_DIR) + "kinova/urdf/kinova.urdf");

    /// kinova joint PD controller
    Eigen::VectorXd jointNominalConfig(kinova->getGeneralizedCoordinateDim()), jointVelocityTarget(kinova->getDOF());
    jointNominalConfig << 0.0, 2.76, -1.57, 0.0, 2.0, 0.0;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
    jointPgain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
    jointDgain << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;

    kinova->setGeneralizedCoordinate(jointNominalConfig);
    kinova->setGeneralizedForce(Eigen::VectorXd::Zero(kinova->getDOF()));
    kinova->setPdGains(jointPgain, jointDgain);
    kinova->setPdTarget(jointNominalConfig, jointVelocityTarget);
    kinova->setName("kinova");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.setMap("simple");
    server.launchServer();
    server.focusOn(kinova);

    /// connect to RBDL
    RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
    std::string modelFile = std::string(URDF_RSC_DIR) + "kinova/urdf/kinova.urdf";
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, false);
    std::cout << "Degree of freedom overview : " << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*model);
    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*model);
    std::cout << "q : " << model->q_size << ", qdot : " << model->qdot_size << std::endl;

    for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
    }

    std::cout<<"mass "<<kinova->getMassMatrix()[0]<<std::endl;

    server.killServer();
}
