
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>

std::string rbdlSourcePath = URDF_RSC_DIR;
RigidBodyDynamics::Model model;
Eigen::VectorXd Q;
Eigen::VectorXd QDot;
Eigen::VectorXd Tau;
Eigen::VectorXd QDDot;

void getModelFromURDF(std::string urdfFileName, bool bBaseType)
{
    std::string modelFile = rbdlSourcePath;
    modelFile.append(urdfFileName);
    std::cout << "get urdf" << std::endl;
    /*
     * URDFReadFromFile
     * fileName is start from home directory
     * bBaseType true is floating base, false is fixed base
     */
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), &model, bBaseType);
    std::cout << "check: " << modelLoaded << std::endl;
}

void Test_DOF()
{
    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(model);

    std::cout << "Name body origins overview:"<<std::endl;
    std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(model);

}

void Test_ForwardDynamics(Eigen::VectorXd newQ, Eigen::VectorXd newTau)
{
    Q = newQ;
    Tau = newTau;

    std::cout << "size(Q, QDot, QDDot, Tau): ( " << Q.size() << ", " << QDot.size() << ", " << QDDot.size() << ", " << Tau.size() << ")" << std::endl;

    std::cout << "desired:" << std::endl << "Q:" << std::endl << Q << std::endl << "QDot:" << std::endl << QDot << std::endl << "QDDot:" << std::endl << QDDot << std::endl;
    std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);

    std::cout << QDDot.transpose() << std::endl;
}

void Test_InverseDynamics()
{
    //double bar base code
    Q << -30 * 3.141592 / 180, -2 * 30 * 3.141592 / 180, -(90 - 30) * 3.141592 / 180;
    std::cout << "Use Inverse dynamics get desired Tau for zero qDDot in desired Q, QDot:" << std::endl;
    std::cout << "desired:" << std::endl << "Q:" << std::endl << Q << std::endl << "QDot:" << std::endl << QDot << std::endl << "QDDot:" << std::endl << QDDot << std::endl;
    std::cout << "Tau: " << std::endl << Tau << std::endl;
    RigidBodyDynamics::InverseDynamics(model, Q, QDot, QDDot, Tau);

    std::cout << Tau.transpose() << std::endl;
}

void Test_MassMatrix(Eigen::VectorXd Q)
{
    Eigen::MatrixXd H;
    H = Eigen::MatrixXd(19,19);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, Q, H, true);
    std::cout<<"Mass matrix :"<<std::endl<<H<<std::endl;
}

void fixedBaseExample()
{
    getModelFromURDF("camel_quad_planar/urdf/camel_quad_planar_stl.urdf",false);
    Test_DOF();
}

void floatingBaseExample()
{
    Q = Eigen::VectorXd(19);
    Q.setZero();
    Q[18] = 1;
    getModelFromURDF("canine/urdf/canineV1.urdf",true);
    Test_DOF();
    Test_MassMatrix(Q);
}

int main()
{
    floatingBaseExample();
    return 0;
}
