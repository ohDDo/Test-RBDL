
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

void Test_ForwardDynamics()
{
    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);

    std::cout << "Forward Dynamics" << std::endl;
    std::cout << "QDDot: "<< QDDot.transpose() << std::endl<< std::endl;
}

void Test_InverseDynamics()
{
    RigidBodyDynamics::InverseDynamics(model, Q, QDot, QDDot, Tau, NULL);

    std::cout << "Inverse dynamics" << std::endl;
    std::cout << "Tau :"<< Tau.transpose() << std::endl<< std::endl;
}

void Test_MassMatrix(Eigen::VectorXd Q)
{
    Eigen::MatrixXd H;
    H = Eigen::MatrixXd(19,19);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, Q, H, true);
    std::cout<<"Mass matrix :"<<std::endl<<H<<std::endl<< std::endl;
}

void fixedBaseExample()
{
    //multi-link example
    getModelFromURDF("camel_quad_planar/urdf/camel_quad_planar_stl.urdf",false);
    Test_DOF();
    Q = Eigen::VectorXd(5);
    QDot = Eigen::VectorXd(5);
    QDDot = Eigen::VectorXd(5);
    Tau = Eigen::VectorXd(5);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    Tau.setZero();
}

void floatingBaseExample()
{
    //quadruped robot example
    getModelFromURDF("canine/urdf/canineV1.urdf",true);
    Test_DOF();
    Q = Eigen::VectorXd(19);
    QDot = Eigen::VectorXd(18);
    QDDot = Eigen::VectorXd(18);
    Tau = Eigen::VectorXd(18);
    Q << -0.00966787, -2.22733e-05, 0.341271, 0, 0, 0, -0.000316542, 0.846003, -1.57948, 0.00246455, 0.821255, -1.53864, -0.000320557, 0.846237, -1.57953, -0.00247643, 0.821701, -1.53906, 1;
//    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    Tau.setZero();
//    Q[18] = 1;
    Test_MassMatrix(Q);
    Test_InverseDynamics();
    Tau << 0 ,-1.30104e-17,      154.596 ,   -0.543163,      1.72995 ,-2.21334e-16   ,  0.971612    , 0.175192   , -0.174462,     0.973037 ,    0.170559  ,  -0.171324 ,    -1.51477 ,      1.2135,    -0.174427,    -0.973042,     0.170706, -0.171319;
    Test_ForwardDynamics();
}

int main()
{
//    fixedBaseExample();
    floatingBaseExample();
    return 0;
}
