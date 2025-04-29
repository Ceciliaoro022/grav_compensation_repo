#include "pinocchio_/rrbot_description.hpp"

using namespace pinocchio;
using namespace Eigen;

Model buildMini2RModel() {
    Model model;
    model.name = "Mini2R";

    JointIndex base_joint_id = model.addJoint(
        0,
        JointModelRevoluteUnaligned(Vector3d(1.0, 0.0, 0.0)),
        SE3::Identity(),
        "base_joint");

    model.appendBodyToJoint(
        base_joint_id,
        Inertia(2.0, Vector3d(0.0, 0.0, 1.0), Matrix3d::Identity() * 0.01),
        SE3::Identity());

    JointIndex joint1_id = model.addJoint(
        model.getJointId("base_joint"),
        JointModelRevoluteUnaligned(Vector3d(0.0, 1.0, 0.0)),
        SE3(Quaterniond::Identity(), Vector3d(0.0, 0.1, 1.95)),
        "joint1");

    model.appendBodyToJoint(
        joint1_id,
        Inertia(1.0, Vector3d(0.0, 0.0, 0.45), Matrix3d::Identity() * 0.01),
        SE3::Identity());

    JointIndex joint2_id = model.addJoint(
        model.getJointId("joint1"),
        JointModelRevoluteUnaligned(Vector3d(0.0, 1.0, 0.0)),
        SE3(Quaterniond::Identity(), Vector3d(0.0, 0.1, 0.95)),
        "joint2");

    model.appendBodyToJoint(
        joint2_id,
        Inertia(1.0, Vector3d(0.0, 0.0, 0.45), Matrix3d::Identity() * 0.01),
        SE3::Identity());

    return model;
}
