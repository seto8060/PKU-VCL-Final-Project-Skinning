#include "StickmanRenderer.h"

#include "Engine/prelude.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCX::Labs::FinalAnimation {
    StickmanRenderer::StickmanRenderer(): LineItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        ) {}

    void StickmanRenderer::BuildFromSkeleton(const Skeleton& skeleton) {
        std::vector<std::uint32_t> indices;
        indices.reserve((skeleton.GetJointCount() - 1) * 2);
        JointColors.resize(skeleton.GetJointCount());

        const int jointCount = skeleton.GetJointCount();
        for (int i = 1; i < jointCount; ++i) {
            int parent = skeleton.joints[i].parent;
            JointType type = ClassifyJoint(skeleton.joints[i].name);
            JointColors[i] = ColorOf(type);
            if (parent < 0) continue;
            if (i == 0 || parent == 0) continue;
            indices.push_back(static_cast<std::uint32_t>(parent));
            indices.push_back(static_cast<std::uint32_t>(i));
        }

        LineItem.UpdateElementBuffer(indices);
    }

    void StickmanRenderer::Render(Engine::GL::UniqueProgram& program, const GlobalPose& pose, const Skeleton& skeleton) {
        auto span = Engine::make_span_bytes<glm::vec3>(pose.position);

        program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 1.0f, 1.0f));

        LineItem.UpdateVertexBuffer("position", span);

        LineItem.Draw({ program.Use() });

        glPointSize(12.0f);
        for (int i = 0 ; i < skeleton.GetJointCount(); i++) {
            TmpPoint[0] = pose.position[i];
            std::span<const glm::vec3> sp(TmpPoint.data(), 1);
            PointItem.UpdateVertexBuffer("position", Engine::make_span_bytes(sp));
            program.GetUniforms().SetByName("u_Color", JointColors[i]);
            PointItem.Draw({ program.Use() });
        }
        glPointSize(1.0f);
    }

    StickmanRenderer::JointType
    StickmanRenderer::ClassifyJoint(const std::string& name) const {
        if (name.find("Hips") != std::string::npos || name.find("Root") != std::string::npos)
            return JointType::Root;

        if (name.find("Spine") != std::string::npos ||
            name.find("Chest") != std::string::npos)
            return JointType::Spine;

        if (name.find("Neck") != std::string::npos ||
            name.find("Head") != std::string::npos)
            return JointType::Head;

        if (name.find("Arm") != std::string::npos ||
            name.find("Shoulder") != std::string::npos ||
            name.find("Hand") != std::string::npos)
            return JointType::Arm;

        if (name.find("Leg") != std::string::npos ||
            name.find("Foot") != std::string::npos ||
            name.find("Toe") != std::string::npos)
            return JointType::Leg;
        return JointType::Other;
    }

    glm::vec3 StickmanRenderer::ColorOf(JointType type) const {
        switch (type) {
        case JointType::Root:  return {1.f, 1.f, 0.f};   // yellow
        case JointType::Spine: return {1.f, 0.5f, 0.f}; // orange
        case JointType::Head:  return {1.f, 0.f, 0.f};  // red
        case JointType::Arm:   return {0.f, 1.f, 1.f};  // cyan
        case JointType::Leg:   return {0.f, 1.f, 0.f};  // green
        default:               return {0.7f, 0.7f, 0.7f};
        }
    }
}
