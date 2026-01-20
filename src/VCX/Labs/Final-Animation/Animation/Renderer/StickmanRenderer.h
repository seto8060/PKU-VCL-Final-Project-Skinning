#pragma once
#include <vector>
#include <array>

#include <glm/glm.hpp>

#include "Engine/GL/RenderItem.h"
#include "Engine/GL/Program.h"

#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"
#include "Labs/Final-Animation/Animation/Pose/Pose.h"

namespace VCX::Labs::FinalAnimation {
    class StickmanRenderer {
    public:
        StickmanRenderer();
    
        void BuildFromSkeleton(const Skeleton& skeleton);
        void Render(Engine::GL::UniqueProgram& program, const GlobalPose& pose, const Skeleton& skeleton);
    
    private:
        enum class JointType {Root, Spine, Head, Arm, Leg, Other};

        JointType ClassifyJoint(const std::string& name) const;
        glm::vec3 ColorOf(JointType type) const;

        Engine::GL::UniqueRenderItem PointItem{
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points
        };

    
        Engine::GL::UniqueIndexedRenderItem LineItem{
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        };

        std::array<glm::vec3, 1> TmpPoint {};
        std::vector<glm::vec3> JointColors;
    };
}