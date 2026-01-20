#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include "Labs/Common/ICase.h"
#include "Labs/Final-Animation/Animation/BVH/BVH.h"
#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"
#include "Labs/Final-Animation/Animation/Renderer/StickmanRenderer.h"
#include "Labs/Final-Animation/Animation/FBX/FBXLoader.h"
#include "Labs/Final-Animation/Animation/FBX/SkeletonBuilder.h"
#include "Labs/Final-Animation/Animation/Retarget/Retargeter.h"
#include "Labs/Final-Animation/Animation/Retarget/FBXAnimSkeleton.h"
#include "Engine/GL/Texture.hpp"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"

#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include <vector>
#include <string>
#include <optional>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/quaternion.hpp>


namespace VCX::Labs::FinalAnimation {

    class CaseFinalAnimation final : public Common::ICase {
    private:
        int CurrentFrame = 0;
        float FrameTime = 0.0f;
        bool Playing = true;

        std::vector<std::string> BVHFiles{
            "01_01.bvh","01_02.bvh","01_03.bvh","01_04.bvh","01_05.bvh","01_06.bvh","01_07.bvh","01_08.bvh","01_09.bvh","01_10.bvh",
            "01_11.bvh","01_12.bvh","01_13.bvh","01_14.bvh"};
        std::vector<std::string> FBXFiles{
            "Timmy.fbx", "Romero.fbx", "Olivia.fbx"
        };
        int CurrentFBXIndex = 0;
        bool ReloadBVH = true;
        int CurrentBVHIndex = 0;
        bool UseDualQuat = false;
        std::optional<BVH::BVHData> BVH;
        Skeleton skeleton; // BVH skeleton

        Engine::GL::UniqueRenderFrame FrameBuffer;
        Engine::GL::UniqueProgram Program;
        StickmanRenderer BVHStickmanRenderer;
        StickmanRenderer FBXStickmanRenderer;

        Engine::Camera Camera { .Eye = glm::vec3(0.0f, 1.5f, 4.0f)};
        Common::OrbitCameraManager CameraManager;

        std::array<Engine::GL::UniqueTexture2D, 2> _textures;
        Engine::GL::UniqueIndexedRenderItem meshItem{
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0)
                .Add<glm::vec3>("normal", Engine::GL::DrawFrequency::Stream, 1),
            Engine::GL::PrimitiveType::Triangles
        };

        bool ReloadFBX = true;
        FBXAsset fbx;
        Skeleton fbxAnimSkeleton; // FBX skeleton
        FBXSkeleton fbxSkeletonFiltered;

        bool ReloadRetarget = true;
        std::unique_ptr<Retargeter> retargeter;

        FBXAnimSkeletonBuildResult fbxAnimBuilt;
        LocalPose fbxPose;
        enum class RenderMode {
            BVHStickman = 0,
            SkinnedMesh = 1,
        };
        RenderMode renderMode = RenderMode::BVHStickman;

    public:
        CaseFinalAnimation();
        ~CaseFinalAnimation() = default;
        std::string_view const GetName() override;

        void OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(
            std::pair<std::uint32_t, std::uint32_t> const desiredSize
        ) override;
        void OnProcessInput(ImVec2 const & pos) override;
    };

}