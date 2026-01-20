#include "Labs/Final-Animation/CaseFinalAnimation.h"
#include "Labs/Final-Animation/Animation/BVH/BVHLoader.h"
#include "Labs/Final-Animation/Animation/Pose/BuildLocalPose.h"
#include "Labs/Final-Animation/Animation/Pose/FK.h"
#include "Labs/Final-Animation/Animation/FBX/FBXLoader.h"
#include "Labs/Final-Animation/Animation/FBX/SkeletonBuilder.h"
#include "Labs/Final-Animation/Animation/Retarget/Retargeter.h"
#include "Labs/Final-Animation/Animation/Retarget/FBXAnimSkeleton.h"
#include "Labs/Final-Animation/Animation/FBX/SkinCPU.h"

#include "Labs/Common/ImGuiHelper.h"

#include "Engine/GL/Texture.hpp"
#include "Engine/GL/RenderItem.h"
#include "Engine/prelude.hpp"
#include <iostream>

namespace VCX::Labs::FinalAnimation {
    static glm::quat SafeNormalize(const glm::quat& q) {
        glm::quat r = q;
        float len = glm::length(r);
        if (len <= 1e-8f) return glm::quat(1,0,0,0);
        return r / len;
    }
    static void PrintVec3(const char* tag, const glm::vec3& v) {std::cout << tag << " = (" << v.x << ", " << v.y << ", " << v.z << ")\n";}
    static void PrintQuat(const char* tag, const glm::quat& q) {glm::quat qq = q; if (qq.w < 0) qq = -qq; std::cout << tag << " = (x=" << qq.x << ", y=" << qq.y << ", z=" << qq.z << ", w=" << qq.w << ")\n";}
    static glm::quat RotFromMat4(const glm::mat4& M) {return SafeNormalize(glm::quat_cast(glm::mat3(M)));}
    static glm::vec3 PosFromMat4(const glm::mat4& M) {return glm::vec3(M[3][0], M[3][1], M[3][2]);}

    static int FindFallbackAnimJoint(int rawJoint, const FBXSkeleton& rawSkel, const std::vector<int>& rawToAnim) {
        int j = rawJoint;
        while (j != -1) {
            if (j >= 0 && j < (int)rawToAnim.size()) {
                int animJ = rawToAnim[j];
                if (animJ >= 0)
                    return animJ;
            }
            j = rawSkel.joints[j].parent;
        }
        return -1;
    }
    
    static void RemapSkinToAnimSkeleton(FBXAsset& fbx, const Skeleton& animSkel, const FBXAnimSkeletonBuildResult& built) {
        const FBXSkeleton& rawSkel = fbx.skeleton;
    
        // 1. rebuild invBindGlobal to match anim skeleton indexing
        std::vector<glm::mat4> newInvBind(animSkel.joints.size(), glm::mat4(1.0f));
    
        for (int animIdx = 0; animIdx < (int)animSkel.joints.size(); ++animIdx) {
            int rawIdx = (animIdx >= 0 && animIdx < (int)built.animToRaw.size()) ? built.animToRaw[animIdx] : -1;
    
            if (rawIdx >= 0 && rawIdx < (int)fbx.skin.invBindGlobal.size()) newInvBind[animIdx] = fbx.skin.invBindGlobal[rawIdx];
            else {
                int p = animSkel.joints[animIdx].parent;
                newInvBind[animIdx] = (p >= 0) ? newInvBind[p] : glm::mat4(1.0f);
            }
        }
    
        fbx.skin.invBindGlobal = std::move(newInvBind);
    
        // 2. remap skin weights: raw -> anim + fallback
        for (auto& vwList : fbx.skin.weights) {
            std::vector<SkinWeight> newList;
            newList.reserve(vwList.size());
    
            for (auto w : vwList) {
                int rawJoint = (int)w.joint;
                int animJoint = (rawJoint >= 0 && rawJoint < (int)built.rawToAnim.size()) ? built.rawToAnim[rawJoint] : -1;
    
                // fallback: find the first AnimSkeleton joint
                if (animJoint < 0) animJoint = FindFallbackAnimJoint(rawJoint, rawSkel, built.rawToAnim);
                if (animJoint < 0 || animJoint >= (int)animSkel.joints.size()) continue;
                w.joint = (uint32_t)animJoint;
                newList.push_back(w);
            }
    
            if (newList.empty()) newList.push_back({0, 1.0f});
    
            float sum = 0.f;
            for (auto& w : newList) sum += w.weight;
            if (sum > 1e-8f) for (auto& w : newList) w.weight /= sum;
            vwList = std::move(newList);
        }
    }
    
    static void DumpSkeletonBrief(const Skeleton& skel, const char* title) {
        std::cout << "====== " << title << " ======\n";
        for (int i = 0; i < (int)skel.joints.size(); ++i) {
            auto const& j = skel.joints[i];
            std::cout << i << ": " << j.name
                      << " parent=" << j.parent
                      << " offset=(" << j.offset.x << "," << j.offset.y << "," << j.offset.z << ")\n";
        }
        std::cout << "=============================\n";
    }
    static Skeleton BuildSkeleton(const BVH::BVHData& bvh) {
        Skeleton skeleton;
        skeleton.joints.resize(bvh.joints.size());
        for (size_t i = 0; i < bvh.joints.size(); ++i) {
            skeleton.joints[i].name = bvh.joints[i].name;
            skeleton.joints[i].parent = bvh.joints[i].parent;
            skeleton.joints[i].offset = bvh.joints[i].offset;
        }
        return skeleton;
    }

    static float EstimateScale_LegLength(const VCX::Labs::FinalAnimation::Skeleton& bvhSkel, const VCX::Labs::FinalAnimation::Skeleton& fbxSkel){
        auto find = [&](const Skeleton& s, const std::string& name)->int{
            for (int i=0;i<(int)s.joints.size();++i) if (s.joints[i].name==name) return i;
            return -1;
        };
    
        int bU = find(bvhSkel, "UpperLeg_L");
        int bL = find(bvhSkel, "LowerLeg_L");
        int bF = find(bvhSkel, "Foot_L");
    
        int fU = find(fbxSkel, "mixamorig5:LeftUpLeg");
        int fL = find(fbxSkel, "mixamorig5:LeftLeg");
        int fF = find(fbxSkel, "mixamorig5:LeftFoot");
    
        auto len3 = [](glm::vec3 a, glm::vec3 b){ return glm::length(a)+glm::length(b); };
    
        if (bU<0||bL<0||bF<0||fU<0||fL<0||fF<0) return 1.0f;
    
        float bvhLen = glm::length(bvhSkel.joints[bL].offset) + glm::length(bvhSkel.joints[bF].offset);
        float fbxLen = glm::length(fbxSkel.joints[fL].offset) + glm::length(fbxSkel.joints[fF].offset);
    
        if (bvhLen < 1e-6f) return 1.0f;
        return fbxLen / bvhLen;
    }

    CaseFinalAnimation::CaseFinalAnimation():Program({ Engine::GL::SharedShader("assets/shaders/mesh_min.vert"),
                                                      Engine::GL::SharedShader("assets/shaders/mesh_min.frag") }) {
        BVH = BVH::BVHLoader::Load("assets/BVH/01_01.bvh");
        fbx = FBXLoader::Load("assets/FBX/Timmy.fbx");
        skeleton = BuildSkeleton(*BVH);
        ReloadBVH = true;
        ReloadFBX = true;
        CameraManager.AutoRotate = false;
        CameraManager.Save(Camera);
    }

    std::string_view const CaseFinalAnimation::GetName() { return "Final Animation";}

    void CaseFinalAnimation::OnSetupPropsUI() {
        ImGui::Text("BVH loader");
        
        if (ImGui::BeginCombo("File", BVHFiles[CurrentBVHIndex].c_str())) {
            for (int i = 0; i < BVHFiles.size(); i++) {
                bool isSelected = CurrentBVHIndex == i;
                if (ImGui::Selectable(BVHFiles[i].c_str(), isSelected)) {
                    CurrentBVHIndex = i;
                    ReloadBVH = true;
                }
                if (isSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::Text("FBX loader");

        if (ImGui::BeginCombo("FBX File", FBXFiles[CurrentFBXIndex].c_str())) {
            for (int i = 0; i < FBXFiles.size(); i++) {
                bool isSelected = (CurrentFBXIndex == i);
                if (ImGui::Selectable(FBXFiles[i].c_str(), isSelected)) {
                    CurrentFBXIndex = i;
                    ReloadFBX = true;
                }
                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::Separator();

        ImGui::Text("Render Mode");
        ImGui::RadioButton("BVH Stickman", (int*)&renderMode, (int)RenderMode::BVHStickman);
        ImGui::RadioButton("Skinned Mesh", (int*)&renderMode, (int)RenderMode::SkinnedMesh);
        if (renderMode == RenderMode::SkinnedMesh) {
            ImGui::Indent();
            ImGui::Checkbox("Use Dual Quaternion Skinning", &UseDualQuat);
            ImGui::Unindent();
        }
    }

    Common::CaseRenderResult CaseFinalAnimation::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        bool ShowBVHStickman = renderMode == RenderMode::BVHStickman;
        bool ShowSkinnedMesh = renderMode == RenderMode::SkinnedMesh;

        static float lastTime = 0.0f;
        float now = ImGui::GetTime();
        float deltaTime = now - lastTime;
        lastTime = now;

        // 1. Reload BVH
        if (ReloadBVH) {
            ReloadBVH = false;
            try {
                std::string path = "assets/BVH/" + BVHFiles[CurrentBVHIndex];
                BVH = BVH::BVHLoader::Load(path);
                skeleton = BuildSkeleton(*BVH);
                BVHStickmanRenderer.BuildFromSkeleton(skeleton);
                ReloadRetarget = true;
            }
            catch (const std::exception& e) {
                std::cerr << "Error loading BVH: " << e.what() << "\n";
                BVH.reset();
            }
        }
        if (!BVH.has_value()) {
            Common::ImageRGB image = Common::CreateCheckboardImageRGB(400, 400);
            _textures[0].Update(image);
            return Common::CaseRenderResult {
                .Fixed = true,
                .Image = _textures[0],
                .ImageSize = {400, 400},
            };
        }

        // 2. Reload FBX
        if (ReloadFBX) {
            ReloadFBX = false;
            try {
                fbx = FBXLoader::Load("assets/FBX/" + FBXFiles[CurrentFBXIndex]);

                fbxAnimBuilt = FBXAnimSkeleton::Build(fbx.skeleton);
                fbxAnimSkeleton = fbxAnimBuilt.skeleton;
                fbxPose = FBXAnimSkeleton::MakeBindPoseLocalPose(fbxAnimBuilt);
                RemapSkinToAnimSkeleton(fbx, fbxAnimSkeleton, fbxAnimBuilt);
                GlobalPose bindG = FK::Compute(fbxAnimSkeleton, fbxPose);
                fbx.skin.invBindGlobal.resize(fbxAnimSkeleton.joints.size());
                for (int j = 0; j < (int)fbxAnimSkeleton.joints.size(); ++j) fbx.skin.invBindGlobal[j] = glm::inverse(bindG.transform[j]);

                FBXStickmanRenderer.BuildFromSkeleton(fbxAnimSkeleton);

                // upload mesh
                meshItem.UpdateVertexBuffer(
                    "position",
                    Engine::make_span_bytes(
                        std::span<const glm::vec3>(fbx.mesh.positions)
                    )
                );
                meshItem.UpdateVertexBuffer(
                    "normal",
                    Engine::make_span_bytes(
                        std::span<const glm::vec3>(fbx.mesh.normals)
                    )
                );
                meshItem.UpdateElementBuffer(fbx.mesh.indices);
                ReloadRetarget = true;
            }
            catch (const std::exception& e) {
                std::cerr << "Error loading FBX: " << e.what() << "\n";
            }
            
        }
        
        //3. Advance frame
        if (Playing) {
            FrameTime += deltaTime;
            float frameDuration = BVH->frameTime;
            while (FrameTime >= frameDuration) {
                FrameTime -= frameDuration;
                CurrentFrame = (CurrentFrame + 1) % BVH->frameCount;
            }
        }

        // 4.Setup
        FrameBuffer.Resize(desiredSize);
        gl_using(FrameBuffer);
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        CameraManager.Update(Camera);
        Program.GetUniforms().SetByName("u_Projection", Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        Program.GetUniforms().SetByName("u_View", Camera.GetViewMatrix());

        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(6.0f);
        
        // 5. BVH pose -> global pose
        LocalPose bvhRef = BuildLocalPose(*BVH, skeleton, 0);
        LocalPose localPose = BuildLocalPose(*BVH, skeleton, CurrentFrame);
        localPose.translation[0] = glm::vec3(0.0f, 0.0f, 0.0f);
        GlobalPose bvhGlobal = FK::Compute(skeleton, localPose);

        // Scale & Center BVH
        for (auto& t : bvhGlobal.position) t *= 6.0f;
        glm::vec3 mn(FLT_MAX), mx(-FLT_MAX);
        for (auto &p : bvhGlobal.position) { mn = glm::min(mn, p); mx = glm::max(mx, p); }
        glm::vec3 c = (mn + mx) * 0.5f;
        for (auto &p : bvhGlobal.position) p -= c;

        // 6.Retarget init
        if (ReloadRetarget) {
            ReloadRetarget = false;
            retargeter = std::make_unique<Retargeter>(skeleton, fbxAnimSkeleton, fbxAnimBuilt.bindLocalRot);
            retargeter->BuildDefaultMapping_MixamoLike();
            retargeter->ComputeSemanticLocalOffsets(bvhRef, fbxPose);
        }

        // 7. Retarget Current Frame
        LocalPose fbxLocal = fbxPose;
        float rootTranslationScale = EstimateScale_LegLength(skeleton, fbxAnimSkeleton);
        fbxLocal = retargeter->RetargetFrame(localPose, rootTranslationScale, fbxPose);

        // 8. FK for FBX skeleton
        GlobalPose fbxGlobal = FK::Compute(fbxAnimSkeleton, fbxLocal);
        SkinCPU(fbx.mesh.positions, fbx.skin, fbxGlobal, fbx.mesh.skinnedPositions, UseDualQuat);

        mn = glm::vec3(FLT_MAX), mx = glm::vec3(-FLT_MAX);
        for (auto &p : fbx.mesh.skinnedPositions) {
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
        }
        c = (mn + mx) * 0.5f;
        for (auto &p : fbx.mesh.skinnedPositions) p -= c;
        for (auto &p : fbx.mesh.skinnedPositions) p += glm::vec3(0.6f, 0.0f, 0.0f);

        // 9. Render
        Program.GetUniforms().SetByName("u_Model", glm::scale(glm::mat4(1), glm::vec3(0.02f)));

        Program.GetUniforms().SetByName("u_Color", glm::vec3(0.9f));
        Program.GetUniforms().SetByName("u_LightDir", glm::normalize(glm::vec3(-1,-1,-1)));
        meshItem.UpdateVertexBuffer(
            "position",
            Engine::make_span_bytes(
                std::span<const glm::vec3>(fbx.mesh.skinnedPositions)
            )
        );
        if (renderMode == RenderMode::BVHStickman) {
            Program.GetUniforms().SetByName("u_Color", glm::vec3(0.2f, 1.0f, 0.2f));
            BVHStickmanRenderer.Render(Program, bvhGlobal, skeleton);
        }
        else meshItem.Draw({Program.Use()});

        // 10. recover the default state
        glLineWidth(1.0f);
        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_DEPTH_TEST);
        
        return Common::CaseRenderResult {
            .Fixed = false,
            .Flipped = true,
            .Image = FrameBuffer.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }
    void CaseFinalAnimation::OnProcessInput(ImVec2 const & pos) {
        CameraManager.ProcessInput(Camera, pos);
    }
}