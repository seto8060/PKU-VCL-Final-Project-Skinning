#include "SkinCPU.h"
#include <cassert>

namespace VCX::Labs::FinalAnimation {
    DualQuat Mat4ToDualQuat(const glm::mat4& m) {
        glm::quat r = glm::quat_cast(m);
    
        glm::vec3 t = glm::vec3(m[3]);
        glm::quat d(
            0.0f,
            0.5f * ( t.x * r.w + t.y * r.z - t.z * r.y ),
            0.5f * (-t.x * r.z + t.y * r.w + t.z * r.x ),
            0.5f * ( t.x * r.y - t.y * r.x + t.z * r.w )
        );
    
        return { r, d };
    }
    void SkinCPU(const std::vector<glm::vec3>& restPositions, const FBXSkin& skin, const GlobalPose& globalPose, std::vector<glm::vec3>& outSkinnedPositions, bool useDualQuat){
        const int vertexCount = (int)restPositions.size();
        const int jointCount  = (int)globalPose.transform.size();

        assert((int)skin.weights.size() == vertexCount);
        assert((int)skin.invBindGlobal.size() == jointCount);

        outSkinnedPositions.resize(vertexCount);

        std::vector<DualQuat> jointDQ;
        std::vector<glm::mat4> jointMat;

        // 1) precompute joint matrices: G * invBind
        if (useDualQuat) {
            jointDQ.resize(jointCount);
            for (int j = 0; j < jointCount; ++j) {
                glm::mat4 M = globalPose.transform[j] * skin.invBindGlobal[j];
                jointDQ[j] = Mat4ToDualQuat(M);
            }
        } else {
            jointMat.resize(jointCount);
            for (int j = 0; j < jointCount; ++j) jointMat[j] = globalPose.transform[j] * skin.invBindGlobal[j];
        }

        // 2) skin vertices
        if (useDualQuat) {
            for (int v = 0; v < vertexCount; ++v) {
                DualQuat dqBlend;
                dqBlend.real = glm::quat(0,0,0,0);
                dqBlend.dual = glm::quat(0,0,0,0);
                const auto& vw = skin.weights[v];
                if (vw.empty()) {
                    outSkinnedPositions[v] = restPositions[v];
                    continue;
                }
            
                glm::quat ref = jointDQ[ vw[0].joint ].real;
            
                for (const auto& w : vw) {
                    const DualQuat& dq = jointDQ[w.joint];
            
                    float sign = (glm::dot(ref, dq.real) < 0.0f) ? -1.0f : 1.0f;
            
                    dqBlend.real += w.weight * sign * dq.real;
                    dqBlend.dual += w.weight * sign * dq.dual;
                }
            
                float len = glm::length(dqBlend.real);
                dqBlend.real /= len;
                dqBlend.dual /= len;
            
                glm::vec3 v0 = restPositions[v];
                glm::quat transQuat = dqBlend.dual * glm::conjugate(dqBlend.real);
                glm::vec3 t = 2.0f * glm::vec3(transQuat.x, transQuat.y, transQuat.z);
            
                glm::vec3 rotated = dqBlend.real * v0;
                outSkinnedPositions[v] = rotated + t;
            }
        } 
        else {
            for (int v = 0; v < vertexCount; ++v) {
                glm::vec4 rest(restPositions[v], 1.0f);
                glm::vec4 skinned(0.0f);

                for (const auto& w : skin.weights[v]) {
                    assert(w.joint < jointMat.size());
                    skinned += w.weight * (jointMat[w.joint] * rest);
                }

                outSkinnedPositions[v] = glm::vec3(skinned);
            }
        }
    }
}