#include "FBXAnimSkeleton.h"
#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"

#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <cctype>
#include <iostream>

namespace VCX::Labs::FinalAnimation {

    static inline bool Contains(const std::string& s, const char* sub) {return s.find(sub) != std::string::npos;}

    std::string FBXAnimSkeleton::BaseName(const std::string& name) {
        auto pos = name.find_last_of(':');
        if (pos == std::string::npos) return name;
        return name.substr(pos + 1);
    }

    bool FBXAnimSkeleton::IsFingerLike(const std::string& base) {
        return Contains(base, "Thumb") || Contains(base, "Index") || Contains(base, "Middle") || Contains(base, "Ring")  || Contains(base, "Pinky");
    }

    bool FBXAnimSkeleton::IsMainSemanticJoint(const std::string& base) {
        // only keep "human semantic joints", exclude scene root
        static const std::unordered_set<std::string> keep = {
            "Hips",

            "Spine", "Spine1", "Spine2",

            "Neck", "Head",

            "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",

            "RightShoulder", "RightArm", "RightForeArm", "RightHand",

            "LHipJoint", "RHipJoint",

            "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",

            "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase",
        };
        return keep.count(base) > 0;
    }


    std::vector<int> FBXAnimSkeleton::SelectSemanticJoints(const FBXSkeleton& raw) {
        std::vector<int> selected;
        selected.reserve(raw.joints.size());

        for (int i = 0; i < (int)raw.joints.size(); ++i) {
            const std::string& n = raw.joints[i].name;
            if (Contains(n, "$AssimpFbx$")) continue;
            std::string base = BaseName(n);
            if (IsFingerLike(base)) continue;
            if (!IsMainSemanticJoint(base)) continue;
            selected.push_back(i);
        }
        return selected;
    }

    glm::vec3 FBXAnimSkeleton::ExtractTranslation(const glm::mat4& m) {return glm::vec3(m[3][0], m[3][1], m[3][2]);}

    glm::quat FBXAnimSkeleton::ExtractRotationOrthonormalized(const glm::mat4& m) {
        // remove scale/shear: orthonormalize 3x3
        glm::vec3 x = glm::vec3(m[0]);
        glm::vec3 y = glm::vec3(m[1]);
        glm::vec3 z = glm::vec3(m[2]);

        if (glm::dot(x, x) < 1e-12f) x = glm::vec3(1,0,0);
        if (glm::dot(y, y) < 1e-12f) y = glm::vec3(0,1,0);

        x = glm::normalize(x);
        y = y - x * glm::dot(x, y);
        if (glm::dot(y, y) < 1e-12f) y = glm::vec3(0,1,0);
        y = glm::normalize(y);
        z = glm::normalize(glm::cross(x, y));

        glm::mat3 R;
        R[0] = x; R[1] = y; R[2] = z;

        return glm::quat_cast(R);
    }

    FBXAnimSkeletonBuildResult FBXAnimSkeleton::Build(const FBXSkeleton& raw) {
        auto selected = SelectSemanticJoints(raw);
        return Build(raw, selected);
    }

    FBXAnimSkeletonBuildResult FBXAnimSkeleton::Build(const FBXSkeleton& raw, const std::vector<int>& selectedRawJoints) {
        FBXAnimSkeletonBuildResult out;

        const int RawN = (int)raw.joints.size();
        out.rawToAnim.assign(RawN, -1);

        std::vector<glm::vec3> worldPos(raw.joints.size());

        for (int i = 0; i < raw.joints.size(); ++i) {
            glm::mat4 M = raw.joints[i].bindPose;
            int p = raw.joints[i].parent;
            while (p != -1) {
                M = raw.joints[p].bindPose * M;
                p = raw.joints[p].parent;
            }
            worldPos[i] = glm::vec3(M[3]); // world translation
        }

        // 1. fill parent chain: required = selected + ancestors
        std::unordered_set<int> required;
        required.reserve(selectedRawJoints.size() * 2);

        for (int idx : selectedRawJoints) {
            int cur = idx;
            while (cur != -1) {
                if (!required.insert(cur).second) break;
                cur = raw.joints[cur].parent;
            }
        }

        // 2. generate "parent first" order ordered
        std::vector<int> ordered;
        ordered.reserve(required.size());
        std::vector<char> vis(RawN, 0);

        std::function<void(int)> dfs = [&](int u) {
            if (u < 0 || u >= RawN) return;
            if (!required.count(u)) return;
            if (vis[u]) return;
            vis[u] = 1;

            int p = raw.joints[u].parent;
            if (p != -1) dfs(p);

            if (!Contains(raw.joints[u].name, "$AssimpFbx$")) ordered.push_back(u);
        };
        for (int idx : selectedRawJoints) dfs(idx);
        for (int u : required) dfs(u);

        // 3. raw<->anim mapping
        out.animToRaw.resize((int)ordered.size(), -1);
        out.rawToAnim.assign((int)raw.joints.size(), -1);

        // 4. build skeleton + bindLocalRot
        auto findSemanticParent = [&](int rawIdx) -> int {
            int p = raw.joints[rawIdx].parent;
            while (p != -1) {
                int animP = out.rawToAnim[p];
                if (animP != -1) {
                    return animP;   // 找到最近的 anim joint
                }
                p = raw.joints[p].parent; // 继续往上爬
            }
            return -1;
        };

        std::vector<glm::mat4> rawGlobal(raw.joints.size(), glm::mat4(1.0f));
        for (int i = 0; i < (int)raw.joints.size(); ++i) {
            glm::mat4 G = raw.joints[i].bindPose; // local
            int p = raw.joints[i].parent;
            while (p != -1) {
                G = raw.joints[p].bindPose * G;
                p = raw.joints[p].parent;
            }
            rawGlobal[i] = G;
        }

        // 5. build skeleton + bindLocalRot
        out.skeleton.joints.clear();
        out.bindLocalRot.clear();
        out.animToRaw.clear();

        out.skeleton.joints.reserve(ordered.size() + 2); // +2 for hip pivots
        out.bindLocalRot.reserve(ordered.size() + 2);
        out.animToRaw.reserve(ordered.size() + 2);

        int animHips = -1;
        int animLeftUpLeg = -1;
        int animRightUpLeg = -1;

        auto PushJoint = [&](const std::string& name,int parent,const glm::vec3& offset,const glm::quat& bindRot,int rawIdxOrMinus1) -> int {
            Joint j;
            j.name   = name; j.parent = parent; j.offset = offset;
            int idx = (int)out.skeleton.joints.size(); 
            out.skeleton.joints.push_back(j);
            out.bindLocalRot.push_back(bindRot); 
            out.animToRaw.push_back(rawIdxOrMinus1);
            if (rawIdxOrMinus1 >= 0) out.rawToAnim[rawIdxOrMinus1] = idx;
            return idx;
        };

        for (int k = 0; k < (int)ordered.size(); ++k) {
            int rawIdx = ordered[k];
            const FBXJoint& rj = raw.joints[rawIdx];
            std::string base = BaseName(rj.name);

            int parentAnim = findSemanticParent(rawIdx);

            auto ComputeOffset = [&](int rawChild, int parentAnimIndex) -> glm::vec3 {
                if (parentAnimIndex < 0) return glm::vec3(0.0f);

                int rawP = out.animToRaw[parentAnimIndex];

                if (rawP < 0) {
                    if (animHips < 0) return glm::vec3(0.0f);
                    rawP = out.animToRaw[animHips];
                }

                return worldPos[rawChild] - worldPos[rawP];
            };

            if (base == "Hips") {
                glm::vec3 offset = ComputeOffset(rawIdx, parentAnim);
                glm::quat bindRot = ExtractRotationOrthonormalized(rj.bindPose);
                int idx = PushJoint(rj.name, parentAnim, offset, bindRot, rawIdx);
                animHips = idx;
                continue;
            }

            // handle LeftUpLeg and RightUpLeg: build pivot joints
            if (base == "LeftUpLeg") {
                if (animHips < 0) throw std::runtime_error("Build: Hips must appear before LeftUpLeg");
            
                glm::vec3 off = ComputeOffset(rawIdx, animHips);
            
                int leftPivot = PushJoint("LeftHipPivot", animHips, off, glm::quat(1,0,0,0), -1);
            
                parentAnim = leftPivot;
                glm::vec3 offset  = glm::vec3(0.0f);
                glm::quat bindRot = ExtractRotationOrthonormalized(rj.bindPose);
            
                animLeftUpLeg = PushJoint(rj.name, parentAnim, offset, bindRot, rawIdx);
                continue;
            }
            
            if (base == "RightUpLeg") {
                if (animHips < 0) throw std::runtime_error("Build: Hips must appear before RightUpLeg");
                glm::vec3 off = ComputeOffset(rawIdx, animHips);
                int rightPivot = PushJoint("RightHipPivot", animHips, off, glm::quat(1,0,0,0), -1);
            
                parentAnim = rightPivot;
                glm::vec3 offset  = glm::vec3(0.0f);
                glm::quat bindRot = ExtractRotationOrthonormalized(rj.bindPose);
                animRightUpLeg = PushJoint(rj.name, parentAnim, offset, bindRot, rawIdx);
                continue;
            }

            glm::vec3 offset = ComputeOffset(rawIdx, parentAnim);
            glm::quat bindRot = ExtractRotationOrthonormalized(rj.bindPose);
            PushJoint(rj.name, parentAnim, offset, bindRot, rawIdx);
        }

        if (animHips < 0) throw std::runtime_error("Build: missing Hips");
        return out;
    }

    LocalPose FBXAnimSkeleton::MakeBindPoseLocalPose(const FBXAnimSkeletonBuildResult& built) {
        LocalPose pose;
        int N = (int)built.skeleton.joints.size();
        pose.translation.resize(N);
        pose.rotation.resize(N);

        for (int i = 0; i < N; ++i) {
            pose.translation[i] = built.skeleton.joints[i].offset;
            pose.rotation[i]    = built.bindLocalRot[i];
        }
        return pose;
    }

}
