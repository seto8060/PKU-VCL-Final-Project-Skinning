#define GLM_ENABLE_EXPERIMENTAL

#include "Retargeter.h"
#include "Labs/Final-Animation/Animation/Pose/FK.h"
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <stdexcept>

namespace VCX::Labs::FinalAnimation {

    glm::quat Retargeter::SafeNormalize(const glm::quat& q) {
        glm::quat r = q;
        float len = glm::length(r);
        if (len <= 1e-8f) return glm::quat(1,0,0,0);
        return r / len;
    }

    glm::quat Retargeter::RotFromMat4(const glm::mat4& M) {return SafeNormalize(glm::quat_cast(glm::mat3(M)));}

    glm::quat Retargeter::RotFromLocal(const glm::quat& q) {return SafeNormalize(q);}

    static std::string NormalizeJointName(const std::string& name){
        size_t pos = name.find(':');
        if (pos != std::string::npos) return name.substr(pos + 1);
        return name;
    }

    Retargeter::Retargeter(const Skeleton& bvhSkel, const Skeleton& fbxAnimSkel,const std::vector<glm::quat>& fbxBindLocalRot)
        : _bvh(bvhSkel), _fbx(fbxAnimSkel), _fbxBindLocalRot(fbxBindLocalRot){
        for (int i = 0; i < (int)_bvh.joints.size(); ++i) _bvhNameToIndex[_bvh.joints[i].name] = i;

        for (int i = 0; i < (int)_fbx.joints.size(); ++i) _fbxNameToIndex[NormalizeJointName(_fbx.joints[i].name)] = i;

        _fbxToBvh.assign(_fbx.joints.size(), -1);

        if (_fbxNameToIndex.count("Hips")) _fbxHips = _fbxNameToIndex.at("Hips");
        if (_fbxNameToIndex.count("LeftHipPivot"))    _fbxLeftHipPivot  = _fbxNameToIndex.at("LeftHipPivot");
        if (_fbxNameToIndex.count("RightHipPivot"))   _fbxRightHipPivot = _fbxNameToIndex.at("RightHipPivot");
    }


    int Retargeter::FindBVH(const std::string& name) const {
        auto it = _bvhNameToIndex.find(name);
        return (it == _bvhNameToIndex.end()) ? -1 : it->second;
    }

    int Retargeter::FindFBX(const std::string& name) const {
        std::string key = NormalizeJointName(name);
        auto it = _fbxNameToIndex.find(key);
        return (it == _fbxNameToIndex.end()) ? -1 : it->second;
    }

    void Retargeter::AddMapByName(const std::string& bvhName, const std::string& fbxName) {
        int b = FindBVH(bvhName);
        int f = FindFBX(fbxName);
        if (b < 0) throw std::runtime_error("Retargeter: BVH joint not found: " + bvhName);
        if (f < 0) throw std::runtime_error("Retargeter: FBX joint not found: " + fbxName);

        _mapping.push_back({b, f});
    }

    void Retargeter::BuildDefaultMapping_MixamoLike() {
        _mapping.clear();
        _semOffsets.clear();
        std::fill(_fbxToBvh.begin(), _fbxToBvh.end(), -1);

        AddMapByName("Hips",       "Hips");

        AddMapByName("LowerBack",  "Spine");
        AddMapByName("Spine",      "Spine1");
        AddMapByName("Spine1",     "Spine2");

        AddMapByName("Neck",       "Neck");
        AddMapByName("Head",       "Head");

        AddMapByName("LeftShoulder","LeftShoulder");
        AddMapByName("LeftArm",     "LeftArm");
        AddMapByName("LeftForeArm", "LeftForeArm");
        AddMapByName("LeftHand",    "LeftHand");

        AddMapByName("RightShoulder","RightShoulder");
        AddMapByName("RightArm",     "RightArm");
        AddMapByName("RightForeArm", "RightForeArm");
        AddMapByName("RightHand",    "RightHand");

        AddMapByName("LeftUpLeg",   "LeftUpLeg");
        AddMapByName("LeftLeg",     "LeftLeg");
        AddMapByName("LeftFoot",    "LeftFoot");
        AddMapByName("LeftToeBase", "LeftToeBase");

        AddMapByName("RightUpLeg",   "RightUpLeg");
        AddMapByName("RightLeg",     "RightLeg");
        AddMapByName("RightFoot",    "RightFoot");
        AddMapByName("RightToeBase", "RightToeBase");

        for (auto const& m : _mapping) _fbxToBvh[m.fbx] = m.bvh;

        _semOffsets.resize(_mapping.size(), glm::quat(1,0,0,0));
    }

    int Retargeter::ResolveBvhSemanticParentForFbx(int fbxIdx) const {
        int p = _fbx.joints[fbxIdx].parent;
        while (p >= 0) {
            int b = _fbxToBvh[p];
            if (b >= 0) return b;
            p = _fbx.joints[p].parent;
        }
        return -1;
    }

    glm::quat Retargeter::ComputeBvhSemanticLocal(const std::vector<glm::mat4>& bvhGlobal,
    int bvhIdx,
                                                int bvhSemanticParent) const
    {
        glm::quat Gs = RotFromMat4(bvhGlobal[bvhIdx]);
        if (bvhSemanticParent < 0) return Gs;

        glm::quat Gp = RotFromMat4(bvhGlobal[bvhSemanticParent]);
        return SafeNormalize(glm::inverse(Gp) * Gs);
    }

    glm::quat Retargeter::ComputeFbxLocalFromGlobal(const std::vector<glm::mat4>& fbxGlobal,
                                                    int fbxIdx) const
    {
        glm::quat Gs = RotFromMat4(fbxGlobal[fbxIdx]);
        int p = _fbx.joints[fbxIdx].parent;
        if (p < 0) return Gs;
        glm::quat Gp = RotFromMat4(fbxGlobal[p]);
        return SafeNormalize(glm::inverse(Gp) * Gs);
    }

    void Retargeter::ComputeSemanticLocalOffsets(const LocalPose& bvhRefPose, const LocalPose& fbxBindPose){
        if (_mapping.empty()) throw std::runtime_error("Retargeter: mapping empty; call BuildDefaultMapping first.");
        if ((int)bvhRefPose.rotation.size() != (int)_bvh.joints.size()) throw std::runtime_error("Retargeter: bvhRefPose rotation size mismatch.");
        if ((int)fbxBindPose.rotation.size() != (int)_fbx.joints.size()) throw std::runtime_error("Retargeter: fbxBindPose rotation size mismatch.");

        // get global pose
        GlobalPose bvhRefG = FK::Compute(_bvh, bvhRefPose);
        GlobalPose fbxBindG = FK::Compute(_fbx, fbxBindPose);

        std::vector<glm::mat4> bvhG = bvhRefG.transform;
        std::vector<glm::mat4> fbxG = fbxBindG.transform;

        _semOffsets.resize(_mapping.size(), glm::quat(1,0,0,0));

        for (int i = 0; i < (int)_mapping.size(); ++i) {
            int b = _mapping[i].bvh, f = _mapping[i].fbx;

            int bvhSemParent = ResolveBvhSemanticParentForFbx(f);// get semantic parent
            glm::quat Rb_sem_ref = ComputeBvhSemanticLocal(bvhG, b, bvhSemParent);// get semantic local
            glm::quat Rf_local_bind = ComputeFbxLocalFromGlobal(fbxG, f);// get fbx local
            _semOffsets[i] = SafeNormalize(Rf_local_bind * glm::inverse(Rb_sem_ref));// compute offset
        }
    }

    LocalPose Retargeter::RetargetFrame(const LocalPose& bvhPose, float rootTranslationScale, const LocalPose& fbxBindPose) const {
        if ((int)bvhPose.rotation.size() != (int)_bvh.joints.size()) throw std::runtime_error("Retargeter: bvhPose rotation size mismatch.");
        if ((int)fbxBindPose.rotation.size() != (int)_fbx.joints.size()) throw std::runtime_error("Retargeter: fbxBindPose rotation size mismatch.");
        if (_mapping.empty() || _semOffsets.size() != _mapping.size()) throw std::runtime_error("Retargeter: offsets not computed; call ComputeSemanticLocalOffsets.");

        // copy bind local & global pose
        LocalPose out = fbxBindPose;
        GlobalPose bvhCurG = FK::Compute(_bvh, bvhPose);
        const std::vector<glm::mat4>& bvhG = bvhCurG.transform;

        // write local rotation
        for (int i = 0; i < (int)_mapping.size(); ++i) {
            int b = _mapping[i].bvh, f = _mapping[i].fbx;

            int bvhSemParent = ResolveBvhSemanticParentForFbx(f);
            glm::quat Rb_sem_cur = ComputeBvhSemanticLocal(bvhG, b, bvhSemParent);
            // compute target local
            out.rotation[f] = SafeNormalize(_semOffsets[i] * Rb_sem_cur);
        }

        // keep identity
        if (_fbxLeftHipPivot >= 0)  out.rotation[_fbxLeftHipPivot]  = glm::quat(1,0,0,0);
        if (_fbxRightHipPivot >= 0) out.rotation[_fbxRightHipPivot] = glm::quat(1,0,0,0);

        // root translation
        int bvhHips = FindBVH("Hips");
        if (bvhHips >= 0 && _fbxHips >= 0) out.translation[_fbxHips] = bvhPose.translation[bvhHips] * rootTranslationScale;

        return out;
    }
}
