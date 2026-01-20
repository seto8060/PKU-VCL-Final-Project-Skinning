#include "Labs/Final-Animation/Animation/BVH/BVHLoader.h"

#include <fstream>
#include <stdexcept>

namespace VCX::Labs::FinalAnimation::BVH {
    static ChannelType ParseChannelType(const std::string& name) {
        if (name == "Xposition") return ChannelType::Xposition;
        if (name == "Yposition") return ChannelType::Yposition;
        if (name == "Zposition") return ChannelType::Zposition;
        if (name == "Xrotation") return ChannelType::Xrotation;
        if (name == "Yrotation") return ChannelType::Yrotation;
        if (name == "Zrotation") return ChannelType::Zrotation;
        throw std::runtime_error("Invalid channel type: " + name);
    }

    BVHData BVHLoader::Load(const std::string& path) {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Failed to open BVH file");

        BVHData bvh;
        Expect(in, "HIERARCHY");
        Expect(in, "ROOT");
        ParseJoint(in, bvh, -1);
        Expect(in, "MOTION");
        ParseMotion(in, bvh);
        return bvh;
    }

    int BVHLoader::ParseJoint(std::istream& in, BVHData& bvh,int parent) {
        std::string name;
        in >> name;

        Joint joint;
        joint.name = name;
        joint.parent = parent;
        int JointIndex = static_cast<int>(bvh.joints.size());
        bvh.joints.push_back(joint);

        Expect(in, "{");

        std::string token;
        while (in >> token) {
            if (token == "}") break;
            if (token == "OFFSET") {
                in >> bvh.joints[JointIndex].offset.x >> bvh.joints[JointIndex].offset.y >> bvh.joints[JointIndex].offset.z;
                continue;
            }
            if (token == "CHANNELS") {
                int channels;
                in >> channels;
                for (int i = 0; i < channels; i++) {
                    std::string Name;
                    in >> Name;

                    bvh.joints[JointIndex].channels.push_back(ParseChannelType(Name));
                    bvh.channelCount++;
                }
                continue;
            }
            if (token == "JOINT") {
                ParseJoint(in, bvh, JointIndex);
                continue;
            }
            if (token == "End") {
                in >> token; // "Site"
                Expect(in, "{");

                for (int i = 0; i < 4; i++) in >> token; // "OFFSET 0 0 0"
                Expect(in, "}");
                continue;
            }
            throw std::runtime_error("Unexpected token in hierarchy: " + token);
        }
        return JointIndex;
    }

    void BVHLoader::ParseMotion(std::istream& in, BVHData& bvh) {
        Expect(in, "Frames:");
        in >> bvh.frameCount;
        Expect(in, "Frame"), Expect(in, "Time:");
        in >> bvh.frameTime;
        bvh.motion.resize(bvh.frameCount);
        for (int i = 0; i < bvh.frameCount; i++) {
            bvh.motion[i].resize(bvh.channelCount);
            for (int j = 0; j < bvh.channelCount; j++) in >> bvh.motion[i][j];
        }
    }

    void BVHLoader::Expect(std::istream& in, const std::string& token) {
        std::string read;
        in >> read;
        if (read != token) throw std::runtime_error("Unexpected token: " + read + ", expected: " + token);
    }
}
