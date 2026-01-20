#pragma once

#include <string>
#include <istream>

#include "Labs/Final-Animation/Animation/BVH/BVH.h"

namespace VCX::Labs::FinalAnimation::BVH {
    class BVHLoader {
    public:
        static BVHData Load(const std::string& path); // load BVH file from disk and return parsed BVHData

    private:
        static int ParseJoint(std::istream& in, BVHData& bvh, int parent); // parse one joint (ROOT or JOINT) recursively and return index of the parsed joint from input

        static void ParseMotion(std::istream& in, BVHData& bvh); // parse MOTION section from input and update BVHData

        static void Expect(std::istream& in, const std::string& token); // A utility function to expect a specific token, otherwise throw error
    };

}