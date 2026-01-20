#include "FBXLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <unordered_map>
#include <iostream>
#include <cassert>
#include <functional>

static glm::mat4 AiToGlm(const aiMatrix4x4& m) {
    glm::mat4 r;
    r[0][0] = m.a1; r[1][0] = m.a2; r[2][0] = m.a3; r[3][0] = m.a4;
    r[0][1] = m.b1; r[1][1] = m.b2; r[2][1] = m.b3; r[3][1] = m.b4;
    r[0][2] = m.c1; r[1][2] = m.c2; r[2][2] = m.c3; r[3][2] = m.c4;
    r[0][3] = m.d1; r[1][3] = m.d2; r[2][3] = m.d3; r[3][3] = m.d4;
    return r;
}

static void CollectNodes(
    aiNode* node,
    int parent,
    std::vector<aiNode*>& outNodes,
    std::unordered_map<std::string, int>& nodeIndex
) {
    int index = (int)outNodes.size();
    outNodes.push_back(node);
    nodeIndex[node->mName.C_Str()] = index;

    for (unsigned i = 0; i < node->mNumChildren; ++i) {
        CollectNodes(node->mChildren[i], index, outNodes, nodeIndex);
    }
}

FBXAsset FBXLoader::Load(const std::string& path) {
    FBXAsset asset;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path,
        aiProcess_Triangulate |
        aiProcess_JoinIdenticalVertices |
        aiProcess_GenNormals
    );

    if (!scene || !scene->HasMeshes()) throw std::runtime_error("Failed to load FBX: " + path);

    // mesh
    asset.mesh.positions.clear();
    asset.mesh.normals.clear();
    asset.mesh.indices.clear();
    uint32_t vertexOffset = 0;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        aiMesh* mesh = scene->mMeshes[m];
    
        // positions / normals
        for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
            asset.mesh.positions.emplace_back(
                mesh->mVertices[i].x,
                mesh->mVertices[i].y,
                mesh->mVertices[i].z
            );
    
            if (mesh->HasNormals()) {
                asset.mesh.normals.emplace_back(
                    mesh->mNormals[i].x,
                    mesh->mNormals[i].y,
                    mesh->mNormals[i].z
                );
            } else {
                asset.mesh.normals.emplace_back(0, 1, 0);
            }
        }
    
        // indices
        for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            assert(face.mNumIndices == 3);
            asset.mesh.indices.push_back(vertexOffset + face.mIndices[0]);
            asset.mesh.indices.push_back(vertexOffset + face.mIndices[1]);
            asset.mesh.indices.push_back(vertexOffset + face.mIndices[2]);
        }
    
        vertexOffset += mesh->mNumVertices;
    }

    // skeleton
    std::vector<aiNode*> nodes;
    std::unordered_map<std::string, int> nodeIndex;
    CollectNodes(scene->mRootNode, -1, nodes, nodeIndex);

    asset.skeleton.joints.resize(nodes.size());

    for (size_t i = 0; i < nodes.size(); ++i) {
        FBXJoint& joint = asset.skeleton.joints[i];
        aiNode* node = nodes[i];

        joint.name = node->mName.C_Str();
        joint.parent = node->mParent ? nodeIndex[node->mParent->mName.C_Str()] : -1;
        joint.bindPose = AiToGlm(node->mTransformation);
        joint.inverseBindPose = glm::inverse(joint.bindPose);
    }

    // skin weights
    size_t totalVertices = 0;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        totalVertices += scene->mMeshes[m]->mNumVertices;
    }

    asset.skin.weights.clear();
    asset.skin.weights.resize(totalVertices);
    vertexOffset = 0;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        aiMesh* mesh = scene->mMeshes[m];

        for (unsigned i = 0; i < mesh->mNumBones; ++i) {
            aiBone* bone = mesh->mBones[i];
            std::string boneName = bone->mName.C_Str();

            if (nodeIndex.find(boneName) == nodeIndex.end()) continue;

            uint32_t jointId = nodeIndex[boneName];

            // overwrite inverse bind pose with Assimp data
            asset.skeleton.joints[jointId].inverseBindPose = AiToGlm(bone->mOffsetMatrix);

            for (unsigned j = 0; j < bone->mNumWeights; ++j) {
                const aiVertexWeight& vw = bone->mWeights[j];
                uint32_t globalVid = vertexOffset + vw.mVertexId;
                asset.skin.weights[globalVid].push_back({jointId, vw.mWeight});
            }
        }
        vertexOffset += mesh->mNumVertices;
    }

    // normalize weights
    for (auto& arr : asset.skin.weights) {
        float sum = 0.0f;
        for (auto& w : arr) sum += w.weight;
        if (sum > 0.0f) {
            for (auto& w : arr) w.weight /= sum;
        }
    }
    return asset;
}
