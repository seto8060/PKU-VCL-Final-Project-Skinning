#include "SkeletonBuilder.h"

#include <unordered_map>
#include <vector>
#include <functional>
#include <iostream>

BuiltSkeleton SkeletonBuilder::BuildFullSkeleton(const FBXSkeleton& raw) {
    FBXSkeleton skeleton;

    const int N = static_cast<int>(raw.joints.size());
    if (N == 0) {
        BuiltSkeleton out;
        out.skeleton = skeleton;
        out.rawToAnim.resize(N);
        return out;
    }

    skeleton.joints.resize(N);

    for (int i = 0; i < N; ++i) skeleton.joints[i] = raw.joints[i];

    std::vector<std::vector<int>> children(N);
    for (int i = 0; i < N; ++i) {
        int p = raw.joints[i].parent;
        if (p >= 0 && p < N) children[p].push_back(i);
    }

    std::vector<int> newOrder;
    newOrder.reserve(N);

    std::vector<bool> visited(N, false);

    std::function<void(int)> dfs = [&](int u) {
        visited[u] = true;
        newOrder.push_back(u);
        for (int v : children[u]) 
            if (!visited[v]) dfs(v);
    };

    for (int i = 0; i < N; ++i) if (raw.joints[i].parent == -1) dfs(i);

    FBXSkeleton reordered;
    reordered.joints.resize(N);

    std::vector<int> oldToNew(N);

    for (int newIdx = 0; newIdx < N; ++newIdx) {
        int oldIdx = newOrder[newIdx];
        reordered.joints[newIdx] = skeleton.joints[oldIdx];
        oldToNew[oldIdx] = newIdx;
    }

    for (int i = 0; i < N; ++i) {
        int& p = reordered.joints[i].parent;
        if (p != -1) p = oldToNew[p];
    }

    BuiltSkeleton out;
    out.skeleton = reordered;
    out.rawToAnim.resize(N);
    for (int newIdx = 0; newIdx < N; ++newIdx) {
        int oldIdx = newOrder[newIdx];
        out.rawToAnim[oldIdx] = newIdx;
    }
    return out;
}
