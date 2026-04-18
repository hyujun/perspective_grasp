#pragma once

#include <map>
#include <vector>

namespace perspective_grasp {

/// Simple disjoint-set with path compression and union by rank.
class UnionFind {
 public:
  int find(int a) {
    if (parent_.find(a) == parent_.end()) {
      parent_[a] = a;
      rank_[a] = 0;
    }
    if (parent_[a] != a) {
      parent_[a] = find(parent_[a]);
    }
    return parent_[a];
  }

  void unite(int a, int b) {
    int ra = find(a);
    int rb = find(b);
    if (ra == rb) return;
    if (rank_[ra] < rank_[rb]) std::swap(ra, rb);
    parent_[rb] = ra;
    if (rank_[ra] == rank_[rb]) ++rank_[ra];
  }

  /// Returns groups: representative -> list of members.
  std::map<int, std::vector<int>> getGroups() const {
    std::map<int, std::vector<int>> groups;
    // Need non-const find for path compression, so copy parent
    auto parent_copy = parent_;
    for (auto& [node, par] : parent_copy) {
      // Path-compressed find on copy
      int root = par;
      while (parent_copy[root] != root) root = parent_copy[root];
      // Compress
      int cur = node;
      while (parent_copy[cur] != root) {
        int next = parent_copy[cur];
        parent_copy[cur] = root;
        cur = next;
      }
      groups[root].push_back(node);
    }
    return groups;
  }

 private:
  std::map<int, int> parent_;
  std::map<int, int> rank_;
};

}  // namespace perspective_grasp
