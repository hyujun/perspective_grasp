#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <set>
#include <vector>

#include "cross_camera_associator/union_find.hpp"

using perspective_grasp::UnionFind;

TEST(UnionFind, FindOnUnknownNodeCreatesSingleton) {
  UnionFind uf;
  EXPECT_EQ(uf.find(7), 7);
  // Second call is idempotent.
  EXPECT_EQ(uf.find(7), 7);
}

TEST(UnionFind, DisjointNodesStayDisjoint) {
  UnionFind uf;
  uf.find(1);
  uf.find(2);
  uf.find(3);
  EXPECT_NE(uf.find(1), uf.find(2));
  EXPECT_NE(uf.find(2), uf.find(3));
  EXPECT_NE(uf.find(1), uf.find(3));
}

TEST(UnionFind, UniteMergesTwoSets) {
  UnionFind uf;
  uf.unite(1, 2);
  EXPECT_EQ(uf.find(1), uf.find(2));
}

TEST(UnionFind, UniteTransitive) {
  UnionFind uf;
  uf.unite(1, 2);
  uf.unite(2, 3);
  EXPECT_EQ(uf.find(1), uf.find(3));
}

TEST(UnionFind, UniteSameElementIsNoOp) {
  UnionFind uf;
  uf.unite(5, 5);
  EXPECT_EQ(uf.find(5), 5);
}

TEST(UnionFind, UniteAlreadyConnectedIsNoOp) {
  UnionFind uf;
  uf.unite(1, 2);
  uf.unite(2, 3);
  // Re-uniting existing group elements should not corrupt structure.
  uf.unite(1, 3);
  auto groups = uf.getGroups();
  ASSERT_EQ(groups.size(), 1u);
  auto& members = groups.begin()->second;
  EXPECT_EQ(members.size(), 3u);
}

TEST(UnionFind, GetGroupsReturnsAllDisjointSets) {
  UnionFind uf;
  uf.unite(1, 2);
  uf.unite(3, 4);
  uf.unite(4, 5);
  uf.find(6);  // singleton

  auto groups = uf.getGroups();
  // 3 groups: {1,2}, {3,4,5}, {6}
  ASSERT_EQ(groups.size(), 3u);

  std::vector<std::size_t> sizes;
  for (auto& [rep, members] : groups) {
    sizes.push_back(members.size());
  }
  std::sort(sizes.begin(), sizes.end());
  EXPECT_EQ(sizes[0], 1u);
  EXPECT_EQ(sizes[1], 2u);
  EXPECT_EQ(sizes[2], 3u);
}

TEST(UnionFind, GetGroupsContainsExpectedMembers) {
  UnionFind uf;
  uf.unite(10, 20);
  uf.unite(20, 30);

  auto groups = uf.getGroups();
  ASSERT_EQ(groups.size(), 1u);
  std::set<int> members(groups.begin()->second.begin(),
                        groups.begin()->second.end());
  EXPECT_EQ(members, (std::set<int>{10, 20, 30}));
}

TEST(UnionFind, PathCompressionKeepsFindConsistent) {
  // Build a long chain then hit find() to flatten.
  UnionFind uf;
  for (int i = 0; i < 10; ++i) {
    uf.unite(i, i + 1);
  }
  // All 11 nodes (0..10) share the same root.
  int root = uf.find(0);
  for (int i = 1; i <= 10; ++i) {
    EXPECT_EQ(uf.find(i), root);
  }
}
