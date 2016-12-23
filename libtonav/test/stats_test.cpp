#include <gtest/gtest.h>

#include "stats_node.h"

TEST(StatsTest, StatsNodeEmptyContent) {
    StatsNode node(nullptr);
    ASSERT_EQ(node.str(), "{\n}");
}

TEST(StatsTest, StatsNodeTerminalContent) {
    std::string content = "This is some example of StatsNode content";
    StatsNode node(nullptr);
    node = content;
    ASSERT_EQ(node.str(), "\"" + content + "\"");
}

TEST(StatsTest, StatsNodeOneLevelHierarchy) {
    StatsNode root_node(nullptr);
    root_node["child"] = "child_content";
    std::cout << root_node.str() << std::endl;
    std::string expected = "{\n    \"child\": \"child_content\"\n}";
    ASSERT_EQ(root_node.str(), expected) << "Got content " << root_node.str();
}

TEST(StatsTest, StatsNodeOneLevelHierarchyTwoKeys) {
    StatsNode root_node(nullptr);
    root_node["child"] = "child_content";
    root_node["child2"] = "child2_content";
    std::cout << root_node.str() << std::endl;
    std::string expected = "{\n    \"child\": \"child_content\",\n    \"child2\": \"child2_content\"\n}";
    ASSERT_EQ(root_node.str(), expected) << "Got content " << root_node.str();
}


TEST(StatsTest, StatsNodeTwoLevelHierarchy) {
    StatsNode root_node(nullptr);
    root_node["child"]["child_child"] = "child_content";
    std::cout << root_node.str() << std::endl;
    std::string expected = "{\n    \"child\": {\n        \"child_child\": \"child_content\"\n    }\n}";
    ASSERT_EQ(root_node.str(), expected) << "Got content " << root_node.str();
}
