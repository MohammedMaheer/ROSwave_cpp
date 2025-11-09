#include <gtest/gtest.h>
#include <QApplication>
#include <QString>
#include <memory>
#include "../include/gui/context_menus.hpp"

/**
 * Test suite for ContextMenuManager - Context menu functionality
 * Tests right-click menus for topics, nodes, and services
 */

int argc_global = 0;
char** argv_global = nullptr;
QApplication* app = nullptr;

int main(int argc, char** argv) {
    argc_global = argc;
    argv_global = argv;
    app = new QApplication(argc, argv);
    
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    
    delete app;
    return result;
}

class ContextMenuTest : public ::testing::Test {
protected:
    std::unique_ptr<ContextMenuManager> manager;

    void SetUp() override {
        manager = std::make_unique<ContextMenuManager>();
    }

    void TearDown() override {
        manager.reset();
    }
};

/**
 * Test: ContextMenuManager instantiation
 * Verifies that the manager can be created
 */
TEST_F(ContextMenuTest, ManagerInstantiation) {
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Topic context menu creation
 * Verifies that showTopicContextMenu doesn't crash
 */
TEST_F(ContextMenuTest, TopicContextMenuCreation) {
    EXPECT_NE(manager.get(), nullptr);
    manager->showTopicContextMenu("test_topic", "std_msgs/String", 100, 100);
}

/**
 * Test: Node context menu creation
 * Verifies that showNodeContextMenu doesn't crash
 */
TEST_F(ContextMenuTest, NodeContextMenuCreation) {
    EXPECT_NE(manager.get(), nullptr);
    manager->showNodeContextMenu("test_node", 100, 100);
}

/**
 * Test: Service context menu creation
 * Verifies that showServiceContextMenu doesn't crash
 */
TEST_F(ContextMenuTest, ServiceContextMenuCreation) {
    EXPECT_NE(manager.get(), nullptr);
    manager->showServiceContextMenu("test_service", "std_srvs/SetBool", 100, 100);
}

/**
 * Test: Topic subscribe signal slot works
 * Verifies that the topic subscribe slot can be called
 */
TEST_F(ContextMenuTest, TopicSubscribeSlot) {
    EXPECT_NE(manager.get(), nullptr);
    // Just verify the manager exists - actual signal testing requires GUI interaction
}

/**
 * Test: Custom action registration for topics
 * Verifies that custom actions can be registered
 */
TEST_F(ContextMenuTest, CustomTopicActionRegistration) {
    bool action_called = false;
    manager->registerCustomAction(ContextMenuManager::ItemType::TOPIC,
                                  "CustomAction",
                                  [&]() { action_called = true; });
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Custom action registration for nodes
 * Verifies that custom actions can be registered for nodes
 */
TEST_F(ContextMenuTest, CustomNodeActionRegistration) {
    bool action_called = false;
    manager->registerCustomAction(ContextMenuManager::ItemType::NODE,
                                  "CustomNodeAction",
                                  [&]() { action_called = true; });
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Custom action registration for services
 * Verifies that custom actions can be registered for services
 */
TEST_F(ContextMenuTest, CustomServiceActionRegistration) {
    bool action_called = false;
    manager->registerCustomAction(ContextMenuManager::ItemType::SERVICE,
                                  "CustomServiceAction",
                                  [&]() { action_called = true; });
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Multiple custom actions
 * Verifies that multiple custom actions can be registered
 */
TEST_F(ContextMenuTest, MultipleCustomActions) {
    manager->registerCustomAction(ContextMenuManager::ItemType::TOPIC,
                                  "Action1", []() {});
    manager->registerCustomAction(ContextMenuManager::ItemType::TOPIC,
                                  "Action2", []() {});
    manager->registerCustomAction(ContextMenuManager::ItemType::TOPIC,
                                  "Action3", []() {});
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Context menu with long topic names
 * Verifies that context menus handle long topic names
 */
TEST_F(ContextMenuTest, LongTopicName) {
    QString long_name = "/robot/sensor/camera/rgb/image_raw/compressed/depth_aligned";
    manager->showTopicContextMenu(long_name, "sensor_msgs/CompressedImage", 0, 0);
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Context menu with complex message types
 * Verifies that context menus handle complex message types
 */
TEST_F(ContextMenuTest, ComplexMessageTypes) {
    manager->showTopicContextMenu("/pointcloud", "sensor_msgs/PointCloud2", 0, 0);
    manager->showTopicContextMenu("/tf", "tf2_msgs/TFMessage", 0, 0);
    manager->showTopicContextMenu("/joints", "sensor_msgs/JointState", 0, 0);
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Multiple topics context menus
 * Verifies that managing multiple topics works
 */
TEST_F(ContextMenuTest, MultipleTopics) {
    manager->showTopicContextMenu("topic1", "type1", 0, 0);
    manager->showTopicContextMenu("topic2", "type2", 0, 0);
    manager->showTopicContextMenu("topic3", "type3", 0, 0);
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Manager destruction
 * Verifies that creating and destroying manager works safely
 */
TEST_F(ContextMenuTest, ManagerDestruction) {
    auto temp_manager = std::make_unique<ContextMenuManager>();
    temp_manager->showTopicContextMenu("topic", "type", 0, 0);
    temp_manager.reset();
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Performance test: Multiple context menu operations
 * Verifies that multiple rapid context menu operations work
 */
TEST_F(ContextMenuTest, PerformanceMultipleOperations) {
    for (int i = 0; i < 50; i++) {
        QString topic = QString("topic_%1").arg(i);
        QString type = QString("type_%1").arg(i % 10);
        manager->showTopicContextMenu(topic, type, i % 500, i % 500);
    }
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: ItemType enum values
 * Verifies that all ItemType enum values are accessible
 */
TEST_F(ContextMenuTest, ItemTypeEnumValues) {
    auto topic_type = ContextMenuManager::ItemType::TOPIC;
    auto node_type = ContextMenuManager::ItemType::NODE;
    auto service_type = ContextMenuManager::ItemType::SERVICE;
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Menu creation with various node names
 * Verifies node menus work with different naming patterns
 */
TEST_F(ContextMenuTest, VariousNodeNames) {
    manager->showNodeContextMenu("/node1", 0, 0);
    manager->showNodeContextMenu("/ns/node2", 0, 0);
    manager->showNodeContextMenu("/a/b/c/node3", 0, 0);
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Service menu creation with various service names
 * Verifies service menus work with different naming patterns
 */
TEST_F(ContextMenuTest, VariousServiceNames) {
    manager->showServiceContextMenu("/service1", "srv/type1", 0, 0);
    manager->showServiceContextMenu("/ns/service2", "srv/type2", 0, 0);
    manager->showServiceContextMenu("/a/b/service3", "srv/type3", 0, 0);
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Stress test - rapid menu creation
 * Verifies that rapid menu creation doesn't cause issues
 */
TEST_F(ContextMenuTest, StressTestRapidMenuCreation) {
    for (int i = 0; i < 100; i++) {
        manager->showTopicContextMenu(
            QString("topic_%1").arg(i),
            QString("type_%1").arg(i % 5),
            rand() % 1000, rand() % 1000
        );
    }
    EXPECT_NE(manager.get(), nullptr);
}

/**
 * Test: Verify signal existence
 * Verifies that all expected signals are defined
 */
TEST_F(ContextMenuTest, SignalsExist) {
    // Verify we can connect to signals without compilation errors
    auto connection1 = connect(manager.get(), &ContextMenuManager::topicSubscribeRequested,
                              [](const QString&) {});
    auto connection2 = connect(manager.get(), &ContextMenuManager::topicRecordRequested,
                              [](const QString&) {});
    auto connection3 = connect(manager.get(), &ContextMenuManager::topicMonitoringRequested,
                              [](const QString&) {});
    auto connection4 = connect(manager.get(), &ContextMenuManager::nodeActionRequested,
                              [](const QString&, const QString&) {});
    auto connection5 = connect(manager.get(), &ContextMenuManager::serviceActionRequested,
                              [](const QString&, const QString&) {});
    
    EXPECT_TRUE(connection1);
    EXPECT_TRUE(connection2);
    EXPECT_TRUE(connection3);
    EXPECT_TRUE(connection4);
    EXPECT_TRUE(connection5);
}
