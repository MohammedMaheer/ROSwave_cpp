#ifndef CONTEXT_MENUS_HPP
#define CONTEXT_MENUS_HPP

#include <QObject>
#include <QMenu>
#include <QAction>
#include <QString>
#include <QClipboard>
#include <QApplication>
#include <functional>
#include <map>
#include <set>

/**
 * @brief Context menu system for topics and nodes
 * 
 * Provides right-click context menus with standard actions:
 * - Copy name/type to clipboard
 * - Subscribe to topic
 * - Record topic
 * - Add to monitoring
 * - View properties
 */
class ContextMenuManager : public QObject {
    Q_OBJECT

public:
    enum class ItemType {
        TOPIC,
        NODE,
        SERVICE
    };

    struct MenuItem {
        QString text;
        QString icon;
        std::function<void()> action;
        bool enabled = true;
    };

    explicit ContextMenuManager(QObject* parent = nullptr);
    ~ContextMenuManager();

    /**
     * @brief Show context menu for a topic
     * @param itemName Topic name
     * @param itemType Type of topic (e.g., "sensor_msgs/PointCloud2")
     * @param x Screen X coordinate
     * @param y Screen Y coordinate
     */
    void showTopicContextMenu(const QString& topicName, 
                              const QString& topicType,
                              int x, int y);

    /**
     * @brief Show context menu for a node
     * @param nodeName Node name
     * @param x Screen X coordinate
     * @param y Screen Y coordinate
     */
    void showNodeContextMenu(const QString& nodeName, int x, int y);

    /**
     * @brief Show context menu for a service
     * @param serviceName Service name
     * @param serviceType Service type
     * @param x Screen X coordinate
     * @param y Screen Y coordinate
     */
    void showServiceContextMenu(const QString& serviceName,
                                const QString& serviceType,
                                int x, int y);

    /**
     * @brief Register custom action callback
     * @param itemType Type of item this action applies to
     * @param actionName Name of the action
     * @param callback Function to call when action triggered
     */
    void registerCustomAction(ItemType itemType, 
                              const QString& actionName,
                              std::function<void()> callback);

signals:
    /**
     * @brief Emitted when topic is selected for subscription
     */
    void topicSubscribeRequested(const QString& topicName);

    /**
     * @brief Emitted when topic recording is requested
     */
    void topicRecordRequested(const QString& topicName);

    /**
     * @brief Emitted when topic is added to monitoring
     */
    void topicMonitoringRequested(const QString& topicName);

    /**
     * @brief Emitted when node action is requested
     */
    void nodeActionRequested(const QString& nodeName, const QString& action);

    /**
     * @brief Emitted when service is called
     */
    void serviceActionRequested(const QString& serviceName, const QString& action);

private slots:
    /**
     * @brief Handle copy action
     */
    void onCopyToClipboard(const QString& text);

    /**
     * @brief Handle subscribe action
     */
    void onSubscribeTopic();

    /**
     * @brief Handle record action
     */
    void onRecordTopic();

    /**
     * @brief Handle monitoring action
     */
    void onMonitorTopic();

    /**
     * @brief Handle node restart action
     */
    void onRestartNode();

    /**
     * @brief Handle node kill action
     */
    void onKillNode();

    /**
     * @brief Handle view logs action
     */
    void onViewNodeLogs();

private:
    /**
     * @brief Create topic context menu
     */
    QMenu* createTopicMenu(const QString& topicName, const QString& topicType);

    /**
     * @brief Create node context menu
     */
    QMenu* createNodeMenu(const QString& nodeName);

    /**
     * @brief Create service context menu
     */
    QMenu* createServiceMenu(const QString& serviceName, const QString& serviceType);

    /**
     * @brief Add standard topic actions to menu
     */
    void addTopicActions(QMenu* menu, const QString& topicName, const QString& topicType);

    /**
     * @brief Add standard node actions to menu
     */
    void addNodeActions(QMenu* menu, const QString& nodeName);

    /**
     * @brief Add standard service actions to menu
     */
    void addServiceActions(QMenu* menu, const QString& serviceName, const QString& serviceType);

    // Current context (for signal emission)
    QString currentTopicName;
    QString currentTopicType;
    QString currentNodeName;
    QString currentServiceName;

    // Custom actions per item type
    std::map<ItemType, std::map<QString, std::function<void()>>> customActions;

    // Action persistence
    std::set<QString> recordedTopics;
    std::set<QString> monitoredTopics;
};

#endif // CONTEXT_MENUS_HPP
