#include "gui/context_menus.hpp"
#include <QMenu>
#include <QAction>
#include <QClipboard>
#include <QApplication>
#include <QMessageBox>
#include <iostream>

ContextMenuManager::ContextMenuManager(QObject* parent)
    : QObject(parent)
{
}

ContextMenuManager::~ContextMenuManager()
{
}

void ContextMenuManager::showTopicContextMenu(const QString& topicName,
                                             const QString& topicType,
                                             int x, int y)
{
    currentTopicName = topicName;
    currentTopicType = topicType;

    QMenu* menu = createTopicMenu(topicName, topicType);
    menu->exec(QPoint(x, y));
    delete menu;
}

void ContextMenuManager::showNodeContextMenu(const QString& nodeName,
                                            int x, int y)
{
    currentNodeName = nodeName;

    QMenu* menu = createNodeMenu(nodeName);
    menu->exec(QPoint(x, y));
    delete menu;
}

void ContextMenuManager::showServiceContextMenu(const QString& serviceName,
                                               const QString& serviceType,
                                               int x, int y)
{
    currentServiceName = serviceName;

    QMenu* menu = createServiceMenu(serviceName, serviceType);
    menu->exec(QPoint(x, y));
    delete menu;
}

void ContextMenuManager::registerCustomAction(ItemType itemType,
                                              const QString& actionName,
                                              std::function<void()> callback)
{
    customActions[itemType][actionName] = callback;
}

QMenu* ContextMenuManager::createTopicMenu(const QString& topicName,
                                           const QString& topicType)
{
    QMenu* menu = new QMenu();
    menu->setTitle(QString("Topic: %1").arg(topicName));

    addTopicActions(menu, topicName, topicType);

    // Add custom actions
    if (customActions.find(ItemType::TOPIC) != customActions.end()) {
        menu->addSeparator();
        for (const auto& [actionName, callback] : customActions[ItemType::TOPIC]) {
            QAction* action = menu->addAction(actionName);
            connect(action, &QAction::triggered, this, [callback]() { callback(); });
        }
    }

    return menu;
}

QMenu* ContextMenuManager::createNodeMenu(const QString& nodeName)
{
    QMenu* menu = new QMenu();
    menu->setTitle(QString("Node: %1").arg(nodeName));

    addNodeActions(menu, nodeName);

    // Add custom actions
    if (customActions.find(ItemType::NODE) != customActions.end()) {
        menu->addSeparator();
        for (const auto& [actionName, callback] : customActions[ItemType::NODE]) {
            QAction* action = menu->addAction(actionName);
            connect(action, &QAction::triggered, this, [callback]() { callback(); });
        }
    }

    return menu;
}

QMenu* ContextMenuManager::createServiceMenu(const QString& serviceName,
                                             const QString& serviceType)
{
    QMenu* menu = new QMenu();
    menu->setTitle(QString("Service: %1").arg(serviceName));

    addServiceActions(menu, serviceName, serviceType);

    // Add custom actions
    if (customActions.find(ItemType::SERVICE) != customActions.end()) {
        menu->addSeparator();
        for (const auto& [actionName, callback] : customActions[ItemType::SERVICE]) {
            QAction* action = menu->addAction(actionName);
            connect(action, &QAction::triggered, this, [callback]() { callback(); });
        }
    }

    return menu;
}

void ContextMenuManager::addTopicActions(QMenu* menu, const QString& topicName,
                                         const QString& topicType)
{
    // Copy name
    {
        QAction* action = menu->addAction("Copy Name");
        connect(action, &QAction::triggered, this, [this, topicName]() {
            onCopyToClipboard(topicName);
        });
    }

    // Copy type
    {
        QAction* action = menu->addAction("Copy Full Type");
        connect(action, &QAction::triggered, this, [this, topicType]() {
            onCopyToClipboard(topicType);
        });
    }

    menu->addSeparator();

    // View message definition
    {
        QAction* action = menu->addAction("View Message Definition");
        connect(action, &QAction::triggered, this, [topicType]() {
            std::cout << "Message type: " << topicType.toStdString() << std::endl;
        });
    }

    menu->addSeparator();

    // Subscribe
    {
        QAction* action = menu->addAction("Subscribe (Show Messages)");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onSubscribeTopic);
    }

    // Record this topic only
    {
        QAction* action = menu->addAction("Record This Topic");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onRecordTopic);
    }

    // Add to monitoring
    {
        QAction* action = menu->addAction("Add to Monitoring");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onMonitorTopic);
    }
}

void ContextMenuManager::addNodeActions(QMenu* menu, const QString& nodeName)
{
    // Copy name
    {
        QAction* action = menu->addAction("Copy Name");
        connect(action, &QAction::triggered, this, [this, nodeName]() {
            onCopyToClipboard(nodeName);
        });
    }

    menu->addSeparator();

    // View node parameters
    {
        QAction* action = menu->addAction("View Node Parameters");
        connect(action, &QAction::triggered, this, [nodeName]() {
            std::cout << "Parameters for node: " << nodeName.toStdString() << std::endl;
        });
    }

    // View node info
    {
        QAction* action = menu->addAction("View Node Info");
        connect(action, &QAction::triggered, this, [nodeName]() {
            std::cout << "Info for node: " << nodeName.toStdString() << std::endl;
        });
    }

    menu->addSeparator();

    // Restart node
    {
        QAction* action = menu->addAction("Restart Node");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onRestartNode);
    }

    // View node logs
    {
        QAction* action = menu->addAction("View Node Logs");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onViewNodeLogs);
    }

    // Kill node
    {
        QAction* action = menu->addAction("Kill Node");
        connect(action, &QAction::triggered, this, &ContextMenuManager::onKillNode);
    }
}

void ContextMenuManager::addServiceActions(QMenu* menu, const QString& serviceName,
                                           const QString& serviceType)
{
    // Copy name
    {
        QAction* action = menu->addAction("Copy Name");
        connect(action, &QAction::triggered, this, [this, serviceName]() {
            onCopyToClipboard(serviceName);
        });
    }

    // Copy type
    {
        QAction* action = menu->addAction("Copy Service Type");
        connect(action, &QAction::triggered, this, [this, serviceType]() {
            onCopyToClipboard(serviceType);
        });
    }

    menu->addSeparator();

    // Call service
    {
        QAction* action = menu->addAction("Call Service");
        connect(action, &QAction::triggered, this, [this, serviceName]() {
            emit serviceActionRequested(serviceName, "call");
        });
    }
}

void ContextMenuManager::onCopyToClipboard(const QString& text)
{
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(text);
    std::cout << "Copied to clipboard: " << text.toStdString() << std::endl;
}

void ContextMenuManager::onSubscribeTopic()
{
    emit topicSubscribeRequested(currentTopicName);
    std::cout << "Subscribing to topic: " << currentTopicName.toStdString() << std::endl;
}

void ContextMenuManager::onRecordTopic()
{
    recordedTopics.insert(currentTopicName);
    emit topicRecordRequested(currentTopicName);
    std::cout << "Recording topic: " << currentTopicName.toStdString() << std::endl;
}

void ContextMenuManager::onMonitorTopic()
{
    monitoredTopics.insert(currentTopicName);
    emit topicMonitoringRequested(currentTopicName);
    std::cout << "Monitoring topic: " << currentTopicName.toStdString() << std::endl;
}

void ContextMenuManager::onRestartNode()
{
    emit nodeActionRequested(currentNodeName, "restart");
    std::cout << "Restarting node: " << currentNodeName.toStdString() << std::endl;
}

void ContextMenuManager::onKillNode()
{
    emit nodeActionRequested(currentNodeName, "kill");
    std::cout << "Killing node: " << currentNodeName.toStdString() << std::endl;
}

void ContextMenuManager::onViewNodeLogs()
{
    emit nodeActionRequested(currentNodeName, "logs");
    std::cout << "Viewing logs for node: " << currentNodeName.toStdString() << std::endl;
}
