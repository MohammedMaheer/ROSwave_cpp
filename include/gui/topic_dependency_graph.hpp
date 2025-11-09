#pragma once

#include <QWidget>
#include <qcustomplot.h>
#include <QString>
#include <QMap>
#include <QList>
#include <QVector>
#include <QPair>
#include <QColor>
#include <memory>
#include <vector>

/**
 * @brief Topic Dependency Graph Widget
 * 
 * Displays publisher-subscriber relationships between ROS2 nodes and topics
 * as an interactive graph with:
 * - Force-directed node layout
 * - Interactive zoom/pan
 * - Node and edge filtering
 * - Real-time updates
 * - Statistics overlay
 * - Export capabilities
 */
class TopicDependencyGraph : public QCustomPlot {
    Q_OBJECT

public:
    /**
     * @brief Node types in the graph
     */
    enum class NodeType {
        TOPIC,      // ROS2 topic node
        NODE,       // ROS2 node
        SERVICE     // ROS2 service
    };

    /**
     * @brief Relationship types
     */
    enum class RelationType {
        PUBLISHES,      // Node publishes to topic
        SUBSCRIBES,     // Node subscribes to topic
        SERVES,         // Node provides service
        DEPENDS_ON      // Node depends on service
    };

    /**
     * @brief Graph node representation
     */
    struct GraphNode {
        QString id;
        QString label;
        NodeType type;
        QColor color;
        double x = 0, y = 0;
        double vx = 0, vy = 0;  // Velocity for force-directed layout
        bool fixed = false;      // Fixed position
        int degree = 0;          // Number of connections
        double frequency_hz = 0; // For topics
    };

    /**
     * @brief Graph edge representation
     */
    struct GraphEdge {
        QString from_id;
        QString to_id;
        RelationType type;
        int message_count = 0;
        double latency_ms = 0;
        double bandwidth_mbps = 0;
    };

    explicit TopicDependencyGraph(QWidget* parent = nullptr);
    ~TopicDependencyGraph() = default;

    /**
     * @brief Add a node to the graph
     */
    void add_node(const GraphNode& node);

    /**
     * @brief Add an edge (relationship) to the graph
     */
    void add_edge(const GraphEdge& edge);

    /**
     * @brief Clear all nodes and edges
     */
    void clear_graph();

    /**
     * @brief Build graph from ROS2 system information
     */
    void build_from_ros2_data(const std::vector<QString>& topics,
                             const std::vector<QString>& nodes,
                             const std::vector<QString>& services);

    /**
     * @brief Update graph layout (force-directed)
     */
    void update_layout(int iterations = 50);

    /**
     * @brief Refresh visualization
     */
    void refresh();

    /**
     * @brief Get node by ID
     */
    const GraphNode* get_node(const QString& node_id) const;

    /**
     * @brief Set node as selected
     */
    void select_node(const QString& node_id);

    /**
     * @brief Highlight path from node A to node B
     */
    void highlight_path(const QString& start_node, const QString& end_node);

    /**
     * @brief Clear highlighting
     */
    void clear_highlights();

    /**
     * @brief Filter nodes by type
     */
    void filter_by_type(NodeType type, bool show);

    /**
     * @brief Filter nodes by name pattern
     */
    void filter_by_name(const QString& pattern);

    /**
     * @brief Show/hide node labels
     */
    void set_show_labels(bool show);

    /**
     * @brief Show/hide edge labels
     */
    void set_show_edge_labels(bool show);

    /**
     * @brief Set physics simulation enabled
     */
    void set_physics_enabled(bool enabled);

    /**
     * @brief Get statistics about the graph
     */
    struct GraphStats {
        int node_count = 0;
        int edge_count = 0;
        int topic_count = 0;
        int node_count_ros2 = 0;
        int service_count = 0;
        double avg_degree = 0;
        double graph_diameter = 0;
    };
    GraphStats get_statistics() const;

    /**
     * @brief Export graph as PNG
     */
    bool export_as_image(const QString& file_path, int width = 1280, int height = 720);

    /**
     * @brief Export graph as SVG
     */
    bool export_as_svg(const QString& file_path);

    /**
     * @brief Export graph as JSON
     */
    bool export_as_json(const QString& file_path) const;

    /**
     * @brief Get all paths from node A to node B
     */
    std::vector<std::vector<QString>> find_paths(const QString& start_node,
                                                 const QString& end_node) const;

    /**
     * @brief Get strongly connected components
     */
    std::vector<std::vector<QString>> find_components() const;

signals:
    void node_clicked(const QString& node_id);
    void node_double_clicked(const QString& node_id);
    void edge_clicked(const QString& from_id, const QString& to_id);
    void graph_updated();
    void selection_changed(const QList<QString>& selected_nodes);

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    struct Layout {
        const GraphNode* node = nullptr;
        double x = 0, y = 0;
        double vx = 0, vy = 0;
    };

    void render_graph();
    void draw_nodes();
    void draw_edges();
    void apply_force_directed_layout();
    double calculate_repulsion(double distance) const;
    double calculate_attraction(double distance) const;
    
    const GraphNode* find_node_at(double x, double y) const;
    double calculate_distance(const GraphNode& n1, const GraphNode& n2) const;
    std::vector<QString> find_shortest_path(const QString& start, const QString& end) const;
    
    void draw_statistics_overlay();
    void center_view();
    void fit_to_nodes();

    std::vector<GraphNode> nodes_;
    std::vector<GraphEdge> edges_;
    std::map<QString, size_t> node_index_;
    
    QList<QString> selected_nodes_;
    QString highlighted_start_node_;
    QString highlighted_end_node_;
    
    bool show_labels_ = true;
    bool show_edge_labels_ = false;
    bool physics_enabled_ = true;
    bool is_panning_ = false;
    QPoint last_mouse_pos_;
    
    double zoom_level_ = 1.0;
    double pan_x_ = 0, pan_y_ = 0;
    
    // Physics parameters
    double repulsion_strength_ = 500;
    double attraction_strength_ = 0.5;
    double damping_ = 0.9;
    
    // Rendering
    QColor color_topic_ = Qt::blue;
    QColor color_node_ = Qt::green;
    QColor color_service_ = Qt::red;
    QColor color_selected_ = Qt::yellow;
    QColor color_edge_normal_ = Qt::gray;
    QColor color_edge_highlight_ = Qt::cyan;
};
