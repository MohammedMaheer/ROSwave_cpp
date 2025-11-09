/**
 * ROS2 Dashboard - Web Frontend
 * Real-time dashboard for remote ROS2 system monitoring
 */

class DashboardClient {
    constructor() {
        this.apiBaseUrl = localStorage.getItem('apiUrl') || 'http://localhost:8080/api';
        this.wsUrl = localStorage.getItem('wsUrl') || 'ws://localhost:8081';
        this.authToken = localStorage.getItem('authToken') || '';
        this.autoReconnect = localStorage.getItem('autoReconnect') !== 'false';
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000;
        
        this.init();
    }

    /**
     * Initialize dashboard
     */
    async init() {
        this.setupEventListeners();
        this.setupSettings();
        this.connectWebSocket();
        await this.loadInitialData();
        this.startAutoRefresh();
    }

    /**
     * Setup event listeners
     */
    setupEventListeners() {
        // Page navigation
        document.querySelectorAll('.nav-link').forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                this.navigatePage(link.dataset.page);
            });
        });

        // Settings modal
        const settingsBtn = document.getElementById('settings-btn');
        const settingsModal = document.getElementById('settings-modal');
        const closeBtn = document.querySelector('.close');

        settingsBtn.addEventListener('click', () => {
            settingsModal.classList.add('show');
        });

        closeBtn.addEventListener('click', () => {
            settingsModal.classList.remove('show');
        });

        settingsModal.addEventListener('click', (e) => {
            if (e.target === settingsModal) {
                settingsModal.classList.remove('show');
            }
        });

        document.getElementById('save-settings').addEventListener('click', () => {
            this.saveSettings();
        });

        // Search
        document.getElementById('topics-search').addEventListener('input', (e) => {
            this.filterTopics(e.target.value);
        });

        document.getElementById('nodes-search').addEventListener('input', (e) => {
            this.filterNodes(e.target.value);
        });

        // Duration select
        document.getElementById('duration-select').addEventListener('change', () => {
            this.loadMetrics();
        });
    }

    /**
     * Setup settings modal with current values
     */
    setupSettings() {
        document.getElementById('dashboard-url').value = this.apiBaseUrl;
        document.getElementById('websocket-url').value = this.wsUrl;
        document.getElementById('auth-token').value = this.authToken;
        document.getElementById('auto-reconnect').checked = this.autoReconnect;
    }

    /**
     * Save settings from modal
     */
    saveSettings() {
        this.apiBaseUrl = document.getElementById('dashboard-url').value;
        this.wsUrl = document.getElementById('websocket-url').value;
        this.authToken = document.getElementById('auth-token').value;
        this.autoReconnect = document.getElementById('auto-reconnect').checked;

        localStorage.setItem('apiUrl', this.apiBaseUrl);
        localStorage.setItem('wsUrl', this.wsUrl);
        localStorage.setItem('authToken', this.authToken);
        localStorage.setItem('autoReconnect', this.autoReconnect);

        this.showToast('Settings saved. Reconnecting...', 'success');
        
        if (this.ws) {
            this.ws.close();
        }
        this.connectWebSocket();
    }

    /**
     * Connect to WebSocket server
     */
    connectWebSocket() {
        try {
            this.ws = new WebSocket(this.wsUrl);

            this.ws.onopen = () => {
                this.updateConnectionStatus(true);
                this.reconnectAttempts = 0;
                this.showToast('Connected to dashboard', 'success');
                
                // Subscribe to metrics channel
                this.ws.send(JSON.stringify({
                    action: 'subscribe',
                    channel: 'metrics'
                }));
            };

            this.ws.onmessage = (event) => {
                this.handleWebSocketMessage(JSON.parse(event.data));
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.showToast('Connection error', 'error');
            };

            this.ws.onclose = () => {
                this.updateConnectionStatus(false);
                this.attemptReconnect();
            };
        } catch (error) {
            console.error('Failed to connect:', error);
            this.updateConnectionStatus(false);
            this.attemptReconnect();
        }
    }

    /**
     * Attempt to reconnect
     */
    attemptReconnect() {
        if (!this.autoReconnect) return;

        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            const delay = this.reconnectDelay * this.reconnectAttempts;
            console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            
            setTimeout(() => {
                this.connectWebSocket();
            }, delay);
        } else {
            this.showToast('Max reconnection attempts reached', 'error');
        }
    }

    /**
     * Handle WebSocket messages
     */
    handleWebSocketMessage(message) {
        switch (message.type) {
            case 'metrics_update':
                this.updateMetricsDisplay(message.data);
                break;
            case 'alert':
                this.showToast(message.message, 'warning');
                break;
            case 'topic_discovered':
                this.loadTopics();
                break;
            case 'node_discovered':
                this.loadNodes();
                break;
        }
    }

    /**
     * Update connection status display
     */
    updateConnectionStatus(connected) {
        const status = document.getElementById('connection-status');
        if (connected) {
            status.textContent = '● Connected';
            status.className = 'status connected';
        } else {
            status.textContent = '● Disconnected';
            status.className = 'status disconnected';
        }
    }

    /**
     * Navigate to a page
     */
    navigatePage(page) {
        // Hide all pages
        document.querySelectorAll('.page').forEach(p => p.style.display = 'none');
        
        // Update nav active state
        document.querySelectorAll('.nav-link').forEach(link => {
            link.classList.remove('active');
        });
        
        // Show selected page
        document.getElementById(page + '-page').style.display = 'block';
        
        // Update nav active
        document.querySelector(`[data-page="${page}"]`).classList.add('active');
        
        // Load page data
        this.loadPageData(page);
    }

    /**
     * Load page-specific data
     */
    async loadPageData(page) {
        switch (page) {
            case 'overview':
                await this.loadStatus();
                break;
            case 'topics':
                await this.loadTopics();
                break;
            case 'nodes':
                await this.loadNodes();
                break;
            case 'metrics':
                await this.loadMetrics();
                break;
            case 'services':
                await this.loadServices();
                break;
            case 'health':
                await this.loadHealth();
                break;
        }
    }

    /**
     * Load initial data
     */
    async loadInitialData() {
        await this.loadStatus();
    }

    /**
     * Start auto-refresh timer
     */
    startAutoRefresh() {
        setInterval(() => {
            this.loadStatus();
        }, 2000);
    }

    /**
     * API request helper
     */
    async apiCall(endpoint, options = {}) {
        const url = `${this.apiBaseUrl}${endpoint}`;
        const headers = {
            'Content-Type': 'application/json',
            ...options.headers
        };

        if (this.authToken) {
            headers['Authorization'] = `Bearer ${this.authToken}`;
        }

        try {
            const response = await fetch(url, {
                ...options,
                headers
            });

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            return await response.json();
        } catch (error) {
            console.error('API call failed:', error);
            this.showToast(`API Error: ${error.message}`, 'error');
            return null;
        }
    }

    /**
     * Load dashboard status
     */
    async loadStatus() {
        const data = await this.apiCall('/status');
        if (!data) return;

        const metrics = await this.apiCall('/metrics');
        if (!metrics) return;

        // Update stats
        const cpu = metrics.cpu_percent || 0;
        const memory = metrics.memory_mb || 0;
        const temp = metrics.temperature_c || 0;

        document.getElementById('cpu-usage').textContent = cpu.toFixed(1) + '%';
        document.getElementById('cpu-bar').style.width = Math.min(cpu, 100) + '%';

        document.getElementById('memory-usage').textContent = memory.toFixed(0) + ' MB';
        document.getElementById('memory-bar').style.width = Math.min((memory / 4000) * 100, 100) + '%';

        document.getElementById('temperature').textContent = temp.toFixed(1) + '°C';
        document.getElementById('temp-bar').style.width = Math.min((temp / 100) * 100, 100) + '%';

        // Update status info
        const statusInfo = document.getElementById('status-info');
        statusInfo.innerHTML = `
            <p><strong>Status:</strong> ${data.status}</p>
            <p><strong>Version:</strong> ${data.version}</p>
            <p><strong>Uptime:</strong> ${this.formatDuration(data.uptime_seconds)}</p>
            <p><strong>ROS2:</strong> ${data.ros2_detected ? 'Detected' : 'Not found'}</p>
            <p><strong>Domain ID:</strong> ${data.ros2_domain_id}</p>
        `;

        // Load topic count
        const topics = await this.apiCall('/topics');
        if (topics) {
            document.getElementById('topic-count').textContent = topics.count || 0;
        }
    }

    /**
     * Load topics
     */
    async loadTopics() {
        const data = await this.apiCall('/topics');
        if (!data) return;

        const list = document.getElementById('topics-list');
        list.innerHTML = '';

        data.topics.forEach(topic => {
            const item = document.createElement('div');
            item.className = 'list-item';
            item.innerHTML = `
                <div>
                    <div class="list-item-title">${topic.name}</div>
                    <div class="list-item-detail">${topic.type}</div>
                    <div class="list-item-detail">
                        Pub: ${topic.publishers} | Sub: ${topic.subscribers} | Rate: ${topic.frequency_hz.toFixed(1)} Hz
                    </div>
                </div>
            `;
            list.appendChild(item);
        });

        if (data.topics.length === 0) {
            list.innerHTML = '<p class="loading">No topics found</p>';
        }
    }

    /**
     * Filter topics by search
     */
    filterTopics(query) {
        const items = document.querySelectorAll('#topics-list .list-item');
        items.forEach(item => {
            const title = item.querySelector('.list-item-title').textContent.toLowerCase();
            item.style.display = title.includes(query.toLowerCase()) ? '' : 'none';
        });
    }

    /**
     * Load nodes
     */
    async loadNodes() {
        const data = await this.apiCall('/nodes');
        if (!data) return;

        const list = document.getElementById('nodes-list');
        list.innerHTML = '';

        data.nodes.forEach(node => {
            const item = document.createElement('div');
            item.className = 'list-item';
            item.innerHTML = `
                <div>
                    <div class="list-item-title">${node.full_name}</div>
                    <div class="list-item-detail">
                        Pub: ${node.publishers.length} | Sub: ${node.subscribers.length}
                    </div>
                </div>
            `;
            list.appendChild(item);
        });

        if (data.nodes.length === 0) {
            list.innerHTML = '<p class="loading">No nodes found</p>';
        }
    }

    /**
     * Filter nodes by search
     */
    filterNodes(query) {
        const items = document.querySelectorAll('#nodes-list .list-item');
        items.forEach(item => {
            const title = item.querySelector('.list-item-title').textContent.toLowerCase();
            item.style.display = title.includes(query.toLowerCase()) ? '' : 'none';
        });
    }

    /**
     * Load metrics history
     */
    async loadMetrics() {
        const duration = document.getElementById('duration-select').value;
        const data = await this.apiCall(`/metrics/history?duration_minutes=${duration}`);
        if (!data) return;

        // Placeholder for chart rendering
        const chart = document.getElementById('metrics-chart');
        chart.innerHTML = `
            <p class="loading">Chart rendering would use Chart.js with ${data.count} data points</p>
        `;
    }

    /**
     * Load services
     */
    async loadServices() {
        const data = await this.apiCall('/services');
        if (!data) return;

        const list = document.getElementById('services-list');
        list.innerHTML = '';

        data.services.forEach(service => {
            const item = document.createElement('div');
            item.className = 'list-item';
            item.innerHTML = `
                <div>
                    <div class="list-item-title">${service.name}</div>
                    <div class="list-item-detail">${service.type}</div>
                </div>
            `;
            list.appendChild(item);
        });

        if (data.services.length === 0) {
            list.innerHTML = '<p class="loading">No services found</p>';
        }
    }

    /**
     * Load health status
     */
    async loadHealth() {
        const data = await this.apiCall('/health');
        if (!data) return;

        const container = document.getElementById('health-status');
        container.innerHTML = '';

        Object.entries(data.checks).forEach(([name, check]) => {
            const item = document.createElement('div');
            item.className = `health-item ${check.status}`;
            item.innerHTML = `
                <div class="health-title">${name.replace(/_/g, ' ')}</div>
                <div class="health-message">${check.message}</div>
            `;
            container.appendChild(item);
        });
    }

    /**
     * Update metrics display from WebSocket
     */
    updateMetricsDisplay(data) {
        const cpu = data.cpu_percent || 0;
        const memory = data.memory_mb || 0;
        const temp = data.temperature_c || 0;

        // Smoothly update progress bars
        const updateValue = (id, bar, value, max) => {
            document.getElementById(id).textContent = value.toFixed(1) + (bar === 'temp' ? '°C' : '%');
            document.getElementById(bar).style.width = Math.min((value / max) * 100, 100) + '%';
        };

        updateValue('cpu-usage', 'cpu-bar', cpu, 100);
        updateValue('memory-usage', 'memory-bar', memory, 4000);
        updateValue('temperature', 'temp-bar', temp, 100);
    }

    /**
     * Show toast notification
     */
    showToast(message, type = 'info') {
        const container = document.getElementById('toast-container');
        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        toast.innerHTML = `
            <span>${message}</span>
            <button onclick="this.parentElement.remove()" style="background: none; border: none; cursor: pointer;">×</button>
        `;
        container.appendChild(toast);

        setTimeout(() => {
            toast.remove();
        }, 3000);
    }

    /**
     * Format duration in seconds to readable format
     */
    formatDuration(seconds) {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        return `${hours}h ${minutes}m`;
    }
}

// Initialize dashboard when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    new DashboardClient();
});
