/**
 * OctoMap Web Viewer - Main Application
 * Connects to ROS2 via rosbridge and visualizes OctoMap
 */

(function () {
    'use strict';

    // DOM Elements
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.getElementById('status-text');
    const voxelCountEl = document.getElementById('voxel-count');
    const fpsEl = document.getElementById('fps');
    const resetViewBtn = document.getElementById('reset-view');
    const toggleAxesBtn = document.getElementById('toggle-axes');
    const toggleSubscribeBtn = document.getElementById('toggle-subscribe');
    const statsEl = document.getElementById('stats');

    // Initialize viewer
    const container = document.getElementById('canvas-container');
    const viewer = new OctomapViewer(container);

    // FPS tracking
    let frameCount = 0;
    let lastFpsUpdate = Date.now();

    // ROS connection state
    let ros = null;
    let markerSubscriber = null;
    let isSubscribed = false;

    function setStatus(status, text) {
        statusDot.className = 'status-dot ' + status;
        statusText.textContent = text;
    }

    function addDebugMsg(msg) {
        console.log(msg);
        if (!statsEl) return;

        const div = document.createElement('div');
        div.textContent = msg;
        div.style.fontSize = '11px';
        div.style.color = '#888';
        div.style.marginTop = '2px';

        // Keep only last 5 messages
        const debugMsgs = statsEl.querySelectorAll('div[style*="font-size: 11px"]');
        if (debugMsgs.length >= 5) {
            statsEl.removeChild(debugMsgs[0]);
        }
        statsEl.appendChild(div);
    }

    function connect() {
        setStatus('connecting', '连接中...');

        // 使用 CONFIG 中的动态 URL，支持局域网访问
        ros = new ROSLIB.Ros({
            url: CONFIG.ROSBRIDGE_URL
        });

        ros.on('connection', () => {
            console.log('Connected to rosbridge at ' + CONFIG.ROSBRIDGE_URL);
            setStatus('connected', '已连接');
            addDebugMsg('已连接: ' + CONFIG.ROSBRIDGE_URL);

            subscribeToMarkers();
        });

        ros.on('error', (error) => {
            console.error('Rosbridge error:', error);
            setStatus('disconnected', '连接错误');
            addDebugMsg('WebSocket 错误');
        });

        ros.on('close', () => {
            console.log('Rosbridge connection closed');
            setStatus('disconnected', '已断开');
            addDebugMsg('WebSocket 连接断开');
            isSubscribed = false;
            updateSubscribeButton();

            // Auto reconnect after 3 seconds
            setTimeout(() => {
                addDebugMsg('3秒后重连...');
                connect();
            }, 3000);
        });
    }

    function subscribeToMarkers() {
        if (markerSubscriber) {
            markerSubscriber.unsubscribe();
        }

        markerSubscriber = new ROSLIB.Topic({
            ros: ros,
            name: CONFIG.MARKER_TOPIC,
            messageType: CONFIG.MARKER_MESSAGE_TYPE,
            throttle_rate: CONFIG.THROTTLE_RATE
        });

        markerSubscriber.subscribe((message) => {
            viewer.updateFromMarkerArray(message);
            updateStats();
        });

        isSubscribed = true;
        updateSubscribeButton();
        console.log('Subscribed to ' + CONFIG.MARKER_TOPIC);
    }

    function unsubscribeFromMarkers() {
        if (markerSubscriber) {
            markerSubscriber.unsubscribe();
            markerSubscriber = null;
        }
        isSubscribed = false;
        updateSubscribeButton();
        console.log('Unsubscribed from ' + CONFIG.MARKER_TOPIC);
        addDebugMsg('已停止订阅');
    }

    function toggleSubscription() {
        if (isSubscribed) {
            unsubscribeFromMarkers();
        } else {
            if (ros && ros.isConnected) {
                subscribeToMarkers();
                addDebugMsg('已启动订阅');
            } else {
                addDebugMsg('未连接，无法订阅');
            }
        }
    }

    function updateSubscribeButton() {
        if (toggleSubscribeBtn) {
            toggleSubscribeBtn.textContent = isSubscribed ? '停止订阅' : '启动订阅';
            toggleSubscribeBtn.style.background = isSubscribed ? '#c93838' : '#38c95a';
        }
    }

    function updateStats() {
        // Update voxel count
        voxelCountEl.textContent = viewer.getVoxelCount().toLocaleString();

        // Update FPS
        frameCount++;
        const now = Date.now();
        const elapsed = now - lastFpsUpdate;

        if (elapsed >= 1000) {
            const fps = Math.round(frameCount * 1000 / elapsed);
            fpsEl.textContent = fps;
            frameCount = 0;
            lastFpsUpdate = now;
        }
    }

    // Button handlers
    if (resetViewBtn) {
        resetViewBtn.addEventListener('click', () => {
            viewer.resetView();
        });
    }

    if (toggleAxesBtn) {
        toggleAxesBtn.addEventListener('click', () => {
            viewer.toggleAxes();
        });
    }

    if (toggleSubscribeBtn) {
        toggleSubscribeBtn.addEventListener('click', toggleSubscription);
    }

    const gridHeightEl = document.getElementById('grid-height');
    if (gridHeightEl) {
        gridHeightEl.addEventListener('change', (event) => {
            viewer.setGridHeight(event.target.value);
        });
    }

    // Start connection
    connect();

})();

