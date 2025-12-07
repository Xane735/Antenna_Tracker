/**
 * tracker-client.js - Client-side JavaScript for Antenna Tracker UI
 * Handles WebSocket connections and data formatting
 */

// ============= Configuration =============
const WS_RECONNECT_DELAY = 2000; // ms
const UPDATE_ANIMATION_DURATION = 300; // ms

// ============= WebSocket Connection =============
class TrackerWebSocket {
    constructor(url) {
        this.url = url;
        this.ws = null;
        this.reconnectTimer = null;
        this.onMessage = null;
        this.onStatusChange = null;
        this.connect();
    }

    connect() {
        try {
            this.ws = new WebSocket(this.url);
            
            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.updateConnectionStatus(true);
                if (this.reconnectTimer) {
                    clearTimeout(this.reconnectTimer);
                    this.reconnectTimer = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (this.onMessage) {
                        this.onMessage(data);
                    }
                } catch (error) {
                    console.error('Error parsing message:', error);
                }
            };

            this.ws.onclose = () => {
                console.log('WebSocket disconnected');
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
        } catch (error) {
            console.error('Failed to create WebSocket:', error);
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (!this.reconnectTimer) {
            this.reconnectTimer = setTimeout(() => {
                console.log('Attempting to reconnect...');
                this.connect();
            }, WS_RECONNECT_DELAY);
        }
    }

    updateConnectionStatus(connected) {
        if (this.onStatusChange) {
            this.onStatusChange(connected);
        }
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
        }
    }
}

// ============= Data Formatting Utilities =============

/**
 * Format latitude/longitude with proper precision
 */
function formatLatLon(value) {
    if (value === null || value === undefined) return '--';
    return value.toFixed(7) + '°';
}

/**
 * Format altitude in meters
 */
function formatAltitude(value) {
    if (value === null || value === undefined) return '--';
    return value.toFixed(2) + ' m';
}

/**
 * Format angle in degrees
 */
function formatAngle(value) {
    if (value === null || value === undefined) return '--';
    return value.toFixed(2) + '°';
}

/**
 * Format distance with appropriate units (m or km)
 */
function formatDistance(meters) {
    if (meters === null || meters === undefined) return '--';
    
    if (meters < 1000) {
        return meters.toFixed(1) + ' m';
    } else {
        return (meters / 1000).toFixed(2) + ' km';
    }
}

/**
 * Format GPS accuracy (eph/epv)
 */
function formatAccuracy(eph, epv) {
    if (eph === null && epv === null) return '--';
    
    const hAcc = eph !== null ? eph.toFixed(2) : '--';
    const vAcc = epv !== null ? epv.toFixed(2) : '--';
    
    return `${hAcc} / ${vAcc} m`;
}

/**
 * Format GPS fix type with color coding
 */
function formatFixType(fixType) {
    if (fixType === null || fixType === undefined) return '--';
    
    const fixTypes = {
        0: 'No Fix',
        1: 'No Fix',
        2: '2D Fix',
        3: '3D Fix',
        4: 'DGPS',
        5: 'RTK Float',
        6: 'RTK Fixed'
    };
    
    return fixTypes[fixType] || 'Unknown';
}

/**
 * Get color class based on fix type quality
 */
function getFixTypeClass(fixType) {
    if (fixType === null || fixType === undefined) return '';
    if (fixType >= 5) return 'good'; // RTK
    if (fixType >= 3) return 'warning'; // 3D Fix or DGPS
    return 'bad'; // Poor fix
}

/**
 * Get color class based on satellite count
 */
function getSatelliteClass(sats) {
    if (sats === null || sats === undefined) return '';
    if (sats >= 10) return 'good';
    if (sats >= 6) return 'warning';
    return 'bad';
}

/**
 * Format speed in m/s and km/h
 */
function formatSpeed(metersPerSecond) {
    if (metersPerSecond === null || metersPerSecond === undefined) return '--';
    
    const ms = metersPerSecond.toFixed(1);
    const kmh = (metersPerSecond * 3.6).toFixed(1);
    
    return `${ms} m/s (${kmh} km/h)`;
}

/**
 * Format heading with cardinal direction
 */
function formatHeading(degrees) {
    if (degrees === null || degrees === undefined) return '--';
    
    const cardinals = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
                      'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
    
    const index = Math.round(((degrees % 360) / 22.5)) % 16;
    const cardinal = cardinals[index];
    
    return `${degrees.toFixed(1)}° (${cardinal})`;
}

/**
 * Format timestamp as relative time
 */
function formatRelativeTime(timestamp) {
    if (!timestamp) return '--';
    
    const now = Date.now() / 1000;
    const diff = now - timestamp;
    
    if (diff < 1) return 'Just now';
    if (diff < 60) return `${Math.floor(diff)}s ago`;
    if (diff < 3600) return `${Math.floor(diff / 60)}m ago`;
    return `${Math.floor(diff / 3600)}h ago`;
}

/**
 * Calculate distance between two GPS coordinates (Haversine formula)
 */
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Earth's radius in meters
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}

/**
 * Calculate bearing between two GPS coordinates
 */
function calculateBearing(lat1, lon1, lat2, lon2) {
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
              Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    
    const θ = Math.atan2(y, x);
    
    return (θ * 180 / Math.PI + 360) % 360; // Bearing in degrees
}

// ============= UI Update Functions =============

/**
 * Update DOM element with animation
 */
function updateElement(id, value, className = null) {
    const element = document.getElementById(id);
    if (!element) return;
    
    // Add flash animation on change
    if (element.textContent !== value) {
        element.style.transition = 'none';
        element.style.opacity = '0.5';
        
        setTimeout(() => {
            element.textContent = value;
            element.style.transition = `opacity ${UPDATE_ANIMATION_DURATION}ms`;
            element.style.opacity = '1';
        }, 50);
    }
    
    // Update class if provided
    if (className !== null) {
        element.className = 'value ' + className;
    }
}

/**
 * Update base station GPS display
 */
function updateBaseGPS(data) {
    if (!data) {
        updateElement('baseLat', '--');
        updateElement('baseLon', '--');
        updateElement('baseAlt', '--');
        updateElement('baseAccuracy', '--');
        updateElement('baseFix', '--');
        updateElement('baseSats', '--');
        return;
    }
    
    updateElement('baseLat', formatLatLon(data.lat));
    updateElement('baseLon', formatLatLon(data.lon));
    updateElement('baseAlt', formatAltitude(data.alt));
    updateElement('baseAccuracy', formatAccuracy(data.eph, data.epv));
    updateElement('baseFix', formatFixType(data.fix_type), getFixTypeClass(data.fix_type));
    updateElement('baseSats', data.sats || '--', getSatelliteClass(data.sats));
}

/**
 * Update drone GPS display
 */
function updateDroneGPS(data) {
    if (!data) {
        updateElement('droneLat', '--');
        updateElement('droneLon', '--');
        updateElement('droneAlt', '--');
        updateElement('droneAccuracy', '--');
        updateElement('droneHeading', '--');
        updateElement('droneSpeed', '--');
        updateElement('droneSats', '--');
        return;
    }
    
    updateElement('droneLat', formatLatLon(data.lat));
    updateElement('droneLon', formatLatLon(data.lon));
    updateElement('droneAlt', formatAltitude(data.alt));
    updateElement('droneAccuracy', formatAccuracy(data.eph, data.epv));
    updateElement('droneHeading', formatHeading(data.heading));
    updateElement('droneSpeed', formatSpeed(data.speed));
    updateElement('droneSats', data.sats || '--', getSatelliteClass(data.sats));
}

/**
 * Update tracker state display
 */
function updateTrackerState(data) {
    // Distance
    updateElement('distance', formatDistance(data.distance));
    
    // World angles
    const worldAngles = (data.world_az !== null && data.world_el !== null)
        ? `${formatAngle(data.world_az)} / ${formatAngle(data.world_el)}`
        : '--';
    updateElement('worldAngles', worldAngles);
    
    // Servo angles
    const servoAngles = (data.servo_az !== null && data.servo_el !== null)
        ? `${formatAngle(data.servo_az)} / ${formatAngle(data.servo_el)}`
        : '--';
    updateElement('servoAngles', servoAngles);
    
    // Flip status
    const flipStatus = data.used_flip ? 'FLIPPED' : 'NORMAL';
    updateElement('flipStatus', flipStatus, data.used_flip ? 'warning' : '');
    
    // Base mode
    updateElement('baseMode', data.base_mode.toUpperCase());
    
    // Last update
    updateElement('lastUpdate', formatRelativeTime(data.timestamp));
    
    // Tracking status
    const statusBadge = document.getElementById('trackingStatus');
    if (statusBadge) {
        if (data.is_tracking) {
            statusBadge.textContent = 'TRACKING';
            statusBadge.className = 'status-badge status-tracking';
        } else {
            statusBadge.textContent = 'IDLE';
            statusBadge.className = 'status-badge status-idle';
        }
    }
}

/**
 * Update connection status indicator
 */
function updateConnectionStatus(connected) {
    const indicator = document.getElementById('connectionStatus');
    if (!indicator) return;
    
    if (connected) {
        indicator.textContent = '● Connected';
        indicator.className = 'connection-status connected';
    } else {
        indicator.textContent = '● Disconnected';
        indicator.className = 'connection-status disconnected';
    }
}

/**
 * Main update handler for all data
 */
function handleDataUpdate(data) {
    updateBaseGPS(data.base);
    updateDroneGPS(data.drone);
    updateTrackerState(data);
    
    // Optional: Update map if implemented
    if (typeof updateMap === 'function') {
        updateMap(data);
    }
}

// ============= Map Integration Utilities =============

/**
 * Initialize Leaflet map (example implementation)
 * Uncomment and modify based on your preferred mapping library
 */
function initializeLeafletMap() {
    // Example Leaflet.js initialization
    // Requires: <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    //           <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    
    /*
    const map = L.map('map').setView([13.0276802, 77.5629616], 13);
    
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);
    
    const baseMarker = L.marker([0, 0], {
        icon: L.divIcon({
            className: 'base-marker',
            html: '📍',
            iconSize: [30, 30]
        })
    }).addTo(map);
    
    const droneMarker = L.marker([0, 0], {
        icon: L.divIcon({
            className: 'drone-marker',
            html: '✈️',
            iconSize: [30, 30]
        })
    }).addTo(map);
    
    const trackingLine = L.polyline([], {
        color: '#60a5fa',
        weight: 2,
        dashArray: '5, 5'
    }).addTo(map);
    
    return { map, baseMarker, droneMarker, trackingLine };
    */
}

/**
 * Update map markers and tracking line
 */
function updateMap(data) {
    // Example implementation for Leaflet
    /*
    if (!window.mapObjects) return;
    
    const { map, baseMarker, droneMarker, trackingLine } = window.mapObjects;
    
    if (data.base && data.base.lat && data.base.lon) {
        const basePos = [data.base.lat, data.base.lon];
        baseMarker.setLatLng(basePos);
        
        if (data.drone && data.drone.lat && data.drone.lon) {
            const dronePos = [data.drone.lat, data.drone.lon];
            droneMarker.setLatLng(dronePos);
            
            // Update tracking line
            trackingLine.setLatLngs([basePos, dronePos]);
            
            // Auto-fit bounds
            const bounds = L.latLngBounds([basePos, dronePos]);
            map.fitBounds(bounds, { padding: [50, 50] });
        }
    }
    */
}

/**
 * Initialize Google Maps (alternative implementation)
 */
function initializeGoogleMap() {
    // Example Google Maps initialization
    // Requires: <script src="https://maps.googleapis.com/maps/api/js?key=YOUR_API_KEY"></script>
    
    /*
    const mapOptions = {
        center: { lat: 13.0276802, lng: 77.5629616 },
        zoom: 13,
        mapTypeId: 'hybrid'
    };
    
    const map = new google.maps.Map(document.getElementById('map'), mapOptions);
    
    const baseMarker = new google.maps.Marker({
        map: map,
        icon: {
            url: 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="30" height="30"><text y="20" font-size="24">📍</text></svg>',
            scaledSize: new google.maps.Size(30, 30)
        }
    });
    
    const droneMarker = new google.maps.Marker({
        map: map,
        icon: {
            url: 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="30" height="30"><text y="20" font-size="24">✈️</text></svg>',
            scaledSize: new google.maps.Size(30, 30)
        }
    });
    
    const trackingLine = new google.maps.Polyline({
        map: map,
        strokeColor: '#60a5fa',
        strokeWeight: 2,
        strokeOpacity: 0.8
    });
    
    return { map, baseMarker, droneMarker, trackingLine };
    */
}

// ============= Initialization =============

/**
 * Initialize the tracker client
 */
function initializeTrackerClient() {
    console.log('Initializing Antenna Tracker Client...');
    
    // Determine WebSocket URL
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    // Create WebSocket connection
    const ws = new TrackerWebSocket(wsUrl);
    
    // Set up message handler
    ws.onMessage = handleDataUpdate;
    
    // Set up connection status handler
    ws.onStatusChange = updateConnectionStatus;
    
    // Initialize map (uncomment based on your choice)
    // window.mapObjects = initializeLeafletMap();
    // window.mapObjects = initializeGoogleMap();
    
    // Store WebSocket instance globally for debugging
    window.trackerWS = ws;
    
    console.log('Tracker client initialized');
}

// ============= Export Functions for External Use =============

// Make utilities available globally
window.TrackerUtils = {
    formatLatLon,
    formatAltitude,
    formatAngle,
    formatDistance,
    formatAccuracy,
    formatFixType,
    formatSpeed,
    formatHeading,
    formatRelativeTime,
    calculateDistance,
    calculateBearing,
    getFixTypeClass,
    getSatelliteClass
};

// ============= Auto-initialize on page load =============
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeTrackerClient);
} else {
    initializeTrackerClient();
}