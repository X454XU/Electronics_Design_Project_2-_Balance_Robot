// Ensure websocket.js is loaded first
if (typeof WebSocket !== 'undefined' && typeof sendCommand !== 'undefined') {
    console.log('WebSocket and sendCommand available');
} else {
    console.error('WebSocket or sendCommand not available');
}

function updateSpeed(speed) {
    document.getElementById('speed').innerText = speed;
}

// Listen for WebSocket messages and update speed if message contains speed data
socket.onmessage = function(event) {
    console.log('Message from server: ', event.data);
    if (event.data.startsWith('SPEED_')) {
        const speed = event.data.split('_')[1];
        updateSpeed(speed);
    }
};
