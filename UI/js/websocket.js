const socket = new WebSocket('ws://your_ip:8080');

socket.onopen = function() {
    console.log('WebSocket is connected.');
};

socket.onmessage = function(event) {
    console.log('Message from server: ', event.data);
    if (event.data.startsWith('SPEED_')) {
        const speed = event.data.split('_')[1];
        document.getElementById('speed').innerText = speed;
    }
};

socket.onclose = function() {
    console.log('WebSocket is closed.');
};

function sendCommand(command) {
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(command);
    } else {
        console.error('WebSocket is not open. Ready state: ' + socket.readyState);
    }
}
