document.addEventListener('DOMContentLoaded', (event) => {
    function updateSpeed() {
        fetch('/get_speed')
            .then(response => response.json())
            .then(data => {
                document.getElementById('speed').textContent = data.speed;
                console.log("speed", data.speed);
            })
            .catch(error => console.error('Error fetching speed:', error));
            
    }

    // Update the speed every 10 milliseconds
    setInterval(updateSpeed, 10);
});