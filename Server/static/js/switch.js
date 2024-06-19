document.getElementById('toggle-switch').addEventListener('change', function() {
    if (this.checked) {
        fetch('/mode', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ 'mode': 'auto' })
        })
        .then(response => response.json())
        .catch(error => {
            console.error('Error:', error);
        });
    } else {
        fetch('/mode', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ 'mode': 'manual' })
        })
        .then(response => response.json())
        .catch(error => {
            console.error('Error:', error);
        });
    }
});
