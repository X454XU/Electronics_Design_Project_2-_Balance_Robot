document.addEventListener('keydown', function(event) {
    const key = event.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
        document.getElementById(`key-${key}`).classList.add('active');
    }
});

document.addEventListener('keyup', function(event) {
    const key = event.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
        document.getElementById(`key-${key}`).classList.remove('active');
    }
});
