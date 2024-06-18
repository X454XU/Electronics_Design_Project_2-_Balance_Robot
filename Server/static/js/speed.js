let currentNumber = 0;

function updateNumber() {
    currentNumber++;
    document.getElementById('speed').innerText = currentNumber;
}

setInterval(updateNumber, 1000); // 每秒更新一次数值
