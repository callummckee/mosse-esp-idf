const frameWidth = 160;
const frameHeight = 120;
const buf_size = 300;
const maxTargetWidth = 2**(Math.floor(Math.log2(frameWidth)));
const maxTargetHeight = 2**(Math.floor(Math.log2(frameHeight)));
console.log(`maxTargetWidth ${maxTargetWidth}, maxTargetHeight ${maxTargetHeight}`);

const frameHistory = new Array(buf_size);

const stream_socket = new WebSocket(`ws://${window.location.hostname}:80/stream_ws`);
const target_socket = new WebSocket(`ws://${window.location.hostname}:80/target_ws`);

const drawCanvas = document.getElementById('draw_canvas');
const confirmedCanvas = document.getElementById('confirmed_canvas');
const streamCtx =  document.getElementById('stream_canvas').getContext('2d');
const targetCanvas = document.getElementById('target_canvas');
const drawCtx = drawCanvas.getContext('2d');
const confirmCtx = confirmedCanvas.getContext('2d');
const targetCtx = targetCanvas.getContext('2d');

let isDrawing = false;
let frameCounter = 0;
let confirmedFrame = -1;
let startX, startY;
let canConfirm = false;
let confirmed = false;
let drawRoi, confirmRoi;
let pos_x, pos_y

window.onload = async () => {
    const response = await fetch('/cfg');
    const data = await response.json();
    document.querySelector('[name="pan_kp"]').value = data.pan_kp.toFixed(4);
    document.querySelector('[name="tilt_kp"]').value = data.tilt_kp.toFixed(4);
    document.querySelector('[name="pan_kd"]').value = data.pan_kd.toFixed(4);
    document.querySelector('[name="tilt_kd"]').value = data.tilt_kd.toFixed(4);
    document.querySelector('[name="pan_ki"]').value = data.pan_ki.toFixed(4);
    document.querySelector('[name="tilt_ki"]').value = data.tilt_ki.toFixed(4);
};

const form = document.getElementById('paramForm');
form.addEventListener('submit', async (event) => {
    event.preventDefault();

    const formData = new FormData(form);
    const data = Object.fromEntries(formData);

    for (let key in data) data[key] = parseFloat(data[key]) || 0;
    await fetch('/update', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data) 
    });
});


stream_socket.onmessage = async function(event) {
    const buffer = await event.data.arrayBuffer();
    const pixels = new Uint8Array(buffer);
    if (confirmed) {
        pos_x = pixels[0];
        pos_y = pixels[1];
    }
    const imageData = pixels.subarray(2);
    frameHistory[frameCounter % buf_size] = new Uint8Array(imageData);
    frameCounter++;
    const blob = new Blob([imageData], { type: 'image/jpeg' });
    const bitmap = await createImageBitmap(blob);
    if (streamCtx) {
        streamCtx.drawImage(bitmap, 0, 0);
    }
    if (confirmed) {
        confirmCtx.clearRect(0, 0, drawCanvas.width, drawCanvas.height);
        confirmCtx.strokeStyle = '#008000';
        confirmCtx.lineWidth = 1;
        confirmCtx.strokeRect(pos_x, pos_y, confirmRoi.w, confirmRoi.h);
    }
}

target_socket.onmessage = async function(event) {
}

document.addEventListener('keydown', async (e) => {
    if(e.key == "Enter") {
        if (confirmed) {
            confirmed = false;
            confirmedFrame = -1;
            confirmCtx.clearRect(0, 0, confirmedCanvas.width, confirmedCanvas.height);
        }
        else if (canConfirm) {
            confirmed = true;
            canConfirm = false;
            drawCtx.clearRect(0, 0, drawCanvas.width, drawCanvas.height);
            confirmCtx.strokeStyle = '#FF0000';
            confirmCtx.lineWidth = 1;
            confirmCtx.setLineDash([2, 2]); 
            confirmCtx.strokeRect(drawRoi.x, drawRoi.y, drawRoi.w, drawRoi.h);
            confirmedFrame = (frameCounter - 1) % buf_size;
            confirmRoi = {...drawRoi };
            console.log("confirmRoi: ", confirmRoi);
            console.log("confirmed frame length: ", frameHistory[confirmedFrame].length);
            const blob = new Blob([frameHistory[confirmedFrame]], { type: 'image/jpeg' });
            const bitmap = await createImageBitmap(blob);
            targetCtx.clearRect(0, 0, targetCanvas.width, targetCanvas.height);
            targetCtx.drawImage(bitmap, confirmRoi.x, confirmRoi.y, confirmRoi.w, confirmRoi.h, 0, 0, confirmRoi.w, confirmRoi.h);
            const imageData = targetCtx.getImageData(0, 0, confirmRoi.w, confirmRoi.h);
            const rawPixels = new Uint8Array(imageData.width * imageData.height);
            for (let i = 0; i < rawPixels.length; i++) {
                rawPixels[i] = imageData.data[i * 4];
            }
            const combinedData = new Uint8Array(rawPixels.length + 4);
            combinedData[0] = confirmRoi.h;
            combinedData[1] = confirmRoi.w;
            combinedData[2] = confirmRoi.x;
            combinedData[3] = confirmRoi.y;
            combinedData.set(rawPixels, 4);
            target_socket.send(combinedData.buffer);
        }
    }
});

// Capture Mouse Events
drawCanvas.addEventListener('mousedown', (e) => {
    if (!confirmed) {
        isDrawing = true;
    }
    canConfirm = false;
    // offsetX/Y gives coords relative to the canvas element itself
    startX = e.offsetX * (drawCanvas.width / drawCanvas.clientWidth);
    startY = e.offsetY * (drawCanvas.height / drawCanvas.clientHeight);
});

drawCanvas.addEventListener('mousemove', (e) => {
    if (!isDrawing) return;

    let currentX = e.offsetX * (drawCanvas.width / drawCanvas.clientWidth);
    let currentY = e.offsetY * (drawCanvas.height / drawCanvas.clientHeight);

    let dx = currentX - startX;
    if (Math.abs(dx) > maxTargetWidth) {
        currentX = startX + Math.sign(dx) * maxTargetWidth;
    }

    let dy = currentY - startY;
    if (Math.abs(dy) > maxTargetHeight) {
        currentY = startY + Math.sign(dy) * maxTargetHeight;
    }

    drawCtx.clearRect(0, 0, drawCanvas.width, drawCanvas.height);
    drawCtx.strokeStyle = '#00ffcc';
    drawCtx.lineWidth = 1;
    drawCtx.setLineDash([2, 2]);
    drawCtx.strokeRect(startX, startY, currentX - startX, currentY - startY);
});

drawCanvas.addEventListener('mouseup', (e) => {
    if (!isDrawing) return;
    isDrawing = false;
    canConfirm = true;

    let endX = e.offsetX * (drawCanvas.width / drawCanvas.clientWidth);
    let endY = e.offsetY * (drawCanvas.height / drawCanvas.clientHeight);

    let dx = endX - startX;
    if (Math.abs(dx) > maxTargetWidth) {
        endX = startX + Math.sign(dx) * maxTargetWidth;
    }

    let dy = endY - startY;
    if (Math.abs(dy) > maxTargetHeight) {
        endY = startY + Math.sign(dy) * maxTargetHeight;
    }

    drawRoi = {
        x: Math.round(Math.min(startX, endX)),
        y: Math.round(Math.min(startY, endY)),
        w: Math.round(Math.abs(endX - startX)),
        h: Math.round(Math.abs(endY - startY))
    };

    console.log("ROI Selected (Sensor Coords):", drawRoi);
});
