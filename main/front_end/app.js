const frameWidth = 160;
const frameHeight = 120;
const buf_size = 300;

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

    const currentX = e.offsetX * (drawCanvas.width / drawCanvas.clientWidth);
    const currentY = e.offsetY * (drawCanvas.height / drawCanvas.clientHeight);

    // Clear previous rectangle
    drawCtx.clearRect(0, 0, drawCanvas.width, drawCanvas.height);

    // Draw new selection box
    drawCtx.strokeStyle = '#00ffcc';
    drawCtx.lineWidth = 1;
    drawCtx.setLineDash([2, 2]); // "Marching Ants" look
    drawCtx.strokeRect(startX, startY, currentX - startX, currentY - startY);
});

drawCanvas.addEventListener('mouseup', (e) => {
    if (!isDrawing) return;
    isDrawing = false;
    canConfirm = true;

    const endX = e.offsetX * (drawCanvas.width / drawCanvas.clientWidth);
    const endY = e.offsetY * (drawCanvas.height / drawCanvas.clientHeight);

    drawRoi = {
        x: Math.round(Math.min(startX, endX)),
        y: Math.round(Math.min(startY, endY)),
        w: Math.round(Math.abs(endX - startX)),
        h: Math.round(Math.abs(endY - startY))
    };

    console.log("ROI Selected (Sensor Coords):", drawRoi);
});

// my current thinking on this is we don't pause, we store the last \approx 300 frames in a circular buffer that we access and send back to the esp32 when confirmed perhaps by click in the stream canvas, image needs to be padded up to the next square power of 2 for optimisation on the esp32, browser will do the padding
// TOMORROW LETS SEND PORTION OUT THE SELECTED FRAME AND SEND IT BACK TO THE ESP32
