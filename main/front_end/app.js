const frameWidth = 96;
const frameHeight = 96;
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
let drawRoi, confirmRoi

function extractTarget(roi, image) {
    let target = new Uint8Array(roi.w * roi.h + 2);
    target[0] = roi.h;
    target[1] = roi.w;
    for (let y = 0; y < roi.h; y++) {
        for (let x = 0; x < roi.w; x++) {
            target[2 + (y * roi.w + x)] = image[(roi.y + y) * 96 + (roi.x + x)];
        }
    }
    return target;
    
}


stream_socket.onmessage = async function(event) {
    const buffer = await event.data.arrayBuffer();
    console.log(`Buffer received: ${buffer.bytelength} bytes`);
    const pixels = new Uint8Array(buffer);
    frameHistory[frameCounter % buf_size] = new Uint8Array(pixels, pixels.length);
    frameCounter++;
    const blob = new Blob([pixels], { type: 'image/jpeg' });
    const bitmap = await createImageBitmap(blob);
    if (streamCtx) {
        streamCtx.drawImage(bitmap, 0, 0);
    }
}

target_socket.onmessage = async function(event) {
    const buffer = await event.data.arrayBuffer();
    const pixels = new Uint8Array(buffer, 2);
    const view = new DataView(buffer);
    const rows = view.getUint8(0);
    const cols = view.getUint8(1);

    const imgData = new ImageData(cols, rows);
    for (let i = 0; i < pixels.length; i++) {
        const val = pixels[i];
        const stride = i * 4;
        imgData.data[stride] = val;
        imgData.data[stride + 1] = val;
        imgData.data[stride + 2] = val;
        imgData.data[stride + 3] = 255;
    }
    if(targetCtx) {
        targetCtx.clearRect(0, 0, targetCanvas.width, targetCanvas.height);
        targetCtx.putImageData(imgData, 0, 0);
    }
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
            target_socket.send(extractTarget(confirmRoi, frameHistory[confirmedFrame]));
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

    // Final ROI in 96x96 sensor coordinates
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
