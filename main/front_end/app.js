const socket = new WebSocket(`ws://${window.location.hostname}:80/ws`);
const views = {
    0: document.getElementById('affine-1').getContext('2d'),
    1: document.getElementById('affine-2').getContext('2d'),
    2: document.getElementById('affine-3').getContext('2d'),
    3: document.getElementById('affine-4').getContext('2d'),
    4: document.getElementById('affine-5').getContext('2d'),
    5: document.getElementById('affine-6').getContext('2d'),
    6: document.getElementById('affine-7').getContext('2d'),
    7: document.getElementById('affine-8').getContext('2d'),
    8: document.getElementById('affine-9').getContext('2d')
}


socket.onmessage = async function(event) {
    const buffer = await event.data.arrayBuffer();
    console.log(`Buffer received: ${buffer.bytelength} bytes`);
    const view = new DataView(buffer);
    const id = view.getUint8(0);
    console.log(`Processing image ID: ${id}`);
    const pixels = new Uint8Array(buffer, 5);

    const imgData = new ImageData(96, 96);
    for (let i = 0; i < pixels.length; i++) {
        const val = pixels[i];
        const stride = i * 4;
        imgData.data[stride] = val;
        imgData.data[stride + 1] = val;
        imgData.data[stride + 2] = val;
        imgData.data[stride + 3] = 255;
    }

    if (views[id]) {
        views[id].putImageData(imgData, 0, 0);
    }
}




