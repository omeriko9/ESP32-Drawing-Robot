// script.js

const canvas = document.getElementById('drawingCanvas');
const ctx = canvas.getContext('2d');
const smoothingCheckbox = document.getElementById('smoothingCheckbox');
const backButton = document.getElementById('backButton');
const clearButton = document.getElementById('clearButton');
const uploadDrawingButton = document.getElementById('uploadDrawingButton');
const drawButton = document.getElementById('drawButton');
const penUpButton = document.getElementById('btnPickPenUp');
const penDownButton = document.getElementById('btnPickPenDown');
const drawCircleButton = document.getElementById('btnDrawCircle');
const drawingZone = document.getElementById('drawingZone');
const savedStrokes = []; // Stores all strokes

let xMin = -10, xMax = 10;
let yMin = 5, yMax = 24;
let isDrawing = false;
let points = [];

const resizeCanvas = () => {
  canvas.width = drawingZone.clientWidth;
  canvas.height = drawingZone.clientHeight;
  ctx.lineWidth = 5;
  ctx.lineCap = 'round';
  ctx.strokeStyle = 'black';
};
resizeCanvas();

const startDrawing = (event) => {
  isDrawing = true;
  points = [{ x: getX(event), y: getY(event) }];
};

const stopDrawing = () => {
  if (!isDrawing) return;
  isDrawing = false;
  if (points.length > 0) {
    savedStrokes.push([...points]);
  }
  points = [];
  redrawCanvas();
};

const draw = (event) => {
  if (!isDrawing) return;
  points.push({ x: getX(event), y: getY(event) });
  redrawCanvas();
};

const redrawCanvas = () => {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  savedStrokes.forEach((stroke) => drawStroke(stroke, smoothingCheckbox.checked));
  drawStroke(points, smoothingCheckbox.checked);
};

const drawStroke = (stroke, smooth) => {
  if (stroke.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(stroke[0].x, stroke[0].y);

  if (smooth) {
    for (let i = 1; i < stroke.length - 1; i++) {
      const midPoint = {
        x: (stroke[i].x + stroke[i + 1].x) / 2,
        y: (stroke[i].y + stroke[i + 1].y) / 2,
      };
      ctx.quadraticCurveTo(stroke[i].x, stroke[i].y, midPoint.x, midPoint.y);
    }
    ctx.lineTo(stroke[stroke.length - 1].x, stroke[stroke.length - 1].y);
  } else {
    for (let i = 1; i < stroke.length; i++) {
      ctx.lineTo(stroke[i].x, stroke[i].y);
    }
  }
  ctx.stroke();
};

backButton.addEventListener('click', () => {
  if (savedStrokes.length > 0) {
    savedStrokes.pop();
    redrawCanvas();
  }
});

clearButton.addEventListener('click', () => {
  savedStrokes.length = 0;
  redrawCanvas();
});

penUpButton.addEventListener('click', () => {
  fetch("/penUp", { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
});

penDownButton.addEventListener('click', () => {
  fetch("/penUp?down=true", { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
});

drawCircleButton.addEventListener('click', () => {
  const url = `/drawCircle?x=${circleX.value}&y=${circleY.value}&r=${circleR.value}&s=${circleSteps.value}&lc=${leftServoCorrection.value}&rc=${rightServoCorrection.value}`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
});

const setCorrections = () => {
  const url = `/setCorrections?lc=${leftServoCorrection.value}&rc=${rightServoCorrection.value}`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
};

drawButton.addEventListener("click", () => {
  uploadDrawing();
  const speedDelay = document.getElementById('speedDelay');
  const url = `/draw?speedDelay=${speedDelay ? speedDelay.value : 0}&penUp=${penUpAngle.value}&penDown=${penDownAngle.value}`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
});

const interpolate = (value, srcMin, srcMax, dstMin, dstMax) => {
  if (srcMax === srcMin) {
    console.error("Source range cannot be zero.");
    return dstMin; // Safe fallback
  }
  return dstMin + ((value - srcMin) * (dstMax - dstMin)) / (srcMax - srcMin);
};

let mirrorXAxis = true;
let mirrorYAxis = true;

const uploadDrawing = () => {
  if (!savedStrokes || savedStrokes.length === 0) {
    console.error("No strokes to upload.");
    return;
  }

  const srcXMin = 0;
  const srcXMax = canvas.width;
  const srcYMin = 0;
  const srcYMax = canvas.height;

  let totalPoints = 0;
  for (const stroke of savedStrokes) {
    totalPoints += stroke.length;
  }

  const totalFloatCount = totalPoints * 2 + savedStrokes.length * 2;
  const floatView = new Float32Array(totalFloatCount);

  let index = 0;
  for (const stroke of savedStrokes) {
    for (const coord of stroke) {
      const interpolatedX = interpolate(coord.x, srcXMin, srcXMax, xMin, xMax);
      const interpolatedY = interpolate(coord.y, srcYMin, srcYMax, yMax, yMin);
      const finalX = mirrorYAxis ? xMin + xMax - interpolatedX : interpolatedX;
      const finalY = mirrorXAxis ? yMin + yMax - interpolatedY : interpolatedY;
      floatView[index++] = finalX;
      floatView[index++] = finalY;
    }
    floatView[index++] = -300;
    floatView[index++] = -300;
  }

  console.log("Interpolated and serialized data:", floatView);

  fetch("/upload2", {
    method: "POST",
    headers: { "Content-Type": "application/octet-stream" },
    body: floatView,
  })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
};

const getX = (event) => {
  const rect = canvas.getBoundingClientRect();
  return (event.touches ? event.touches[0].clientX : event.clientX) - rect.left;
};

const getY = (event) => {
  const rect = canvas.getBoundingClientRect();
  return (event.touches ? event.touches[0].clientY : event.clientY) - rect.top;
};

canvas.addEventListener('mousedown', startDrawing);
canvas.addEventListener('mouseup', stopDrawing);
canvas.addEventListener('mousemove', draw);
canvas.addEventListener('touchstart', startDrawing);
canvas.addEventListener('touchend', stopDrawing);
canvas.addEventListener('touchmove', draw);

window.addEventListener('resize', () => {
  const tempCanvas = document.createElement('canvas');
  tempCanvas.width = canvas.width;
  tempCanvas.height = canvas.height;
  tempCanvas.getContext('2d').drawImage(canvas, 0, 0);
  resizeCanvas();
  ctx.drawImage(tempCanvas, 0, 0);
});

const handleFileUpload = (event) => {
  const file = event.target.files[0];
  if (!file) {
    alert("Please select a file to upload.");
    return;
  }

  const formData = new FormData();
  formData.append("file", file, file.name);

  fetch("/uploadFile", {
    method: "POST",
    body: formData,
  })
    .then(response => response.text())
    .then(data => {
      console.log("Response from ESP32:", data);
      alert(data);
    })
    .catch(error => console.error("Error:", error));
};

const toggleControls = () => {
  const controlsDiv = document.getElementById('controls');
  let isHidden = controlsDiv.style.display === 'none' || controlsDiv.style.display === '';
  controlsDiv.style.display = isHidden ? 'flex' : 'none';
}
