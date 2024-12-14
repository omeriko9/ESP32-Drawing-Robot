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


let mirrorXAxis = true;
let mirrorYAxis = true;

// Update the existing resizeCanvas function with the new logic
const resizeCanvas = () => {
  const aspectRatio = (xMax - xMin) / (yMax - yMin);

  // Get available space
  const maxHeight = window.innerHeight * 0.6; // 60% of viewport height
  const maxWidth = window.innerWidth * 0.8;  // 80% of viewport width

  // Calculate dimensions based on the aspect ratio
  let canvasHeight = maxHeight;
  let canvasWidth = canvasHeight * aspectRatio;

  if (canvasWidth > maxWidth) {
    canvasWidth = maxWidth;
    canvasHeight = canvasWidth / aspectRatio;
  }

  // Apply the calculated dimensions to the canvas
  canvas.width = canvasWidth;
  canvas.height = canvasHeight;

  // Dynamically adjust the parent div
  drawingZone.style.width = `${canvasWidth}px`;
  drawingZone.style.height = `${canvasHeight}px`;

  // Update canvas context styles
  ctx.lineWidth = 5;
  ctx.lineCap = 'round';
  ctx.strokeStyle = 'black';

  console.log(`Canvas resized to: ${canvasWidth} x ${canvasHeight}`);
  console.log(`Drawing zone resized to: ${drawingZone.style.width} x ${drawingZone.style.height}`);
}


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

const setCorrections = () => {
  const url = `/setCorrections?lc=${leftServoCorrection.value}&rc=${rightServoCorrection.value}`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
};


const getRobotParams = () => {
  console.log(`getting robot parameters...`);
  fetch("/get_params", {
    method: "GET"
  })
    .then(response => response.text())
    .then(data => {
      // Split the response into values
      const [t1correction, t2correction, link1, link2, penUpAngle, penDownAngle, baseWidth, speedDelay] = data.split(";").map(parseFloat);

      // Update HTML input fields
      document.getElementById("leftServoCorrection").value = t1correction;
      document.getElementById("rightServoCorrection").value = t2correction;
      document.getElementById("link1Length").value = link1;
      document.getElementById("link2Length").value = link2;
      document.getElementById("penUpAngle").value = penUpAngle;
      document.getElementById("penDownAngle").value = penDownAngle;
      document.getElementById("bwLength").value = baseWidth;
      document.getElementById("speedDelay").value = speedDelay; // Ensure speedDelay is included
      console.log(`got response: l1corr: ${t1correction}, etc`);
    })
    .catch(error => {
      console.error("Error fetching robot parameters:", error);
    });
};

const sendRobotParams = () => {
  console.log(`collecting robot parameters...`);

  // Collect values from the HTML input fields
  const t1correction = parseFloat(document.getElementById("leftServoCorrection").value) || 0;
  const t2correction = parseFloat(document.getElementById("rightServoCorrection").value) || 0;
  const link1 = parseFloat(document.getElementById("link1Length").value) || 0;
  const link2 = parseFloat(document.getElementById("link2Length").value) || 0;
  const penUpAngle = parseFloat(document.getElementById("penUpAngle").value) || 0;
  const penDownAngle = parseFloat(document.getElementById("penDownAngle").value) || 0;
  const baseWidth = parseFloat(document.getElementById("bwLength").value) || 0;
  const drawDelay = parseInt(document.getElementById("speedDelay").value, 10) || 0; // Ensure integer

  // Construct the URL with parameters
  const url = `/set_params?t1=${t1correction}&t2=${t2correction}&link1=${link1}&link2=${link2}&penUp=${penUpAngle}&penDown=${penDownAngle}&baseWidth=${baseWidth}&delay=${drawDelay}`;

  console.log(`sending params via GET: ${url}`);

  // Send the GET request
  fetch(url, {
    method: "GET"
  })
    .then(response => {
      if (response.ok) {
        console.log("Parameters sent successfully");
      } else {
        console.error("Failed to send parameters");
      }
    })
    .catch(error => {
      console.error("Error sending parameters:", error);
    });
};

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

const interpolate = (value, srcMin, srcMax, dstMin, dstMax) => {
  if (srcMax === srcMin) {
    console.error("Source range cannot be zero.");
    return dstMin; // Safe fallback
  }
  return dstMin + ((value - srcMin) * (dstMax - dstMin)) / (srcMax - srcMin);
};

const prepareDrawing = () => {
  if (!savedStrokes || savedStrokes.length === 0) {
    console.error("No strokes to upload.");
    return;
  }

  const totalPoints = savedStrokes.reduce((sum, stroke) => sum + stroke.length, 0);
  const totalFloatCount = totalPoints * 2 + savedStrokes.length * 2;
  const floatView = new Float32Array(totalFloatCount);

  let index = 0;
  for (const stroke of savedStrokes) {
    for (const coord of stroke) {
      let interpolatedX = interpolate(coord.x, 0, canvas.width, xMin, xMax);
      let interpolatedY = interpolate(coord.y, 0, canvas.height, yMax, yMin); // Note: Y inverted

      if (mirrorYAxis) interpolatedX = xMin + xMax - interpolatedX;
      if (mirrorXAxis) interpolatedY = yMin + yMax - interpolatedY;

      floatView[index++] = interpolatedX;
      floatView[index++] = interpolatedY;
    }
    // Add "pen up" marker
    floatView[index++] = -300;
    floatView[index++] = -300;
  }

  console.log("Interpolated and serialized data:", floatView);
  uploadDrawing(floatView);
};


const prepareDrawingOld = () => {
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
  
  uploadDrawing(floatView);

};

const uploadDrawing = (floatView) => {
  fetch("/uploadDrawing", {
    method: "POST",
    headers: { "Content-Type": "application/octet-stream" },
    body: floatView,
  })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
};

const sendDrawCommad = () => {

  sendRobotParams();  
  const url = `/draw`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
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

console.log('test');

/* Event Listeners */


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

  sendRobotParams();
  const url = `/drawCircle?x=${circleX.value}&y=${circleY.value}&r=${circleR.value}&s=${circleSteps.value}&lc=${leftServoCorrection.value}&rc=${rightServoCorrection.value}`;
  console.log(`URL: ${url}`);
  fetch(url, { method: "GET" })
    .then(response => response.text())
    .then(data => console.log("Response from ESP32:", data))
    .catch(error => console.error("Error during fetch:", error));
});

drawButton.addEventListener("click", () => {
  prepareDrawing();
  uploadDrawing();
  sendDrawCommad();
});

canvas.addEventListener('mousedown', startDrawing);
canvas.addEventListener('mouseup', stopDrawing);
canvas.addEventListener('mousemove', draw);
canvas.addEventListener('touchstart', startDrawing);
canvas.addEventListener('touchend', stopDrawing);
canvas.addEventListener('touchmove', draw);

document.getElementById('xMinInput').addEventListener('input', () => {
  xMin = parseFloat(document.getElementById('xMinInput').value) || xMin;
  resizeCanvas();
});

document.getElementById('xMaxInput').addEventListener('input', () => {
  xMax = parseFloat(document.getElementById('xMaxInput').value) || xMax;
  resizeCanvas();
});

document.getElementById('yMinInput').addEventListener('input', () => {
  yMin = parseFloat(document.getElementById('yMinInput').value) || yMin;
  resizeCanvas();
});

document.getElementById('yMaxInput').addEventListener('input', () => {
  yMax = parseFloat(document.getElementById('yMaxInput').value) || yMax;
  resizeCanvas();
});



// Ensure it gets called on window resize
window.addEventListener('resize', () => {
  const tempCanvas = document.createElement('canvas');
  tempCanvas.width = canvas.width;
  tempCanvas.height = canvas.height;
  tempCanvas.getContext('2d').drawImage(canvas, 0, 0);

  resizeCanvas();
  ctx.drawImage(tempCanvas, 0, 0);
});


getRobotParams();