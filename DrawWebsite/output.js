// Given parameters for the 5-bar mechanism
const baseWidth = 7.0 * 10;  // Distance between the two ground hinges
const link1 = 10.0 * 10;      // Length of the first link from each hinge
const link2 = 11.0 * 10;      // Length of the second link from each hinge

// CanvasOutput and context
let canvasoutput = document.getElementById('mechanismCanvas');
let ctx5bar = canvasoutput.getContext('2d');
const trailCanvas = document.getElementById('trailCanvas');
const trailCtx = trailCanvas.getContext('2d');

// Midpoint of the canvasoutput
let midXoutput = parseFloat(document.getElementById('midXoutput').value);
let midYoutput = parseFloat(document.getElementById('midYoutput').value);

// End effector position
let endXoutput = parseFloat(document.getElementById('endXoutput').value);
let endYoutput = parseFloat(document.getElementById('endYoutput').value);




// Coordinates offset and scale directions
function canvasToMechanismCoords(cx, cy) {
  // Adjust based on midpoint
  return {
    x: cx - midXoutput, 
    y: -(cy - midYoutput)
  };
}

function mechanismToCanvasCoords(mx, my) {
  return {
    x: mx + midXoutput,
    y: midYoutput - my
  };
}

// Convert radians to degrees
function rad2deg(r) {
  return r * (180 / Math.PI);
}

// The inverseKinematics function adapted from C to JavaScript
function inverseKinematics(x, y) {
    // Distance from left hinge (0,0) to target
    const RA = Math.sqrt(x*x + y*y);

    // Distance from right hinge (baseWidth,0) to target
    const dx = x - baseWidth;
    const RB = Math.sqrt(dx*dx + y*y);
    
    // Basic reachability check
    const maxReach = link1 + link2;
    if (RA > maxReach || RB > maxReach) {
        return null; // unreachable
    }

    let cos_g1 = (link1*link1 + RA*RA - link2*link2) / (2.0 * link1 * RA);
    let cos_g2 = (link1*link1 + RB*RB - link2*link2) / (2.0 * link1 * RB);

    if (Math.abs(cos_g1) > 1.0 || Math.abs(cos_g2) > 1.0) {
        return null; // no valid solution
    }

    let g1 = Math.acos(cos_g1);
    let g2 = Math.acos(cos_g2);

    let phiA = Math.atan2(y, x);
    let phiB = Math.atan2(y, x - baseWidth);

    let theta1 = phiA + g1;
    let theta2 = phiB - g2;

    let theta1_deg = rad2deg(theta1);
    let theta2_deg = rad2deg(theta2);

    return {theta1: theta1, theta2: theta2, theta1_deg: theta1_deg, theta2_deg: theta2_deg};
}

// Function to draw a small angle arc with label and value
function drawAngleArc(centerX, centerY, startAngle, endAngle, radius, label, angleValue) {
    // Draw the arc
    ctx5bar.beginPath();
    ctx5bar.arc(centerX, centerY, radius, startAngle, endAngle);
    ctx5bar.strokeStyle = "purple";
    ctx5bar.lineWidth = 1;
    ctx5bar.stroke();

    // Calculate mid-point angle for labeling
    let midAngle = (startAngle + endAngle) / 2;

    // Ensure midAngle is upright (convert to range [-π/2, π/2] for readable text)
    if (midAngle > Math.PI / 2) {
        midAngle -= Math.PI; // Flip angle for upright text
    } else if (midAngle < -Math.PI / 2) {
        midAngle += Math.PI;
    }

    // Calculate label position
    const labelX = centerX + (radius + 15) * Math.cos((startAngle + endAngle) / 2);
    const labelY = centerY + (radius + 15) * Math.sin((startAngle + endAngle) / 2);

    // Format the angle value
    const formattedValue = angleValue.toFixed(2);
    const fullLabel = `${label}: ${formattedValue}°`;

    // Draw the text at the calculated position
    ctx5bar.save();
    ctx5bar.translate(labelX, labelY);
    ctx5bar.rotate(Math.PI);
    ctx5bar.scale(-1,1) 
    ctx5bar.font = "12px Arial";
    ctx5bar.fillStyle = "black";
    ctx5bar.textAlign = "center";
    ctx5bar.textBaseline = "middle";
    ctx5bar.fillText(fullLabel, 0, 0);
    ctx5bar.restore();
}

// Draw the 5-bar mechanism on canvasoutput
function drawMechanism(x, y) {
    const ik = inverseKinematics(x, y);
    if (!ik) {
        return;
    }

    // Add a dot at the current (x, y) on the trail canvasoutput
     const canvasCoords = mechanismToCanvasCoords(x, y);
     trailCtx.beginPath();
     trailCtx.arc(canvasCoords.x, canvasCoords.y, 2, 0, 2 * Math.PI); // Small dot
     trailCtx.fillStyle = 'rgba(0, 100, 255, 0.5)'; // Semi-transparent blue
     trailCtx.fill();

    let t1ang = rad2deg(ik.theta1);
    let t2ang = rad2deg(ik.theta2);
    theta1val.value = t1ang.toFixed(2);
    theta2val.value = t2ang.toFixed(2);

    const A = {x:0, y:0};
    const B = {x:baseWidth, y:0};

    const theta1 = ik.theta1;
    const theta2 = ik.theta2;

    const A_elbow = {
        x: A.x + link1 * Math.cos(theta1),
        y: A.y + link1 * Math.sin(theta1)
    };
    const B_elbow = {
        x: B.x + link1 * Math.cos(theta2),
        y: B.y + link1 * Math.sin(theta2)
    };

    ctx5bar.clearRect(0, 0, canvasoutput.width, canvasoutput.height);

    ctx5bar.save();
    ctx5bar.translate(midXoutput, midYoutput);
    ctx5bar.scale(1, -1);

    ctx5bar.strokeStyle = "#000";
    ctx5bar.lineWidth = 2;
    ctx5bar.beginPath();
    ctx5bar.moveTo(A.x, A.y);
    ctx5bar.lineTo(B.x, B.y);
    ctx5bar.stroke();

    ctx5bar.fillStyle = "#000";
    ctx5bar.beginPath();
    ctx5bar.arc(A.x, A.y, 5, 0, 2*Math.PI);
    ctx5bar.fill();

    ctx5bar.beginPath();
    ctx5bar.arc(B.x, B.y, 5, 0, 2*Math.PI);
    ctx5bar.fill();

    ctx5bar.strokeStyle = "#0072b2";
    ctx5bar.lineWidth = 4;
    ctx5bar.beginPath();
    ctx5bar.moveTo(A.x, A.y);
    ctx5bar.lineTo(A_elbow.x, A_elbow.y);
    ctx5bar.stroke();

    ctx5bar.beginPath();
    ctx5bar.moveTo(B.x, B.y);
    ctx5bar.lineTo(B_elbow.x, B_elbow.y);
    ctx5bar.stroke();

    ctx5bar.strokeStyle = "#d55e00";
    ctx5bar.beginPath();
    ctx5bar.moveTo(A_elbow.x, A_elbow.y);
    ctx5bar.lineTo(x, y);
    ctx5bar.stroke();

    ctx5bar.beginPath();
    ctx5bar.moveTo(B_elbow.x, B_elbow.y);
    ctx5bar.lineTo(x, y);
    ctx5bar.stroke();

    ctx5bar.fillStyle = "#cc79a7";
    ctx5bar.beginPath();
    ctx5bar.arc(x, y, 6, 0, 2*Math.PI);
    ctx5bar.fill();

    // Adding label for end effector position
    ctx5bar.save();
    ctx5bar.scale(1, -1); // Flip text back to normal orientation
    ctx5bar.fillStyle = "black";
    ctx5bar.font = "14px Arial";
    ctx5bar.textAlign = "left";
    ctx5bar.textBaseline = "middle";
    ctx5bar.fillText(`(${(x/10).toFixed(1)}, ${(y/10).toFixed(1)})`, x + 10, -y); // Positioning label next to end effector
    ctx5bar.restore();

    drawAngleArc(
        A.x,
        A.y,
        0,
        theta1,
        30, // Radius for the arc
        "θ1",
        t1ang
    );

    drawAngleArc(
        B.x,
        B.y,
        0,
        theta2,
        30, // Radius for the arc
        "θ2",
        t2ang
    );

    ctx5bar.restore();
}

// Initial draw
function initialDraw() {
  const cw = parseInt(document.getElementById('canvasWidth').value, 10);
  const ch = parseInt(document.getElementById('canvasHeight').value, 10);
  canvasoutput.width = cw;
  canvasoutput.height = ch;
  midXoutput = parseFloat(document.getElementById('midXoutput').value);
  midYoutput = parseFloat(document.getElementById('midYoutput').value);
  endXoutput = parseFloat(document.getElementById('endXoutput').value);
  endYoutput = parseFloat(document.getElementById('endYoutput').value);
  drawMechanism(endXoutput, endYoutput);
}
initialDraw();

document.getElementById('drawButton').addEventListener('click', () => {
  initialDraw();
});


// Resize trail canvasoutput to match the main canvasoutput
function resizeTrailCanvas() {
    const container = document.getElementById('canvasContainer');
    const mechanismCanvas = document.getElementById('mechanismCanvas');
    const trailCanvas = document.getElementById('trailCanvas');

    // Set internal resolution of trailCanvas
    trailCanvas.width = mechanismCanvas.width;
    trailCanvas.height = mechanismCanvas.height;

    // Ensure container dimensions match canvasoutput
    container.style.width = mechanismCanvas.offsetWidth + 'px';
    container.style.height = mechanismCanvas.offsetHeight + 'px';

    const mechanismStyle = window.getComputedStyle(mechanismCanvas);
    const trailStyle = window.getComputedStyle(trailCanvas);

    console.log(`MechanismCanvas computed style: width=${mechanismStyle.width}, height=${mechanismStyle.height}`);
    console.log(`TrailCanvas computed style: width=${trailStyle.width}, height=${trailStyle.height}`);
    
    console.log(`trailCanvas.height: ${trailCanvas.height}, trailCanvas.width: ${trailCanvas.width}`);
    console.log(`mechanismCanvas.height: ${mechanismCanvas.height}, mechanismCanvas.width: ${mechanismCanvas.width}`);
    console.log(`trailCanvas.style.height: ${trailCanvas.style.height}, trailCanvas.style.width: ${trailCanvas.style.width}`);

}


// Call resize function whenever necessary
resizeTrailCanvas();
window.addEventListener('resize', resizeTrailCanvas);







let isDragging = false;

// Check if mouse down is on the end effector
canvasoutput.addEventListener('mousedown', (e) => {
  const rect = canvasoutput.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
  // Convert to mechanism coordinates
  const mechPt = canvasToMechanismCoords(mx, my);

  // Check if (mechPt.x, mechPt.y) is near the end effector
  const dist = Math.sqrt((mechPt.x - endXoutput)**2 + (mechPt.y - endYoutput)**2);
  if (dist < 10) {
    isDragging = true;
  }
});

canvasoutput.addEventListener('mousemove', (e) => {
  if (!isDragging) return;
  
  const rect = canvasoutput.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
  const mechPt = canvasToMechanismCoords(mx, my);

  // Update the end effector position
  // Check IK feasibility:
  const ik = inverseKinematics(mechPt.x, mechPt.y);
  if (ik) {
    endXoutput = mechPt.x;
    endYoutput = mechPt.y;
    drawMechanism(endXoutput, endYoutput);
  }
});

canvasoutput.addEventListener('mouseup', () => {
  isDragging = false;
});

canvasoutput.addEventListener('mouseleave', () => {
  isDragging = false;
});


// Ensure mechanismCanvas is resized when dimensions are updated
document.getElementById('drawButton').addEventListener('click', () => {
    resizeTrailCanvas();
});

//Create a tooltip div
const tooltip = document.createElement('div');
tooltip.style.position = 'absolute';
tooltip.style.background = '#fff';
tooltip.style.border = '1px solid #000';
tooltip.style.padding = '5px';
tooltip.style.fontSize = '12px';
tooltip.style.pointerEvents = 'none';
tooltip.style.display = 'none';
tooltip.style.fontFamily = 'Arial'
document.body.appendChild(tooltip);

canvasoutput.addEventListener('mousemove', (e) => {
  const rect = canvasoutput.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;

  // Convert to mechanism coordinates
  const mechPt = canvasToMechanismCoords(mx, my);

  // Show tooltip
  tooltip.style.left = `${e.pageX + 10}px`; // Offset for visibility
  tooltip.style.top = `${e.pageY + 10}px`;
  tooltip.innerHTML = `x: ${(mechPt.x/10).toFixed(1)}, y: ${(mechPt.y/10).toFixed(1)}`;
  tooltip.style.display = 'block';

});

canvasoutput.addEventListener('mouseleave', () => {
  // Hide tooltip when leaving the canvasoutput
  tooltip.style.display = 'none';
});


function drawCircleGradually(centerX, centerY, radius, steps = 500, delay = 10) {
    let step = 0;
    clearTrail();

    function drawStep() {
        if (step >= steps) return; // Stop when all steps are complete

        // Calculate the current (x, y) on the circle
        const angle = (2 * Math.PI * step) / steps;
        const x = centerX + radius * Math.cos(angle);
        const y = centerY + radius * Math.sin(angle);

        endXoutput = x;
        endYoutput = y;

        // Call drawMechanism for the current (x, y)
        drawMechanism(x, y);

        // Increment the step and schedule the next one
        step++;
        setTimeout(drawStep, delay);
    }

    drawStep(); // Start the animation
}

function clearTrail() {
    trailCtx.clearRect(0, 0, trailCanvas.width, trailCanvas.height);
}

function drawFromFloatArray(floatArray, delay)
{
    let step = 0;
    clearTrail();
    function drawStep() {
        if (step >= floatArray.length) return; // Stop when all steps are complete

        let x = floatArray[step] * 10;
        let y = floatArray[step+1] * 10;

        if (x==-3000 && y==-3000)
        {
            setTimeout(drawStep, delay);

            step+=2;
            return;
        }

        endXoutput = x;
        endYoutput = y;
        

        // Call drawMechanism for the current (x, y)
        drawMechanism(x, y);

        // Increment the step and schedule the next one
        step+=2;
        setTimeout(drawStep, delay);
    }

    drawStep(); // Start the animation
}


