const fixedMinX = 1.0;
const fixedMaxX = 3.0;
const minHeight = -0.1;
const maxHeight = 0.2;
const FOCUS_HALF_WIDTH = 0.4;
const CELL_SIZE = 0.2;

function generateMockPointCloud() {
  const points = [];
  for (let x = 1.0; x <= 3.0; x += 0.05) {
    for (let y = -0.8; y <= 0.8; y += 0.05) {
      const distFromCenter = Math.sqrt((x - 2.0) ** 2 + y ** 2);
      const baseHeight = -0.05 + 0.2 * Math.exp(-distFromCenter * 1.5);
      const noise = (Math.random() - 0.5) * 0.04;
      const z = baseHeight + noise;
      points.push(x, y, z);
    }
  }
  return new Float32Array(points);
}

const pointCloudData = generateMockPointCloud();
let minY = Infinity, maxY = -Infinity;
let filteredCount = 0;

for (let i = 0; i < pointCloudData.length; i += 3) {
  const x = pointCloudData[i];
  const y = pointCloudData[i + 1];
  const z = pointCloudData[i + 2];

  if (x >= fixedMinX && x <= fixedMaxX && z >= minHeight && z <= maxHeight) {
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
    filteredCount++;
  }
}

console.log("Mock total points:", pointCloudData.length / 3);
console.log("Filtered count:", filteredCount);
console.log("minY:", minY, "maxY:", maxY);
