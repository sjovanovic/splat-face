/**
 * Generates Gaussian Splat object from set of keypoints [{x, y, z, name}, ...], 
 * triangle indices [[18, 98, 226], ...] 
 * and 2D context from canvas element
 * 
 * @param {Array} keypoints 
 * @param {Array} triangleIndices 
 * @param {CanvasRenderingContext2D} ctx 
 */

export function genSplatFromTrianglesAnd2dContext(keypoints, triangleIndices, ctx, splatData){

    let spread = 2
    let min = 0
    let max = 500

    let points = []
    let triangleIndex = {}, v0, v1, v2, cnt=0;

    //let names = [] // names: "faceOval", "lips", "rightEye", "rightEyebrow", "leftEye", "leftEyebrow"

    for(let i=0; i<triangleIndices.length; i++){
        v0 = [ keypoints[triangleIndices[i][0]].x, keypoints[triangleIndices[i][0]].y, keypoints[triangleIndices[i][0]].z ]
        v1 = [ keypoints[triangleIndices[i][1]].x, keypoints[triangleIndices[i][1]].y, keypoints[triangleIndices[i][1]].z ]
        v2 = [ keypoints[triangleIndices[i][2]].x, keypoints[triangleIndices[i][2]].y, keypoints[triangleIndices[i][2]].z ]

        getEvenlySpreadPointsInTriangle(v0, v1, v2, spread, (point)=>{
            points.push(point)
            triangleIndex[cnt] = i
            cnt = cnt + 1
        })
    }

    if(!splatData) {
        splatData = {
            positions: new Float32Array(cnt * 3),
            colors: new Uint8Array(cnt * 4),
            scales: new Float32Array(cnt * 3),
            rotations: new Float32Array(cnt * 4)
        }
    }
    let {positions, colors, scales, rotations} = splatData

    let color, itriangle, quaternion, eigenValues, covarianceMatrix;
    let scale = 0.0020

    let scls = {x:0.003, y:0.003, z: 0.003}

    for(let i=0;i<cnt;i++){

        positions[i*3] = norm(points[i][0], min, max)
        positions[i*3+1] = norm(points[i][1], min, max)
        positions[i*3+2] = norm(points[i][2], min, max)

        color = ctx.getImageData(points[i][0], points[i][1], 1, 1).data;
        colors[i*4] = color[0]
        colors[i*4+1] = color[1]
        colors[i*4+2] = color[2]
        colors[i*4+3] = color[3]

        if(itriangle !== triangleIndex[i]){
            itriangle = triangleIndex[i]

            v0 = [ keypoints[triangleIndices[itriangle][0]].x, keypoints[triangleIndices[itriangle][0]].y, keypoints[triangleIndices[itriangle][0]].z ]
            v1 = [ keypoints[triangleIndices[itriangle][1]].x, keypoints[triangleIndices[itriangle][1]].y, keypoints[triangleIndices[itriangle][1]].z ]
            v2 = [ keypoints[triangleIndices[itriangle][2]].x, keypoints[triangleIndices[itriangle][2]].y, keypoints[triangleIndices[itriangle][2]].z ]

            quaternion = getTriangleRotationQuaternion(v0, v1, v2)
            
            // let nearest = findNearestIndices(triangleIndices, itriangle), nearestPoints = []
            // for(let j=0;j<nearest.length; j++){
            //     let n = nearest[j]
            //     if(!keypoints[n]) continue;
            //     nearestPoints.push([ keypoints[n].x, keypoints[n].y, keypoints[n].z ])
            // }
            // nearest = null
            // if(nearestPoints.length === 0) nearestPoints.push([0, 0, 0])
            // covarianceMatrix = computeCovarianceMatrix(nearestPoints)
            // try{
            //     eigenValues = computeEigenvalues(covarianceMatrix)
            // }catch(err) {
            //     if(!eigenValues) eigenValues = [1, 1, 1]
            // }

            
            
        }

        rotations[i*4] = quaternion[0]
        rotations[i*4+1] = quaternion[1]
        rotations[i*4+2] = quaternion[2]
        rotations[i*4+3] = quaternion[3]


        // rotations[i*4] = 0
        // rotations[i*4+1] = 0
        // rotations[i*4+2] = 0
        // rotations[i*4+3] = 1


        // if(splatData.customRotations){
        //     rotations[i*4] = splatData.customRotations.x
        //     rotations[i*4+1] = splatData.customRotations.y
        //     rotations[i*4+2] = splatData.customRotations.z
        //     rotations[i*4+3] = splatData.customRotations.w
        // }


        
        // let spread = computeGaussianSplatSpread([v0, v1, v2], points[i])
        // scales[i*3] = spread[0] * 0.005
        // scales[i*3+1] = spread[1] * 0.005 
        // scales[i*3+2] = spread[2] * 0.005

        scales[i*3] = scale
        scales[i*3+1] = scale
        scales[i*3+2] = scale


        if(splatData.customScales){
            scales[i*3] = splatData.customScales.x
            scales[i*3+1] = splatData.customScales.y
            scales[i*3+2] = splatData.customScales.z
        }

        
        /*
        *   [
        *     [xx, xy, xz],
        *     [yx, yy, yz],
        *     [zx, zy, zz]
        *   ]
        */


        // scales[i*3] = scale + covarianceMatrix[0][0] * 0.00001
        // scales[i*3+1] = scale + covarianceMatrix[1][1] * 0.00001
        // scales[i*3+2] = scale + covarianceMatrix[2][2] * 0.00001

    }

    if(splatData.displacementOrigin && splatData.displacementDirection){
        // @param {Number} A Adjust A to control the maximum displacement.
        // @param {Number} sigma Adjust σ to control how far the influence of the vector extends.
        gaussianDisplacementInPlace(positions, splatData.displacementOrigin, splatData.displacementDirection, 0.02, 0.05);
    }

    return splatData
}

export function getTrianglesFromKeypointsAndIndices(keypoints, indices){
    let triangles = []
    for(let i=0;i<indices.length;i++){
        triangles.push([
            keypoints[indices[i][0]].x,
            keypoints[indices[i][0]].y,
            keypoints[indices[i][0]].z
        ])
        triangles.push([
            keypoints[indices[i][1]].x,
            keypoints[indices[i][1]].y,
            keypoints[indices[i][1]].z
        ])
        triangles.push([
            keypoints[indices[i][2]].x,
            keypoints[indices[i][2]].y,
            keypoints[indices[i][2]].z
        ])
    }
    return triangles
}

function norm(value, min, max){
    return (value - min) / (max - min)
}

function getEvenlySpreadPointsInTriangle(v0, v1, v2, spread, callback) {
    // Calculate edge vectors
    const edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    const edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    // Calculate the number of steps based on the spread value
    const length1 = Math.hypot(edge1[0], edge1[1], edge1[2]);
    const length2 = Math.hypot(edge2[0], edge2[1], edge2[2]);
    const steps1 = Math.ceil(length1 / spread);
    const steps2 = Math.ceil(length2 / spread);

    // Iterate over the triangle using barycentric coordinates
    for (let i = 0; i <= steps1; i++) {
        const u = i / steps1;
        for (let j = 0; j <= steps2; j++) {
            const v = j / steps2;
            if (u + v <= 1) { // Ensure the point is inside the triangle
                // Calculate the point using barycentric coordinates
                const point = [
                    v0[0] + u * edge1[0] + v * edge2[0],
                    v0[1] + u * edge1[1] + v * edge2[1],
                    v0[2] + u * edge1[2] + v * edge2[2]
                ];
                callback(point); // Call the callback with the calculated point
            }
        }
    }
}

function getTriangleRotationQuaternion(v0, v1, v2) {
    // Calculate two edge vectors of the triangle
    const edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    const edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    // Calculate the normal vector of the triangle using the cross product
    const normal = [
        edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0]
    ];

    // Normalize the normal vector
    const length = Math.hypot(normal[0], normal[1], normal[2]);
    normal[0] /= length;
    normal[1] /= length;
    normal[2] /= length;

    // Define the reference up vector (world's up vector)
    const up = [0, 1, 0];

    // Calculate the rotation axis using the cross product of the up vector and the normal
    const axis = [
        up[1] * normal[2] - up[2] * normal[1],
        up[2] * normal[0] - up[0] * normal[2],
        up[0] * normal[1] - up[1] * normal[0]
    ];

    // Normalize the rotation axis
    const axisLength = Math.hypot(axis[0], axis[1], axis[2]);
    if (axisLength === 0) {
        // If the axis length is 0, the triangle is already aligned with the up vector
        return [0, 0, 0, 1]; // Identity quaternion
    }
    axis[0] /= axisLength;
    axis[1] /= axisLength;
    axis[2] /= axisLength;

    // Calculate the rotation angle using the dot product
    const angle = Math.acos(up[0] * normal[0] + up[1] * normal[1] + up[2] * normal[2]);

    // Construct the quaternion from the axis-angle representation
    const halfAngle = angle / 2;
    const sinHalfAngle = Math.sin(halfAngle);
    return [
        axis[0] * sinHalfAngle, // x
        axis[1] * sinHalfAngle, // y
        axis[2] * sinHalfAngle, // z
        Math.cos(halfAngle)     // w
    ];
}

function getTriangleSurfaceArea(v0, v1, v2) {
    // Calculate two edge vectors of the triangle
    const edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    const edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    // Calculate the cross product of the two edge vectors
    const cross = [
        edge1[1] * edge2[2] - edge1[2] * edge2[1], // x component
        edge1[2] * edge2[0] - edge1[0] * edge2[2], // y component
        edge1[0] * edge2[1] - edge1[1] * edge2[0]  // z component
    ];

    // Calculate the magnitude of the cross product vector
    const magnitude = Math.hypot(cross[0], cross[1], cross[2]);

    // The area is half the magnitude of the cross product
    return magnitude / 2;
}

function getKNearestNeighbors(vertices, targetIndex, k) {
    if (k >= vertices.length) return vertices; // Return all vertices if k is too large

    const target = vertices[targetIndex];
    const distances = [];

    // Calculate squared distances to avoid expensive square root operations
    for (let i = 0; i < vertices.length; i++) {
        if (i === targetIndex) continue; // Skip the target vertex itself
        const dx = vertices[i][0] - target[0];
        const dy = vertices[i][1] - target[1];
        const dz = vertices[i][2] - target[2];
        const squaredDistance = dx * dx + dy * dy + dz * dz;
        distances.push({ index: i, distance: squaredDistance });
    }

    // Sort by distance (ascending order)
    distances.sort((a, b) => a.distance - b.distance);

    // Extract the k nearest neighbors
    const nearestNeighbors = [];
    for (let i = 0; i < k; i++) {
        nearestNeighbors.push(vertices[distances[i].index]);
    }

    return nearestNeighbors;
}


/**
 * Given array of vertices, returns covariance matrix
 * 
 * @param {Array} vertices [[x, y, z], ...]
 * @returns {Array} covariance matrix 
 * 
 * [
 *   [xx, xy, xz],
 *   [yx, yy, yz],
 *   [zx, zy, zz]
 * ]
 */
function computeCovarianceMatrix(vertices) {
    const n = vertices.length;
    if (n === 0) throw new Error("Vertex array is empty");

    // Initialize sums for x, y, z and their products
    let sumX = 0, sumY = 0, sumZ = 0;
    let sumXX = 0, sumYY = 0, sumZZ = 0;
    let sumXY = 0, sumXZ = 0, sumYZ = 0;

    // Compute sums
    for (const vertex of vertices) {
        const [x, y, z] = vertex;
        sumX += x;
        sumY += y;
        sumZ += z;
        sumXX += x * x;
        sumYY += y * y;
        sumZZ += z * z;
        sumXY += x * y;
        sumXZ += x * z;
        sumYZ += y * z;
    }

    // Compute means
    const meanX = sumX / n;
    const meanY = sumY / n;
    const meanZ = sumZ / n;

    // Compute covariance matrix elements
    const covXX = (sumXX / n) - (meanX * meanX);
    const covYY = (sumYY / n) - (meanY * meanY);
    const covZZ = (sumZZ / n) - (meanZ * meanZ);
    const covXY = (sumXY / n) - (meanX * meanY);
    const covXZ = (sumXZ / n) - (meanX * meanZ);
    const covYZ = (sumYZ / n) - (meanY * meanZ);

    // Construct the covariance matrix
    return [
        [covXX, covXY, covXZ],
        [covXY, covYY, covYZ],
        [covXZ, covYZ, covZZ]
    ];
}

// returns all point indices from triangles that touch given point
function findNearestIndices(indices, pointIndex) {
    let nearestPoints = [pointIndex]
    for(let i=0; i<indices.length; i++){
        if(indices[i][0] === pointIndex) {
            if(!nearestPoints.includes(indices[i][1])) {
                nearestPoints.push(indices[i][1])
            }
            if(!nearestPoints.includes(indices[i][2])) {
                nearestPoints.push(indices[i][2])
            }
        }else if(indices[i][1] === pointIndex) {
            if(!nearestPoints.includes(indices[i][0])) {
                nearestPoints.push(indices[i][0])
            }
            if(!nearestPoints.includes(indices[i][2])) {
                nearestPoints.push(indices[i][2])
            }
        }else if(indices[i][2] === pointIndex) {
            if(!nearestPoints.includes(indices[i][0])) {
                nearestPoints.push(indices[i][0])
            }
            if(!nearestPoints.includes(indices[i][1])) {
                nearestPoints.push(indices[i][1])
            }
        }
    }
    return nearestPoints
}

function computeEigenvalues(covarianceMatrix) {
    const [a, b, c] = covarianceMatrix[0];
    const [d, e, f] = covarianceMatrix[1];
    const [g, h, i] = covarianceMatrix[2];

    // Coefficients of the characteristic equation: λ^3 + pλ^2 + qλ + r = 0
    const p = -(a + e + i);
    const q = a * e + a * i + e * i - b * d - c * g - f * h;
    const r = a * (f * h - e * i) - b * (d * i - f * g) + c * (d * h - e * g);

    // Helper function to solve cubic equation
    const solveCubic = (p, q, r) => {
        // Using Cardano's formula for solving cubic equations
        const discriminant = (18 * p * q * r) - (4 * p ** 3 * r) + (p ** 2 * q ** 2) - (4 * q ** 3) - (27 * r ** 2);
        if (discriminant > 0) {
            // Three distinct real roots
            const u = Math.acos((9 * p * q - 2 * p ** 3 - 27 * r) / (2 * Math.sqrt((p ** 2 - 3 * q) ** 3)));
            const t = 2 * Math.sqrt(p ** 2 - 3 * q);
            return [
                (-t * Math.cos(u / 3) - p) / 3,
                (-t * Math.cos((u + 2 * Math.PI) / 3) - p) / 3,
                (-t * Math.cos((u + 4 * Math.PI) / 3) - p) / 3
            ];
        } else if (discriminant === 0) {
            // Multiple roots
            const root1 = (-p + Math.sqrt(p ** 2 - 3 * q)) / 3;
            const root2 = (-p - Math.sqrt(p ** 2 - 3 * q)) / 3;
            return [root1, root2, root2];
        } else {
            // One real root and two complex roots (not handled here)
            throw new Error("Complex eigenvalues are not supported");
        }
    };

    // Solve the cubic equation to find eigenvalues
    const eigenvalues = solveCubic(p, q, r);

    // Sort eigenvalues in descending order
    eigenvalues.sort((a, b) => b - a);

    return eigenvalues;
}

function calculateSpreadForTriangle(v0, v1, v2) {
    // Calculate two edge vectors of the triangle
    const edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    const edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    // Calculate the normal vector of the triangle using the cross product
    const normal = [
        edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0]
    ];


    let max = Math.max(normal[0], normal[1], normal[2]) + 0.003
    normal[0] = max - normal[0]
    normal[1] = max - normal[1]
    normal[2] = max - normal[2]


    return normal
}

function computeGaussianSplatSpread(triangleVertices, point) {
    // Unpack triangle vertices
    const [A, B, C] = triangleVertices;

    // Compute edge vectors of the triangle
    const edgeAB = [B[0] - A[0], B[1] - A[1], B[2] - A[2]];
    const edgeAC = [C[0] - A[0], C[1] - A[1], C[2] - A[2]];

    // Compute the normal vector of the triangle (perpendicular to the plane)
    const normal = [
        edgeAB[1] * edgeAC[2] - edgeAB[2] * edgeAC[1],
        edgeAB[2] * edgeAC[0] - edgeAB[0] * edgeAC[2],
        edgeAB[0] * edgeAC[1] - edgeAB[1] * edgeAC[0]
    ];

    // Normalize the normal vector
    const normalLength = Math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2);
    const normalizedNormal = [
        normal[0] / normalLength,
        normal[1] / normalLength,
        normal[2] / normalLength
    ];

    // Compute vectors from the point to the triangle vertices
    const vectorPA = [A[0] - point[0], A[1] - point[1], A[2] - point[2]];
    const vectorPB = [B[0] - point[0], B[1] - point[1], B[2] - point[2]];
    const vectorPC = [C[0] - point[0], C[1] - point[1], C[2] - point[2]];

    // Project vectors onto the triangle's plane
    const projectOntoPlane = (vector) => {
        const dot = vector[0] * normalizedNormal[0] + vector[1] * normalizedNormal[1] + vector[2] * normalizedNormal[2];
        return [
            vector[0] - dot * normalizedNormal[0],
            vector[1] - dot * normalizedNormal[1],
            vector[2] - dot * normalizedNormal[2]
        ];
    };

    const projectedPA = projectOntoPlane(vectorPA);
    const projectedPB = projectOntoPlane(vectorPB);
    const projectedPC = projectOntoPlane(vectorPC);

    // Compute the lengths of the projected vectors (spread in the plane)
    const lengthPA = Math.sqrt(projectedPA[0] ** 2 + projectedPA[1] ** 2 + projectedPA[2] ** 2);
    const lengthPB = Math.sqrt(projectedPB[0] ** 2 + projectedPB[1] ** 2 + projectedPB[2] ** 2);
    const lengthPC = Math.sqrt(projectedPC[0] ** 2 + projectedPC[1] ** 2 + projectedPC[2] ** 2);

    // Compute the average spread in the plane
    const averageSpreadInPlane = (lengthPA + lengthPB + lengthPC) / 3;

    // Compute the spread in the normal direction (small value, since the splat is flat)
    const spreadInNormalDirection = 0.01; // Adjust this based on your needs

    // Return the spread as a 3D vector: [spread_x, spread_y, spread_z]
    return [averageSpreadInPlane, averageSpreadInPlane, spreadInNormalDirection];
}

/**
 * 
    // Example usage
    const pointCloud = new Float32Array([
        1, 2, 3, // Point 1
        4, 5, 6, // Point 2
        7, 8, 9, // Point 3
        // Add more points as needed
    ]);

    const rayOrigin = [0, 0, 0]; // Ray origin
    const rayDirection = [1, 0, 0]; // Ray direction

    gaussianDisplacementInPlace(pointCloud, rayOrigin, rayDirection, 0.5, 0.2);

    console.log(pointCloud); // Modified in place
 * @param {Float32Array} pointCloud 
 * @param {Array} rayOrigin 
 * @param {Array} rayDirection 
 * @param {Number} A Adjust A to control the maximum displacement.
 * @param {Number} sigma Adjust σ to control how far the influence of the vector extends.
 */
function gaussianDisplacementInPlace(pointCloud, rayOrigin, rayDirection, A = 1.0, sigma = 1.0) {
    // Normalize the ray direction
    const rayLength = Math.sqrt(rayDirection[0] ** 2 + rayDirection[1] ** 2 + rayDirection[2] ** 2);
    const rayDirNorm = [
        rayDirection[0] / rayLength,
        rayDirection[1] / rayLength,
        rayDirection[2] / rayLength,
    ];

    // Precompute constants for the Gaussian function
    const twoSigmaSquared = 2 * sigma ** 2;

    // Iterate over the point cloud (assumed to be a Float32Array with x, y, z coordinates)
    for (let i = 0; i < pointCloud.length; i += 3) {
        const x = pointCloud[i];
        const y = pointCloud[i + 1];
        const z = pointCloud[i + 2];

        // Vector from ray origin to the point
        const px = x - rayOrigin[0];
        const py = y - rayOrigin[1];
        const pz = z - rayOrigin[2];

        // Project the vector onto the ray direction
        const dot = px * rayDirNorm[0] + py * rayDirNorm[1] + pz * rayDirNorm[2];
        const projectionX = dot * rayDirNorm[0];
        const projectionY = dot * rayDirNorm[1];
        const projectionZ = dot * rayDirNorm[2];

        // Perpendicular vector (point to ray)
        const perpendicularX = px - projectionX;
        const perpendicularY = py - projectionY;
        const perpendicularZ = pz - projectionZ;

        // Perpendicular distance (squared to avoid Math.sqrt)
        const distanceSquared = perpendicularX ** 2 + perpendicularY ** 2 + perpendicularZ ** 2;

        // Compute the Gaussian weight
        const w = A * Math.exp(-distanceSquared / twoSigmaSquared);

        // Compute the displacement (along the ray direction)
        const displacementX = w * rayDirNorm[0];
        const displacementY = w * rayDirNorm[1];
        const displacementZ = w * rayDirNorm[2];

        // Update the point in place
        pointCloud[i] += displacementX;
        pointCloud[i + 1] += displacementY;
        pointCloud[i + 2] += displacementZ;
    }
}

/**
 *  // Example usage
    const pointCloud = new Float32Array([
        1, 2, 3, // Point 1
        4, 5, 6, // Point 2
        7, 8, 9, // Point 3
        // Add more points as needed
    ]);

    const rayOrigin = [0, 0, 0]; // Ray origin
    const rayDirection = [1, 0, 0]; // Ray direction
    const maxDistance = 5; // Maximum allowed distance

    const result = findNearestPointOnRay(pointCloud, rayOrigin, rayDirection, maxDistance);
    console.log(result); // { point: [x, y, z], index: pointIndex } or null
 * @param {*} pointCloud 
 * @param {*} rayOrigin 
 * @param {*} rayDirection 
 * @param {*} maxDistance 
 * @returns 
 */
export function findNearestPointOnRay(pointCloud, rayOrigin, rayDirection, maxDistance) {
    // Normalize the ray direction
    const rayLength = Math.sqrt(rayDirection[0] ** 2 + rayDirection[1] ** 2 + rayDirection[2] ** 2);
    const rayDirNorm = [
        rayDirection[0] / rayLength,
        rayDirection[1] / rayLength,
        rayDirection[2] / rayLength,
    ];

    let nearestPointIndex = -1;
    let minDistanceSquared = Infinity;

    // Precompute maxDistance squared to avoid Math.sqrt in the loop
    const maxDistanceSquared = maxDistance ** 2;

    // Iterate over the point cloud (assumed to be a Float32Array with x, y, z coordinates)
    for (let i = 0; i < pointCloud.length; i += 3) {
        const x = pointCloud[i];
        const y = pointCloud[i + 1];
        const z = pointCloud[i + 2];

        // Vector from ray origin to the point
        const px = x - rayOrigin[0];
        const py = y - rayOrigin[1];
        const pz = z - rayOrigin[2];

        // Project the vector onto the ray direction
        const dot = px * rayDirNorm[0] + py * rayDirNorm[1] + pz * rayDirNorm[2];
        const projectionX = dot * rayDirNorm[0];
        const projectionY = dot * rayDirNorm[1];
        const projectionZ = dot * rayDirNorm[2];

        // Perpendicular vector (point to ray)
        const perpendicularX = px - projectionX;
        const perpendicularY = py - projectionY;
        const perpendicularZ = pz - projectionZ;

        // Squared perpendicular distance (avoid Math.sqrt for performance)
        const distanceSquared = perpendicularX ** 2 + perpendicularY ** 2 + perpendicularZ ** 2;

        // Check if the point is within maxDistance and closer than the current nearest point
        if (distanceSquared <= maxDistanceSquared && distanceSquared < minDistanceSquared) {
            minDistanceSquared = distanceSquared;
            nearestPointIndex = i;
        }
    }

    // Return the nearest point (x, y, z) and its index if found
    if (nearestPointIndex !== -1) {
        return {
            point: [
                pointCloud[nearestPointIndex],
                pointCloud[nearestPointIndex + 1],
                pointCloud[nearestPointIndex + 2],
            ],
            index: nearestPointIndex / 3, // Convert to point index
        };
    }

    return null; // No point found within maxDistance
}

function getDirectionVector(origin, destination, normalize = true) {
    // Calculate the direction vector
    const direction = [
        destination[0] - origin[0],
        destination[1] - origin[1],
        destination[2] - origin[2],
    ];

    // Normalize the vector if requested
    if (normalize) {
        const length = Math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2);
        direction[0] /= length;
        direction[1] /= length;
        direction[2] /= length;
    }

    return direction;
}

export function midpointSubdivisionWithDynamicOffset(vertices, indices, scaleFactor = 0.1) {
    const newVertices = [...vertices];
    const newIndices = [];
    const midpointCache = {}; // Cache to avoid duplicate midpoints

    // Helper function to calculate the midpoint and offset it
    function getOffsetMidpointIndex(v1, v2, normal, offsetStrength) {
        const key = `${Math.min(v1, v2)}_${Math.max(v1, v2)}`;
        if (midpointCache[key]) {
            return midpointCache[key];
        }

        // Calculate midpoint
        const x = (vertices[v1 * 3] + vertices[v2 * 3]) / 2;
        const y = (vertices[v1 * 3 + 1] + vertices[v2 * 3 + 1]) / 2;
        const z = (vertices[v1 * 3 + 2] + vertices[v2 * 3 + 2]) / 2;

        // Offset midpoint along the normal
        const offsetX = x + normal[0] * offsetStrength;
        const offsetY = y + normal[1] * offsetStrength;
        const offsetZ = z + normal[2] * offsetStrength;

        // Add new vertex
        const newIndex = newVertices.length / 3;
        newVertices.push(offsetX, offsetY, offsetZ);

        // Cache the midpoint
        midpointCache[key] = newIndex;
        return newIndex;
    }

    // Helper function to calculate the normal of a triangle
    function calculateNormal(a, b, c) {
        const ab = [
            vertices[b * 3] - vertices[a * 3],
            vertices[b * 3 + 1] - vertices[a * 3 + 1],
            vertices[b * 3 + 2] - vertices[a * 3 + 2],
        ];
        const ac = [
            vertices[c * 3] - vertices[a * 3],
            vertices[c * 3 + 1] - vertices[a * 3 + 1],
            vertices[c * 3 + 2] - vertices[a * 3 + 2],
        ];

        // Cross product of AB and AC
        const normal = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];

        // Normalize the normal vector
        const length = Math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2);
        return [normal[0] / length, normal[1] / length, normal[2] / length];
    }

    // Helper function to calculate the signed distance of a point to a plane
    function signedDistanceToPlane(point, planeNormal, planePoint) {
        const d = -(
            planeNormal[0] * planePoint[0] +
            planeNormal[1] * planePoint[1] +
            planeNormal[2] * planePoint[2]
        );
        return (
            planeNormal[0] * point[0] +
            planeNormal[1] * point[1] +
            planeNormal[2] * point[2] +
            d
        ) / Math.sqrt(planeNormal[0] ** 2 + planeNormal[1] ** 2 + planeNormal[2] ** 2);
    }

    // Helper function to find connected vertices
    function findConnectedVertices(vertexIndex, indices) {
        const connected = new Set();
        for (let i = 0; i < indices.length; i += 3) {
            const triangle = [indices[i], indices[i + 1], indices[i + 2]];
            if (triangle.includes(vertexIndex)) {
                triangle.forEach((v) => {
                    if (v !== vertexIndex) connected.add(v);
                });
            }
        }
        return Array.from(connected);
    }

    // Iterate through each triangle
    for (let i = 0; i < indices.length; i += 3) {
        const a = indices[i];
        const b = indices[i + 1];
        const c = indices[i + 2];

        // Calculate the normal of the current triangle
        const normal = calculateNormal(a, b, c);

        // Find connected vertices (neighbors)
        const neighbors = [
            ...findConnectedVertices(a, indices),
            ...findConnectedVertices(b, indices),
            ...findConnectedVertices(c, indices),
        ].filter((v) => v !== a && v !== b && v !== c); // Exclude current triangle vertices

        // Calculate signed distances of neighbors to the triangle's plane
        const distances = neighbors.map((v) => {
            const point = [vertices[v * 3], vertices[v * 3 + 1], vertices[v * 3 + 2]];
            const planePoint = [vertices[a * 3], vertices[a * 3 + 1], vertices[a * 3 + 2]];
            return signedDistanceToPlane(point, normal, planePoint);
        });

        // Normalize distances to [-1, 1]
        const maxDistance = Math.max(...distances.map(Math.abs));
        const normalizedDistances = distances.map((d) => (maxDistance !== 0 ? d / maxDistance : 0));

        // Average the normalized distances to get the offset strength
        const averageDistance = normalizedDistances.reduce((sum, d) => sum + d, 0) / normalizedDistances.length;
        const offsetStrength = averageDistance * scaleFactor;

        // Get midpoints and offset them along the normal
        const m1 = getOffsetMidpointIndex(a, b, normal, offsetStrength);
        const m2 = getOffsetMidpointIndex(b, c, normal, offsetStrength);
        const m3 = getOffsetMidpointIndex(c, a, normal, offsetStrength);

        // Add new triangles
        newIndices.push(a, m1, m3);
        newIndices.push(m1, b, m2);
        newIndices.push(m3, m2, c);
        newIndices.push(m1, m2, m3);
    }

    return { vertices: newVertices, indices: newIndices };
}