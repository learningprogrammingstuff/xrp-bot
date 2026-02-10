// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpExchange;

import java.awt.Desktop;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.List;

/**
 * Embedded HTTP server for 3D visualization of robot pose and map.
 * Serves a Three.js-based WebGL visualization page and provides a JSON API for state updates.
 */
public class VisualizationServer {
  private static final int DEFAULT_PORT = 5800;
  
  private final HttpServer server;
  private final int port;
  
  // Current state
  private volatile double robotX = 0.0;
  private volatile double robotY = 0.0;
  private volatile double robotTheta = 0.0;
  private volatile List<MapPoint> mapPoints = List.of();
  private volatile double beamEndX = 0.0;
  private volatile double beamEndY = 0.0;
  private volatile SimWorld simWorld;
  private volatile OccupancyMapper mapper;
  private volatile EKFLocalizer localizer;

  /**
   * Creates a new visualization server on the default port.
   */
  public VisualizationServer() throws IOException {
    this(DEFAULT_PORT);
  }

  /**
   * Creates a new visualization server on the specified port.
   * @param port The port to listen on
   */
  public VisualizationServer(int port) throws IOException {
    this.port = port;
    this.server = HttpServer.create(new InetSocketAddress(port), 0);
    
    // Set up HTTP endpoints
    server.createContext("/", this::handleRoot);
    server.createContext("/api/state", this::handleState);
    server.createContext("/api/resetMap", this::handleResetMap);
    server.createContext("/api/setOrigin", this::handleSetOrigin);
    
    server.setExecutor(null); // Use default executor
  }

  /**
   * Starts the HTTP server and opens the browser.
   */
  public void start() {
    server.start();
    System.out.println("Visualization server started on port " + port);
    
    // Open browser
    try {
      if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
        Desktop.getDesktop().browse(new URI("http://localhost:" + port));
        System.out.println("Opening browser to http://localhost:" + port);
      }
    } catch (Exception e) {
      System.err.println("Could not open browser: " + e.getMessage());
      System.out.println("Please manually navigate to http://localhost:" + port);
    }
  }

  /**
   * Stops the HTTP server.
   */
  public void stop() {
    server.stop(0);
  }

  /**
   * Updates the robot pose.
   */
  public void updatePose(double x, double y, double theta) {
    this.robotX = x;
    this.robotY = y;
    this.robotTheta = theta;
  }

  /**
   * Updates the map points.
   */
  public void updateMap(List<MapPoint> points) {
    this.mapPoints = points;
  }

  /**
   * Updates the ultrasonic beam endpoint.
   */
  public void updateBeam(double range, double robotX, double robotY, double robotTheta) {
    double sensorX = robotX + 2.0 * Math.cos(robotTheta);
    double sensorY = robotY + 2.0 * Math.sin(robotTheta);
    this.beamEndX = sensorX + range * Math.cos(robotTheta);
    this.beamEndY = sensorY + range * Math.sin(robotTheta);
  }

  /**
   * Sets the simulated world for visualization.
   */
  public void setSimWorld(SimWorld world) {
    this.simWorld = world;
  }

  /**
   * Sets the occupancy mapper (used for reset map control).
   */
  public void setMapper(OccupancyMapper m) {
    this.mapper = m;
  }

  /**
   * Sets the EKF localizer (used for set-origin control).
   */
  public void setLocalizer(EKFLocalizer l) {
    this.localizer = l;
  }

  /**
   * Handles the root endpoint - serves the HTML/JS visualization.
   */
  private void handleRoot(HttpExchange exchange) throws IOException {
    String html = getVisualizationHTML();
    byte[] response = html.getBytes(StandardCharsets.UTF_8);
    
    exchange.getResponseHeaders().set("Content-Type", "text/html; charset=UTF-8");
    exchange.sendResponseHeaders(200, response.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(response);
    }
  }

  /**
   * Handles the /api/resetMap endpoint - clears the occupancy grid and map file.
   */
  private void handleResetMap(HttpExchange exchange) throws IOException {
    if (mapper != null) {
      mapper.clearMap();
    }
    byte[] response = "{\"status\":\"ok\"}".getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "application/json");
    exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(200, response.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(response);
    }
  }

  /**
   * Handles the /api/setOrigin endpoint - re-defines the current robot pose as origin (0,0).
   * Transforms all stored map points and resets the estimator pose.
   */
  private void handleSetOrigin(HttpExchange exchange) throws IOException {
    if (localizer != null && mapper != null) {
      double curX = localizer.getX();
      double curY = localizer.getY();
      double curTheta = localizer.getTheta();

      // Transform map points to new coordinate frame
      mapper.setOrigin(curX, curY);

      // Reset the pose estimator to (0, 0) keeping current heading
      localizer.reset(0.0, 0.0, curTheta);
    }
    byte[] response = "{\"status\":\"ok\"}".getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "application/json");
    exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(200, response.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(response);
    }
  }

  /**
   * Handles the /api/state endpoint - returns current state as JSON.
   */
  private void handleState(HttpExchange exchange) throws IOException {
    StringBuilder json = new StringBuilder();
    json.append("{\n");
    json.append("  \"robot\": {\n");
    json.append("    \"x\": ").append(robotX).append(",\n");
    json.append("    \"y\": ").append(robotY).append(",\n");
    json.append("    \"theta\": ").append(robotTheta).append("\n");
    json.append("  },\n");
    json.append("  \"beam\": {\n");
    json.append("    \"endX\": ").append(beamEndX).append(",\n");
    json.append("    \"endY\": ").append(beamEndY).append("\n");
    json.append("  },\n");
    json.append("  \"mapPoints\": [\n");
    
    List<MapPoint> points = this.mapPoints;
    for (int i = 0; i < points.size(); i++) {
      MapPoint p = points.get(i);
      json.append("    {\"x\": ").append(p.x).append(", \"y\": ").append(p.y).append("}");
      if (i < points.size() - 1) {
        json.append(",");
      }
      json.append("\n");
    }
    
    json.append("  ],\n");
    json.append("  \"world\": {\n");
    json.append("    \"width\": ").append(simWorld != null ? simWorld.getRoomWidth() : 120).append(",\n");
    json.append("    \"height\": ").append(simWorld != null ? simWorld.getRoomHeight() : 96).append(",\n");
    json.append("    \"obstacles\": [\n");
    
    if (simWorld != null) {
      List<double[]> obstacles = simWorld.getObstacles();
      for (int i = 0; i < obstacles.size(); i++) {
        double[] obs = obstacles.get(i);
        json.append("      {\"minX\": ").append(obs[0])
            .append(", \"minY\": ").append(obs[1])
            .append(", \"maxX\": ").append(obs[2])
            .append(", \"maxY\": ").append(obs[3]).append("}");
        if (i < obstacles.size() - 1) {
          json.append(",");
        }
        json.append("\n");
      }
    }
    
    json.append("    ]\n");
    json.append("  }\n");
    json.append("}\n");
    
    byte[] response = json.toString().getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "application/json");
    exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(200, response.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(response);
    }
  }

  /**
   * Generates the HTML page with embedded Three.js visualization.
   */
  private String getVisualizationHTML() {
    return """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>XRP Robot Visualization</title>
  <style>
    body {
      margin: 0;
      overflow: hidden;
      font-family: Arial, sans-serif;
    }
    #container {
      width: 100vw;
      height: 100vh;
    }
    #info {
      position: absolute;
      top: 10px;
      left: 10px;
      color: white;
      background: rgba(0, 0, 0, 0.7);
      padding: 15px;
      border-radius: 5px;
      font-size: 14px;
      z-index: 100;
    }
    #info h2 {
      margin: 0 0 10px 0;
      font-size: 18px;
    }
    .btn {
      margin-top: 8px;
      padding: 6px 16px;
      color: white;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 14px;
      display: block;
      width: 100%;
    }
    #resetBtn { background: #cc3333; }
    #resetBtn:hover { background: #ee4444; }
    #setOriginBtn { background: #3366cc; }
    #setOriginBtn:hover { background: #4488ee; }
  </style>
</head>
<body>
  <div id="container"></div>
  <div id="info">
    <h2>XRP Robot Pose</h2>
    <div>X: <span id="posX">0.00</span> in</div>
    <div>Y: <span id="posY">0.00</span> in</div>
    <div>Heading: <span id="heading">0.0</span>&deg;</div>
    <div>Map Points: <span id="mapCount">0</span></div>
    <button id="resetBtn" class="btn">Reset Map</button>
    <button id="setOriginBtn" class="btn">Set Origin (0,0)</button>
  </div>

  <script type="importmap">
    {
      "imports": {
        "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
        "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
      }
    }
  </script>

  <script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);

    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 5000);
    camera.position.set(60, 80, 80);
    camera.lookAt(60, 0, 48);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.getElementById('container').appendChild(renderer.domElement);

    // Orbit controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(50, 100, 50);
    scene.add(directionalLight);

    // Grid floor
    const gridSize = 120;
    const gridHelper = new THREE.GridHelper(gridSize, 24, 0x444444, 0x222222);
    gridHelper.position.y = 0;
    gridHelper.position.x = gridSize / 2;
    gridHelper.position.z = gridSize / 2;
    scene.add(gridHelper);

    // Robot (colored box with arrow)
    const robotGeometry = new THREE.BoxGeometry(4, 2, 6);
    const robotMaterial = new THREE.MeshStandardMaterial({ color: 0x00ff88 });
    const robot = new THREE.Mesh(robotGeometry, robotMaterial);
    robot.position.y = 1;
    scene.add(robot);

    // Arrow to show direction
    const arrowGeometry = new THREE.ConeGeometry(1, 3, 8);
    const arrowMaterial = new THREE.MeshStandardMaterial({ color: 0xff4444 });
    const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
    arrow.rotation.x = Math.PI / 2;
    arrow.position.z = 4;
    robot.add(arrow);

    // Ultrasonic beam
    const beamMaterial = new THREE.LineBasicMaterial({ color: 0xffff00 });
    const beamGeometry = new THREE.BufferGeometry();
    const beamLine = new THREE.Line(beamGeometry, beamMaterial);
    scene.add(beamLine);

    // Map points – rendered as tiny point sprites (dots)
    const MAX_POINTS = 100000;
    const pointPositions = new Float32Array(MAX_POINTS * 3);
    const pointGeometry = new THREE.BufferGeometry();
    pointGeometry.setAttribute('position', new THREE.BufferAttribute(pointPositions, 3));
    pointGeometry.setDrawRange(0, 0);
    const pointMaterial = new THREE.PointsMaterial({
      color: 0xff6600,
      size: 1.5,
      sizeAttenuation: true
    });
    const mapPointCloud = new THREE.Points(pointGeometry, pointMaterial);
    scene.add(mapPointCloud);

    // World walls
    const wallMaterial = new THREE.MeshBasicMaterial({
      color: 0x4444ff,
      transparent: true,
      opacity: 0.2,
      side: THREE.DoubleSide
    });
    const wallsGroup = new THREE.Group();
    scene.add(wallsGroup);

    // Ground-truth obstacles (semi-transparent reference - robot does NOT see these)
    const obstaclesGroup = new THREE.Group();
    scene.add(obstaclesGroup);

    let worldInitialized = false;

    // Reset map button handler
    document.getElementById('resetBtn').addEventListener('click', async () => {
      try { await fetch('/api/resetMap', { method: 'POST' }); } catch(e) { console.error(e); }
    });

    // Set Origin button handler
    document.getElementById('setOriginBtn').addEventListener('click', async () => {
      try { await fetch('/api/setOrigin', { method: 'POST' }); } catch(e) { console.error(e); }
    });

    // Update function
    async function update() {
      try {
        const response = await fetch('/api/state');
        const data = await response.json();

        // Update robot pose
        robot.position.x = data.robot.x;
        robot.position.z = data.robot.y;
        robot.rotation.y = -data.robot.theta;

        // Update beam
        const beamStart = new THREE.Vector3(data.robot.x, 0, data.robot.y);
        const beamEnd = new THREE.Vector3(data.beam.endX, 0, data.beam.endY);
        beamGeometry.setFromPoints([beamStart, beamEnd]);

        // Update map points – render as tiny dots (point cloud)
        const numPts = Math.min(data.mapPoints.length, MAX_POINTS);
        for (let i = 0; i < numPts; i++) {
          pointPositions[i * 3]     = data.mapPoints[i].x;
          pointPositions[i * 3 + 1] = 0.15;
          pointPositions[i * 3 + 2] = data.mapPoints[i].y;
        }
        pointGeometry.setDrawRange(0, numPts);
        pointGeometry.attributes.position.needsUpdate = true;

        // Initialize world once
        if (!worldInitialized && data.world) {
          const width = data.world.width;
          const height = data.world.height;

          // Create walls
          const wallHeight = 10;
          const wallThickness = 0.5;

          // Front wall (y=0)
          const wall1 = new THREE.Mesh(
            new THREE.BoxGeometry(width, wallHeight, wallThickness),
            wallMaterial
          );
          wall1.position.set(width/2, wallHeight/2, 0);
          wallsGroup.add(wall1);

          // Back wall (y=height)
          const wall2 = new THREE.Mesh(
            new THREE.BoxGeometry(width, wallHeight, wallThickness),
            wallMaterial
          );
          wall2.position.set(width/2, wallHeight/2, height);
          wallsGroup.add(wall2);

          // Left wall (x=0)
          const wall3 = new THREE.Mesh(
            new THREE.BoxGeometry(wallThickness, wallHeight, height),
            wallMaterial
          );
          wall3.position.set(0, wallHeight/2, height/2);
          wallsGroup.add(wall3);

          // Right wall (x=width)
          const wall4 = new THREE.Mesh(
            new THREE.BoxGeometry(wallThickness, wallHeight, height),
            wallMaterial
          );
          wall4.position.set(width, wallHeight/2, height/2);
          wallsGroup.add(wall4);

          // Ground-truth obstacles (faint reference - not visible to robot)
          const obstacleMaterial = new THREE.MeshStandardMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.15
          });
          data.world.obstacles.forEach(obs => {
            const obsWidth = obs.maxX - obs.minX;
            const obsDepth = obs.maxY - obs.minY;
            const obsMesh = new THREE.Mesh(
              new THREE.BoxGeometry(obsWidth, 5, obsDepth),
              obstacleMaterial
            );
            obsMesh.position.set(
              obs.minX + obsWidth/2,
              2.5,
              obs.minY + obsDepth/2
            );
            obstaclesGroup.add(obsMesh);
          });

          worldInitialized = true;
        }

        // Update HUD
        document.getElementById('posX').textContent = data.robot.x.toFixed(2);
        document.getElementById('posY').textContent = data.robot.y.toFixed(2);
        document.getElementById('heading').textContent = (data.robot.theta * 180 / Math.PI).toFixed(1);
        document.getElementById('mapCount').textContent = data.mapPoints.length;

      } catch (error) {
        console.error('Error fetching state:', error);
      }
    }

    // Animation loop
    function animate() {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }

    // Handle window resize
    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // Start animation and update loop
    animate();
    setInterval(update, 100); // Update every 100ms
    update(); // Initial update
  </script>
</body>
</html>
""";
  }
}
