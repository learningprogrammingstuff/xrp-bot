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
 * Embedded HTTP server for 2D visualization of robot pose and map.
 * Serves an HTML5 Canvas visualization page (no external dependencies) and provides a JSON API for state updates.
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
  private volatile double totalDistance = 0.0;
  private volatile double ultrasonicRange = 0.0;
  private volatile SimWorld simWorld;
  private volatile OccupancyMapper mapper;
  private volatile EKFLocalizer localizer;

  // Rangefinder debug telemetry
  private volatile int rfDebugRawCounts = 0;
  private volatile double rfDebugVoltage = 0.0;
  private volatile double rfDebugClampedMeters = 0.0;
  private volatile double rfDebugRawMeters = 0.0;
  private volatile double rfDebugFilteredMeters = 0.0;
  private volatile boolean rfDebugValid = false;

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
   * Updates the distance measurements.
   * @param totalDist Total distance traveled by the robot (inches)
   * @param ultraRange Current ultrasonic range reading (inches)
   */
  public void updateDistances(double totalDist, double ultraRange) {
    this.totalDistance = totalDist;
    this.ultrasonicRange = ultraRange;
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
   * Updates the rangefinder debug telemetry displayed in the viewer overlay.
   *
   * @param rawCounts   Simulated 12-bit ADC raw counts for AnalogInput 2
   * @param voltage     AnalogInput 2 voltage (0–5 V)
   * @param clampedM    Clamped distance in meters (hardware-realistic, 0–4 m)
   * @param rawM        Unclamped raycast distance in meters
   * @param filteredM   Filtered clamped distance in meters (after median + outlier gate)
   * @param valid       Whether the filter considers the reading valid
   */
  public void updateRangefinderDebug(int rawCounts, double voltage,
      double clampedM, double rawM, double filteredM, boolean valid) {
    this.rfDebugRawCounts = rawCounts;
    this.rfDebugVoltage = voltage;
    this.rfDebugClampedMeters = clampedM;
    this.rfDebugRawMeters = rawM;
    this.rfDebugFilteredMeters = filteredM;
    this.rfDebugValid = valid;
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
    json.append("  \"distances\": {\n");
    json.append("    \"total\": ").append(totalDistance).append(",\n");
    json.append("    \"ultrasonic\": ").append(ultrasonicRange).append("\n");
    json.append("  },\n");
    json.append("  \"rangefinderDebug\": {\n");
    json.append("    \"rawCounts\": ").append(rfDebugRawCounts).append(",\n");
    json.append("    \"voltage\": ").append(rfDebugVoltage).append(",\n");
    json.append("    \"clampedMeters\": ").append(rfDebugClampedMeters).append(",\n");
    json.append("    \"rawMeters\": ").append(rfDebugRawMeters).append(",\n");
    json.append("    \"filteredMeters\": ").append(rfDebugFilteredMeters).append(",\n");
    json.append("    \"valid\": ").append(rfDebugValid).append("\n");
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
   * Generates the HTML page with embedded Canvas 2D visualization.
   * Uses no external dependencies – works fully offline.
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
      background: #1a1a2e;
    }
    canvas {
      display: block;
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
    #centerBtn { background: #33aa33; }
    #centerBtn:hover { background: #44cc44; }
  </style>
</head>
<body>
  <canvas id="canvas"></canvas>
  <div id="info">
    <h2>XRP Robot Pose</h2>
    <div>X: <span id="posX">0.00</span> in</div>
    <div>Y: <span id="posY">0.00</span> in</div>
    <div>Heading: <span id="heading">0.0</span>&deg;</div>
    <div>Map Points: <span id="mapCount">0</span></div>
    <div>Distance Traveled: <span id="distTotal">0.00</span> in</div>
    <div>Ultrasonic Range: <span id="distUltra">0.00</span> in</div>
    <hr style="border-color:#555;margin:8px 0">
    <h3 style="margin:0 0 6px 0;font-size:14px;color:#ffcc00">Rangefinder Debug</h3>
    <div>Raw Counts: <span id="rfCounts">0</span></div>
    <div>Voltage: <span id="rfVoltage">0.000</span> V</div>
    <div>Clamped: <span id="rfClamped">0.000</span> m</div>
    <div>Raw (unclamped): <span id="rfRaw">0.000</span> m</div>
    <div>Filtered: <span id="rfFiltered">0.000</span> m</div>
    <div>Valid: <span id="rfValid" style="font-weight:bold">--</span></div>
    <button id="resetBtn" class="btn">Reset Map</button>
    <button id="setOriginBtn" class="btn">Set Origin (0,0)</button>
    <button id="centerBtn" class="btn">Center to Robot</button>
  </div>

  <script>
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');

    // Camera state (world coordinates at center of view)
    let camX = 0, camY = 0;
    let scale = 4; // pixels per inch
    let robotState = { x: 0, y: 0, theta: 0 };
    let mapPoints = [];
    let beamEnd = { endX: 0, endY: 0 };
    let distances = { total: 0, ultrasonic: 0 };

    // Resize canvas to fill window
    function resize() {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    }
    window.addEventListener('resize', resize);
    resize();

    // --- Pan & Zoom ---
    let isDragging = false, dragStartX = 0, dragStartY = 0, camStartX = 0, camStartY = 0;
    canvas.addEventListener('mousedown', e => {
      isDragging = true;
      dragStartX = e.clientX;
      dragStartY = e.clientY;
      camStartX = camX;
      camStartY = camY;
    });
    canvas.addEventListener('mousemove', e => {
      if (!isDragging) return;
      camX = camStartX - (e.clientX - dragStartX) / scale;
      camY = camStartY + (e.clientY - dragStartY) / scale;
    });
    canvas.addEventListener('mouseup', () => { isDragging = false; });
    canvas.addEventListener('mouseleave', () => { isDragging = false; });
    canvas.addEventListener('wheel', e => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.1 : 0.9;
      scale = Math.max(0.5, Math.min(50, scale * factor));
    }, { passive: false });

    // World-to-screen transform
    function toScreen(wx, wy) {
      const sx = canvas.width / 2 + (wx - camX) * scale;
      const sy = canvas.height / 2 - (wy - camY) * scale;
      return [sx, sy];
    }

    // Draw the scene
    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#1a1a2e';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Grid lines
      ctx.strokeStyle = '#333344';
      ctx.lineWidth = 0.5;
      const gridSpacing = 12; // 12 inches = 1 foot
      const left = camX - canvas.width / 2 / scale;
      const right = camX + canvas.width / 2 / scale;
      const bottom = camY - canvas.height / 2 / scale;
      const top = camY + canvas.height / 2 / scale;

      const gxStart = Math.floor(left / gridSpacing) * gridSpacing;
      const gxEnd = Math.ceil(right / gridSpacing) * gridSpacing;
      const gyStart = Math.floor(bottom / gridSpacing) * gridSpacing;
      const gyEnd = Math.ceil(top / gridSpacing) * gridSpacing;

      for (let gx = gxStart; gx <= gxEnd; gx += gridSpacing) {
        const [sx] = toScreen(gx, 0);
        ctx.beginPath();
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, canvas.height);
        ctx.stroke();
      }
      for (let gy = gyStart; gy <= gyEnd; gy += gridSpacing) {
        const [, sy] = toScreen(0, gy);
        ctx.beginPath();
        ctx.moveTo(0, sy);
        ctx.lineTo(canvas.width, sy);
        ctx.stroke();
      }

      // Origin axes
      const [ox, oy] = toScreen(0, 0);
      ctx.strokeStyle = 'rgba(255,100,100,0.4)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, canvas.height); ctx.stroke();
      ctx.strokeStyle = 'rgba(100,255,100,0.4)';
      ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(canvas.width, oy); ctx.stroke();

      // Map points (orange dots)
      ctx.fillStyle = '#ff6600';
      for (const p of mapPoints) {
        const [sx, sy] = toScreen(p.x, p.y);
        ctx.beginPath();
        ctx.arc(sx, sy, Math.max(2, scale * 0.6), 0, Math.PI * 2);
        ctx.fill();
      }

      // Ultrasonic beam (yellow line)
      const [bsx, bsy] = toScreen(robotState.x, robotState.y);
      const [bex, bey] = toScreen(beamEnd.endX, beamEnd.endY);
      ctx.strokeStyle = '#ffff00';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(bsx, bsy);
      ctx.lineTo(bex, bey);
      ctx.stroke();

      // Robot body (green rectangle with red heading arrow)
      const [rx, ry] = toScreen(robotState.x, robotState.y);
      const rw = 4 * scale, rh = 6 * scale;
      ctx.save();
      ctx.translate(rx, ry);
      ctx.rotate(-robotState.theta);
      ctx.fillStyle = '#00ff88';
      ctx.fillRect(-rw / 2, -rh / 2, rw, rh);
      // Heading arrow
      ctx.fillStyle = '#ff4444';
      ctx.beginPath();
      ctx.moveTo(0, -rh / 2 - 4 * scale / 2);
      ctx.lineTo(-rw / 4, -rh / 2 + 2);
      ctx.lineTo(rw / 4, -rh / 2 + 2);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      requestAnimationFrame(draw);
    }

    // Reset map button
    document.getElementById('resetBtn').addEventListener('click', async () => {
      try { await fetch('/api/resetMap', { method: 'POST' }); } catch(e) { console.error(e); }
    });

    // Set Origin button
    document.getElementById('setOriginBtn').addEventListener('click', async () => {
      try { await fetch('/api/setOrigin', { method: 'POST' }); } catch(e) { console.error(e); }
    });

    // Center to Robot button
    document.getElementById('centerBtn').addEventListener('click', () => {
      camX = robotState.x;
      camY = robotState.y;
    });

    // Fetch state from server
    async function update() {
      try {
        const response = await fetch('/api/state');
        const data = await response.json();
        robotState = data.robot;
        beamEnd = data.beam;
        mapPoints = data.mapPoints;
        distances = data.distances || { total: 0, ultrasonic: 0 };

        // Update HUD
        document.getElementById('posX').textContent = data.robot.x.toFixed(2);
        document.getElementById('posY').textContent = data.robot.y.toFixed(2);
        document.getElementById('heading').textContent = (data.robot.theta * 180 / Math.PI).toFixed(1);
        document.getElementById('mapCount').textContent = data.mapPoints.length;
        document.getElementById('distTotal').textContent = distances.total.toFixed(2);
        document.getElementById('distUltra').textContent = distances.ultrasonic.toFixed(2);

        // Update rangefinder debug panel
        const rf = data.rangefinderDebug;
        if (rf) {
          document.getElementById('rfCounts').textContent = rf.rawCounts;
          document.getElementById('rfVoltage').textContent = rf.voltage.toFixed(3);
          document.getElementById('rfClamped').textContent = rf.clampedMeters.toFixed(3);
          document.getElementById('rfRaw').textContent = rf.rawMeters.toFixed(3);
          document.getElementById('rfFiltered').textContent = rf.filteredMeters.toFixed(3);
          const validEl = document.getElementById('rfValid');
          validEl.textContent = rf.valid ? 'YES' : 'NO';
          validEl.style.color = rf.valid ? '#00ff88' : '#ff4444';
        }
      } catch (error) {
        console.error('Error fetching state:', error);
      }
    }

    // Handle window resize
    window.addEventListener('resize', () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    });

    // Start
    draw();
    setInterval(update, 100);
    update();
  </script>
</body>
</html>
""";
  }
}
