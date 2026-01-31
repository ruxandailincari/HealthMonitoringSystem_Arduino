const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");
const WebSocket = require("ws");

const wss = new WebSocket.Server({ port: 8080 });
console.log("WebSocket server running on ws://localhost:8080");

let port;

try {
  port = new SerialPort({
    path: "COM8",
    baudRate: 115200,
    autoOpen: true
  });

  const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

  parser.on("data", line => {
    try {
      JSON.parse(line);
      wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(line);
        }
      });
    } catch {}
  });

  port.on("open", () => {
    console.log("Arduino connected on COM8");
  });

} catch (err) {
  console.log("Arduino not connected. Server runs without serial.");
}