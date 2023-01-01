const express = require("express");
const bp = require("body-parser");
const app = express();
const port = 5000;

app.use("/", express.static("site"));

app.post("/run", bp.json(), (req, res) => {
  console.log(req.body);
  res.sendStatus(200);
  run(req.body);
});

app.listen(port, () => console.log(`Server listening on port: ${port}`));

async function run(opts) {
  const spawner = require("child_process").spawnSync;
  const proc = spawner("python", ["./src/main.py", ...opts], {
    encoding: "utf8",
  });
  if (proc.error) {
    console.log("ERROR: ", proc.error);
  }
  console.log("stdout: ", proc.stdout);
  console.log("stderr: ", proc.stderr);
  console.log("exit code: ", proc.status);
}
