// BGN - Configuration
let canvasConf = {
  graphs: true,
  axis: false,
  velocity: 1,
  w: 0,
  h: 0,
}

let fmtConf = {
  showV: false,
  showN_z: false,
  showDisk: false,
  showBound: false,
  miliseconds: 1,
  sizeDisk: 90,
  sizeWaypoints: 5,
  sampleOstacles: 20,
  sampleN: 1500
}

let nodesConfig = {
  shouldMove: true,

}

let initNodeConfig = {
  showDisk: false,
  showLabel: true,
  showBound: true,
  colorDisk: '#00D1EC',
  colorPoint: '#ffffff',
  colorBound: '#00D1EC',
};

let goalNodeConfig = {
  showDisk: true,
  showLabel: true,
  showBound: true,
  colorDisk: '#00D1EC',
  colorPoint: '#ffffff',
  colorBound: '#00D1EC',
}

let obstacleNodeConfig = {
  showDisk: true,
  showLabel: false,
  showBound: true,
  colorDisk: '#ec0000',
  colorPoint: '#ffffff',
  colorBound: '#ec0000',
}

let headNodeConfig = {
  showDisk: false,
  showLabel: false,
  showBound: true,
  colorDisk: '#00ec23',
  colorPoint: '#ffffff',
  colorBound: '#0ded00',
}

let pastNodeConfig = {
  showDisk: false,
  showLabel: false,
  showBound: false,
  colorDisk: '#00ec23',
  colorPoint: '#808080',
  colorBound: '#043801',
}

// END - Configuration

let points = []
let tension = 1;

let canvas = document.querySelector('canvas')
canvas.width = document.getElementById('container').offsetWidth;
canvas.height = document.getElementById('container').offsetHeight;

canvasConf.h = canvas.height;
canvasConf.w = canvas.width;

let ctx = canvas.getContext('2d')
let nodes = []

// GLOBAL VARIABLES
// Environment
let time = 0;
let coord_x_0 = canvasConf.w / 2;
let coord_y_0 = canvasConf.h / 2;
let colors = ['#00897b', '#009688', '#e0f2f1', '#e0f2f1 ']
let paramLine = 15;

// FMT VARIABLES
let saved = [];
let memory = [];
let E = [];
let V = [];
let V_unvisited = [];
let V_open = [];
let V_closed = [];
let x_init = newNode(
  coord_x_0 - 200,
  coord_y_0 + 200,
  'x_init',
  0);

let x_obstacles = []
let r_obstacle = 80;
for (let i = 0; i < fmtConf.sampleOstacles; i++) {
  let x_obstacle = newNode(
    getRandom(0, canvasConf.w),
    getRandom(0, canvasConf.h),
    'x_obstacle',
    1)
  x_obstacle.disk = r_obstacle
  x_obstacles.push(x_obstacle)
}

let x_goal = newNode(
  coord_x_0 + 200,
  coord_y_0 - 200,
  'x_goal',
  1)

let r_goal = 50;
x_goal.disk = r_goal;
/*
* FMT*
*/
V = SampleFree(fmtConf.sampleN);
V.push(x_init);
V_unvisited = V.filter(x => { return x_init != x });

V_open = [x_init]
V_closed = []

let z = x_init
let r_n = fmtConf.sizeDisk;

let N_z = Near(
  V.filter(x => { return z != x }),
  z,
  r_n
)
Save(N_z, z)


let N_x = [];
let X_near = [];
let V_open_new = [];
let zett = []

// BGN - WHILE
function next() {
  V_open_new = [];
  N_z = Near(
    V_open.filter(x => { return z != x }),
    z,
    r_n
  )
  X_near = N_z.filter(x => V_unvisited.includes(x));
  let y_min;
  for (let i = 0; i < X_near.length; i++) {
    let x = X_near[i];
    N_x = Near(
      V.filter(x_ => { return x_ != x }),
      x,
      r_n
    )
    Save(N_x, x);
    let Y_near = N_x.filter(x => V_open.includes(x));
    for (let j = 0; j < Y_near.length; j++) {
      let y = Y_near[j];
      if (!j) y_min = y

      if (y_min.cost + Cost(y_min, x) > y.cost + Cost(y, x)) {
        y_min = y;
      }
    }
    if (CollisionFree(y_min, x)) {
      E.push([y_min, x])
      V_open_new.push(x)
      V_unvisited = V_unvisited.filter(x_ => { return x_ != x });
      x.cost = y_min.cost + Cost(y_min, x)
      x.label = x.cost.toFixed(0)
    }
  }
  V_open_new.forEach(v_ => {
    V_open.push(v_)
  })
  V_open = V_open.filter(x_ => { return x_ != z })
  V_closed.push(z);
  if (!V_open.length) {
    throw Error("NO SOLUTION");
  }

  for (let i = 0; i < V_open.length; i++) {
    if (!i) z = V_open[i]
    if (V_open[i].cost < z.cost) z = V_open[i]
  }
}
// END - WHILE 

let max_iteraions = 10000;
(function myLoop(i) {
  setTimeout(function () {
    // BGN - Your code here
    try {
      zett.push(z)
      if (!isGoal(z)) {
        next()
      }
      else {
        console.log("PATH FOUND :D ")
        i = 0;
        Path(z, V_open.concat(V_closed), E)
        return;
      }
      if (--i > 0) myLoop(i);
    }
    catch (e) {
      console.error(e);
    }
    // END - Your code here

  }, fmtConf.miliseconds)
})(max_iteraions);
// }
let path = []

function Path(x_f, nodes, edges) {
  let edge;
  for (let i = 0; i < edges.length; i++) {
    if (edges[i][1] == x_f) {
      edge = edges[i];
      // console.log("Found final node", edges[i])
      path.push([edge[1], edge[0]])
    }
  }

  let maxIterations = 1000;
  while (edge[0] != x_init && maxIterations > 0) {
    for (let i = 0; i < edges.length; i++) {
      if (edges[i][1] == edge[0]) {
        edge = edges[i];
        path.push([edge[1], edge[0]])
        // console.log("Found root node", edges[i])
      }
    }
    maxIterations--;
  }

  points_ = unflatten(path)

  function unflatten(arr) {
    return arr.reduce((prevProps, nextProps) => {
      return prevProps.concat(Array.isArray(nextProps) ? unflatten(nextProps) : nextProps)
    }, [])
  }

  points_ = calcWaypoints(points_);
  (function myLoop(i) {
    setTimeout(function () {
      points.push(points_[i - 1])
      if (--i) myLoop(i);
    }, 40)
  })(points_.length);
}

/*
* A function that returns a set of 
* n € N points(samples) sampled 
* independently and identically from 
* the uniform distribution on Xfree.
*/
function SampleFree(n) {
  let sample = []
  for (let i = 0; i < n; i++) {
    sample.push(newNode(
      getRandom(0, canvasConf.w),
      getRandom(0, canvasConf.h),
      // 'f_' + i,
      '',
      1));
  }
  return sample
}

/*
* A function that stores in memory a 
* set of samples V0 associated with 
* sample v
*/
function Save(V0, v) {
  memory.push({
    'it': v,
    'has': V0
  })
}

/*
* A function that returns the set of 
* samples {u € V: ||u - v|| < r}
*/
function Near(V_, v, r) {
  // Near checks first to see if the required set of samples has already been computed and saved using Save
  // in which case it loads the set from memory, 
  for (let i = 0; i < memory.length; i++) {
    if (memory[i].it == v) {
      // console.log("Memory used")
      return memory[i].has;
    }
  }
  // otherwise it computes the required set from scratch.
  // console.log("Calculating...")
  return V_.filter(u => {
    return magnitude(u, v) < r
  })
}

function magnitude(u, v) {
  return Math.sqrt(
    (u.x - v.x) * (u.x - v.x) +
    (u.y - v.y) * (u.y - v.y)
  )
}

function isGoal(x) {
  return magnitude(x, x_goal) < r_goal;
}

/* 
* let CollisionFree (u, v) denote the boolean function which is true 
* if and only if the line joining u and v does not intersect an obstacle.
*
* TODO: Improve collision algorithm
*/
function CollisionFree(y_min_, x_) {
  return !isCollision(x_)
}

function isCollision(x_) {
  for (let i = 0; i < x_obstacles.length; i++) {
    let x_obstacle = x_obstacles[i];
    if (magnitude(x_, x_obstacle) < r_obstacle) return true
  }
  return false;
}

/*
* let Cost(u, v) be the cost of the straight line 
* joining u and v (in the current setup 
* Cost(u, v) = kv 2 uk
*/
function Cost(u, v) {
  return Math.sqrt(
    (v.x - u.x) * (v.x - u.x) +
    (v.y - u.y) * (v.y - u.y)
  )
}


function newNode(x_init, y_init, label, c) {
  return {
    x0: x_init,
    y0: y_init,
    x: x_init,
    y: y_init,
    label: label,
    rad: 3,
    disk: fmtConf.sizeDisk,
    rgba: colors[c],
    cost: 0,
    changeColor: function (i) { this.rgba = colors[c] }
  }
}


function getRandom(min, max) {
  return Math.random() * (max - min) + min;
}

function reset() {
  for (var j = 0; j < nodes.length; j++) {
    nodes[j].x = nodes[j].x0;
    nodes[j].y = nodes[j].y0;
  }
  time = 0;
}

function draw() {
  if (nodesConfig.shouldMove)
    time++;

  ctx.clearRect(0, 0, canvasConf.w, canvasConf.h);
  // BGN - Axis draw
  if (canvasConf.axis) {
    ctx.strokeStyle = "#ffffff";
    ctx.beginPath();
    ctx.moveTo(coord_x_0, 0);
    ctx.lineTo(coord_x_0, canvas.height);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, coord_y_0);
    ctx.lineTo(canvas.width, coord_y_0);
    ctx.stroke();
  }
  // END - Axis draw

  ctx.globalCompositeOperation = 'lighter';
  ctx.fillStyle = colors[2];
  ctx.strokeStyle = colors[3];

  // BGN - Title
  // ctx.font = "20px Georgia";
  // ctx.textAlign = "center";
  // ctx.fillText("FMT*", coord_x_0, 70);
  // END - Title

  // BGN - Nodes

  function show(x, config) {
    ctx.font = "15px Georgia";
    var currentNode = x;

    // BGN - Draw node
    // Point
    ctx.fillStyle = config.colorPoint;
    ctx.strokeStyle = config.colorPoint;
    ctx.beginPath();
    ctx.arc(currentNode.x, currentNode.y, currentNode.rad, 0, Math.PI * 2, true);
    ctx.fill();
    ctx.closePath();

    // Bound
    if (config.showBound) {
      ctx.strokeStyle = config.colorBound;
      let tmp = ctx.lineWidth
      ctx.beginPath();
      ctx.lineWidth = 4;
      ctx.arc(currentNode.x, currentNode.y, (currentNode.rad + 6), 0, Math.PI * 2, true);
      ctx.stroke();
      ctx.closePath();
      ctx.lineWidth = tmp;
    }

    ctx.fillStyle = '#ffffff';
    // Label
    if (canvasConf.graphs && config.showLabel) {
      ctx.font = "12px Georgia";
      ctx.fillText(currentNode.label, currentNode.x - 0, currentNode.y - paramLine);
    }

    ctx.strokeStyle = config.colorDisk;
    // Disk
    if (config.showDisk) {
      ctx.beginPath();
      ctx.arc(currentNode.x, currentNode.y, (currentNode.disk), 0, Math.PI * 2, true);
      ctx.stroke();
      ctx.closePath();
    }

    // END - Draw node
  }

  show(x_init, initNodeConfig)
  show(x_goal, goalNodeConfig)
  for (let i = 0; i < x_obstacles.length; i++) {
    show(x_obstacles[i], obstacleNodeConfig)

  }

  for (let i = 0; i < zett.length; i++) {
    if (i == zett.length - 1)
      show(zett[i], headNodeConfig)
    else
      show(zett[i], pastNodeConfig)
  }


  // BGN - V
  ctx.font = "5px Georgia";
  for (var i = 0; i < V.length && fmtConf.showV; i++) {
    var currentNode = V[i];
    ctx.fillStyle = '#ffffff';
    ctx.strokeStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(currentNode.x, currentNode.y, currentNode.rad, 0, Math.PI * 2, true);
    ctx.fill();
    ctx.closePath();
  }
  // END - V

  // BGN - N_z
  ctx.font = "5px Georgia";
  for (var i = 0; i < X_near.length && fmtConf.showN_z; i++) {
    var currentNode = X_near[i];
    ctx.strokeStyle = '#00D1EC';
    ctx.beginPath();
    ctx.arc(currentNode.x, currentNode.y, (currentNode.rad + 5), 0, Math.PI * 2, true);
    ctx.stroke();
    ctx.closePath();
  }
  // END - N_z

  // BGN - Path
  for (let i = 0; i < points.length - 1; i++) {
    ctx.beginPath();
    ctx.strokeStyle = "#ec0400";
    ctx.lineWidth = 8;
    ctx.moveTo(points[i].x, points[i].y);
    ctx.lineTo(points[i + 1].x, points[i + 1].y);
    ctx.stroke();
  }
  // END - Path

  // BGN - Edges
  for (let i = 0; i < E.length; i++) {
    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#808080";
    ctx.moveTo(E[i][0].x, E[i][0].y);
    ctx.lineTo(E[i][1].x, E[i][1].y);
    ctx.stroke();
  }
  // END - Edges
}

function findDistance(p1, p2) {
  return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
}

window.requestAnimFrame = (function () {
  return window.requestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.mozRequestAnimationFrame ||
    function (callback) {
      window.setTimeout(callback, 1000 / 60);
    };
})();


(function init() {
  canvasConf.w = canvas.width;
  canvasConf.h = canvas.height;
  coord_x_0 = canvasConf.w / 2;
  coord_y_0 = canvasConf.h / 2;
})();

window.onresize = function () {
  canvas.width = document.getElementById('container').offsetWidth;
  canvas.height = document.getElementById('container').offsetHeight;

};

(function loop() {
  draw();
  requestAnimFrame(loop);
})();

// https://stackoverflow.com/questions/23939588/how-to-animate-drawing-lines-on-canvas
function calcWaypoints(vertices) {
  var waypoints = [];
  for (var i = 1; i < vertices.length; i++) {
    var pt0 = vertices[i - 1];
    var pt1 = vertices[i];
    var dx = pt1.x - pt0.x;
    var dy = pt1.y - pt0.y;
    for (var j = 0; j < fmtConf.sizeWaypoints; j++) {
      var x = pt0.x + dx * j / fmtConf.sizeWaypoints;
      var y = pt0.y + dy * j / fmtConf.sizeWaypoints;
      waypoints.push({ x: x, y: y });
    }
  }
  return waypoints;
}