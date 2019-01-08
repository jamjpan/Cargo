const DEBUG = false

var socket                  = io()
var main_container          = document.getElementById("main")
var dom_footer              = document.querySelector("footer")

function footer(string) {
  dom_footer.innerText = string
  window.setTimeout(() => dom_footer.innerText = "", 3000)
}
// Editor ----------------------------------------------------------------------
var editor                  = document.getElementById("editor")
var editor_selector         = document.getElementById("editor_selector")
var editor_window           = document.getElementById("editor_window")
var editor_toolbar_close    = document.getElementById("editor_toolbar_close")
var editor_toolbar_reset    = document.getElementById("editor_toolbar_reset")
var editor_toolbar_load_cpp = document.getElementById("editor_toolbar_load_cpp")
var editor_toolbar_load_h   = document.getElementById("editor_toolbar_load_h")
var cm_options = {
  mode: "text/x-c++src",
  lineNumbers: true,
  viewportMargin: Infinity,
  extraKeys: {
    "F11": (cm) => { beg_fullscreen(cm) },
    "Esc": (cm) => { end_fullscreen(cm) }
  },
  foldGutter: true,
  gutters: ["CodeMirror-linenumbers", "CodeMirror-foldgutter"],
  theme: "tomorrow-night-bright",
}
var editor_cpp  = CodeMirror.fromTextArea(document.getElementById("editor_cpp_container"), cm_options)
var editor_h    = CodeMirror.fromTextArea(document.getElementById("editor_h_container")  , cm_options)

editor_selector         .addEventListener("change", editor_load_example)
editor_toolbar_close    .addEventListener("click" , editor_toggle)
editor_toolbar_reset    .addEventListener("click" , editor_reset)
editor_toolbar_load_cpp .addEventListener("change", editor_load_cpp)
editor_toolbar_load_h   .addEventListener("change", editor_load_h)

editor_reset()
editor_toggle()

function editor_toggle()    { editor.hidden = (editor.hidden == true ? false : true) }
function beg_fullscreen(cm) { cm.setOption("fullScreen", !cm.getOption("fullScreen")) }
function end_fullscreen(cm) { if (cm.getOption("fullScreen")) cm.setOption("fullScreen", false) }
function editor_reset()     {
  editor_cpp.setSize(editor_window.offsetWidth/2 - 1, editor_window.offsetHeight - 2)
  editor_h  .setSize(editor_window.offsetWidth/2 - 1, editor_window.offsetHeight - 2)
  editor_cpp.setValue(cc_template)
  editor_h  .setValue(h_template)
}
function editor_load_cpp(e) { file_import(e, (val) => { if (val != false) editor_cpp.setValue(val) }) }
function editor_load_h(e)   { file_import(e, (val) => { if (val != false) editor_h  .setValue(val) }) }
function editor_load_example() {
  if (editor_selector.value == "**custom**") editor_reset()
  else socket.emit('client cmd load example', editor_selector.value)
}
function file_import(e, callback) {
  var file = null
  file = e.target.files[0]
  if (!file)
    return false
  var reader = new FileReader()
  reader.onload = (e) => { callback(e.target.result) }
  reader.readAsText(file)
}
// Controls --------------------------------------------------------------------
var controls            = new dat.GUI({ autoPlace: false, width: 280 })
var controllers         = {}
// Controls: rsalgorithm (code editor)
var controls_editor     = controls.addFolder("rsalgorithm")
var controls_editor_params = {
  'show/hide editor': editor_toggle,
  'compile':          compile,
  'compile + run':    compile_run
}
controls_editor.add(controls_editor_params, 'show/hide editor')
controls_editor.add(controls_editor_params, 'compile')
controls_editor.add(controls_editor_params, 'compile + run')
// Controls: simulation settings
var controls_settings   = controls.addFolder("simulation settings")
var controls_settings_params = {
  'road': 'bj5',
  'vehicles': '5k',
  'vehicle capacity': '3',
  'vehicle speed': '10 m/sec',
  'customers scale': '1.0x (~9.7/sec)',
  'delay tolerance': '6 min',
  'match period (sec)': 60,
  'runtime mode': 'dynamic',
  'routing mode': 'merge'
}
controls_road_selector = controls_settings.add(controls_settings_params, 'road', ['bj5', 'mny', 'cd1']).onFinishChange(change_road)
controls_settings.add(controls_settings_params, 'vehicles', ['1k', '5k', '10k', '15k', '20k', '25k', '30k', '35k', '40k', '45k', '50k'])
controls_settings.add(controls_settings_params, 'vehicle capacity', [1, 3, 6, 9])
controls_settings.add(controls_settings_params, 'vehicle speed', ['10 m/sec'])
controls_settings.add(controls_settings_params, 'customers scale', ['0.5x (~4.9/sec)', '1.0x (~9.7/sec)', '2.0x (~19.0/sec)', '4.0x (~36.4/sec)'])
controls_settings.add(controls_settings_params, 'delay tolerance', ['2 min', '4 min', '6 min', '8 min'])
controls_settings.add(controls_settings_params, 'match period (sec)')
controls_settings.add(controls_settings_params, 'routing mode', ['strict', 'merge'])
controls_settings.add(controls_settings_params, 'runtime mode', ['static', 'dynamic'])
// Controls: main
var controls_params = {
  'show road':        true,
  'compile + run':    compile_run,
  'stop simulation':  stop
}
controls.add(controls_params, 'show road').onFinishChange(toggle_visible_road)
controls.add(controls_params, 'compile + run')
controls.add(controls_params, 'stop simulation')

main_container.appendChild(controls.domElement)

function toggle_visible_road(toggle) { Simulation.road_network.material.visible = toggle }
function problem_str() {
  return "rs-"  + controls_settings_params["road"]
         + "-m" + controls_settings_params["vehicles"]
         + "-c" + controls_settings_params["vehicle capacity"]
         + "-d" + controls_settings_params["delay tolerance"].split(" ")[0]
         + "-s" + controls_settings_params["vehicle speed"].split(" ")[0]
         + "-x" + controls_settings_params["customers scale"].split(" ")[0].slice(0, -1)
}
// Visualization ---------------------------------------------------------------
const bg_color = 0x000000  // black
const rn_color = 0x132a42  // dark blue
const rn_linewidth = 1
const customer_scales_lookup = {  // hack.. but better than asking the server
  "bj5": { "0.5x": 8754, "1.0x": 17467, "2.0x": 34152, "4.0x": 65500 },
  "cd1": { "0.5x": 4534, "1.0x": 8922,  "2.0x": 18356, "4.0x": 42573 },
  "mny": { "0.5x": 2390, "1.0x": 5033,  "2.0x": 10512, "4.0x": 22165 }
}

var viz                 = document.getElementById("main")
var vertexshader        = document.getElementById("vertexshader")
var fragmentshader      = document.getElementById("fragmentshader")
var vertexshader_cust   = document.getElementById("vertexshader_cust")
var fragmentshader_cust = document.getElementById("fragmentshader_cust")

var Simulation = {

  // Call init() only ONCE
  init() {
    this.aspect     = viz.offsetWidth/viz.offsetHeight
    this.clock      = new THREE.Clock()
    this.stats      = new Stats()
    this.scene      = new THREE.Scene()
    this.camera     = new THREE.OrthographicCamera(-0.1, 0.1, 0.1/this.aspect, -0.1/this.aspect, 1, 3)
    this.box        = new THREE.Box2()
    this.center     = new THREE.Vector2
    this.renderer   = new THREE.WebGLRenderer({ alpha: true})
    this.controls   = new THREE.MapControls(this.camera, this.renderer.domElement)

    this.stats      .showPanel(0) // 0: fps, 1: ms, 2: mb, 3+: custom
    this.renderer   .setSize(viz.offsetWidth, viz.offsetHeight)
    this.renderer   .setClearColor (bg_color, 1)
    this.controls   .enableDamping = true
    this.controls   .dampingFactor = 0.25
    this.controls   .enableKeys = false
    this.controls   .enableRotate = false
    this.controls   .screenSpacePanning = true
    viz.appendChild(this.renderer.domElement)
    viz.appendChild(this.stats.dom);

    window.addEventListener('resize', () => {
      this.aspect = viz.offsetWidth/viz.offsetHeight
      this.renderer.setSize(viz.offsetWidth, viz.offsetHeight)
      this.camera.top = 0.1/this.aspect
      this.camera.bottom = -0.1/this.aspect
      this.camera.updateProjectionMatrix()
    }, false)
  },

  // Reset the scene, but doesn't change the settings
  // (can't figure out how to set values in dat.gui)
  // Call whenever "run" is issued
  reset() {
    clear3(this.scene)  // erases everything!

    change_road(controls_settings_params["road"])

    this.started = false
    this.uniforms = {
      t:    { value: 0.0 },  // current time
      zoom: { value: 1.0 }   // camera zoom
    }
    this.nvehl = Number(controls_settings_params["vehicles"].split("k")[0])*1000
    this.vehicle_particles = new THREE.Points(this.vehicle_geometry(this.nvehl), new THREE.ShaderMaterial({
      uniforms: this.uniforms,
      vertexShader: vertexshader.text,
      fragmentShader: fragmentshader.text
    }))
    this.vehicle_speed     = Number(controls_settings_params["vehicle speed"].split(" ")[0])
    this.vehicle_position  = this.vehicle_particles.geometry.attributes.position.array
    this.vehicle_target0   = this.vehicle_particles.geometry.attributes.target0.array
    this.vehicle_target1   = this.vehicle_particles.geometry.attributes.target1.array
    this.vehicle_dur0      = this.vehicle_particles.geometry.attributes.dur0.array
    this.vehicle_dur1      = this.vehicle_particles.geometry.attributes.dur1.array
    this.vehicle_t0        = this.vehicle_particles.geometry.attributes.t0.array
    this.vehicle_loads     = this.vehicle_particles.geometry.attributes.load.array
    this.scene.add(this.vehicle_particles)

    this.ncust = customer_scales_lookup[controls_settings_params["road"]][controls_settings_params["customers scale"].split(" ")[0]]
    this.customer_particles = new THREE.Points(this.customer_geometry(this.ncust), new THREE.ShaderMaterial({
      uniforms: this.uniforms,
      vertexShader: vertexshader_cust.text,
      fragmentShader: fragmentshader_cust.text
    }))
    this.customer_position = this.customer_particles.geometry.attributes.position.array
    this.customer_sizes    = this.customer_particles.geometry.attributes.size.array
    this.scene.add(this.customer_particles)
  },

  // Return a vehicle geometry for vehicle points object
  vehicle_geometry(nvehl) {
    var temp_vehicle_geom   = new THREE.BufferGeometry()
    var init_pos            = new Array(this.nvehl*3).fill(0.0);
    var init_ones           = new Array(this.nvehl  ).fill(1.0);
    var init_zeros          = new Array(this.nvehl  ).fill(0.0);
    // force the bounding box to avoid recomputing each frame
    init_pos.push(this.box.min['x'], this.box.min['y'], 0)
    init_pos.push(this.box.max['x'], this.box.max['y'], 0)
    // fill ones and zeros to the correct size
    init_ones .push(1.0, 1.0)
    init_zeros.push(0.0, 0.0)
    temp_vehicle_geom.addAttribute('position', new THREE.Float32BufferAttribute(init_pos,   3))
    temp_vehicle_geom.addAttribute('target0',  new THREE.Float32BufferAttribute(init_pos,   3))
    temp_vehicle_geom.addAttribute('target1',  new THREE.Float32BufferAttribute(init_pos,   3))
    temp_vehicle_geom.addAttribute('dur0',     new THREE.Float32BufferAttribute(init_ones , 1))
    temp_vehicle_geom.addAttribute('dur1',     new THREE.Float32BufferAttribute(init_ones,  1))
    temp_vehicle_geom.addAttribute('t0',       new THREE.Float32BufferAttribute(init_zeros, 1))
    temp_vehicle_geom.addAttribute('load',     new THREE.Float32BufferAttribute(init_zeros, 1))
    return temp_vehicle_geom
  },

  // Return a customer geometry for customer points object
  customer_geometry(ncust) {
    var temp_customer_geom  = new THREE.BufferGeometry()
    var init_pos            = new Array(this.ncust*3).fill(0.0);
    var init_zeros          = new Array(this.ncust  ).fill(0.0);
    // force the bounding box to avoid recomputing each frame
    init_pos.push(this.box.min['x'], this.box.min['y'], 0)
    init_pos.push(this.box.max['x'], this.box.max['y'], 0)
    // fill zeros to the correct size
    init_zeros.push(0.0, 0.0)
    temp_customer_geom.addAttribute('position',new THREE.Float32BufferAttribute(init_pos,   3))
    temp_customer_geom.addAttribute('size',    new THREE.Float32BufferAttribute(init_zeros, 1))
    return temp_customer_geom
  }
}

function change_road(road) {
  Simulation.box.makeEmpty()
  if (Simulation.road_network) clear1(Simulation.road_network)
  if (road == "bj5") {
    Simulation.rn_positions = bj5_positions
    Simulation.rn_nodes     = bj5_nodes
    Simulation.rn_weights   = bj5_weights
  } else if (road == "mny") {
    Simulation.rn_positions = mny_positions
    Simulation.rn_nodes     = mny_nodes
    Simulation.rn_weights   = mny_weights
  } else if (road == "cd1") {
    Simulation.rn_positions = cd1_positions
    Simulation.rn_nodes     = cd1_nodes
    Simulation.rn_weights   = cd1_weights
  }
  var temp_buffer_arr = new Float32Array(Simulation.rn_positions.length/2*3);
  var temp_buffer_arr_idx = 0
  for (var i = 0; i < Simulation.rn_positions.length; i+=2) {
    temp_buffer_arr[temp_buffer_arr_idx++] = Simulation.rn_positions[i]
    temp_buffer_arr[temp_buffer_arr_idx++] = Simulation.rn_positions[i+1]
    temp_buffer_arr[temp_buffer_arr_idx++] = 0
    Simulation.box.expandByPoint(new THREE.Vector2(Simulation.rn_positions[i], Simulation.rn_positions[i+1]))
  }
  Simulation.box.getCenter(Simulation.center)

  // Create a new LineSegments and add it to the scene
  Simulation.road_network = new THREE.LineSegments(
    new THREE.BufferGeometry().addAttribute('position', new THREE.Float32BufferAttribute(temp_buffer_arr, 3)),
    new THREE.LineBasicMaterial({
      color: rn_color,
      linewidth: rn_linewidth
    })
  )
  Simulation.road_network.matrixAutoUpdate = false
  Simulation.scene.add(Simulation.road_network)

  // Reset the camera and controls to point to the new LineSegments
  Simulation.camera.position.set(Simulation.center['x'], Simulation.center['y'], 2)
  Simulation.controls.target = new THREE.Vector3(Simulation.center['x'], Simulation.center['y'], 0)
}

// Pass this.scene to clear the entire scene
// https://stackoverflow.com/questions/30359830/how-do-i-clear-three-js-scene
function clear3(obj) {
  while (obj.children.length > 0) {
    clear3(obj.children[0])
    obj.remove(obj.children[0])
  }
  if (obj.geometry) obj.geometry.dispose()
  if (obj.material) obj.material.dispose()
  if (obj.texture)  obj.texture .dispose()
}

function clear1(obj) {
  Simulation.scene.remove(obj)
  if (obj.geometry) obj.geometry.dispose()
  if (obj.material) obj.material.dispose()
  if (obj.texture)  obj.texture .dispose()
}

function animate() {
  Simulation.stats.begin()
  Simulation.controls.update()
  Simulation.vehicle_particles.geometry.attributes.position.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.target0.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.target1.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.dur0.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.dur1.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.t0.needsUpdate = true
  Simulation.vehicle_particles.geometry.attributes.load.needsUpdate = true
  Simulation.customer_particles.geometry.attributes.position.needsUpdate = true
  Simulation.customer_particles.geometry.attributes.size.needsUpdate = true
  Simulation.uniforms.t.value = Simulation.clock.getElapsedTime()
  Simulation.uniforms.zoom.value = Simulation.camera.zoom
  Simulation.road_network.material.linewidth = 1.0*Simulation.camera.zoom
  Simulation.renderer.render(Simulation.scene, Simulation.camera)
  Simulation.stats.end()
  requestAnimationFrame(animate)
}

Simulation.init(); Simulation.reset(); animate()

function weight(u, v) {
  var key1 = u.toString()+"_"+v.toString()
  var key2 = v.toString()+"_"+u.toString()
  if      (key1 in Simulation.rn_weights)   return Simulation.rn_weights[key1]
  else if (key2 in Simulation.rn_weights)   return Simulation.rn_weights[key2]
  else                                      return 0
}
// Socket controls -----------------------------------------------------------
var cargofeed = document.getElementById("cargofeed")
var algfeed   = document.getElementById("algfeed")

socket.on("server error"        , (data) => { /* handle error */ })
socket.on("server msg"          , (data) => { log_cargo_append(data) })
socket.on('server msg alg'      , (data) => { log_alg_print(data) })
socket.on('server msg vehicle'  , (data) => {
  if (DEBUG) console.log(data)
  var vid = Number(data.vid) - 1  // 0-indexed here, 1-indexed in Cargo
  var i = 3*vid
  // Place vehicle at data.position + overshoot in direction of target0
  // and update target1
  if (data.position in Simulation.rn_nodes
   && data.target0  in Simulation.rn_nodes
   && data.target1  in Simulation.rn_nodes) {
    var x0 = Simulation.rn_nodes[data.position][0]
    var y0 = Simulation.rn_nodes[data.position][1]
    var x1 = Simulation.rn_nodes[data.target0 ][0]
    var y1 = Simulation.rn_nodes[data.target0 ][1]
    var w = weight(data.position, data.target0)
    var delta = -Number(data.overshoot)
    var d = delta/w
    var x = d*(x1-x0)+x0
    var y = d*(y1-y0)+y0

    Simulation.vehicle_position[i  ] = x
    Simulation.vehicle_position[i+1] = y
    Simulation.vehicle_target0 [i  ] = Simulation.rn_nodes[data.target0][0]  // lng
    Simulation.vehicle_target0 [i+1] = Simulation.rn_nodes[data.target0][1]  // lat
    Simulation.vehicle_target1 [i  ] = Simulation.rn_nodes[data.target1][0]
    Simulation.vehicle_target1 [i+1] = Simulation.rn_nodes[data.target1][1]

    Simulation.vehicle_dur0[vid] = (w-delta)/Simulation.vehicle_speed
    Simulation.vehicle_dur1[vid] = weight(data.target0, data.target1)/Simulation.vehicle_speed
    Simulation.vehicle_t0[vid]   = Simulation.clock.getElapsedTime()

    if (DEBUG) {
      console.log(`w=${w},d=${d},x0=${x0},y0=${y0},x1=${x1},y1=${y1},
         dur0=${Simulation.vehicle_dur0[vid]},dur1=${Simulation.vehicle_dur1[vid]},
         t0=${Simulation.vehicle_t0[vid]},t=${Simulation.uniforms.t.value}`)
      console.log('got V, set x,y to ', x, y)
    }
  }
})
socket.on('server msg begin route'  , (data) => {
  if (DEBUG) console.log(data)
  var vid = Number(data.vid)-1  // 0-indexed here, 1-indexed in Cargo
  var i = 3*vid
  // Initialize first three points
  if (data.route.length > 2) {
    Simulation.vehicle_position[i  ] = Simulation.rn_nodes[data.route[0]][0]  // lng
    Simulation.vehicle_position[i+1] = Simulation.rn_nodes[data.route[0]][1]  // lat
    Simulation.vehicle_target0 [i  ] = Simulation.rn_nodes[data.route[1]][0]
    Simulation.vehicle_target0 [i+1] = Simulation.rn_nodes[data.route[1]][1]
    Simulation.vehicle_target1 [i  ] = Simulation.rn_nodes[data.route[2]][0]
    Simulation.vehicle_target1 [i+1] = Simulation.rn_nodes[data.route[2]][1]

    Simulation.vehicle_dur0[vid] = weight(data.route[0], data.route[1])/Simulation.vehicle_speed
    Simulation.vehicle_dur1[vid] = weight(data.route[1], data.route[2])/Simulation.vehicle_speed
    Simulation.vehicle_t0[vid] = Simulation.clock.getElapsedTime()
  } else {  // could be an idling vehicle
    Simulation.vehicle_position[i  ] = Simulation.rn_nodes[data.route[0]][0]  // lng
    Simulation.vehicle_position[i+1] = Simulation.rn_nodes[data.route[0]][1]  // lat
    Simulation.vehicle_target0 [i  ] = Simulation.rn_nodes[data.route[0]][0]
    Simulation.vehicle_target0 [i+1] = Simulation.rn_nodes[data.route[0]][1]
    Simulation.vehicle_target1 [i  ] = Simulation.rn_nodes[data.route[0]][0]
    Simulation.vehicle_target1 [i+1] = Simulation.rn_nodes[data.route[0]][1]

    Simulation.vehicle_dur0[vid] = 1
    Simulation.vehicle_dur1[vid] = 1
    Simulation.vehicle_t0[vid] = Simulation.clock.getElapsedTime()
  }
})
socket.on('server msg customer' , (data) => {
  for (var k = 0; k < data.length; k++) {
    var cid = Number(data[k][0]) - 1 - Simulation.nvehl
    var i = 3*cid
    var loc = data[k][1]
    Simulation.customer_position[i  ] = Simulation.rn_nodes[loc][0]
    Simulation.customer_position[i+1] = Simulation.rn_nodes[loc][1]
    Simulation.customer_sizes[cid] = 3.0
  }
})
socket.on('server msg customer_timeout', (data) => {
  for (var k = 0; k < data.length; k++) {
    var cid = Number(data[k]) - 1 - Simulation.nvehl
    Simulation.customer_sizes[cid] = 0.0
  }
})
socket.on('server msg pickup'   , (data) => {
  for (var k = 0; k < data.length; k++) {
    var cid = Number(data[k]) - 1 - Simulation.nvehl
    Simulation.customer_sizes[cid] = 0.0
  }
})
socket.on('server msg load'     , (data) => {
  for (var i = 0; i < data.length; i++) {
    var idx = Number(data[i])
    if (idx < 0) {
      Simulation.vehicle_loads[-(idx+1)] = Simulation.vehicle_loads[-(idx+1)] - 1
      if (DEBUG) console.log(Simulation.vehicle_loads[-(idx+1)])
    }
    else {
      Simulation.vehicle_loads[idx-1] = Simulation.vehicle_loads[idx-1] + 1
      if (DEBUG) console.log(Simulation.vehicle_loads[idx-1])
    }
  }
})
socket.on('server msg initialize'   , (data) => {
  // Initially animate the road network when client connects
  Simulation.nvehl = data.nvehl
  Simulation.ncust = data.ncust
  Simulation.init()
  change_road(controls_settings_params["road"])
  Simulation.vehicle_speed = Number(controls_settings_params["vehicle speed"].split(" ")[0])
  animate()
  if (Simulation.started) {
    // Enter only if client presses 'start'
    Simulation.reset()
    socket.emit('client cmd run')
  }
})
socket.on('server msg started'  , () => { Simulation.clock.start() })
socket.on('server msg load_example', (data) => {
  editor_cpp.setValue(data[0])
  editor_h  .setValue(data[1])
})

// Why we call this?
// socket.emit('client query initialize', problem_str())
// socket.emit('client query running')

reset_feeds()

function reset_feeds() {
  cargofeed.value = "cargo\n"
  algfeed.value   = "rsalgorithm\n"
}

function log_cargo_append(data) { cargofeed.value += data; cargofeed.scrollTop = cargofeed.scrollHeight }
function log_alg_print(data) { algfeed.value = data; algfeed.scrollTop = algfeed.scrollHeight }

function compile() {
  log_cargo_append("Compiling...\n")
  socket.emit("client cmd compile_h" , editor_h  .getValue())
  socket.emit("client cmd compile_cc", editor_cpp.getValue())
  socket.emit("client msg compile_opts", {
    problem: problem_str(),
    road:    controls_settings_params["road"],
    speed:   controls_settings_params["vehicle speed"].split(" ")[0],
    mat:     controls_settings_params["match period (sec)"],
    mode:    controls_settings_params["runtime mode"],
    route:   controls_settings_params["routing mode"]
  })
}

function compile_run() {
  if (!Simulation.started) {
    compile()
    // TODO: Don't continue if compile fails
    Simulation.started = true
    log_cargo_append("Started\n")
    socket.emit('client query initialize', problem_str())
  } else {
    footer("Already started.")
  }
}

function stop() {
  Simulation.clock.stop()
  Simulation.started = false
  socket.emit('client cmd stop')
  if (!Simulation.started) {
    footer("Not running.")
  } else {
    log_cargo_append("Stopped\n")
  }
}

// Drop the curtains
var loader = document.getElementById("loader")
loader.style.backgroundColor = "transparent"
loader.innerText = "Initialized"
window.setTimeout(() => { loader.style.display = "none" }, 1000)

