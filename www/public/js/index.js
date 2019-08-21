const DEBUG             = false
// Style -----------------------------------------------------------------------
// https://github.com/morhetz/gruvbox
// const bg_color          = 0x1d2021  // bg0_h
// const rn_color          = 0x32302f  // bg0_s
const bg_color          = 0xffffff
const rn_color          = 0xeeeeee
const rn_linewidth      = 2
const rn_linewidth_modulate = true  // set false to disable thicker lines at high zoom
// const line_color0       = 0xfb4934  // red (167)
// const line_color1       = 0x83a598  // blue (109)
// const line_color2       = 0xfabd2f  // yellow (214)
const line_color0       = 0xff0000  // red
const line_color1       = 0x6f688e  // purple grey-ish
const line_color2       = 0x00ff00  // green
const line_color3       = 0x0000ff  // blue
// const hi_color          = (locate in index.html > fragment_hi)
// const cust_color        = (locate in index.html > fragment_cust)
// const vehl_color        = (locate in index.html > fragment_vehl)
// -----------------------------------------------------------------------------
// NOTES
// - Raycaster disabled for now
// - Raycaster precision is not great at these scales. It computes intersection
//   using world-space. The size of each point in this space is very small
//   (order 10^-4) because the world unit is longitude/latitude coordinates. It
//   might be better to convert road network coordinates to UTM. Then the
//   precision might be better.
// - Raycaster threshold parameter determines if an intersection occurs. It is
//   the diameter of a circle centered around mouse click. From testing, a
//   value of 0.00002 seems to be just enough to register an intersection if the
//   click is right in the middle of the particle; a value of 0.00010 seems to be
//   enough to register click on edge of particle.
// - TODO: decide to support multi-users or single-user (look into electron app?)
// - RSAlgorithm::pause() blocks the listen and step threads until user hits "Enter".
//   Unfortunately I cannot get node to pass button press from client to trigger
//   and "enter" in cargo's stdin. I tried child.stdin.write() but got EPIPE error.
//   I saw there is a bug in 10x so I upgraded node to nightly but still error.
//   I also tried simply console.log but that did nothing.  On c++ side I also
//   tried std::cin.get(), getline(), and std::cin >> x. For now, user has to
//   physically press enter in the server window to continue.
const lu_scales = {  // hack.. but better than asking server
  "bj5": { "0.5x": 8754, "1.0x": 17467, "2.0x": 34152, "4.0x": 65500 },
  "cd1": { "0.5x": 4534, "1.0x":  8922, "2.0x": 18356, "4.0x": 42573 },
  "mny": { "0.5x": 2390, "1.0x":  5033, "2.0x": 10512, "4.0x": 22165 }}
var socket              = io()
var main_container      = document.getElementById("main")
var dom_footer          = document.querySelector("footer")
// var info_content        = document.querySelector("#info > p")
// var resume              = document.getElementById("resume")
// var resume_btn          = document.querySelector("#resume > button")

function footer(string) { dom_footer.innerText = string }
// Editor + Log ----------------------------------------------------------------
var log                 = document.getElementById("log")
var log_show_hide       = document.getElementById("log_show_hide")
var editor              = document.getElementById("editor")
var editor_selector     = document.getElementById("editor_selector")
var editor_window       = document.getElementById("editor_window")
var editor_tool_close   = document.getElementById("editor_toolbar_close")
var editor_tool_reset   = document.getElementById("editor_toolbar_reset")
var editor_load_cpp     = document.getElementById("editor_toolbar_load_cpp")
var editor_load_h       = document.getElementById("editor_toolbar_load_h")
var cm_options = {
  mode: "text/x-c++src",
  lineNumbers: true,
  viewportMargin: Infinity,
  extraKeys: {
    "F11": (cm) => { beg_fscrn(cm) },
    "Esc": (cm) => { end_fscrn(cm) }
  },
  foldGutter: true,
  gutters: ["CodeMirror-linenumbers", "CodeMirror-foldgutter"],
  theme: "tomorrow-night-bright",
}
var editor_cpp          = CodeMirror.fromTextArea(document.getElementById("editor_cpp_container"), cm_options)
var editor_h            = CodeMirror.fromTextArea(document.getElementById("editor_h_container"  ), cm_options)
log_show_hide           .addEventListener("click" , log_toggle)
editor_selector         .addEventListener("change", editor_load_example)
editor_tool_close       .addEventListener("click" , editor_toggle)
editor_tool_reset       .addEventListener("click" , editor_reset)
editor_load_cpp         .addEventListener("change", editor_load_cpp)
editor_load_h           .addEventListener("change", editor_load_h)

editor_reset(); editor_toggle()

function log_toggle()   { log.style.bottom = (log.style.bottom.split("px")[0] < 0 ? "0px" : "-240px") }
function end_fscrn(cm)  { if (cm.getOption("fullScreen")) cm.setOption("fullScreen", false) }
function beg_fscrn(cm)  {     cm.setOption("fullScreen", !cm.getOption("fullScreen")) }
function editor_toggle(){ editor.hidden = (editor.hidden == true ? false : true) }
function editor_reset() {
  editor_cpp.setSize(editor_window.offsetWidth/2 - 1, editor_window.offsetHeight - 2)
  editor_h  .setSize(editor_window.offsetWidth/2 - 1, editor_window.offsetHeight - 2)
  editor_cpp.setValue(cc_template)
  editor_h  .setValue(h_template)
}
function editor_load_cpp(e)     { file_import(e, (val) => { if (val != false) editor_cpp.setValue(val) }) }
function editor_load_h(e)       { file_import(e, (val) => { if (val != false) editor_h  .setValue(val) }) }
function editor_load_example()  {
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
  'show/hide editor'    : editor_toggle,
  'compile'             : compile,
  'compile + run'       : compile_run
}
controls_editor.add(controls_editor_params, 'show/hide editor')
controls_editor.add(controls_editor_params, 'compile')
controls_editor.add(controls_editor_params, 'compile + run')
// Controls: simulation settings
var controls_settings   = controls.addFolder("simulation settings")
var controls_settings_params = {
  'road'                : 'bj5',
  'vehicles'            : '5k',
  'vehicle capacity'    : '3',
  'vehicle speed'       : '10 m/sec',
  'customers scale'     : '1.0x (~9.7/sec)',
  'delay tolerance'     : '6 min',
  'match period (sec)'  :  60,
  'runtime mode'        : 'dynamic',
  'routing mode'        : 'merge'
}
controls_road_selector  = controls_settings.add(controls_settings_params, 'road', ['bj5', 'mny', 'cd1', 'bj5_directed', 'mny_directed', 'cd1_directed']).onFinishChange(change_road)
controls_settings.add(controls_settings_params, 'vehicles', ['1k', '5k', '10k', '15k', '20k', '25k', '30k', '35k', '40k', '45k', '50k'])
controls_settings.add(controls_settings_params, 'vehicle capacity', [1, 3, 6, 9])
controls_settings.add(controls_settings_params, 'vehicle speed', ['10 m/sec', '15 m/sec', '20 m/sec', '25 m/sec'])
controls_settings.add(controls_settings_params, 'customers scale', ['0.5x (~4.9/sec)', '1.0x (~9.7/sec)', '2.0x (~19.0/sec)', '4.0x (~36.4/sec)'])
controls_settings.add(controls_settings_params, 'delay tolerance', ['2 min', '4 min', '6 min', '8 min'])
controls_settings.add(controls_settings_params, 'match period (sec)')
controls_settings.add(controls_settings_params, 'routing mode', ['strict', 'merge'])
controls_settings.add(controls_settings_params, 'runtime mode', ['static', 'dynamic'])
// Controls: main
var controls_params     = {
  'show road'           : true,
  'compile + run'       : compile_run,
  'stop simulation'     : stop
}
controls.add(controls_params, 'show road'       ).onFinishChange(toggle_visible_road)
controls.add(controls_params, 'compile + run'   )
controls.add(controls_params, 'stop simulation' )

main_container.appendChild(controls.domElement)

function toggle_visible_road(toggle) { Simulation.road_network.material.visible = toggle }
function problem_str() {
  return "rs-"  + controls_settings_params["road"]
         + "-m" + controls_settings_params["vehicles"]
         + "-c" + controls_settings_params["vehicle capacity"]
         + "-d" + controls_settings_params["delay tolerance"].split(" ")[0]
         + "-s" + controls_settings_params["vehicle speed"  ].split(" ")[0]
         + "-x" + controls_settings_params["customers scale"].split(" ")[0].slice(0, -1)
}
// Visualization ---------------------------------------------------------------
var viz           = document.getElementById("main")
var vertex_vehl   = document.getElementById("vertex_vehl")
var vertex_cust   = document.getElementById("vertex_cust")
var fragment_vehl = document.getElementById("fragment_vehl")
var fragment_cust = document.getElementById("fragment_cust")

// Temporary memory, safe to clear
var workspace = {
  selected_vehicles     : [],
  selected_customers    : [],
  lines                 : [],
  routes                : [],
  highlights            : []
}
var Simulation = {
  // Call init() only ONCE
  init() {
    this.aspect         = viz.offsetWidth/viz.offsetHeight
    this.clock          = new THREE.Clock()
    this.stats          = new Stats()
    this.scene          = new THREE.Scene()
    this.camera         = new THREE.OrthographicCamera(-0.1, 0.1, 0.1/this.aspect, -0.1/this.aspect, 1, 3)
    this.box            = new THREE.Box2()
    this.center         = new THREE.Vector2
    this.renderer       = new THREE.WebGLRenderer({ alpha: true})
    this.controls       = new THREE.MapControls(this.camera, this.renderer.domElement)
    this.raycaster      = new THREE.Raycaster()
    this.mouse          = new THREE.Vector2()

    this.stats          .showPanel(0) // 0: fps, 1: ms, 2: mb, 3+: custom
    this.renderer       .setSize(viz.offsetWidth, viz.offsetHeight)
    this.renderer       .setClearColor (bg_color, 1)
    this.controls       .enableDamping = true
    this.controls       .dampingFactor = 0.25
    this.controls       .enableKeys = false
    this.controls       .enableRotate = false
    this.controls       .screenSpacePanning = true
    this.raycaster      .params.Points.threshold = 0.000100
    viz.appendChild(this.renderer.domElement)
    viz.appendChild(this.stats.dom);

    window.addEventListener('resize', () => {
      this.aspect = viz.offsetWidth/viz.offsetHeight
      this.renderer.setSize(viz.offsetWidth, viz.offsetHeight)
      this.camera.top = 0.1/this.aspect
      this.camera.bottom = -0.1/this.aspect
      this.camera.updateProjectionMatrix()
    }, false)

    // window.addEventListener('click', capture_mouse, false)
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
      vertexShader: vertex_vehl.text,
      fragmentShader: fragment_vehl.text
    }))
    this.vehicle_speed     = Number(controls_settings_params["vehicle speed"].split(" ")[0])
    this.vehicle_position  = this.vehicle_particles.geometry.attributes.position.array
    this.vehicle_target0   = this.vehicle_particles.geometry.attributes.target0.array
    this.vehicle_target1   = this.vehicle_particles.geometry.attributes.target1.array
    this.vehicle_dur0      = this.vehicle_particles.geometry.attributes.dur0.array
    this.vehicle_dur1      = this.vehicle_particles.geometry.attributes.dur1.array
    this.vehicle_t0        = this.vehicle_particles.geometry.attributes.t0.array
    this.vehicle_loads     = this.vehicle_particles.geometry.attributes.load.array
    this.vehicle_sizes     = this.vehicle_particles.geometry.attributes.size.array
    this.scene.add(this.vehicle_particles)

    this.ncust = lu_scales[controls_settings_params["road"]][controls_settings_params["customers scale"].split(" ")[0]]
    this.customer_particles = new THREE.Points(this.customer_geometry(this.ncust), new THREE.ShaderMaterial({
      uniforms: this.uniforms,
      vertexShader: vertex_cust.text,
      fragmentShader: fragment_cust.text
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
    temp_vehicle_geom.addAttribute('size',     new THREE.Float32BufferAttribute(init_ones,  1))
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
  },

  // Return a line geometry for gui line command
  clinev_geometry(id_1, id_2) {
    var temp_geom = new THREE.BufferGeometry()
    var i_cust = 3*(Number(id_1) - this.nvehl - 1)
    var i_vehl = 3*(Number(id_2) - 1)
    var pos = [this.customer_position[i_cust], this.customer_position[i_cust+1], 0,
               this.vehicle_position [i_vehl], this.vehicle_position [i_vehl+1], 0]
    // console.log(`id_1: ${id_1}, i_cust: ${i_cust}, id_2: ${id_2}, i_vehl: ${i_vehl}, pos: ${pos}`)
    temp_geom.addAttribute('position', new THREE.Float32BufferAttribute(pos, 3))
    return temp_geom
  },

  clinec_geometry(id_1, id_2) {
    var temp_geom = new THREE.BufferGeometry()
    var i_cust1 = 3*(Number(id_1) - this.nvehl - 1)
    var i_cust2 = 3*(Number(id_2) - this.nvehl - 1)
    var pos = [this.customer_position[i_cust1], this.customer_position[i_cust1+1], 0,
               this.customer_position[i_cust2], this.customer_position[i_cust2+1], 0]
    console.log(`id_1: ${id_1}, i_cust1: ${i_cust1}, id_2: ${id_2}, i_cust2: ${i_cust2}, pos: ${pos}`)
    temp_geom.addAttribute('position', new THREE.Float32BufferAttribute(pos, 3))
    return temp_geom
  },

  // Return a "route" geometry for gui route command
  route_geometry(data) {
    var temp_geom = new THREE.BufferGeometry()
    var pos = []
    for (var i = 0; i < data.length; i++) {
      pos.push(Simulation.rn_nodes[data[i]][0])
      pos.push(Simulation.rn_nodes[data[i]][1])
      pos.push(0)
    }
    temp_geom.addAttribute('position', new THREE.Float32BufferAttribute(pos, 3))
    return temp_geom
  },

  // Return a highlight (point) geometry for gui hi command
  highlight_geometry(id, is_cust) {
    var temp_geom = new THREE.BufferGeometry()
    var pos = (is_cust ? [this.customer_position[3*(Number(id)-this.nvehl-1)], this.customer_position[3*(Number(id)-this.nvehl-1)+1], 0]
                       : [this.vehicle_position[3*(Number(id)-1)], this.vehicle_position[3*(Number(id)-1)+1], 0])
    temp_geom.addAttribute('position',new THREE.Float32BufferAttribute(pos, 3))
    temp_geom.addAttribute('size',    new THREE.Float32BufferAttribute([3.0], 1))
    return temp_geom
  },
}

function change_road(road) {
  Simulation.box.makeEmpty()
  if (Simulation.road_network) clear1(Simulation.road_network)
  if (road == "bj5" || road == "bj5_directed") {
    Simulation.rn_positions = bj5_positions
    Simulation.rn_nodes     = bj5_nodes
    Simulation.rn_weights   = bj5_weights
  } else if (road == "mny" || road == "mny_directed") {
    Simulation.rn_positions = mny_positions
    Simulation.rn_nodes     = mny_nodes
    Simulation.rn_weights   = mny_weights
  } else if (road == "cd1" || road == "cd1_directed") {
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
  zoomTo(Simulation.center['x'], Simulation.center['y'])
}

// Move and point camera to lng, lat at zoom level
function zoomTo(lng, lat/*, zoom*/) {
  Simulation.camera.position.set(lng, lat, 2)
  Simulation.controls.target = new THREE.Vector3(lng, lat, 0)
}

// Pass Simulation.scene to clear the entire scene
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

// https://threejs.org/docs/index.html#api/en/core/Raycaster
// function capture_mouse(e) {
//   e.preventDefault()
//   Simulation.mouse.x =  (e.clientX/Simulation.renderer.domElement.clientWidth )*2 - 1
//   Simulation.mouse.y = -(e.clientY/Simulation.renderer.domElement.clientHeight)*2 + 1
//   Simulation.raycaster.setFromCamera(Simulation.mouse, Simulation.camera)
// 
//   var intersects = Simulation.raycaster.intersectObject(Simulation.vehicle_particles)
//   if (intersects.length > 0) deselect_all_vehicles()
//   for (var i = 0; i < intersects.length; i++) { get_vehicle_info(intersects[i].index) }
// 
//   intersects = Simulation.raycaster.intersectObject(Simulation.customer_particles)
//   if (intersects.length > 0) deselect_all_customers()
//   for (var i = 0; i < intersects.length; i++) { get_customer_info(intersects[i].index) }
// }
// 
// function deselect_all_vehicles() {
//   for (var i = 0; i < workspace.selected_vehicles.length; i++)
//     Simulation.vehicle_sizes[workspace.selected_vehicles[i]] = 1.0
//   workspace.selected_vehicles = []
//   clear_info()
// }
// 
// function deselect_all_customers() {
//   for (var i = 0; i < workspace.selected_customers.length; i++)
//     Simulation.customer_sizes[workspace.selected_customers[i]] = 3.0
//   workspace.selected_customers = []
//   clear_info()
// }
// 
// function get_vehicle_info(vehl_id) {
//   workspace.selected_vehicles.push(vehl_id)
//   Simulation.vehicle_sizes[vehl_id] = 4  // quadruple the size
//   socket.emit("client cmd get vehicle info", vehl_id+1)  // 0-indexed in Three, but 1-indexed in Cargo
// }
// 
// function get_customer_info(cust_id) {
//   workspace.selected_customers.push(cust_id)
//   Simulation.customer_sizes[cust_id] = 12 // quadruple size (original is 3)
//   socket.emit("client cmd get customer info", cust_id+Simulation.nvehl+1)
// }
// 
// function clear_info() {
//   info_content.innerHTML = ""
// }

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
  Simulation.vehicle_particles.geometry.attributes.size.needsUpdate = true
  Simulation.customer_particles.geometry.attributes.position.needsUpdate = true
  Simulation.customer_particles.geometry.attributes.size.needsUpdate = true
  Simulation.uniforms.t.value = Simulation.clock.getElapsedTime()
  Simulation.uniforms.zoom.value = Simulation.camera.zoom
  // Simulation.road_network.material.linewidth = rn_linewidth*Simulation.camera.zoom
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
socket.on("server msg t"        , (data) => { log_alg_flush() })
socket.on("server msg alg"      , (data) => { log_alg_append(data) })
socket.on("server msg vehicle"  , (data) => {
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
// socket.on('server msg vehicle info' , (data) => {
//   info_content.innerHTML = ""
//   info_content.innerHTML += "ID: "+data["vehl_id"]
//   info_content.innerHTML += "Load: "+Simulation.vehicle_loads[data["vehl_id"]]+"<br/>"
//   info_content.innerHTML += "Origin: "+data["origin"]+"<br/>"
//   info_content.innerHTML += "Destination: "+data["destination"]+"<br/>"
//   info_content.innerHTML += "Capacity: "+data["capacity"]+"<br/>"
//   info_content.innerHTML += "Matches: "+data["matches"]+"<br/>"
// })
// socket.on('server msg customer info' , (data) => {
//   info_content.innerHTML = ""
//   info_content.innerHTML += "ID: "+data["cust_id"]
//   info_content.innerHTML += "Origin: "+data["origin"]+"<br/>"
//   info_content.innerHTML += "Destination: "+data["destination"]+"<br/>"
//   info_content.innerHTML += "Early: "+data["early"]+"<br/>"
//   info_content.innerHTML += "Late: "+data["late"]+"<br/>"
//   info_content.innerHTML += "Load: "+data["load"]+"<br/>"
//   info_content.innerHTML += "AssignedTo: "+data["assignedTo"]+"<br/>"
// })
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

socket.on('server cmd gui clinev', (data) => {
  // console.log(data)
  var geometry = Simulation.clinev_geometry(data[0], data[1])
  var material = new THREE.LineDashedMaterial({
    color    : line_color0,
    linewidth: 6,
    dashSize : 0.00006,
    gapSize  : 0.00006 })
  var line = new THREE.Line(geometry, material)
  line.computeLineDistances()
  Simulation.scene.add(line)
  workspace.lines.push(line)
})

socket.on('server cmd gui clinec', (data) => {
  // console.log(data)
  console.log("Got clinec");
  var geometry = Simulation.clinec_geometry(data[0], data[1])
  var material = new THREE.LineDashedMaterial({
    color    : line_color3,
    linewidth: 6,
    dashSize : 0.00006,
    gapSize  : 0.00006 })
  var line = new THREE.Line(geometry, material)
  line.computeLineDistances()
  Simulation.scene.add(line)
  workspace.lines.push(line)
})

socket.on('server cmd gui route', (data) => {
  // console.log(data)
  var geometry = Simulation.route_geometry(data[1])
  var material = new THREE.LineDashedMaterial({
    color    : (data[0] == true ? line_color1 : line_color2),
    linewidth: 6,
    dashSize : (data[0] == true ? 0 : 0.00040),
    gapSize  : (data[0] == true ? 0 : 0.00040) })
  var route = new THREE.Line(geometry, material)
  if (data[0] == false) route.computeLineDistances()
  Simulation.scene.add(route)
  workspace.routes.push(route)
})

socket.on('server cmd gui hi', (data) => {
  // console.log(data)
  var geometry = Simulation.highlight_geometry(data[0], data[1])
  var point = new THREE.Points(geometry, new THREE.ShaderMaterial({
      uniforms: Simulation.uniforms,
      vertexShader: vertex_cust.text,
      fragmentShader: fragment_hi.text
    }))
  //var material = new THREE.PointsMaterial({ color: hi_color, size: 6.0 });
  //var point = new THREE.Points(geometry, material)
  Simulation.scene.add(point)
  workspace.highlights.push(point)
})

socket.on('server cmd gui center', (data) => {
  // console.log(data)
  zoomTo(Simulation.rn_nodes[data][0], Simulation.rn_nodes[data][1])
})

socket.on('server cmd gui reset', () => {
  for (var i = 0; i < workspace.lines.length; i++) Simulation.scene.remove(workspace.lines[i])
  for (var i = 0; i < workspace.routes.length; i++) Simulation.scene.remove(workspace.routes[i])
  for (var i = 0; i < workspace.highlights.length; i++) Simulation.scene.remove(workspace.highlights[i])
  workspace.lines      = []
  workspace.routes     = []
  workspace.highlights = []
})

// socket.on('server cmd enable resume', toggle_resume)

reset_feeds()
// toggle_resume()

// function toggle_resume(){ resume.hidden = (resume.hidden == true ? false : true) }

// resume_btn.addEventListener('click', () => {
//   socket.emit("client cmd resume")
//   toggle_resume
// })

function reset_feeds() {
  cargofeed.value = "cargo\n"
  algfeed.value   = "rsalgorithm\n"
}

function log_cargo_append(data) { cargofeed.value += data; cargofeed.scrollTop = cargofeed.scrollHeight }
function log_alg_append(data) { algfeed.value += data; algfeed.scrollTop = algfeed.scrollHeight }
function log_alg_flush() { algfeed.value = ""; algfeed.scrollTop = algfeed.scrollHeight }

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
    footer("Running...")
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
document.getElementById("loader").style.display = "none"

