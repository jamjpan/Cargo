/** Dec 19, 2018 **/
const PORT           = 3000
const PROB_PATH      = "/home/jpan/Projects/PhD/Datasets/3-Problem-Instances/"
const ROAD_PATH      = "/home/jpan/Projects/PhD/Datasets/1-Road-Networks/"
const LIBCARGO_PATH  = "/home/jpan/Projects/PhD/Software/Cargo/lib"
const INCLUDE_PATH   = "/home/jpan/Projects/PhD/Software/Cargo/include"
const LIBMETIS_PATH  = "/usr/local/lib"
// -----------------------------------------------------------------------------
const express        = require('express')
const cargo          = express()
const http           = require('http').Server(cargo)
const io             = require('socket.io')(http)
const spawnSync      = require('child_process').spawnSync
const spawn          = require('child_process').spawn
const fs             = require('fs')
const readline       = require('readline')
const stripAnsi      = require('strip-ansi')
const Tail           = require('tail').Tail
// State store (maybe it would be nice if we could connect to the SQLite db
// while it's running instead of using .dat file?)
const Simulation = {
  reset() {
    this.process     = null   // store cargo running process (kill on stop)
    this.feed        = null   // store cargoweb.feed         (kill on stop)
    this.dat         = null   // store cargoweb.dat          (kill on stop)
    this.running     = false  // (false on stop)
    this.initialized = false  // (TODO is this flag necessary?)
    this.vehicles    = {}     // (load on problem) k: vehl_id; v: vehicle {}
    this.customers   = {}     // (load on problem) k: cust_id; v: customer {}
    this.nvehl       =  0     // (load on problem)
    this.ncust       =  0     // (load on problem)
    this.appearances = {}     // (load on problem) k: early;   v: [customer]
    this.t           = -1     // (update on dat line)
    this.routes      = {}     // (update on dat R) k: vehl_id; v: [node_id]
    this.route_index = {}     // (update on dat V) k: vehl_id; v: int
    this.matches     = {}     // (update on dat M) k: cust_id; v: vehl_id
  },

  load_problem(prob) {
    fs.readFile(`${PROB_PATH}${prob}.instance`, { encoding: "utf-8" }, (error, data) => {
      var line = data.split('\n')
      for (var i = 6; i < line.length; i++) {  // skip 6 lines of header
        var col = line[i].split('\t')
        var id = Number(col[0])
        var origin = Number(col[1])
        var destination = Number(col[2])
        var load = Number(col[3])
        var early = Number(col[4])
        var late = Number(col[5])
        if (load > 0) {  // got a customer
          if (early in Simulation.appearances)
            Simulation.appearances[early].push([id, origin])
          else
            Simulation.appearances[early] = [[id, origin]]
          Simulation.customers[id] = {
            "origin"      : origin,
            "destination" : destination,
            "early"       : early,
            "late"        : late,
            "load"        : load,
            "assignedTo"  : null
          }
          // console.log(id, Simulation.customers[id])
          Simulation.ncust++
        } else {  // got a vehicle
          Simulation.vehicles[id] = {
            "origin"      : origin,
            "destination" : destination,
            "capacity"    : load,
            "matches"     : []
          }
          Simulation.nvehl++
        }
      }
    })
  }
}
cargo.use('/static'                      , express.static(__dirname + '/public'))
cargo.use('/static/css/bootstrap.min.css', express.static(__dirname + '/node_modules/bootstrap/dist/css/bootstrap.min.css'))
cargo.get('/', (req, res) => res.sendFile(__dirname + '/public/index.html'))

// Socket events
io.on('connection', (socket) => {
  console.log('a user connected!')

  socket.on('disconnect', () => { console.log('user disconnected') })

  socket.on('client query initialize', (data) => {
    if (!Simulation.initialized) {
      console.log("Resetting the server")
      Simulation.reset()
      Simulation.load_problem(data)
      Simulation.initialized = true
      emit_init(io)
    } else
      emit_init(io)
  })

  socket.on('client query running', () => {
    socket.emit('server msg running', Simulation.running) })

  socket.on('client cmd compile_h', (data) => {
    try {
      fs.writeFileSync('cargoweb.cc', data)
    } catch (err) {
      socket.emit('server error', err)
      console.log(err)
    }
  })

  socket.on('client cmd compile_cc', (data) => {
    try {
      fs.appendFileSync('cargoweb.cc', data)
    } catch (err) {
      socket.emit('server error', err)
      console.log(err)
    }
  })

  socket.on('client msg compile_opts', (data) => {
    // console.log(data)
    Simulation.reset()
    Simulation.load_problem(data.problem)
    var static_mode = (data.mode == 'dynamic' ? "false" : "true")
    var routing_mode = (data.routing == 'merge' ? "false" : "true")
    try {
      fs.appendFileSync('cargoweb.cc',
        `//
        int main() {
          Options option;
          option.path_to_roadnet    = "${ROAD_PATH}${data.road}.rnet";
          option.path_to_problem    = "${PROB_PATH}${data.problem}.instance";
          option.matching_period    = ${data.mat};
          option.strict_mode        = ${routing_mode};
          option.static_mode        = ${static_mode};
          Cargo cargo(option);
          CargoWeb cw;
          cargo.start(cw);
        }
        `)
      const compile = spawnSync('g++', [
          'cargoweb.cc',
          '-std=c++11',
          '-O3',
          '-Wall',
          '-L'+LIBCARGO_PATH, '-lcargo',
          '-L'+LIBMETIS_PATH, '-lmetis',
          '-pthread',
          '-lrt',
          '-ldl',
          '-l:libglpk.a',
          '-I'+INCLUDE_PATH,
          '-Iglpk',
          '-o', 'cargoweb'], {
      })
      /*socket*/io.emit('server msg', stripAnsi(compile.stdout.toString()))
      /*socket*/io.emit('server msg', stripAnsi(compile.stderr.toString()))
      /*socket*/io.emit('server msg', "done\n")
    } catch (err) {
      socket.emit('server error', err.message)
      console.log(err.message)
    }
  })

  socket.on('client cmd run', () => {
    try {
      fs.unlinkSync('cargoweb.feed')
    } catch (err) {}
    // TODO Allow use to manually launch the process from terminal
    Simulation.process = spawn('./cargoweb', [], { detached: false  })
    Simulation.running = true,
    /*socket*/io.emit('server msg running', Simulation.running)
    var rl = readline.createInterface({
      input: Simulation.process.stdout
    })
    rl.on('line', parse_cargo_log)
    // Simulation.process.stdout.on('data', parse_cargo_log)
    // Simulation.process.stderr.on('data', (data) => { /*socket*/io.emit('server msg', stripAnsi(data.toString())) })
    Simulation.process.on('error', (data) => {
      /*socket*/io.emit('server msg', stripAnsi(data.toString()))
      close(io)
    })
    Simulation.process.on('close', (data) => {
      /*socket*/io.emit('server msg', "Closed\n")
      close(io)
    })
    const watcher = fs.watch('.', { persistent: false }, (eventType, filename) => {
      if (eventType == 'rename' && filename == 'cargoweb.feed') {
        watcher.close()
        emit_dat(io)
      }
    })
  })

  socket.on('client cmd stop', () => { close(io) })

  socket.on('client cmd load example', (data) => {
    var cc = fs.readFileSync("public/example/"+data+".cc", { "encoding": "utf-8" })
    var h  = fs.readFileSync("public/example/"+data+".h" , { "encoding": "utf-8" })
    io.emit('server msg load_example', [cc, h])
  })

  socket.on('client cmd get vehicle info', (vehl_id) => {
    console.log("get vehicle info", vehl_id)
    console.log(Simulation.vehicles[vehl_id])
    io.emit('server msg vehicle info', {
      "vehl_id"     : vehl_id-1,
      "origin"      : Simulation.vehicles[vehl_id]["origin"],
      "destination" : Simulation.vehicles[vehl_id]["destination"],
      "capacity"    : Simulation.vehicles[vehl_id]["capacity"],
      "matches"     : JSON.stringify(Simulation.vehicles[vehl_id]["matches"])
    })
  })

  socket.on('client cmd get customer info', (cust_id) => {
    console.log("get customer info", cust_id)
    console.log(Simulation.customers[cust_id])
    io.emit('server msg customer info', {
      "cust_id"     : cust_id-1,
      "origin"      : Simulation.customers[cust_id]["origin"],
      "destination" : Simulation.customers[cust_id]["destination"],
      "load"        : Simulation.customers[cust_id]["load"],
      "early"       : Simulation.customers[cust_id]["early"],
      "late"        : Simulation.customers[cust_id]["late"],
      "assignedTo"  : Simulation.customers[cust_id]["assignedTo"]
    })
  })

  // Throws EPIPE for some reason
  // socket.on('client cmd resume', () => {
  //   // Simulation.process.stdin.write("hello")
  //   // Simulation.process.stdin.end()
  //   process.stdin.write("hello\n")
  // })

})

function parse_cargo_log(line) {
  if (line.length > 0) {
    line = stripAnsi(line)
           if (line.lastIndexOf("gui clinev"    , 0) == 0) { io.emit('server cmd gui clinev', line.slice(11).split(' '))
    } else if (line.lastIndexOf("gui clinec"    , 0) == 0) { io.emit('server cmd gui clinec', line.slice(11).split(' '))
    } else if (line.lastIndexOf("gui hi cust"   , 0) == 0) { io.emit('server cmd gui hi'    , [line.slice(12), true])
    } else if (line.lastIndexOf("gui hi vehl"   , 0) == 0) { io.emit('server cmd gui hi'    , [line.slice(12), false])
    } else if (line.lastIndexOf("gui center"    , 0) == 0) { io.emit('server cmd gui center', line.slice(11))
    } else if (line.lastIndexOf("gui route cur" , 0) == 0) { io.emit('server cmd gui route' , [true, line.slice(14).split(' ')])
    } else if (line.lastIndexOf("gui route new" , 0) == 0) { io.emit('server cmd gui route' , [false, line.slice(14).split(' ')])
    } else if (line.lastIndexOf("gui reset"     , 0) == 0) { io.emit('server cmd gui reset')
    } else {
      console.log(line)
      io.emit('server msg', line+'\n')
    }
  }
  // }
}

function emit_init(io) {
  io.emit('server msg initialize', {
    nvehl: Simulation.nvehl,
    ncust: Simulation.ncust
  })
}

function emit_dat(io) {
  io.emit('server msg started')

  fs.writeFileSync('cargoweb.dat', "")  // clear any existing contents and ensure file exists
  Simulation.dat = new Tail('cargoweb.dat', { fromBeginning: true, follow: false })
  Simulation.dat.on('line', (line) => {
    var col = line.toString().split(' ')
    var t = Number(col[0])
    if (Simulation.t != t) {
      if (t in Simulation.appearances)
        io.emit('server msg customer', Simulation.appearances[t])
      Simulation.t = t
      io.emit('server msg t', t)
    }
    // console.log(line)
    if (col[1] == "R") {
      var vid = col[2]
      var route = col.slice(3, col.length)
      Simulation.route_index[vid] = 2  // reset the current location index
      if (vid in Simulation.routes) {
        Simulation.routes[vid] = route
        // console.log('emit update route', route.slice(0, 3))
        io.emit('server msg update route', { vid: vid, route: route.slice(0, 3) })
      } else {
        Simulation.routes[vid] = route
        // console.log('emit begin route', route.slice(0, 3))
        io.emit('server msg begin route', { vid: vid, route: route.slice(0, 3) })
      }
    } else if (col[1] == "V") {
      var vid  = col[2]
      var loc0 = col[3]
      var nnd  = col[4]
      if (vid in Simulation.routes) {
        if (loc0 == Simulation.routes[vid][Simulation.route_index[vid]-1]) {
          var loc1 = Simulation.routes[vid][Simulation.route_index[vid]]
          var loc2 = Simulation.routes[vid][++Simulation.route_index[vid]]
          // console.log('emit vehicle', vid)
          io.emit('server msg vehicle', {
            vid: vid, position: loc0, target0: loc1, target1: loc2, overshoot: nnd})
        }
      }
    } else if (col[1] == "P") {
      // console.log('emit pickup', col.slice(2, col.length))
      io.emit('server msg pickup', col.slice(2, col.length))
    // } else if (col[1] == "D") {
    //   io.emit('server msg dropoff', col.slice(2, col.length))
    // }
    } else if (col[1] == "L") {
      // console.log('emit load', col.slice(2, col.length))
      io.emit('server msg load', col.slice(2, col.length))
    } else if (col[1] == "T") {
      // console.log('emit timeout', col.slice(2, col.length))
      io.emit('server msg customer_timeout', col.slice(2, col.length))
    } else if (col[1] == "M") {
      var vid = col[2]
      var custs = col.slice(3, col.length)
      for (var i = 0; i < custs.length; i++) {
        var cid = Number(custs[i])
        if (Number(custs[i]) < 0) {
          Simulation.matches[-cid] = null
          Simulation.vehicles[vid]["matches"] = Simulation.vehicles[vid]["matches"].filter(elem => elem != cid)
        }
        else {
          Simulation.matches[cid] = vid
          Simulation.vehicles[vid]["matches"].push(cid)
        }
      }
    }
  })

  Simulation.feed = fs.createReadStream('cargoweb.feed', { autoClose: false })
  Simulation.feed.on('data', (data) => { /*socket*/io.emit('server msg alg', stripAnsi(data.toString())) })
}

function close(io) {
  console.log("Closing the server")
  if (Simulation.process) Simulation.process.kill()
  Simulation.running = false
  if (Simulation.feed) {
    Simulation.feed.destroy()
    Simulation.feed = null
  }
  if (Simulation.dat) {
    Simulation.dat.unwatch()
    Simulation.dat = null
  }
  Simulation.initialized = false
  /*socket*/io.emit('server msg running', Simulation.running)
}

http.listen(PORT, () => { console.log(`Cargo listening on port ${PORT}!`) })

