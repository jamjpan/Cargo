/** Dec 19, 2018 **/
const express = require('express')
const cargo = express()
const http = require('http').Server(cargo)
const port = 3000
const io = require('socket.io')(http)
const spawnSync = require('child_process').spawnSync
const spawn = require('child_process').spawn
const fs = require('fs')
const readline = require('readline')
const stripAnsi = require('strip-ansi')
const Tail = require('tail').Tail

// Set static files + Bootstrap
cargo.use('/static', express.static(__dirname + '/public'))
cargo.use('/static/css/bootstrap.min.css', express.static(__dirname + '/node_modules/bootstrap/dist/css/bootstrap.min.css'))
cargo.use('/static/css/bootstrap.min.css', express.static(__dirname + '/node_modules/bootstrap/dist/css/bootstrap.min.css'))

// Main routes
cargo.get('/', (req, res) => res.send('Hello world!'))

// State store
var Simulation = {
  reset() {
    this.process = null
    this.feed = null
    this.dat = null
    this.running = false
    this.initialized = false
    this.vehicles = {}
    this.routes = {}
    this.route_index = {}
    this.customers = {}
    this.nvehl = 0
    this.ncust = 0
    this.t = -1
  }
}

// Socket events
io.on('connection', (socket) => {
  console.log('a user connected!')

  socket.on('disconnect', () => { console.log('user disconnected') })

  // Called whenever a new client connects so as to find nvehl and ncust,
  // passed to client's own initialization procedure
  socket.on('client query initialize', (data) => {
    if (!Simulation.initialized) {
      console.log("Resetting the server")
      Simulation.reset()
      const rl = readline.createInterface({
        input: fs.createReadStream(`../data/benchmark/${data}.instance`),
        crlfDelay: Infinity
      })
      rl.on('line', (line) => {
        col = line.split('\t')
        if (Number(col[0])) {
          if (Number(col[3]) > 0) {
            var early = Number(col[4])
            var id = Number(col[0])
            var origin = col[1]
            if (early in Simulation.customers)
              Simulation.customers[early].push([id, origin])
            else
              Simulation.customers[early] = [[id, origin]]
            Simulation.ncust++
          } else {
            Simulation.nvehl++
          }
        }
      })
      Simulation.initialized = true
      rl.on('close', () => { emit_init(io) })
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
    var static_mode = (data.mode == 'dynamic' ? "false" : "true")
    try {
      fs.appendFileSync('cargoweb.cc',
        `//
        int main() {
          Options option;
          option.path_to_roadnet="../data/roadnetwork/${data.road}.rnet";
          option.path_to_gtree="../data/roadnetwork/${data.road}.gtree";
          option.path_to_edges="../data/roadnetwork/${data.road}.edges";
          option.path_to_problem="../data/benchmark/${data.problem}.instance";
          option.path_to_solution="cargoweb.sol";
          option.path_to_dataout="cargoweb.dat";
          option.time_multiplier=1;
          option.vehicle_speed=${data.speed};
          option.matching_period=${data.mat};
          option.strict_mode=false;
          option.static_mode=${static_mode};
          Cargo cargo(option);
          CargoWeb cw;
          cargo.start(cw);
          return 0;
        }
        `)
      const compile = spawnSync('g++', [
          'cargoweb.cc',
          '-std=c++11',
          '-O3',
          '-Wall',
          '-L../lib', '-lcargo',
          '-L/usr/local/lib', '-lmetis',
          '-pthread',
          '-lrt',
          '-ldl',
          '-I../include',
          '-o', 'cargoweb'], {
      })
      /*socket*/io.emit('server msg compile', stripAnsi(compile.stdout.toString()))
      /*socket*/io.emit('server msg compile', stripAnsi(compile.stderr.toString()))
      /*socket*/io.emit('server msg', "done")
    } catch (err) {
      socket.emit('server error', err.message)
      console.log(err.message)
    }
  })

  socket.on('client cmd run', () => {
    try {
      fs.unlinkSync('cargoweb.feed')
    } catch (err) {}
    Simulation.process = spawn('./cargoweb', [], { detached: false })
    Simulation.running = true
    /*socket*/io.emit('server msg running', Simulation.running)
    Simulation.process.stdout.on('data', (data) => { /*socket*/io.emit('server msg', stripAnsi(data.toString())) })
    Simulation.process.stderr.on('data', (data) => { /*socket*/io.emit('server msg', stripAnsi(data.toString())) })
    Simulation.process.on('error', (data) => {
      /*socket*/io.emit('server msg', stripAnsi(data.toString()))
      close(io)
    })
    Simulation.process.on('close', (data) => {
      /*socket*/io.emit('server msg', "Closed")
      close(io)
    })
    Simulation.vehicles = {}
    const watcher = fs.watch('.', { persistent: false }, (eventType, filename) => {
      if (eventType == 'rename' && filename == 'cargoweb.feed') {
        watcher.close()
        emit_dat(io)
      }
    })
  })

  socket.on('client cmd stop', () => { close(io) })
})

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
      if (t in Simulation.customers) {
        // console.log('emit customer', Simulation.customers[t])
        io.emit('server msg customer', Simulation.customers[t])
      }
      Simulation.t = t
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

// Start the server
http.listen(port, () => { console.log(`Cargo listening on port ${port}!`) })

