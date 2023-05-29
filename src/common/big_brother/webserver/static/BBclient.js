/* BigBrother client side code to communicate with webserver */

// socketIO
var socket = io.connect('http://' + document.domain + ':' + location.port + '/web', {
	reconnection: false
});

// ROS info elements
const car_status = document.getElementById("car-status")
const nodes = document.getElementById("nodes");
const topics = document.getElementById("topics");

// AS info elements
const blue_cones = document.getElementById("blue-cones");
const yellow_cones = document.getElementById("yellow-cones");
const orange_cones = document.getElementById("orange-cones");
const unknown_cones = document.getElementById("unknown-cones");
const throttle = document.getElementById("throttle");
const steering = document.getElementById("steering");
const control_on = document.getElementById("control-on");
const require_braking = document.getElementById("require-braking");
const mission = document.getElementById("mission");
const mission_finished = document.getElementById("finished");
const res_emergency = document.getElementById("res-emergency");
const res_go = document.getElementById("res-go");
const lap_counter = document.getElementById("lap-counter");
const sta_pos = document.getElementById("sta-pos");
const delta_sta = document.getElementById("delta-sta");
const delta_controller = document.getElementById("delta-controller");
const delta_demands = document.getElementById("delta-demands");
const slam_loop = document.getElementById("slam-loop");
const p2f_distance = document.getElementById("p2f-distance");
const p2f_speed = document.getElementById("p2f-speed");
const vel_x = document.getElementById("vel-x");
const vel_y = document.getElementById("vel-y");

// image element
// const image = document.getElementById("image");

// slam map element
const map = document.getElementById("map");

// log elements
// const log_box = document.getElementById("log-box");
// const log_table = document.getElementById("log-table");
// const log_text = document.getElementById("log-text");
// const log_entry = document.getElementById("log-entry");
// const log_tag = document.getElementById("log-tag");
// const log_button = document.getElementById("log-button");
// const slack_checkbox = document.getElementById("slack-checkbox");

// rosbag elements
// const rosbag_text = document.getElementById("rosbag-topics-text");
// const record_button = document.getElementById("record-button");
// const stop_button = document.getElementById("stop-button");

// event elements
const event_box = document.getElementById("event-box");
const event_table = document.getElementById("event-table");

// variables
var num_nodes = 0				// number of nodes
var num_topics = 0				// number of topics
var entry_num = 0				// log sequence number
 var config_done = false         // config flag
// stop_button.disabled = true     // disable rosbag stop button
// var rosbag_topics_num = 0		// number of rosbag topics in config file
// var refresh = true				// webpage refresh flag for rosbag recording tool


/* ------------------------ Initial config info ------------------------ */

socket.on('config', (msg) => {

	if (config_done || msg.nodes.length == 0 || msg.topics.length == 0) {
		return
	}

	num_nodes = msg.nodes.length-1	// dont count with rosrecord
	num_topics = msg.topics.length

	// config ROS nodes
	for (let i = 0; i < num_nodes; i++) {
		nodes.innerHTML += "<span class=\"dot-grey\" id=\"node" + i + "\"></span>&ensp;" + msg.nodes[i] + "<br>"
	}

	// config ROS topics
	for (let i = 0; i < num_topics; i++) {
		topics.innerHTML += "<span class=\"grey\" id=\"topic" + i + "\">0</span>&ensp;" + msg.topics[i] + "<br>"
	}

	// load log database
	// for (let i = 0; i < msg.log.length; i++) {
	// 	log_table.innerHTML += "<tr><td><span class=\"" + msg.log[i][1] + "\"><b>" + msg.log[i][1] + "</b></span></td>"
	// 						+ "<td>" + msg.log[i][0] + "</td>"
	// 						+ "<td>" + msg.log[i][2] + "</td>"
	// 						+ "<td><i class=\"bi bi-pencil-square\"></i></td>"
	// 						+ "<td><i class=\"bi bi-x-circle\"></i></td></tr>"
	// }
	// entry_num = msg.log.length
	// log_box.scrollTop = log_box.scrollHeight

	// load event database
	for (let i = 0; i < msg.events.length; i++) {
		event_table.innerHTML += "<tr><td>" + msg.events[i][0] + "</td><td>" + msg.events[i][1] + "</td></tr>";
	}
	event_box.scrollTop = event_box.scrollHeight

	// config topics to record
	// for (let i = 0; i < msg.record.length; i++) {
	// 	rosbag_text.innerHTML += '<li class="list-group-item rosbag-topic-item">\
	// 								<div class="custom-control custom-checkbox">\
	// 									<input class="custom-control-input" id="check' + i + '" type="checkbox" checked>\
	// 									<label class="cursor-pointer font-italic d-block custom-control-label" id="check-label' + i + '" for="check' + i + '">' + msg.record[i] + '</label>\
	// 								</div>\
	// 							  </li>'
	// }
	// rosbag_topics_num = msg.record.length

	config_done = true
});


/* ------------- Info message (ROS info + AS info + image) ------------- */

socket.on('info', (msg) => {

	if (!config_done) return

	var rosInfo = msg.rosInfo
	var asInfo = msg.asInfo

	// update car status
	if (msg.carConnected) {
		car_status.innerHTML = "<b>Car online</b>"
		car_status.className = "online"
	}
	else {
		car_status.innerHTML = "<b>Car offline</b>"
		car_status.className = "red"
	}

	// clear info if car disconnected
	if (!msg.carConnected || rosInfo.length == 0 || asInfo.length == 0) {
		clearInfo()
		return
	}


	// --------------- parse ROS info ---------------

	let i = 0

	// ROS nodes
	for (i = 0; i < rosInfo[0].length-1; i++) {
		if (rosInfo[0][i]) document.getElementById("node" + i).className = "dot-green"
		else document.getElementById("node" + i).className = "dot-grey"
	}

	// rosbag recording (record node)
	// if (rosInfo[0][i]) {
	// 	document.getElementById("rosbag-recording").className = "dot-green"
	// 	if (refresh) {
	// 		record_button.disabled = true
	// 		stop_button.disabled = false
	// 	}
	// }
	// else document.getElementById("rosbag-recording").className = "dot-grey"

	// ROS topics
	for (i = 0; i < rosInfo[1].length; i++) {
		document.getElementById("topic" + i).innerHTML = rosInfo[1][i]
		if (rosInfo[1][i] > 0) document.getElementById("topic" + i).className = "green"
		else document.getElementById("topic" + i).className = "grey"
	}


	// --------------- parse AS info ---------------

	// blue cones
	blue_cones.innerHTML = asInfo[0]
	blue_cones.className = "green"

	// yellow cones
	yellow_cones.innerHTML = asInfo[1]
	yellow_cones.className = "green"

	// orange cones
	orange_cones.innerHTML = asInfo[2] + asInfo[3]
	orange_cones.className = "green"

	// unknown cones
	unknown_cones.innerHTML = asInfo[4]
	unknown_cones.className = "green"

	// throttle
	throttle.innerHTML = asInfo[5]
	width = ((asInfo[5] + 1) / 2) * 100
	if (width > 100) width = 100
	throttle.style.width = width + "%"
	throttle.classList.remove(['bar-grey'])
	throttle.classList.remove(['bar-red'])
	throttle.classList.add(['bar-green'])
	if (asInfo[5] == -1) {
		throttle.classList.remove(['bar-green'])
		throttle.classList.add(['bar-red'])
		throttle.style.width = "100%"
	}

	// steering
	steering.innerHTML = asInfo[6]
	width = 100 - (((asInfo[6] + 1) / 2) * 100)
	steering.style.width = width + "%"
	steering.classList.remove(['bar-grey'])
	steering.classList.add(['bar-green'])

	// control on
	if (asInfo[7]) control_on.className = "dot-green"
	else control_on.className = "dot-grey"
	
	// require braking
	if (asInfo[8]) require_braking.className = "dot-green"
	else require_braking.className = "dot-grey"
	
	// mission
	mission.innerHTML = asInfo[9]
	mission.className = "green"

	// mission finished
	if (asInfo[10]) mission_finished.className = "dot-green"
	else mission_finished.className = "dot-grey"

	// lap counter
	lap_counter.innerHTML = asInfo[11]
	lap_counter.className = "green"

	// res emergency
	if (asInfo[12]) res_emergency.className = "dot-green"
	else res_emergency.className = "dot-grey"
	
	// res go
	if (asInfo[13]) res_go.className = "dot-green"
	else res_go.className = "dot-grey"

	// STA position actual
	sta_pos.innerHTML = asInfo[14]
	sta_pos.className = "green"

	// delta STA
	delta_sta.innerHTML = asInfo[15]
	delta_sta.className = "green"

	// delta controller
	delta_controller.innerHTML = asInfo[16]
	delta_controller.className = "green"

	// delta demands
	delta_demands.innerHTML = asInfo[17]
	delta_demands.className = "green"

	// SLAM loop closure
	if (asInfo[18]) slam_loop.className = "dot-green"
	else slam_loop.className = "dot-grey"

	// p2f distance
	p2f_distance.innerHTML = asInfo[19]
	p2f_distance.className = "green"

	// p2f speed
	p2f_speed.innerHTML = asInfo[20]
	p2f_speed.className = "green"

	// velocity X
	vel_x.innerHTML = asInfo[21]
	vel_x.className = "green"
	
	// velocity Y
	vel_y.innerHTML = asInfo[22]
	vel_y.className = "green"


	// --------------- image ---------------

	// load image
	// if (msg.image != null) {
	// 	image.style.visibility = 'visible'
	// 	image.src = msg.image
	// }

	// --------------- slam map ---------------
	
	if (msg.slamMap != null) {
		map.style.visibility = 'visible'
		map.src = msg.slamMap
	}

});


function clearInfo() {

	// --------------- clear ROS info ---------------

	let i = 0

	// ROS nodes
	for (i = 0; i < num_nodes; i++) {
		document.getElementById("node" + i).className = "dot-grey"
	}

	// rosbag recording (record node)
	// document.getElementById("rosbag-recording").className = "dot-grey"

	// ROS topics
	for (i = 0; i < num_topics; i++) {
		document.getElementById("topic" + i).innerHTML = "0"
		document.getElementById("topic" + i).className = "grey"
	}


	// --------------- clear AS info ---------------

	// blue cones
	blue_cones.innerHTML = 0
	blue_cones.className = "grey"

	// yellow cones
	yellow_cones.innerHTML = 0
	yellow_cones.className = "grey"

	// orange cones
	orange_cones.innerHTML = 0
	orange_cones.className = "grey"

	// unknown cones
	unknown_cones.innerHTML = 0
	unknown_cones.className = "grey"

	// throttle
	throttle.innerHTML = 0
	throttle.style.width = "50%"
	throttle.classList.remove(['bar-red'])
	throttle.classList.remove(['bar-green'])
	throttle.classList.add(['bar-grey'])

	// steering
	steering.innerHTML = 0
	steering.style.width = "50%"
	steering.classList.remove(['bar-green'])
	steering.classList.add(['bar-grey'])

	// control on
	control_on.className = "dot-grey"
	
	// require braking
	require_braking.className = "dot-grey"
	
	// mission
	mission.innerHTML = 0
	mission.className = "grey"

	// mission finished
	mission_finished.className = "dot-grey"

	// lap counter
	lap_counter.innerHTML = 0
	lap_counter.className = "grey"

	// res emergency
	res_emergency.className = "dot-grey"
	
	// res go
	res_go.className = "dot-grey"

	// STA position actual
	sta_pos.innerHTML = 0
	sta_pos.className = "grey"

	// delta STA
	delta_sta.innerHTML = 0
	delta_sta.className = "grey"

	// delta controller
	delta_controller.innerHTML = 0
	delta_controller.className = "grey"

	// delta demands
	delta_demands.innerHTML = 0
	delta_demands.className = "grey"

	// SLAM loop closure
	slam_loop.className = "dot-grey"

	// p2f distance
	p2f_distance.innerHTML = 0
	p2f_distance.className = "grey"

	// p2f speed
	p2f_speed.innerHTML = 0
	p2f_speed.className = "grey"

	// velocity X
	vel_x.innerHTML = 0
	vel_x.className = "grey"

	// velocity Y
	vel_y.innerHTML = 0
	vel_y.className = "grey"


	// --------------- clear image ---------------

	// image.style.visibility = 'hidden'

	// ---------------- clear map ----------------

	map.style.visibility = 'hidden'
}


/* ------------------------------ Logging ------------------------------ */

// log function
// function log(time, tag, entry, slack, send) {

// 	// write entry on page
// 	log_table.innerHTML += "<tr><td><span class=\"" + tag + "\"><b>" + tag + "</b></span></td>"
// 						+ "<td>" + time + "</td>"
// 						+ "<td>" + entry + "</td>"
// 						+ "<td><i class=\"bi bi-pencil-square\"></i></td>"
// 						+ "<td><i class=\"bi bi-x-circle\"></i></td></tr>"

// 	// scroll down
// 	log_box.scrollTop = log_box.scrollHeight

// 	// send entry to server
// 	if (send)
// 		socket.emit('new_log', {"entry_num": entry_num, "time": time, "tag": tag, "data": entry, "slack": slack});

// 	// update entry num
// 	entry_num = entry_num + 1
// }


// // when a new entry is submitted
// log_button.addEventListener("click", function() {

// 	// check empty log
// 	if (log_entry.value == "") return

// 	// log entry and send to server
// 	log(currentTime(), log_tag.value, log_entry.value, slack_checkbox.checked, true)

// 	// clear input
// 	log_entry.value = ""
// });


// // receive log entries from other clients
// socket.on('add_log', (msg) => {

// 	// if it is new, log entry but dont send to server
// 	if (msg.entry_num >= entry_num) {
// 		log(msg.time, msg.tag, msg.data, false, false)
// 		entry_num = msg.entry_num + 1
// 	}
// });

// // TEMPORARY - receive log entries from backend
// socket.on('add_backend_log', (msg) => {

// 	log(msg.time, msg.tag, msg.data, false, false)
// });


/* ---------------------------- Rosbag tool ---------------------------- */

// start recording rosbag
// record_button.addEventListener("click", function() {

// 	// recording flag
// 	refresh = false

// 	// toggle buttons
// 	record_button.disabled = true
// 	stop_button.disabled = false

// 	// rosbag topics
// 	record_topics = []
// 	for (let i = 0; i < rosbag_topics_num; i++) {
// 		if (document.getElementById("check" + i).checked) {
// 			record_topics.push(document.getElementById("check-label" + i).innerText)
// 		}
// 	}

// 	// send record signal
// 	socket.emit('rosbag_record', {"topics": record_topics});

// 	// log entry data
// 	var entry = "Started recording rosbag. Recorded topics: "
// 	for (let i = 0; i < record_topics.length-1; i++) {
// 		entry += record_topics[i] + ', '
// 	}
// 	entry += record_topics[record_topics.length-1] + '.'

// 	// log entry and send to server
// 	log(currentTime(), "Rosbag", entry, false, true)
// });


// // stop recording rosbag
// stop_button.addEventListener("click", function() {

// 	// recording flag
// 	refresh = false

// 	// toggle buttons
// 	record_button.disabled = false
// 	stop_button.disabled = true

// 	// send stop signal
// 	socket.emit('rosbag_stop');

// 	// log entry and send to server
// 	log(currentTime(), "Rosbag", "Stopped recording rosbag.", false, true)
// });


// // toggle rosbag buttons if other clients start recording
// socket.on('rosbag_record', () => {

// 	// recording flag
// 	refresh = false

// 	// toggle buttons
// 	record_button.disabled = true
// 	stop_button.disabled = false
// });


// // toggle rosbag buttons if other clients stop recording
// socket.on('rosbag_stop', () => {

// 	// recording flag
// 	refresh = false

// 	// toggle buttons
// 	record_button.disabled = false
// 	stop_button.disabled = true
// });


/* ------------------------------- Events ------------------------------ */

socket.on('events', (msg) => {

	// write events on table
	for (let i = 0; i < msg.events.length; i++) {
		event_table.innerHTML += "<tr><td>" + msg.events[i][0] + "</td><td>" + msg.events[i][1] + "</td></tr>";
	}

	// scroll down
	event_box.scrollTop = event_box.scrollHeight
});


/* ------------------------- Auxiliar functions ------------------------ */

function currentTime() {
	// return current time in format "hh:mm"

	var date = new Date()
	var time = ""

	var hours = date.getHours()
	if (Math.floor(hours/10) < 1) time += "0"
	time += hours + ":"

	var mins = date.getMinutes()
	if (Math.floor(mins/10) < 1) time += "0"
	time += mins

	return time
}