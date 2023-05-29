#!/usr/bin/env python3

from flask_socketio import SocketIO
from flask import Flask, render_template

app = Flask(__name__)
socketio = SocketIO(app)


@app.route('/')
def index():
    return render_template('index.html')


##################### Connection functions #####################

@socketio.on('connect', namespace='/web')
def connect_web():
    print('[INFO] Web client connected')
    # ask backend for initial config when a new client connects
    socketio.emit('config_request', namespace='/backend')


@socketio.on('disconnect', namespace='/web')
def disconnect_web():
    print('[INFO] Web client disconnected')


@socketio.on('connect', namespace='/backend')
def connect_node():
    print('[INFO] Backend connected')


@socketio.on('disconnect', namespace='/backend')
def disconnect_node():
    print('[INFO] Backend disconnected')


################### Communication functions ####################

@socketio.on('config', namespace='/backend')
def handle_config(message):
    # receive initial config from backend and forward it to clients
    socketio.emit('config', message, namespace='/web')


@socketio.on('info', namespace='/backend')
def handle_info(message):
    # receive info from backend and forward it to clients
    socketio.emit('info', message, namespace='/web')


@socketio.on('new_log', namespace='/web')
def handle_log(message):
    # receive a new log entry from a client and forward it to the backend and all other clients
    socketio.emit('new_log', message, namespace='/backend')
    socketio.emit('add_log', message, namespace='/web')

@socketio.on('new_backend_log', namespace='/backend')
def handle_log(message):
    # receive a new log entry from a client and forward it to the backend and all other clients
    socketio.emit('new_log', message, namespace='/backend')
    socketio.emit('add_backend_log', message, namespace='/web')

@socketio.on('rosbag_record', namespace='/web')
def handle_record(message):
    # forward rosbag info to backend to start recording rosbag
    socketio.emit('rosbag_record', message, namespace='/backend')
    # alert other clients that rosbag is recording
    socketio.emit('rosbag_record', namespace='/web')


@socketio.on('rosbag_stop', namespace='/web')
def handle_stop():
    # alert backend to tell the car to stop recording
    socketio.emit('rosbag_stop', namespace='/backend')
    # alert other clients that rosbag stopped recording
    socketio.emit('rosbag_stop', namespace='/web')


@socketio.on('events', namespace='/backend')
def handle_event(message):
    # receive event from backend and forward it to clients
    socketio.emit('events', message, namespace='/web')


################################################################

if __name__ == "__main__":
    print('[INFO] Starting server at http://localhost:3000')
    # run server
    socketio.run(app=app, host='0.0.0.0', port=3000)
