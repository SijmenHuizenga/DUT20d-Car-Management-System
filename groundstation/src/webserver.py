from state import statemanager as state
from flask import Flask, send_from_directory

app = Flask(__name__, static_folder='../../operator/build/', static_url_path='')


def start_server():
    app.run()


@app.route('/')
def root():
    return app.send_static_file('index.html')


@app.route('/state')
def home():
    return state.state

