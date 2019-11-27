from flask import Flask
app = Flask(__name__)


def start_server():
    app.run()


@app.route('/')
def hello_world():
    return 'Hello, World!'
