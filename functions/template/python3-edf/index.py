# Copyright (c) Alex Ellis 2017. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for full license information.

from flask import Flask, request
from function import handler
import scheddl
import os

runtime = int(os.getenv("EDFRUNTIME", 0))
deadline = int(os.getenv("EDFDEADLINE", 0))
period = int(os.getenv("EDFPERIOD", 0))
threaded = bool(os.getenv("THREADED", False))

app = Flask(__name__)

# distutils.util.strtobool() can throw an exception
def is_true(val):
    return len(val) > 0 and val.lower() == "true" or val == "1"

@app.before_request
def fix_transfer_encoding():
    """
    Sets the "wsgi.input_terminated" environment flag, thus enabling
    Werkzeug to pass chunked requests as streams.  The gunicorn server
    should set this, but it's not yet been implemented.
    """

    transfer_encoding = request.headers.get("Transfer-Encoding", None)
    if transfer_encoding == u"chunked":
        request.environ["wsgi.input_terminated"] = True

@app.route("/", defaults={"path": ""}, methods=["POST", "GET"])
@app.route("/<path:path>", methods=["POST", "GET"])
def main_route(path):
    raw_body = os.getenv("RAW_BODY", "false")

    as_text = True

    if is_true(raw_body):
        as_text = False
    
    ret = handler.handle(request.get_data(as_text=as_text))
    return ret

if __name__ == '__main__':
    if runtime != 0 and deadline != 0 and period != 0:
        dl_args = (
            runtime  * 1000 * 1000, # runtime in nanoseconds
            deadline * 1000 * 1000, # deadline in nanoseconds
            period   * 1000 * 1000  # time period in nanoseconds
        )
        scheddl.set_deadline(*dl_args)
        
    # Enable multithreading so the HTTP server can keep 
    # persistent TCP sockets open without blocking
    app.run(host="0.0.0.0", port=5000, threaded=True)
