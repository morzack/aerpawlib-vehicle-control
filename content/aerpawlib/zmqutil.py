import asyncio
import zmq

ZMQ_PROXY_IN_PORT = "5570"
ZMQ_PROXY_OUT_PORT = "5571"

ZMQ_TYPE_TRANSITION = "state_transition"
ZMQ_TYPE_FIELD_REQUEST = "field_request"
ZMQ_TYPE_FIELD_CALLBACK = "field_callback"

def run_zmq_proxy():
    # TODO make use asynico. for now must be separate process
    zmq_context = zmq.Context()
    
    p_sub = zmq_context.socket(zmq.XSUB)
    p_pub = zmq_context.socket(zmq.XPUB)

    p_sub.bind(f"tcp://*:{ZMQ_PROXY_IN_PORT}")
    p_pub.bind(f"tcp://*:{ZMQ_PROXY_OUT_PORT}")

    print("[aerpawlib] launching zmq proxy")
    zmq.proxy(p_sub, p_pub)

