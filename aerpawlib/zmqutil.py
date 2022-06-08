import asyncio
import zmq

ZMQ_PROXY_IN_PORT = "5570"
ZMQ_PROXY_OUT_PORT = "5571"

def run_zmq_proxy():
    # TODO make use asynico. for now must be separate process
    zmq_context = zmq.Context()
    
    p_sub = zmq_context.socket(zmq.XSUB)
    p_pub = zmq_context.socket(zmq.XPUB)

    p_sub.bind(f"tcp://127.0.0.1:{ZMQ_PROXY_IN_PORT}")
    p_pub.bind(f"tcp://127.0.0.1:{ZMQ_PROXY_OUT_PORT}")

    print("[aerpawlib] launching zmq proxy")
    zmq.proxy(p_sub, p_pub)

