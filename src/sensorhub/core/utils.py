
import ssl
from pathlib import Path
from typing import Optional

def ssl_context_from_config(certfile: Optional[str], keyfile: Optional[str], cafile: Optional[str], require_client_cert: bool = False):
    if not certfile or not keyfile:
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=certfile, keyfile=keyfile)
    if require_client_cert:
        ctx.verify_mode = ssl.CERT_REQUIRED
        if cafile:
            ctx.load_verify_locations(cafile)
    return ctx
