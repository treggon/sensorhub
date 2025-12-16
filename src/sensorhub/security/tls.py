
import ssl

def configure_server_ssl(certfile: str, keyfile: str, cafile: str | None = None, require_client_cert: bool = False) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile, keyfile)
    if require_client_cert:
        ctx.verify_mode = ssl.CERT_REQUIRED
        if cafile:
            ctx.load_verify_locations(cafile)
    return ctx
