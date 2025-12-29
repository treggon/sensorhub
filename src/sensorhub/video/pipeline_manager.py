
import os, signal, shlex, subprocess, time
from dataclasses import dataclass
from typing import Optional, Literal

Backend = Literal["ffmpeg", "gstreamer"]

@dataclass
class RTSPSettings:
    enable: bool = True
    name: str = "usb_cam0"          # RTSP path in server (MediaMTX)
    bitrate_kbps: int = 2500
    fps: int = 30
    width: int = 1280
    height: int = 720
    gop: int = 60
    codec: Literal["h264", "vp8", "vp9"] = "h264"
    low_latency: bool = True

@dataclass
class SourceSpec:
    type: Literal["device", "rtsp"]
    device: Optional[str] = None     # '/dev/video0'
    url: Optional[str] = None        # 'rtsp://user:pass@ip/path'

@dataclass
class PipelineSpec:
    id: str
    backend: Backend
    source: SourceSpec
    rtsp: RTSPSettings
    target_host: str = "127.0.0.1"
    target_port: int = 8554          # MediaMTX default RTSP port
    mjpeg_enable: bool = True        # keep MJPEG API available

@dataclass
class PipelineState:
    id: str
    backend: Backend
    pid: Optional[int] = None
    command: Optional[str] = None
    running: bool = False
    rtsp_url: Optional[str] = None
    codec: str = "h264"
    fps: int = 30
    bitrate_kbps: int = 2500
    width: int = 1280
    height: int = 720
    gop: int = 60

class PipelineManager:
    """
    Spawns and manages video encode pipelines that publish to an RTSP server (MediaMTX).
    """
    def __init__(self):
        self._pipelines: dict[str, PipelineState] = {}

    # ---------------- FFmpeg command builders ----------------

    def _ffmpeg_for_device(self, spec: PipelineSpec) -> str:
        dev = spec.source.device or "/dev/video0"
        rtsp_url = f"rtsp://{spec.target_host}:{spec.target_port}/{spec.rtsp.name}"
        LL = spec.rtsp.low_latency
        if spec.rtsp.codec == "h264":
            return (
                f"ffmpeg -hide_banner -loglevel warning "
                f"-f v4l2 -framerate {spec.rtsp.fps} -video_size {spec.rtsp.width}x{spec.rtsp.height} -i {dev} "
                f"-c:v libx264 {'-tune zerolatency ' if LL else ''}-preset veryfast "
                f"-b:v {spec.rtsp.bitrate_kbps}k -maxrate {spec.rtsp.bitrate_kbps}k -bufsize {spec.rtsp.bitrate_kbps}k "
                f"-g {spec.rtsp.gop} -keyint_min {spec.rtsp.gop} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )
        elif spec.rtsp.codec == "vp8":
            # libvpx realtime: cpu-used (speed), deadline realtime, lag-in-frames 0, row-mt 1
            return (
                f"ffmpeg -hide_banner -loglevel warning "
                f"-f v4l2 -framerate {spec.rtsp.fps} -video_size {spec.rtsp.width}x{spec.rtsp.height} -i {dev} "
                f"-c:v libvpx -b:v {spec.rtsp.bitrate_kbps}k "
                f"{'-deadline realtime -cpu-used 6 -lag-in-frames 0 ' if LL else '-quality good -cpu-used 2 '} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )
        else:  # vp9
            return (
                f"ffmpeg -hide_banner -loglevel warning "
                f"-f v4l2 -framerate {spec.rtsp.fps} -video_size {spec.rtsp.width}x{spec.rtsp.height} -i {dev} "
                f"-c:v libvpx-vp9 -b:v {spec.rtsp.bitrate_kbps}k "
                f"{'-quality realtime -speed 6 -deadline realtime -row-mt 1 -tile-columns 2 -lag-in-frames 0 -error-resilient 1 ' if LL else '-quality good -speed 2 '} "
                f"-g {spec.rtsp.gop} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )

    def _ffmpeg_for_rtsp(self, spec: PipelineSpec) -> str:
        url = spec.source.url
        rtsp_url = f"rtsp://{spec.target_host}:{spec.target_port}/{spec.rtsp.name}"
        LL = spec.rtsp.low_latency
        if spec.rtsp.codec == "h264":
            return (
                f"ffmpeg -hide_banner -loglevel warning -rtsp_transport tcp -i {url} "
                f"-c:v libx264 {'-tune zerolatency ' if LL else ''}-preset veryfast "
                f"-b:v {spec.rtsp.bitrate_kbps}k -maxrate {spec.rtsp.bitrate_kbps}k -bufsize {spec.rtsp.bitrate_kbps}k "
                f"-g {spec.rtsp.gop} -keyint_min {spec.rtsp.gop} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )
        elif spec.rtsp.codec == "vp8":
            return (
                f"ffmpeg -hide_banner -loglevel warning -rtsp_transport tcp -i {url} "
                f"-c:v libvpx -b:v {spec.rtsp.bitrate_kbps}k "
                f"{'-deadline realtime -cpu-used 6 -lag-in-frames 0 ' if LL else '-quality good -cpu-used 2 '} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )
        else:
            return (
                f"ffmpeg -hide_banner -loglevel warning -rtsp_transport tcp -i {url} "
                f"-c:v libvpx-vp9 -b:v {spec.rtsp.bitrate_kbps}k "
                f"{'-quality realtime -speed 6 -deadline realtime -row-mt 1 -tile-columns 2 -lag-in-frames 0 -error-resilient 1 ' if LL else '-quality good -speed 2 '} "
                f"-g {spec.rtsp.gop} "
                f"-f rtsp -rtsp_transport tcp {rtsp_url}"
            )

    # ---------------- GStreamer command builders ----------------
    def _gst_for_device(self, spec: PipelineSpec) -> str:
        dev = spec.source.device or "/dev/video0"
        rtsp = f"rtsp://{spec.target_host}:{spec.target_port}/{spec.rtsp.name}"
        if spec.rtsp.codec == "h264":
            # zerolatency: speed-preset veryfast, tune=zerolatency; push using rtspclientsink
            return (
                f"gst-launch-1.0 v4l2src device={dev} ! "
                f"video/x-raw,framerate={spec.rtsp.fps}/1,width={spec.rtsp.width},height={spec.rtsp.height} ! "
                f"videoconvert ! x264enc tune=zerolatency speed-preset=veryfast key-int-max={spec.rtsp.gop} ! "
                f"rtph264pay config-interval=1 pt=96 ! rtspclientsink location={rtsp}"
            )
        elif spec.rtsp.codec == "vp8":
            return (
                f"gst-launch-1.0 v4l2src device={dev} ! "
                f"video/x-raw,framerate={spec.rtsp.fps}/1,width={spec.rtsp.width},height={spec.rtsp.height} ! "
                f"videoconvert ! vp8enc deadline=1 cpu-used=6 target-bitrate={spec.rtsp.bitrate_kbps*1000} ! "
                f"rtpvp8pay pt=96 ! rtspclientsink location={rtsp}"
            )
        else:
            return (
                f"gst-launch-1.0 v4l2src device={dev} ! "
                f"video/x-raw,framerate={spec.rtsp.fps}/1,width={spec.rtsp.width},height={spec.rtsp.height} ! "
                f"videoconvert ! vp9enc deadline=1 cpu-used=6 lag-in-frames=0 row-mt=1 "
                f"target-bitrate={spec.rtsp.bitrate_kbps*1000} ! rtpvp9pay pt=96 ! rtspclientsink location={rtsp}"
            )

    def _gst_for_rtsp(self, spec: PipelineSpec) -> str:
        src = spec.source.url
        rtsp = f"rtsp://{spec.target_host}:{spec.target_port}/{spec.rtsp.name}"
        # Ingest RTSP -> transcode -> republish
        if spec.rtsp.codec == "h264":
            return (
                f"gst-launch-1.0 rtspsrc location={src} protocols=tcp ! rtph264depay ! h264parse ! "
                f"avdec_h264 ! videoconvert ! x264enc tune=zerolatency speed-preset=veryfast key-int-max={spec.rtsp.gop} ! "
                f"rtph264pay pt=96 ! rtspclientsink location={rtsp}"
            )
        elif spec.rtsp.codec == "vp8":
            return (
                f"gst-launch-1.0 rtspsrc location={src} protocols=tcp ! decodebin ! videoconvert ! "
                f"vp8enc deadline=1 cpu-used=6 target-bitrate={spec.rtsp.bitrate_kbps*1000} ! "
                f"rtpvp8pay pt=96 ! rtspclientsink location={rtsp}"
            )
        else:
            return (
                f"gst-launch-1.0 rtspsrc location={src} protocols=tcp ! decodebin ! videoconvert ! "
                f"vp9enc deadline=1 cpu-used=6 lag-in-frames=0 row-mt=1 target-bitrate={spec.rtsp.bitrate_kbps*1000} ! "
                f"rtpvp9pay pt=96 ! rtspclientsink location={rtsp}"
            )

    # ---------------- Lifecycle ----------------
    def _build_cmd(self, spec: PipelineSpec) -> str:
        if spec.backend == "ffmpeg":
            return self._ffmpeg_for_device(spec) if spec.source.type == "device" else self._ffmpeg_for_rtsp(spec)
        else:
            return self._gst_for_device(spec) if spec.source.type == "device" else self._gst_for_rtsp(spec)

    def start(self, spec: PipelineSpec) -> PipelineState:
        cmd = self._build_cmd(spec)
        proc = subprocess.Popen(shlex.split(cmd))
        st = PipelineState(
            id=spec.id, backend=spec.backend, pid=proc.pid, command=cmd, running=True,
            rtsp_url=f"rtsp://{spec.target_host}:{spec.target_port}/{spec.rtsp.name}",
            codec=spec.rtsp.codec, fps=spec.rtsp.fps, bitrate_kbps=spec.rtsp.bitrate_kbps,
            width=spec.rtsp.width, height=spec.rtsp.height, gop=spec.rtsp.gop
        )
        self._pipelines[spec.id] = st
        return st

    def stop(self, id: str) -> bool:
        st = self._pipelines.get(id)
        if not st or not st.pid:
            return False
        try:
            os.kill(st.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        st.running = False
        return True

    def list(self):
        return list(self._pipelines.values())

    def patch(self, id: str, **changes) -> Optional[PipelineState]:
        st = self._pipelines.get(id)
        if not st: return None
        # For simplicity: stop & rebuild with updated settings
        self.stop(id)
        # Reconstruct spec from state + changes (you may want persistence)
        spec = PipelineSpec(
            id=id,
            backend=changes.get("backend", st.backend),
            source=SourceSpec(type=changes.get("source_type", "device"),
                              device=changes.get("device"),
                              url=changes.get("url")),
            rtsp=RTSPSettings(
                enable=True,
                name=changes.get("name", st.id),
                bitrate_kbps=changes.get("bitrate_kbps", st.bitrate_kbps),
                fps=changes.get("fps", st.fps),
                width=changes.get("width", st.width),
                height=changes.get("height", st.height),
                gop=changes.get("gop", st.gop),
                codec=changes.get("codec", st.codec),
                low_latency=changes.get("low_latency", True),
            ),
            target_host=changes.get("target_host", "127.0.0.1"),
            target_port=changes.get("target_port", 8554),
            mjpeg_enable=True,
        )
        return self.start(spec)
