"""WAV/MP3 audio playback helpers for script steps."""

import audioop
import os
import shutil
import subprocess
import threading
import time
import uuid
import wave
from pathlib import Path


TARGET_SAMPLE_RATE = 16000
CHUNK_FRAMES = 1024
SUPPORTED_AUDIO_BACKENDS = ("auto", "g1", "system", "none")
SUPPORTED_AUDIO_EXTENSIONS = (".wav", ".mp3")
DEFAULT_AUDIO_VOLUME = 100


def _is_cancelled(cancel_event):
    return cancel_event is not None and cancel_event.is_set()


def normalize_audio_volume(volume):
    """Return a G1 AudioClient volume value clamped to 0..100."""
    if volume is None:
        return DEFAULT_AUDIO_VOLUME
    value = int(round(float(volume)))
    return max(0, min(100, value))


def _set_client_volume(client, volume):
    normalized = normalize_audio_volume(volume)
    try:
        client.SetVolume(normalized)
    except AttributeError:
        print("[AUDIO] G1 AudioClient does not expose SetVolume; keep current robot volume")
    except Exception as exc:
        print(f"[AUDIO] G1 SetVolume({normalized}) failed: {exc}")
    return normalized


def stop_g1_audio_stream(stream_name="music", timeout=2.0):
    """Ask the robot audio service to stop a named stream immediately."""
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    client = AudioClient()
    client.Init()
    try:
        client.SetTimeout(float(timeout))
    except Exception:
        pass
    return client.PlayStop(str(stream_name or "music"))


def set_g1_audio_volume(volume=DEFAULT_AUDIO_VOLUME, timeout=2.0):
    """Set the robot-side G1 audio volume, if the SDK supports it."""
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    client = AudioClient()
    client.Init()
    try:
        client.SetTimeout(float(timeout))
    except Exception:
        pass
    return _set_client_volume(client, volume)


def _read_wav_pcm16_mono_chunks(path, target_sample_rate=TARGET_SAMPLE_RATE, cancel_event=None):
    """Yield PCM16 mono chunks from a WAV file at the requested sample rate."""
    with wave.open(str(path), "rb") as wav:
        input_rate = int(wav.getframerate())
        input_channels = int(wav.getnchannels())
        sample_width = int(wav.getsampwidth())
        ratecv_state = None

        while True:
            if _is_cancelled(cancel_event):
                return

            data = wav.readframes(CHUNK_FRAMES)
            if not data:
                return

            if sample_width != 2:
                data = audioop.lin2lin(data, sample_width, 2)
            if input_channels > 1:
                data = audioop.tomono(data, 2, 0.5, 0.5)
            if input_rate != target_sample_rate:
                data, ratecv_state = audioop.ratecv(
                    data,
                    2,
                    1,
                    input_rate,
                    target_sample_rate,
                    ratecv_state,
                )
            if data:
                yield data


def _find_ffmpeg():
    configured = os.environ.get("G1_TEACH_FFMPEG")
    if configured:
        return configured
    return shutil.which("ffmpeg")


def _terminate_process(process):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def _read_ffmpeg_pcm16_mono_chunks(path, target_sample_rate=TARGET_SAMPLE_RATE, cancel_event=None):
    """Yield PCM16 mono chunks by decoding MP3 through ffmpeg."""
    ffmpeg = _find_ffmpeg()
    if not ffmpeg:
        raise RuntimeError(
            "MP3 playback requires ffmpeg. Install ffmpeg on PATH, or set G1_TEACH_FFMPEG "
            "to the ffmpeg executable path."
        )

    command = [
        ffmpeg,
        "-nostdin",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(path),
        "-vn",
        "-f",
        "s16le",
        "-acodec",
        "pcm_s16le",
        "-ac",
        "1",
        "-ar",
        str(int(target_sample_rate)),
        "pipe:1",
    ]
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        creationflags=creationflags,
    )
    bytes_per_chunk = CHUNK_FRAMES * 2
    try:
        while True:
            if _is_cancelled(cancel_event):
                break
            data = process.stdout.read(bytes_per_chunk)
            if not data:
                break
            yield data

        if _is_cancelled(cancel_event):
            _terminate_process(process)
            return

        return_code = process.wait()
        if return_code != 0:
            stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
            detail = f": {stderr}" if stderr else ""
            raise RuntimeError(f"ffmpeg failed to decode {Path(path).name}{detail}")
    finally:
        if process.poll() is None:
            _terminate_process(process)
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()


def _read_pcm16_mono_chunks(path, target_sample_rate=TARGET_SAMPLE_RATE, cancel_event=None):
    suffix = Path(path).suffix.lower()
    if suffix == ".wav":
        yield from _read_wav_pcm16_mono_chunks(path, target_sample_rate, cancel_event)
        return
    if suffix == ".mp3":
        yield from _read_ffmpeg_pcm16_mono_chunks(path, target_sample_rate, cancel_event)
        return
    raise ValueError(
        f"unsupported audio format: {Path(path).suffix or '<none>'}; "
        f"supported formats: {', '.join(SUPPORTED_AUDIO_EXTENSIONS)}"
    )


def _play_audio_on_g1(
    path,
    stream_name="music",
    timeout=10.0,
    cancel_event=None,
    volume=DEFAULT_AUDIO_VOLUME,
):
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    client = AudioClient()
    client.Init()
    try:
        client.SetTimeout(float(timeout))
    except Exception:
        pass
    _set_client_volume(client, volume)

    stream_id = f"{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"
    try:
        for chunk in _read_pcm16_mono_chunks(path, cancel_event=cancel_event):
            if _is_cancelled(cancel_event):
                break
            ret = client.PlayStream(str(stream_name), stream_id, chunk)
            ret_code = ret[0] if isinstance(ret, tuple) else int(ret)
            if ret_code != 0:
                raise RuntimeError(f"G1 PlayStream returned non-zero: {ret_code}")
    finally:
        if _is_cancelled(cancel_event):
            try:
                client.PlayStop(str(stream_name or "music"))
            except Exception:
                pass


def _play_audio_on_system(path, cancel_event=None):
    import pyaudio

    pa = pyaudio.PyAudio()
    stream = None
    try:
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=TARGET_SAMPLE_RATE,
            output=True,
            frames_per_buffer=CHUNK_FRAMES,
        )
        for chunk in _read_pcm16_mono_chunks(path, cancel_event=cancel_event):
            if _is_cancelled(cancel_event):
                break
            stream.write(chunk)
    finally:
        if stream is not None:
            try:
                stream.stop_stream()
            finally:
                stream.close()
        pa.terminate()


def play_audio_file(
    path,
    backend="auto",
    stream_name="music",
    timeout=10.0,
    cancel_event=None,
    volume=DEFAULT_AUDIO_VOLUME,
):
    """Play one WAV or MP3 file using the requested backend.

    `auto` prefers the G1 AudioClient and falls back to local PyAudio output.
    The function is blocking; use AudioPlayback for async delayed playback.
    """
    audio_path = Path(path).resolve()
    if not audio_path.exists():
        raise FileNotFoundError(f"audio file not found: {audio_path}")
    if audio_path.suffix.lower() not in SUPPORTED_AUDIO_EXTENSIONS:
        raise ValueError(
            f"unsupported audio format: {audio_path.suffix or '<none>'}; "
            f"supported formats: {', '.join(SUPPORTED_AUDIO_EXTENSIONS)}"
        )

    normalized_backend = str(backend or "auto").strip().lower()
    if normalized_backend not in SUPPORTED_AUDIO_BACKENDS:
        raise ValueError(
            f"unsupported audio backend: {backend}; choose one of {', '.join(SUPPORTED_AUDIO_BACKENDS)}"
        )
    if normalized_backend == "none":
        print(f"[AUDIO] skip {audio_path.name} backend=none")
        return True

    if normalized_backend == "g1":
        _play_audio_on_g1(
            audio_path,
            stream_name=stream_name,
            timeout=timeout,
            cancel_event=cancel_event,
            volume=volume,
        )
        return True

    if normalized_backend == "system":
        _play_audio_on_system(audio_path, cancel_event=cancel_event)
        return True

    try:
        _play_audio_on_g1(
            audio_path,
            stream_name=stream_name,
            timeout=timeout,
            cancel_event=cancel_event,
            volume=volume,
        )
        return True
    except Exception as g1_exc:
        if _is_cancelled(cancel_event):
            return True
        print(f"[AUDIO] g1 backend failed, falling back to system audio: {g1_exc}")
        _play_audio_on_system(audio_path, cancel_event=cancel_event)
        return True


def play_wav_file(
    path,
    backend="auto",
    stream_name="music",
    timeout=10.0,
    cancel_event=None,
    volume=DEFAULT_AUDIO_VOLUME,
):
    """Backward-compatible alias for play_audio_file."""
    return play_audio_file(
        path,
        backend=backend,
        stream_name=stream_name,
        timeout=timeout,
        cancel_event=cancel_event,
        volume=volume,
    )


class AudioPlayback:
    """Small handle for delayed optional-background audio playback."""

    def __init__(
        self,
        path,
        backend="auto",
        delay=0.0,
        stream_name="music",
        timeout=10.0,
        async_play=True,
        volume=DEFAULT_AUDIO_VOLUME,
    ):
        self.path = Path(path).resolve()
        self.backend = backend
        self.delay = max(0.0, float(delay or 0.0))
        self.stream_name = str(stream_name or "music")
        self.timeout = float(timeout)
        self.async_play = bool(async_play)
        self.volume = normalize_audio_volume(volume)
        self._thread = None
        self._cancel_event = threading.Event()
        self.error = None

    def start(self):
        if not self.async_play:
            self._run()
            if self.error is not None:
                raise self.error
            return self

        self._thread = threading.Thread(
            target=self._run,
            name=f"AudioPlayback:{self.path.name}",
            daemon=False,
        )
        self._thread.start()
        return self

    def join(self):
        if self._thread is not None:
            self._thread.join()
        if self.error is not None:
            raise self.error

    def cancel(self):
        self._cancel_event.set()
        if str(self.backend or "").strip().lower() in {"auto", "g1"}:
            try:
                stop_g1_audio_stream(self.stream_name, timeout=min(float(self.timeout), 2.0))
            except Exception as exc:
                print(f"[AUDIO] G1 PlayStop failed: {exc}")

    def _run(self):
        try:
            if self.delay > 0 and self._cancel_event.wait(self.delay):
                return
            if self._cancel_event.is_set():
                return
            print(
                f"[AUDIO] play {self.path.name} backend={self.backend} "
                f"stream={self.stream_name}"
            )
            play_audio_file(
                self.path,
                backend=self.backend,
                stream_name=self.stream_name,
                timeout=self.timeout,
                cancel_event=self._cancel_event,
                volume=self.volume,
            )
        except BaseException as exc:
            self.error = exc
            print(f"[AUDIO] playback failed: {exc}")


def describe_audio_step(
    path,
    backend="auto",
    delay=0.0,
    async_play=True,
    stream_name="music",
    volume=DEFAULT_AUDIO_VOLUME,
):
    mode = "async" if async_play else "blocking"
    return (
        f"path={Path(path).name} backend={backend} delay={float(delay or 0.0):.2f}s "
        f"mode={mode} stream={stream_name} volume={normalize_audio_volume(volume)}"
    )
