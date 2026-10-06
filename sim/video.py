"""Writing an animation to a file."""

import shutil
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, PillowWriter


def save_animation(anim, path, fps, dpi=None):
    """Write `anim` to `path`: an .mp4 video, or a .gif."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".gif":
        writer = PillowWriter(fps=fps)
    else:
        if shutil.which("ffmpeg") is None:  # no system ffmpeg: use the one shipped with imageio-ffmpeg
            import imageio_ffmpeg

            plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        writer = FFMpegWriter(fps=fps)
    anim.save(str(path), writer=writer, dpi=dpi)
