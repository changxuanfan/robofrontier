"""Frozen per-run policy observation formats; absent fields mean legacy PNG."""
from copy import deepcopy

DEFAULT_PROFILE = 'jpeg_896x560_q85'
LEGACY_PROFILE = 'png_1280x800'
PROFILES = {
    DEFAULT_PROFILE: {'width': 896, 'height': 560, 'format': 'JPEG', 'quality': 85, 'extension': '.jpg'},
    LEGACY_PROFILE: {'width': 1280, 'height': 800, 'format': 'PNG', 'extension': '.png'},
}


def observation_profile(config):
    name = config.get('observation_image_profile', LEGACY_PROFILE)
    if not isinstance(name, str) or name not in PROFILES:
        raise ValueError('Unknown observation image profile: ' + str(name))
    return deepcopy(PROFILES[name])
