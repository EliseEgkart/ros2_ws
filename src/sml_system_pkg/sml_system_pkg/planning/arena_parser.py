"""Arena helper functions for Plan D."""

import json
from pathlib import Path


def load_station_coord_json(path, logger=None):
    """Load station coordinates from JSON.

    Expected schema:
        {"station_coordinates": {"0": {"x": 0.0, "y": 0.0}, ...}}

    Returns:
        dict[int, tuple[float, float]]
    """
    coords = {}
    if not path:
        return coords

    p = Path(path).expanduser()
    if not p.exists():
        if logger:
            logger.warn(f'station coord JSON not found: {p}')
        return coords

    try:
        data = json.loads(p.read_text(encoding='utf-8'))
        raw = data.get('station_coordinates', data)
        for key, value in raw.items():
            sid = int(key)
            coords[sid] = (float(value['x']), float(value['y']))
        if logger:
            logger.info(
                f'station coord JSON loaded: {len(coords)} stations, path={p}'
            )
    except Exception as exc:
        if logger:
            logger.warn(f'failed to load station coord JSON: {exc}')
        return {}

    return coords