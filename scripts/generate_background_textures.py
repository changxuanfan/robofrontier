"""Create the deterministic, visual-only textures for the complex background."""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
SIZE = 512


def periodic_noise(seed, scale):
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:SIZE, 0:SIZE]
    field = np.zeros((SIZE, SIZE), dtype=np.float32)
    for _ in range(48):
        fx, fy = rng.integers(-scale, scale + 1, size=2)
        if fx == 0 and fy == 0:
            fx = 1
        phase = rng.uniform(0, 2 * np.pi)
        amplitude = rng.uniform(0.4, 1.0) / max(1, (fx * fx + fy * fy) ** 0.35)
        field += amplitude * np.sin(2 * np.pi * (fx * x + fy * y) / SIZE + phase)
    field -= field.mean()
    field /= max(float(field.std()), 1e-6)
    return field


def flecked_texture(seed, base, spread, fleck_palette, fleck_count, radius_range, scale):
    rng = np.random.default_rng(seed)
    noise = periodic_noise(seed + 100, scale)
    image = np.empty((SIZE, SIZE, 3), dtype=np.float32)
    for channel in range(3):
        channel_noise = periodic_noise(seed + 200 + channel, scale)
        image[:, :, channel] = base[channel] + spread * (0.7 * noise + 0.3 * channel_noise)
    result = Image.fromarray(np.uint8(np.clip(image, 0, 255)), "RGB")
    draw = ImageDraw.Draw(result, "RGBA")
    for _ in range(fleck_count):
        x, y = (int(value) for value in rng.integers(0, SIZE, size=2))
        radius = int(rng.integers(radius_range[0], radius_range[1] + 1))
        color = fleck_palette[int(rng.integers(0, len(fleck_palette)))]
        alpha = int(rng.integers(75, 170))
        for ox in (-SIZE, 0, SIZE):
            for oy in (-SIZE, 0, SIZE):
                draw.ellipse((x + ox - radius, y + oy - radius,
                              x + ox + radius, y + oy + radius),
                             fill=(*color, alpha))
    return result


def cube_texture(seed, dominant, other_colors):
    """Make multicolor patches while retaining the named cube's dominant hue."""
    rng = np.random.default_rng(seed)
    palette = np.asarray([dominant, *other_colors], dtype=np.float32)
    # Coarse cells create visible patches; the dominant material occupies 68%.
    labels = rng.choice(len(palette), size=(32, 32), p=[0.68, 0.08, 0.08, 0.08, 0.08])
    labels = np.repeat(np.repeat(labels, 16, axis=0), 16, axis=1)
    image = palette[labels]
    noise = rng.normal(0, 9, size=(SIZE, SIZE, 1))
    return Image.fromarray(np.uint8(np.clip(image + noise, 0, 255)), "RGB")


def pattern_texture(kind):
    light, dark = (220, 220, 220), (35, 35, 35)
    image = Image.new("RGB", (SIZE, SIZE), light)
    draw = ImageDraw.Draw(image)
    if kind == "striped":
        band = SIZE // 8
        for index in range(8):
            if index % 2:
                draw.rectangle((index * band, 0, (index + 1) * band - 1, SIZE - 1), fill=dark)
    elif kind == "dotted":
        spacing = SIZE // 4
        radius = SIZE // 16
        for row in range(4):
            for col in range(4):
                x, y = spacing // 2 + col * spacing, spacing // 2 + row * spacing
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=dark)
    elif kind == "checkered":
        cell = SIZE // 8
        for row in range(8):
            for col in range(8):
                if (row + col) % 2:
                    draw.rectangle((col * cell, row * cell,
                                    (col + 1) * cell - 1, (row + 1) * cell - 1), fill=dark)
    else:
        raise ValueError(f"Unknown pattern: {kind}")
    return image


def main():
    output = ROOT / "assets/backgrounds"
    output.mkdir(parents=True, exist_ok=True)
    table = flecked_texture(
        1729, (164, 153, 132), 11,
        [(190, 67, 63), (55, 99, 171), (71, 129, 79),
         (214, 185, 111), (87, 81, 72), (225, 215, 191)],
        1450, (1, 4), 18)
    floor = flecked_texture(
        2718, (119, 126, 132), 20,
        [(147, 72, 68), (63, 91, 139), (102, 123, 88),
         (173, 151, 115), (73, 77, 82), (183, 183, 174)],
        420, (1, 7), 9)
    table.save(output / "table-terrazzo.png", optimize=True)
    floor.save(output / "floor-mottled.png", optimize=True)
    cube_colors = {
        "blue": (20, 89, 242),
        "red": (230, 26, 26),
        "green": (20, 179, 56),
    }
    distractors = [(218, 194, 143), (83, 85, 87)]
    for index, (name, dominant) in enumerate(cube_colors.items()):
        other_colors = [color for key, color in cube_colors.items() if key != name] + distractors
        cube_texture(3811 + index, dominant, other_colors).save(
            output / f"cube-{name}.png", optimize=True)
    for name in ("striped", "dotted", "checkered"):
        pattern_texture(name).save(output / f"cube-{name}.png", optimize=True)


if __name__ == "__main__":
    main()
