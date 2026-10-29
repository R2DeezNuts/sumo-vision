Import("env")

from pathlib import Path


PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
ENV_PATH = PROJECT_DIR / ".env"

REQUIRED = ("WIFI_SSID", "WIFI_PASSWORD", "LAPTOP_IP")
DEFAULTS = {
    "LAPTOP_PORT": "5005",
    "CONTROL_PORT": "5006",
    "STATIC_IP": "192.168.1.145",
    "GATEWAY_IP": "192.168.1.1",
    "SUBNET_MASK": "255.255.255.0",
    "DNS_IP": "192.168.1.1",
    "FRAME_SIZE": "QVGA",
    "JPEG_QUALITY": "12",
    "TARGET_FPS": "20",
    "UDP_PAYLOAD_SIZE": "1400",
}

FRAME_SIZES = {
    "QQVGA": "FRAMESIZE_QQVGA",
    "HQVGA": "FRAMESIZE_HQVGA",
    "QVGA": "FRAMESIZE_QVGA",
    "CIF": "FRAMESIZE_CIF",
    "VGA": "FRAMESIZE_VGA",
    "SVGA": "FRAMESIZE_SVGA",
    "XGA": "FRAMESIZE_XGA",
}


def parse_env(path):
    values = {}
    if not path.exists():
        raise RuntimeError(
            "Missing .env. Create it from .env.example and set WIFI_SSID, "
            "WIFI_PASSWORD and LAPTOP_IP."
        )

    for line_number, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise RuntimeError(f"Invalid .env line {line_number}: {raw_line}")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        values[key] = value
    return values


def as_int(values, key, minimum, maximum):
    try:
        value = int(values[key])
    except ValueError as exc:
        raise RuntimeError(f"{key} must be an integer") from exc
    if value < minimum or value > maximum:
        raise RuntimeError(f"{key} must be between {minimum} and {maximum}")
    return value


def validate_ipv4(values, key):
    parts = values[key].split(".")
    if len(parts) != 4 or any(not part.isdigit() or int(part) < 0 or int(part) > 255 for part in parts):
        raise RuntimeError(f"{key} must be a valid IPv4 address")


config = parse_env(ENV_PATH)
for key, value in DEFAULTS.items():
    config.setdefault(key, value)

missing = [key for key in REQUIRED if not config.get(key)]
if missing:
    raise RuntimeError(f"Missing required .env values: {', '.join(missing)}")

frame_size = config["FRAME_SIZE"].upper()
if frame_size not in FRAME_SIZES:
    allowed = ", ".join(sorted(FRAME_SIZES))
    raise RuntimeError(f"FRAME_SIZE must be one of: {allowed}")

laptop_port = as_int(config, "LAPTOP_PORT", 1, 65535)
control_port = as_int(config, "CONTROL_PORT", 1, 65535)
jpeg_quality = as_int(config, "JPEG_QUALITY", 4, 63)
target_fps = as_int(config, "TARGET_FPS", 1, 60)
udp_payload_size = as_int(config, "UDP_PAYLOAD_SIZE", 256, 1400)

for key in ("LAPTOP_IP", "STATIC_IP", "GATEWAY_IP", "SUBNET_MASK", "DNS_IP"):
    validate_ipv4(config, key)

env.Append(
    CPPDEFINES=[
        ("WIFI_SSID", env.StringifyMacro(config["WIFI_SSID"])),
        ("WIFI_PASSWORD", env.StringifyMacro(config["WIFI_PASSWORD"])),
        ("LAPTOP_IP", env.StringifyMacro(config["LAPTOP_IP"])),
        ("LAPTOP_PORT", laptop_port),
        ("CONTROL_PORT", control_port),
        ("STATIC_IP", env.StringifyMacro(config["STATIC_IP"])),
        ("GATEWAY_IP", env.StringifyMacro(config["GATEWAY_IP"])),
        ("WIFI_SUBNET_MASK", env.StringifyMacro(config["SUBNET_MASK"])),
        ("DNS_IP", env.StringifyMacro(config["DNS_IP"])),
        ("CAMERA_FRAME_SIZE", FRAME_SIZES[frame_size]),
        ("CAMERA_JPEG_QUALITY", jpeg_quality),
        ("TARGET_FPS", target_fps),
        ("UDP_PAYLOAD_SIZE", udp_payload_size),
    ]
)
