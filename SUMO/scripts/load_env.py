from pathlib import Path

Import("env")

PROJECT_DIR = Path(env["PROJECT_DIR"])
ENV_FILE = PROJECT_DIR / ".env"
REQUIRED_KEYS = ("WIFI_SSID", "WIFI_PASSWORD")
OPTIONAL_DEFAULTS = {
    "UDP_PORT": "9999",
    "STATIC_IP": "192.168.1.148",
    "GATEWAY_IP": "192.168.1.1",
    "SUBNET_MASK": "255.255.255.0",
    "DNS_IP": "192.168.1.1",
    "START_BOOST_MS": "2000",
    "START_RIGHT_FWD": "30",
    "START_LEFT_FWD": "23",
    "RUN_RIGHT_FWD": "0",
    "RUN_LEFT_FWD": "-7",
    "TRIM_RIGHT_FWD": "-6",
    "TRIM_LEFT_FWD": "0",
    "TRIM_RIGHT_BACK": "-3",
    "TRIM_LEFT_BACK": "0",
}


def parse_env_file(path):
    values = {}
    if not path.exists():
        raise RuntimeError(
            "Falta el archivo .env. Copia .env.example a .env y rellena WIFI_SSID y WIFI_PASSWORD."
        )

    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise RuntimeError(f"Linea invalida en .env:{line_number}: {raw_line}")

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        values[key] = value

    return values


def escape_define(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


config = parse_env_file(ENV_FILE)
for key, default_value in OPTIONAL_DEFAULTS.items():
    config.setdefault(key, default_value)

missing = [key for key in REQUIRED_KEYS if not config.get(key)]
if missing:
    raise RuntimeError("Faltan variables en .env: " + ", ".join(missing))

for key in ("UDP_PORT", "START_BOOST_MS", "START_RIGHT_FWD", "START_LEFT_FWD"):
    if not config[key].isdigit():
        raise RuntimeError(f"{key} debe ser numerico.")

for key in ("RUN_RIGHT_FWD", "RUN_LEFT_FWD", "TRIM_RIGHT_FWD", "TRIM_LEFT_FWD", "TRIM_RIGHT_BACK", "TRIM_LEFT_BACK"):
    value = config[key]
    if value.startswith("-"):
        value = value[1:]
    if not value.isdigit():
        raise RuntimeError(f"{key} debe ser un entero.")

for key in ("STATIC_IP", "GATEWAY_IP", "SUBNET_MASK", "DNS_IP"):
    parts = config[key].split(".")
    if len(parts) != 4 or any(not part.isdigit() or int(part) < 0 or int(part) > 255 for part in parts):
        raise RuntimeError(f"{key} debe ser una IPv4 valida.")

env.Append(
    BUILD_FLAGS=[
        f'-D WIFI_SSID="\\"{escape_define(config["WIFI_SSID"])}\\""',
        f'-D WIFI_PASSWORD="\\"{escape_define(config["WIFI_PASSWORD"])}\\""',
        f'-D UDP_PORT={config["UDP_PORT"]}',
        f'-D STATIC_IP="\\"{escape_define(config["STATIC_IP"])}\\""',
        f'-D GATEWAY_IP="\\"{escape_define(config["GATEWAY_IP"])}\\""',
        f'-D WIFI_SUBNET_MASK="\\"{escape_define(config["SUBNET_MASK"])}\\""',
        f'-D DNS_IP="\\"{escape_define(config["DNS_IP"])}\\""',
        f'-D START_BOOST_MS={config["START_BOOST_MS"]}',
        f'-D START_RIGHT_FWD={config["START_RIGHT_FWD"]}',
        f'-D START_LEFT_FWD={config["START_LEFT_FWD"]}',
        f'-D RUN_RIGHT_FWD={config["RUN_RIGHT_FWD"]}',
        f'-D RUN_LEFT_FWD={config["RUN_LEFT_FWD"]}',
        f'-D TRIM_RIGHT_FWD={config["TRIM_RIGHT_FWD"]}',
        f'-D TRIM_LEFT_FWD={config["TRIM_LEFT_FWD"]}',
        f'-D TRIM_RIGHT_BACK={config["TRIM_RIGHT_BACK"]}',
        f'-D TRIM_LEFT_BACK={config["TRIM_LEFT_BACK"]}',
    ]
)
