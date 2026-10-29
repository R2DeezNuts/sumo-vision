#!/usr/bin/env python3
import argparse
import re
import socket
import sys


VALID_MOVES = {"f": "F", "l": "L", "r": "R"}
VALID_SOFT_MOVES = {"rr": "RR", "ll": "LL"}


def parse_args():
    parser = argparse.ArgumentParser(description="Control UDP para el robot SUMO")
    parser.add_argument("ip", nargs="?", help="IP del ESP32")
    parser.add_argument("-p", "--port", type=int, default=9999, help="Puerto UDP del ESP32")
    parser.add_argument("--retries", type=int, default=2, help="Reintentos si no llega respuesta")
    return parser.parse_args()


def normalize_command(raw):
    text = raw.strip()
    compact = re.fullmatch(r"([flrFLR])\s*(\d{1,3})", text)
    if compact:
        text = f"{compact.group(1)} {compact.group(2)}"

    parts = text.split()
    if not parts:
        return None

    action = parts[0].lower()
    if action in {"q", "quit", "exit"}:
        return "QUIT"

    if action in {"s", "stop", "parar"}:
        return "S"

    aliases = {
        "forward": "f",
        "avanzar": "f",
        "left": "l",
        "izquierda": "l",
        "right": "r",
        "derecha": "r",
        "softright": "rr",
        "derechasuave": "rr",
        "softleft": "ll",
        "izquierdasuave": "ll",
    }
    action = aliases.get(action, action)

    if action in VALID_SOFT_MOVES:
        if len(parts) != 3:
            raise ValueError("usa base y delta: rr 25 10 o ll 25 10")
        try:
            base = int(parts[1])
            delta = int(parts[2])
        except ValueError as exc:
            raise ValueError("base y delta deben ser numeros enteros") from exc
        if base < 0 or base > 100 or delta < 0 or delta > 100:
            raise ValueError("base y delta deben estar entre 0 y 100")
        return f"{VALID_SOFT_MOVES[action]} {base} {delta}"

    if action not in VALID_MOVES:
        raise ValueError("comando no reconocido")
    if len(parts) != 2:
        raise ValueError("usa porcentaje: f 60, l 35 o r 35")

    try:
        percent = int(parts[1])
    except ValueError as exc:
        raise ValueError("el porcentaje debe ser un numero entero") from exc

    if percent < 0 or percent > 100:
        raise ValueError("el porcentaje debe estar entre 0 y 100")

    return f"{VALID_MOVES[action]} {percent}"


def main():
    args = parse_args()
    esp32_ip = args.ip or input("IP del ESP32: ").strip()
    if not esp32_ip:
        print("Falta la IP del ESP32.", file=sys.stderr)
        return 2

    target = (esp32_ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    print(f"Enviando UDP a {esp32_ip}:{args.port}")
    print("Comandos: f 60, l 35, r 35, rr 25 10, ll 25 10, s, quit")

    while True:
        try:
            raw = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break

        try:
            command = normalize_command(raw)
        except ValueError as exc:
            print(f"Error: {exc}")
            continue

        if command is None:
            continue
        if command == "QUIT":
            break

        for attempt in range(args.retries + 1):
            sock.sendto(command.encode("ascii"), target)
            print(f"Enviado: {command}" if attempt == 0 else f"Reintento {attempt}: {command}")
            try:
                data, address = sock.recvfrom(256)
            except socket.timeout:
                if attempt == args.retries:
                    print("Robot: sin respuesta")
                continue

            print(f"Robot {address[0]}:{address[1]} -> {data.decode('utf-8', errors='replace')}")
            break

    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
