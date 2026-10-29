#!/usr/bin/env python3
import argparse
import select
import socket
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(description="Calibracion interactiva de motores SUMO")
    parser.add_argument("ip", nargs="?", default="192.168.1.148", help="IP fija del ESP32")
    parser.add_argument("-p", "--port", type=int, default=9999, help="Puerto UDP del ESP32")
    parser.add_argument("--interval", type=float, default=0.8, help="Segundos para observar cada valor")
    parser.add_argument("--max", type=int, default=100, help="Porcentaje maximo de prueba")
    parser.add_argument("--boost-ms", type=int, default=2000, help="Duracion del boost antes de probar marcha")
    parser.add_argument("--start-right", type=int, default=30, help="Boost del motor derecho para pruebas de marcha")
    parser.add_argument("--start-left", type=int, default=23, help="Boost del motor izquierdo para pruebas de marcha")
    return parser.parse_args()


def send_command(sock, target, command, retries=2):
    for _ in range(retries + 1):
        sock.sendto(command.encode("ascii"), target)
        try:
            data, _ = sock.recvfrom(256)
            return data.decode("utf-8", errors="replace")
        except socket.timeout:
            continue
    return "sin respuesta"


def drain_stdin():
    while select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()


def wait_enter_or_timeout(seconds):
    ready, _, _ = select.select([sys.stdin], [], [], seconds)
    if ready:
        sys.stdin.readline()
        return True
    return False


def wait_ready(title):
    print()
    print(f"== {title} ==")
    input("Pulsa Enter para empezar esta fase...")
    drain_stdin()


def calibrate_start(sock, target, label, command_prefix, max_percent, interval):
    wait_ready(f"Arranque desde parado - {label}")
    print("Pulsa Enter justo cuando veas que empieza a moverse desde parado.")

    for percent in range(0, max_percent + 1):
        send_command(sock, target, "S")
        time.sleep(0.25)
        response = send_command(sock, target, f"{command_prefix} {percent}")
        print(f"arranque {label}: {percent:3d}% -> {response}")
        if wait_enter_or_timeout(interval):
            send_command(sock, target, "S")
            print(f"Arranque guardado para {label}: {percent}%")
            return percent

    send_command(sock, target, "S")
    print(f"No se marco arranque para {label}.")
    return None


def calibrate_run(sock, target, label, command_prefix, start_percent, max_percent, interval, boost_seconds):
    wait_ready(f"Marcha tras boost - {label}")
    print("Cada prueba hace boost, baja a marcha, y espera. Pulsa Enter cuando mantenga marcha estable.")

    for percent in range(0, max_percent + 1):
        send_command(sock, target, "S")
        time.sleep(0.25)
        boost_response = send_command(sock, target, f"{command_prefix} {start_percent}")
        time.sleep(boost_seconds)
        run_response = send_command(sock, target, f"{command_prefix} {percent}")
        print(f"marcha {label}: boost {start_percent:3d}% -> run {percent:3d}% | {boost_response} | {run_response}")
        if wait_enter_or_timeout(interval):
            send_command(sock, target, "S")
            print(f"Marcha guardada para {label}: {percent}%")
            return percent

    send_command(sock, target, "S")
    print(f"No se marco marcha estable para {label}.")
    return None


def calibrate_both_run(sock, target, start_right, start_left, max_percent, interval, boost_seconds):
    wait_ready("Marcha ambos motores tras boost")
    start_avg = round((start_right + start_left) / 2)
    print("El robot hara boost con F y luego bajara a la marcha probada.")
    print("Pulsa Enter cuando avance de forma estable.")

    for percent in range(0, max_percent + 1):
        send_command(sock, target, "S")
        time.sleep(0.25)
        boost_response = send_command(sock, target, f"F {start_avg}")
        time.sleep(boost_seconds)
        run_response = send_command(sock, target, f"F {percent}")
        print(f"marcha ambos: boost {start_avg:3d}% -> run {percent:3d}% | {boost_response} | {run_response}")
        if wait_enter_or_timeout(interval):
            send_command(sock, target, "S")
            print(f"Marcha guardada para ambos: {percent}%")
            return percent

    send_command(sock, target, "S")
    print("No se marco marcha estable para ambos.")
    return None


def value_or_dash(value):
    return "-" if value is None else str(value)


def main():
    args = parse_args()
    target = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    boost_seconds = max(0, args.boost_ms) / 1000.0

    print(f"Calibrando contra {args.ip}:{args.port}")
    print("Levanta el robot o ponlo en una zona segura antes de empezar.")
    print("A/B son comandos de calibracion sin trims: A=derecha, B=izquierda.")
    input("Pulsa Enter cuando estes listo...")

    results = {}
    results["start_left"] = calibrate_start(sock, target, "motor izquierdo", "B", args.max, args.interval)
    results["start_right"] = calibrate_start(sock, target, "motor derecho", "A", args.max, args.interval)

    start_left = results["start_left"] if results["start_left"] is not None else args.start_left
    start_right = results["start_right"] if results["start_right"] is not None else args.start_right

    results["run_left"] = calibrate_run(sock, target, "motor izquierdo", "B", start_left, args.max, args.interval, boost_seconds)
    results["run_right"] = calibrate_run(sock, target, "motor derecho", "A", start_right, args.max, args.interval, boost_seconds)
    results["run_both"] = calibrate_both_run(sock, target, start_right, start_left, args.max, args.interval, boost_seconds)

    send_command(sock, target, "S")
    sock.close()

    print()
    print("Resumen:")
    print(f"  Arranque izquierdo: {value_or_dash(results['start_left'])}%")
    print(f"  Arranque derecho:   {value_or_dash(results['start_right'])}%")
    print(f"  Marcha izquierdo:   {value_or_dash(results['run_left'])}%")
    print(f"  Marcha derecho:     {value_or_dash(results['run_right'])}%")
    print(f"  Marcha ambos:       {value_or_dash(results['run_both'])}%")
    print()
    print("Valores sugeridos para .env:")
    print(f"START_BOOST_MS={args.boost_ms}")
    print(f"START_RIGHT_FWD={value_or_dash(results['start_right'])}")
    print(f"START_LEFT_FWD={value_or_dash(results['start_left'])}")
    print("RUN_RIGHT_FWD=0")
    print("RUN_LEFT_FWD=0")
    print()
    print("Luego ajusta RUN_RIGHT_FWD/RUN_LEFT_FWD de 1 en 1 si al avanzar se desvia.")


if __name__ == "__main__":
    raise SystemExit(main())
