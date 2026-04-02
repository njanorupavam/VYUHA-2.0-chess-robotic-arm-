# pc_serial_bridge.py  — v4
# Multi-client WebSocket hub
# Routes:
#   S<pin>:<pulse>   -> Pico serial  (servo command from HTML sliders)
#   MOVE:<type>:...  -> broadcast to HTML calibrator (from chess_vision.py)
#   R                -> Pico serial  (reset all servos)
#   anything else    -> broadcast to all other clients
#
# pip install pyserial websockets
# python pc_serial_bridge.py

import sys
import os
print("[bridge] Starting...", flush=True)

try:
    import serial
    import serial.tools.list_ports
    print("[bridge] pyserial OK", flush=True)
except ImportError:
    print("[bridge] ERROR: pip install pyserial", flush=True)
    input("Press Enter to exit...")
    sys.exit(1)

try:
    import asyncio
    import websockets
    import threading
    print("[bridge] websockets OK", flush=True)
except ImportError:
    print("[bridge] ERROR: pip install websockets", flush=True)
    input("Press Enter to exit...")
    sys.exit(1)

# ── CONFIG ────────────────────────────────────────────────────────────────────
WS_HOST     = "localhost"
WS_PORT     = 8768
BAUD        = 115200
MANUAL_PORT = None   # override e.g. "COM5" if auto-detect picks wrong port

# ── Serial ────────────────────────────────────────────────────────────────────
ser  = None
loop = None

def find_pico_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[bridge] No serial ports detected", flush=True)
        return None
    print("[bridge] Available ports:", flush=True)
    for p in ports:
        print(f"         {p.device}  —  {p.description}  [{p.manufacturer}]", flush=True)
    # Prefer Pico by name
    for p in ports:
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if any(k in desc or k in mfr for k in ["pico","rp2","micropython","raspberry"]):
            print(f"[bridge] Auto-detected Pico on {p.device}", flush=True)
            return p.device
    # Fallback: first port
    print(f"[bridge] Using first port: {ports[0].device}", flush=True)
    return ports[0].device

def open_serial():
    global ser
    port = MANUAL_PORT or find_pico_port()
    if not port:
        print("[bridge] No port — running without Pico", flush=True)
        return False
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        print(f"[bridge] Serial OPEN on {port} @ {BAUD}", flush=True)
        return True
    except serial.SerialException as e:
        print(f"[bridge] Serial FAILED: {e}", flush=True)
        print(f"         Is Thonny or another app using {port}?", flush=True)
        return False

def send_to_pico(cmd):
    global ser
    if ser is None:
        print(f"[bridge] WARNING: no serial — cannot send: {cmd}", flush=True)
        return
    if not ser.is_open:
        print(f"[bridge] WARNING: serial closed — cannot send: {cmd}", flush=True)
        return
    try:
        line = cmd.strip() + '\n'
        ser.write(line.encode('utf-8'))
        ser.flush()
        print(f"[pico ←] {cmd}", flush=True)
    except Exception as e:
        print(f"[bridge] Serial write error: {e}", flush=True)
        ser = None   # mark as dead so reconnect can happen

def serial_reader():
    """Background thread — read responses from Pico and broadcast to all clients."""
    while True:
        try:
            if ser and ser.is_open and ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"[pico →] {line}", flush=True)
                    if loop:
                        asyncio.run_coroutine_threadsafe(
                            broadcast(f"PICO:{line}", None), loop
                        )
        except Exception as e:
            print(f"[serial reader] {e}", flush=True)

# ── WebSocket ─────────────────────────────────────────────────────────────────
connected_clients = set()

async def broadcast(msg, exclude):
    dead = set()
    for client in list(connected_clients):
        if client == exclude:
            continue
        try:
            await client.send(msg)
        except Exception:
            dead.add(client)
    connected_clients.difference_update(dead)

async def ws_handler(websocket):
    connected_clients.add(websocket)
    cid = id(websocket)
    print(f"[bridge] +Client #{cid}  total={len(connected_clients)}", flush=True)
    try:
        async for raw in websocket:
            message = raw.strip()
            if not message:
                continue

            # ── Servo command → Pico only ──────────────────────────────────
            if message.startswith('S') and ':' in message:
                print(f"[bridge] Servo cmd: {message}", flush=True)
                send_to_pico(message)

            # ── Move command → broadcast to HTML calibrator ────────────────
            elif message.upper().startswith('MOVE:'):
                print(f"[bridge] Move cmd: {message}", flush=True)
                await broadcast(message, websocket)

            # ── Reset → Pico ───────────────────────────────────────────────
            elif message == 'R':
                print(f"[bridge] Reset all servos", flush=True)
                send_to_pico('R')

            # ── Anything else → broadcast ──────────────────────────────────
            else:
                print(f"[bridge] Broadcast: {message[:60]}", flush=True)
                await broadcast(message, websocket)

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"[bridge] Handler error: {e}", flush=True)
    finally:
        connected_clients.discard(websocket)
        print(f"[bridge] -Client #{cid}  total={len(connected_clients)}", flush=True)

# ── Main ──────────────────────────────────────────────────────────────────────
async def amain():
    global loop
    loop = asyncio.get_running_loop()

    open_serial()
    threading.Thread(target=serial_reader, daemon=True).start()

    print(f"", flush=True)
    print(f"[bridge] ✓ WebSocket hub ready at  ws://{WS_HOST}:{WS_PORT}", flush=True)
    print(f"", flush=True)
    print(f"[bridge] Connect these clients (in order):", flush=True)
    print(f"[bridge]   1. chess_arm_calibrator.html  — opens in browser", flush=True)
    print(f"[bridge]   2. chess_vision.py             — python chess_vision.py", flush=True)
    print(f"", flush=True)
    print(f"[bridge] Message routing:", flush=True)
    print(f"[bridge]   S<pin>:<pulse>  → Pico serial  (servo move)", flush=True)
    print(f"[bridge]   MOVE:<type>:... → HTML page     (arm sequence)", flush=True)
    print(f"[bridge]   R               → Pico serial  (reset all)", flush=True)
    print(f"", flush=True)

    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        print("\n[bridge] Stopped by user", flush=True)
        if ser and ser.is_open:
            ser.close()
    except Exception as e:
        print(f"[bridge] FATAL: {e}", flush=True)
        import traceback

        traceback.print_exc()
        input("Press Enter to exit...")