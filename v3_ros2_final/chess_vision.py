# chess_vision.py
# Chess board state detection using frame difference (cv2.absdiff)
# Integrated with arm controller via ROS 2 -> robot_arm_node
#
# BOARD ORIENTATION (your setup):
#   TOP-LEFT     = H8
#   TOP-RIGHT    = H1
#   BOTTOM-LEFT  = A8
#   BOTTOM-RIGHT = A1
#
# Install:
#   pip install opencv-python numpy chess requests
#
# Run:
#   python3 chess_vision.py

import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import cv2
import numpy as np
import requests
import time
import chess
import chess.engine
import threading
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── CONFIG ────────────────────────────────────────────────────────────────────
IPCAM_URL        = "http://10.66.85.95:8080/shot.jpg"   # IP Webcam app on phone

STOCKFISH_PATH   = "stockfish"                           # Linux binary name
STOCKFISH_DEPTH  = 15
STOCKFISH_TIME   = 2.0

HUMAN_COLOR      = chess.WHITE
DIFF_THRESHOLD   = 40

SAVE_DIR         = "captured_frames"
SAVE_IMAGES      = True
CORNERS_FILE     = "board_corners.npy"
CALIB_FILE       = os.path.expanduser("~/chess_arm_calib.json")

SERVO_PINS = {
    'base': 0, 'shldr': 1, 'elbow': 2,
    'wrpitch': 3, 'wrroll': 4, 'grip': 5,
}
MOVE_DELAY = 0.3

# ── COORDINATE MAPPING ───────────────────────────────────────────────────────
def idx_to_sq(col, row):
    file_char = 'abcdefgh'[col]
    rank = 8 - row
    return file_char + str(rank)

def sq_to_idx(sq):
    col = 'abcdefgh'.index(sq[0])
    row = 8 - int(sq[1])
    return col, row

# ── GLOBALS ───────────────────────────────────────────────────────────────────
stockfish           = None
game_board          = chess.Board()
warped_before       = None
frame_before        = None
CLICKED_POINTS      = []

ros_node            = None
ros_pub             = None

# ── ROS INIT ──────────────────────────────────────────────────────────────────
def ros_init():
    global ros_node, ros_pub
    rclpy.init()
    ros_node = rclpy.create_node('chess_vision_node')
    ros_pub  = ros_node.create_publisher(String, 'white_move', 10)
    print("[ros] Publisher ready on /white_move")

def ros_publish(uci_move):
    if ros_pub is None:
        print(f"[ros] Not initialized — cannot publish: {uci_move}")
        return
    msg = String()
    msg.data = uci_move
    ros_pub.publish(msg)
    print(f"[ros] Published /white_move: {uci_move}")

def ros_shutdown():
    global ros_node
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()

# ── CALIBRATION ───────────────────────────────────────────────────────────────
def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print(f"[calib] {CALIB_FILE} not found")
        return {}
    try:
        with open(CALIB_FILE) as f:
            data = json.load(f)
        print(f"[calib] Loaded {len(data.get('squares', {}))} squares")
        return data
    except Exception as e:
        print(f"[calib] Failed: {e}")
        return {}

# ── ARM MOVEMENT ──────────────────────────────────────────────────────────────
def send_move_command(move_type, *squares):
    cmd = "MOVE:" + move_type + ":" + ":".join(squares)
    # Publish from_sq + to_sq as UCI to ROS
    if len(squares) >= 2:
        uci = squares[0] + squares[1]
        ros_publish(uci)
    print(f"[arm] {cmd}")

def dispatch_arm_move(move, board_before_push, af, at, calib):
    is_capture   = board_before_push.is_capture(move)
    is_castle    = board_before_push.is_castling(move)
    is_enpassant = board_before_push.is_en_passant(move)
    is_promotion = move.promotion is not None

    if is_castle:
        send_move_command("castle", af, at)
    elif is_promotion:
        send_move_command("promote", af, at)
    elif is_enpassant:
        captured_pawn_sq = at[0] + af[1]
        send_move_command("enpassant", af, at, captured_pawn_sq)
    elif is_capture:
        send_move_command("capture", af, at)
    else:
        send_move_command("normal", af, at)

# ── STOCKFISH ─────────────────────────────────────────────────────────────────
def load_stockfish():
    global stockfish, STOCKFISH_PATH
    import shutil
    sf = shutil.which("stockfish")
    if sf:
        STOCKFISH_PATH = sf
    elif not os.path.exists(STOCKFISH_PATH):
        print("[stockfish] Not found — install with: sudo apt install stockfish")
        return False
    try:
        stockfish = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
        print(f"[stockfish] {stockfish.id.get('name', 'Stockfish')} ready")
        return True
    except Exception as e:
        print(f"[stockfish] {e}")
        return False

def get_arm_move():
    if game_board.is_game_over():
        print(f"[game] GAME OVER — {game_board.result()}")
        return None, None, game_board.result(), None
    try:
        result = stockfish.play(
            game_board,
            chess.engine.Limit(depth=STOCKFISH_DEPTH, time=STOCKFISH_TIME)
        )
        if not result.move:
            return None, None, "no move", None
        move = result.move
        frm  = chess.square_name(move.from_square)
        to   = chess.square_name(move.to_square)
        info = stockfish.analyse(game_board, chess.engine.Limit(depth=10))
        sc   = info['score'].relative
        if sc.is_mate():
            ev = f"Mate in {abs(sc.mate())}"
        else:
            cp = sc.score()
            ev = f"{cp/100:+.2f}" if cp is not None else "?"
        board_snapshot = game_board.copy()
        game_board.push(move)
        print(f"[stockfish] {move.uci()}  eval: {ev}")
        return frm, to, ev, (move, board_snapshot)
    except Exception as e:
        print(f"[stockfish] {e}")
        return None, None, "?", None

# ── CAMERA ────────────────────────────────────────────────────────────────────
def capture_snapshot():
    try:
        resp  = requests.get(IPCAM_URL, timeout=5)
        arr   = np.frombuffer(resp.content, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return frame
    except Exception as e:
        print(f"[camera] {e}")
        return None

def save_frame(frame, name):
    os.makedirs(SAVE_DIR, exist_ok=True)
    p = os.path.join(SAVE_DIR, f"{name}_{int(time.time())}.jpg")
    cv2.imwrite(p, frame)
    return p

# ── WARP ──────────────────────────────────────────────────────────────────────
def order_corners(pts):
    s    = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).flatten()
    return np.float32([
        pts[np.argmin(s)],
        pts[np.argmin(diff)],
        pts[np.argmax(s)],
        pts[np.argmax(diff)],
    ])

def warp_board(frame, corners=None):
    if corners is None:
        h, w = frame.shape[:2]
        m = int(min(h, w) * 0.05)
        corners = np.float32([[m,m],[w-m,m],[w-m,h-m],[m,h-m]])
    corners = order_corners(corners)
    SIZE = 800
    dst  = np.float32([[0,0],[SIZE,0],[SIZE,SIZE],[0,SIZE]])
    M    = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(frame, M, (SIZE, SIZE))
    return warped, M, corners

def warped_to_original(warped_pts, corners, size=800):
    dst  = np.float32([[0,0],[size,0],[size,size],[0,size]])
    Minv = cv2.getPerspectiveTransform(dst, corners)
    pts  = np.array(warped_pts, dtype=np.float32).reshape(-1,1,2)
    return cv2.perspectiveTransform(pts, Minv).reshape(-1,2)

def load_corners():
    if not os.path.exists(CORNERS_FILE):
        return None
    try:
        c = np.load(CORNERS_FILE)
        if c.shape == (4,2):
            print(f"[corners] Loaded from {CORNERS_FILE}")
            return c
    except Exception as e:
        print(f"[corners] {e}")
    return None

def save_corners(corners):
    np.save(CORNERS_FILE, np.array(corners, dtype=np.float32))
    print(f"[corners] Saved to {CORNERS_FILE}")

# ── DIFF ──────────────────────────────────────────────────────────────────────
def diff_squares(wb, wa):
    sz = wb.shape[0] // 8
    gb = cv2.cvtColor(wb, cv2.COLOR_BGR2GRAY)
    ga = cv2.cvtColor(wa, cv2.COLOR_BGR2GRAY)
    out = []
    for row in range(8):
        for col in range(8):
            x1, y1 = col*sz, row*sz
            sb = gb[y1:y1+sz, x1:x1+sz].astype(np.float32)
            sa = ga[y1:y1+sz, x1:x1+sz].astype(np.float32)
            d  = cv2.absdiff(sb, sa)
            sig  = (d > DIFF_THRESHOLD).sum()
            mean = d.mean()
            out.append((idx_to_sq(col, row), sig*mean, sig, mean))
    out.sort(key=lambda x: x[1], reverse=True)
    return out

def detect_human_move(wb, wa):
    scores = diff_squares(wb, wa)
    print("[diff] Top changed squares:")
    for sq, sc, sig, mn in scores[:6]:
        print(f"       {sq}  score={sc:.0f}  sig={sig}  mean={mn:.1f}")
    changed = [(sq, sc) for sq, sc, sig, mn in scores if sc > 500]
    if len(changed) < 2:
        print("[diff] Not enough change — did you move a piece?")
        return None, None
    sq1, sq2 = changed[0][0], changed[1][0]
    p1 = game_board.piece_at(chess.parse_square(sq1))
    p2 = game_board.piece_at(chess.parse_square(sq2))
    if p1 and not p2:
        return sq1, sq2
    elif p2 and not p1:
        return sq2, sq1
    elif p1 and p2:
        return (sq1, sq2) if p1.color == HUMAN_COLOR else (sq2, sq1)
    return sq1, sq2

def apply_human_move(from_sq, to_sq):
    try:
        move  = chess.Move.from_uci(from_sq + to_sq)
        piece = game_board.piece_at(chess.parse_square(from_sq))
        if piece and piece.piece_type == chess.PAWN:
            tr = int(to_sq[1])
            if (piece.color==chess.WHITE and tr==8) or \
               (piece.color==chess.BLACK and tr==1):
                move = chess.Move.from_uci(from_sq+to_sq+'q')
        if move in game_board.legal_moves:
            game_board.push(move)
            print(f"[game] {move.uci()} (legal)")
            return move
        else:
            print(f"[game] ILLEGAL: {from_sq}->{to_sq}")
            print(f"[game] Legal: {[m.uci() for m in list(game_board.legal_moves)[:8]]}")
            return None
    except Exception as e:
        print(f"[game] {e}")
        return None

# ── DRAW ──────────────────────────────────────────────────────────────────────
def draw_board(warped, hf=None, ht=None, af=None, at=None,
               scores=None, ros_ok=False, cal_ok=False):
    vis = warped.copy()
    sz  = vis.shape[0]
    sq  = sz // 8

    for i in range(9):
        cv2.line(vis,(i*sq,0),(i*sq,sz),(55,55,55),1)
        cv2.line(vis,(0,i*sq),(sz,i*sq),(55,55,55),1)

    for row in range(8):
        for col in range(8):
            name = idx_to_sq(col, row)
            cv2.putText(vis, name, (col*sq+3, row*sq+13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (75,75,75), 1)

    def hl(s, color, lbl):
        c, r = sq_to_idx(s)
        x, y = c*sq, r*sq
        cv2.rectangle(vis,(x,y),(x+sq,y+sq),color,3)
        cv2.putText(vis,lbl,(x+4,y+sq-8),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)

    if hf: hl(hf,(0,220,255),"YOU-F")
    if ht: hl(ht,(0,160,200),"YOU-T")
    if af: hl(af,(50,255,50),"ARM-F")
    if at: hl(at,(0,200,0), "ARM-T")

    if scores:
        mx = max(scores[0][1], 1)
        for sn, sc, _, _ in scores[:8]:
            if sc < 200: continue
            inten = min(int(sc/mx*70),70)
            c, r  = sq_to_idx(sn)
            x, y  = c*sq, r*sq
            roi   = vis[y:y+sq, x:x+sq]
            ov    = roi.copy()
            cv2.rectangle(ov,(0,0),(sq,sq),(0,80,255),-1)
            cv2.addWeighted(ov,inten/255,roi,1-inten/255,0,vis[y:y+sq,x:x+sq])

    for col in range(8):
        cv2.putText(vis, str(8-col), (col*sq+sq//2-5, sz-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140,140,140), 1)
    for row in range(8):
        cv2.putText(vis, 'HGFEDCBA'[row], (3, row*sq+sq//2+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140,140,140), 1)

    for lbl,(lx,ly) in [("H8",(4,14)),("H1",(sz-30,14)),
                         ("A8",(4,sz-6)),("A1",(sz-30,sz-6))]:
        cv2.putText(vis,lbl,(lx,ly),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,200,255),2)

    cv2.rectangle(vis,(0,800),(800,840),(15,15,15),-1)
    turn = "YOUR TURN" if game_board.turn==HUMAN_COLOR else "ARM TURN"
    mn   = game_board.fullmove_number
    bar  = f"M{mn} | {turn} | B=before  A=after  C=corners  R=reset  Q=quit"
    cv2.putText(vis,bar,(6,826),cv2.FONT_HERSHEY_SIMPLEX,0.43,(200,200,200),1)
    cv2.circle(vis,(760,820),7,(0,220,80) if ros_ok  else (0,0,180),-1)
    cv2.putText(vis,"ROS",(769,824),cv2.FONT_HERSHEY_SIMPLEX,0.37,(180,180,180),1)
    cv2.circle(vis,(728,820),7,(0,220,80) if cal_ok else (0,0,180),-1)
    cv2.putText(vis,"CAL",(737,824),cv2.FONT_HERSHEY_SIMPLEX,0.37,(180,180,180),1)
    return vis

# ── CLICK ─────────────────────────────────────────────────────────────────────
CORNER_LABELS = [
    "1 — TOP-LEFT  (H8)",
    "2 — TOP-RIGHT (H1)",
    "3 — BOT-RIGHT (A1)",
    "4 — BOT-LEFT  (A8)",
]

def click_event(event, x, y, flags, param):
    global CLICKED_POINTS
    if event == cv2.EVENT_LBUTTONDOWN and len(CLICKED_POINTS) < 4:
        n = len(CLICKED_POINTS)
        CLICKED_POINTS.append((x, y))
        print(f"[click] {CORNER_LABELS[n]} at ({x},{y})")

# ── GAME LOOP ─────────────────────────────────────────────────────────────────
def game_loop(calib):
    global warped_before, frame_before, CLICKED_POINTS

    print()
    print("="*60)
    print("  CHESS VISION — absdiff + Stockfish + ROS Arm")
    print("="*60)
    print(f"  You play  : {'WHITE' if HUMAN_COLOR==chess.WHITE else 'BLACK'}")
    print()
    print("  TOP-LEFT=H8  TOP-RIGHT=H1")
    print("  BOT-LEFT=A8  BOT-RIGHT=A1")
    print()
    print("  Click corners: H8 -> H1 -> A1 -> A8")
    print("  B=before  A=after  C=corners  R=reset  S=fen  Q=quit")
    print("="*60)

    WIN = "Chess Vision"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 820, 860)
    cv2.setMouseCallback(WIN, click_event)

    corners = load_corners()
    hf = ht = af = at = None
    scores  = None
    cal_ok  = len(calib.get('squares', {})) > 0
    ros_ok  = ros_pub is not None

    while True:
        frame = capture_snapshot()
        if frame is None:
            ph = np.zeros((840,820,3),dtype=np.uint8)
            cv2.putText(ph,"No camera signal",(150,400),
                        cv2.FONT_HERSHEY_SIMPLEX,1.2,(0,0,200),2)
            cv2.imshow(WIN,ph)
            key = cv2.waitKey(1500) & 0xFF
        else:
            warped, M, corners = warp_board(frame, corners)

            if len(CLICKED_POINTS) == 4:
                orig = warped_to_original(CLICKED_POINTS, corners)
                corners = orig
                save_corners(corners)
                CLICKED_POINTS.clear()
                warped, M, corners = warp_board(frame, corners)
                print("[click] Board re-warped — verify corner labels")

            vis = draw_board(warped, hf, ht, af, at, scores,
                             ros_ok=ros_ok, cal_ok=cal_ok)

            if 0 < len(CLICKED_POINTS) < 4:
                n = len(CLICKED_POINTS)
                cv2.putText(vis, f"Click: {CORNER_LABELS[n]}",
                            (10,50),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,255,255),2)
                for px,py in CLICKED_POINTS:
                    cv2.circle(vis,(px,py),8,(0,255,255),-1)
                    cv2.drawMarker(vis,(px,py),(0,255,255),cv2.MARKER_CROSS,20,2)

            dot = (0,255,100) if warped_before is not None else (0,0,180)
            cv2.circle(vis,(795,15),8,dot,-1)
            cv2.imshow(WIN, vis)
            key = cv2.waitKey(200) & 0xFF

        if key in [ord('q'),27]:
            break

        elif key in [ord('b'),ord('B')]:
            f = capture_snapshot()
            if f is None: print("[B] Camera failed"); continue
            warped_before, M, corners = warp_board(f, corners)
            frame_before = f.copy()
            if SAVE_IMAGES: save_frame(warped_before,"before")
            hf = ht = af = at = None
            scores = None
            print("[B] Before captured — move your piece then press A")

        elif key in [ord('a'),ord('A')]:
            if warped_before is None:
                print("[A] Press B first"); continue
            f = capture_snapshot()
            if f is None: print("[A] Camera failed"); continue
            wa, M, corners = warp_board(f, corners)
            if SAVE_IMAGES: save_frame(wa,"after")
            print("\n[A] Analyzing diff...")
            scores = diff_squares(warped_before, wa)
            hf, ht = detect_human_move(warped_before, wa)
            if hf is None:
                print("[!] Move not detected — check lighting"); continue
            print(f"\n[detect] Your move: {hf.upper()} -> {ht.upper()}")
            move = apply_human_move(hf, ht)
            if move is None:
                print("[!] Illegal move detected"); continue

            print("[stockfish] Thinking...")
            af, at, ev, move_info = get_arm_move()

            print()
            print("="*56)
            print(f"  YOUR MOVE :  {hf.upper()}  ->  {ht.upper()}")
            if af and at:
                print(f"  ARM MOVE  :  {af.upper()}  ->  {at.upper()}")
                print(f"  EVAL      :  {ev}")
            else:
                print("  ARM MOVE  :  Game over")
            print("="*56)

            if af and at and move_info:
                if not cal_ok:
                    print("[arm] Calib JSON not loaded")
                else:
                    arm_move, board_before = move_info
                    dispatch_arm_move(arm_move, board_before, af, at, calib)

            warped_before = None

        elif key in [ord('c'),ord('C')]:
            corners = None
            CLICKED_POINTS.clear()
            try: os.remove(CORNERS_FILE)
            except FileNotFoundError: pass
            print("[C] Corners cleared — click H8 -> H1 -> A1 -> A8")

        elif key in [ord('r'),ord('R')]:
            game_board.reset()
            warped_before = None
            hf = ht = af = at = None
            scores = None
            print("[R] Game reset")

        elif key in [ord('s'),ord('S')]:
            print(f"\n[FEN]  {game_board.fen()}")
            print(f"[turn] {'White' if game_board.turn==chess.WHITE else 'Black'}")
            print(f"[move] {game_board.fullmove_number}")
            if game_board.is_check(): print("[!!] CHECK!")
            print()

    cv2.destroyAllWindows()

# ── MAIN ──────────────────────────────────────────────────────────────────────
def main():
    print()
    print("="*60)
    print("  Chess Vision  —  absdiff + Stockfish + ROS Arm")
    print("="*60)
    print()

    ros_init()

    if not load_stockfish():
        ros_shutdown()
        return

    print("[init] Testing camera...")
    frame = capture_snapshot()
    if frame is None:
        print(f"[ERROR] Camera not found")
        print(f"        Make sure IP Webcam is running on phone")
        ros_shutdown()
        return
    print(f"[init] Camera OK — {frame.shape[1]}x{frame.shape[0]}")

    calib = load_calibration()

    game_loop(calib)

    if stockfish: stockfish.quit()
    ros_shutdown()
    print("[done] Bye")

if __name__ == "__main__":
    main()
