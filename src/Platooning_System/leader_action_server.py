# leader_action_server.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading

class ActionBroadcastServer:
    """
    리더가 선택한 액션 인덱스(0~4)를
    여러 팔로워에게 동시에 방송(broadcast)하는 TCP 서버.

    프로토콜:
      - broadcast(action_idx)를 호출할 때마다
        "3\n" 처럼 '정수 + 개행' 문자열을 모든 클라이언트에 전송.
      - 팔로워는 여기에 TCP로 붙어서 한 줄씩 읽어가면 됨.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 6000):
        self.host = host
        self.port = port
        self.clients = []
        self._stop = False

    def start(self):
        """서버 스레드를 백그라운드로 시작."""
        t = threading.Thread(target=self._server_loop, daemon=True)
        t.start()

    def _server_loop(self):
        """팔로워들의 TCP 연결을 받아들이는 루프."""
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serv.bind((self.host, self.port))
        serv.listen(5)
        print(f"[LeaderServer] listening on {self.host}:{self.port}")
        serv.settimeout(1.0)

        while not self._stop:
            try:
                conn, addr = serv.accept()
                conn.setblocking(True)
                self.clients.append(conn)
                print(f"[LeaderServer] follower connected from {addr}")
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception:
                # 기타 에러는 일단 무시하고 계속
                pass

        serv.close()

    def broadcast(self, action_idx: int):
        """
        현재 액션 인덱스를 모든 팔로워에게 전송.
        팔로워 코드에서는 int(line)으로 파싱하면 됨.
        """
        msg = f"{int(action_idx)}\n".encode("utf-8")
        dead = []

        for c in self.clients:
            try:
                c.sendall(msg)
            except OSError:
                dead.append(c)

        # 끊어진 연결 정리
        for d in dead:
            try:
                d.close()
            except OSError:
                pass
            if d in self.clients:
                self.clients.remove(d)

    def stop(self):
        self._stop = True
        for c in self.clients:
            try:
                c.close()
            except OSError:
                pass
        self.clients.clear()
