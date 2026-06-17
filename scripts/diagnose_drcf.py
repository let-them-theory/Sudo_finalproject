#!/usr/bin/env python3
# DRCF 컨트롤러 TCP/ICMP 연결 진단 — ros2_control_node 초기화 실패 원인 좁히기용.
"""
DRCF(Doosan 컨트롤러) 연결 진단 스크립트.

ros2_control_node가 `INITIAL STATE CALL FAILURE`로 죽을 때 그 원인을 좁히기 위한 도구.
DRFL C++ 라이브러리 없이 순수 TCP/ICMP 레벨만 검사한다.

검사 항목:
  1. ICMP ping — 네트워크/케이블 도달 가능 여부
  2. TCP 12345 (DRCF) — 컨트롤 포트 응답 (refused/timeout/open?)
  3. TCP 12345 raw recv 5초 — DRCF가 자발적으로 보내는 모니터링 스트림 유무
  4. TCP 20002 (DRL user) — gripper 브리지가 쓰는 포트도 같이 확인
  5. 기존 ros2_control_node 점유 확인 — 다른 클라이언트가 잡고 있는지

사용:
  python3 diagnose_drcf.py                       # 기본 110.120.1.50
  python3 diagnose_drcf.py 192.168.137.50        # 다른 IP
"""
from __future__ import annotations

import socket
import subprocess
import sys
import time

DEFAULT_HOST = "110.120.1.50"
DRCF_PORT = 12345
DRL_USER_PORT = 20002


def ok(msg: str) -> None:
    print(f"  \033[32m✓\033[0m {msg}")


def fail(msg: str) -> None:
    print(f"  \033[31m✗\033[0m {msg}")


def warn(msg: str) -> None:
    print(f"  \033[33m!\033[0m {msg}")


def section(title: str) -> None:
    print(f"\n\033[1m── {title} ──\033[0m")


def check_ping(host: str) -> bool:
    section(f"1. ICMP ping → {host}")
    try:
        r = subprocess.run(
            ["ping", "-c", "3", "-W", "2", host],
            capture_output=True, text=True, timeout=15,
        )
        # 마지막 줄에 "X received" 확인
        if r.returncode == 0 and " 0% packet loss" in r.stdout:
            # RTT 추출
            for line in r.stdout.splitlines()[-2:]:
                if "min/avg/max" in line or "rtt" in line:
                    print(f"    {line.strip()}")
            ok("ping 응답 정상 — 네트워크 도달 가능")
            return True
        else:
            fail(f"ping 손실/실패 (rc={r.returncode})")
            for line in r.stdout.splitlines()[-3:]:
                print(f"    {line}")
            return False
    except subprocess.TimeoutExpired:
        fail("ping 타임아웃")
        return False
    except FileNotFoundError:
        warn("ping 명령 없음 — 건너뜀")
        return True


def check_tcp_connect(host: str, port: int, label: str) -> tuple[bool, socket.socket | None]:
    """TCP 연결만 시도하고 결과 + 소켓(연결 성공 시) 반환."""
    section(f"2. TCP {label} ({host}:{port}) 연결")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    t0 = time.monotonic()
    try:
        s.connect((host, port))
        dt = (time.monotonic() - t0) * 1000
        ok(f"TCP 연결 성공 ({dt:.0f} ms)")
        return True, s
    except socket.timeout:
        fail("TCP 연결 타임아웃 — DRCF가 응답 없음 (포트 닫힘/필터링)")
        s.close()
        return False, None
    except ConnectionRefusedError:
        fail("TCP 연결 거부 — 포트는 열려있는데 서버가 거부 (DRCF 미기동)")
        s.close()
        return False, None
    except OSError as e:
        fail(f"TCP 오류: {e}")
        s.close()
        return False, None


def check_drcf_traffic(sock: socket.socket, seconds: float = 5.0) -> None:
    """DRCF는 연결되면 자발적으로 모니터링 패킷을 흘려보낸다.
    5초 동안 들어오는 바이트를 받아 본다. 없다 = DRCF 응답 정지."""
    section(f"3. DRCF 모니터링 스트림 ({seconds:.0f}초 대기)")
    sock.settimeout(0.5)
    total = 0
    chunks = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            data = sock.recv(4096)
            if not data:
                warn("서버가 연결을 닫음 (FIN)")
                break
            total += len(data)
            chunks += 1
        except socket.timeout:
            continue
        except OSError as e:
            fail(f"recv 오류: {e}")
            break
    if total > 0:
        ok(f"DRCF로부터 {total} bytes 수신 ({chunks} chunks) — 모니터링 스트림 살아있음")
        ok("→ DRCF는 살아있고 패킷도 보내는 중. PC 측 DRFL 라이브러리가 그 패킷을 해석 못 하거나, AUTHORITY 거부 응답일 가능성.")
    else:
        fail(f"{seconds:.0f}초 동안 0 bytes — DRCF 측이 데이터 안 보냄")
        fail("→ DRCF가 hang 상태. 펜던트 측 리셋/재부팅 필요.")


def check_listening_processes() -> None:
    """우리 PC에서 12345 포트로 이미 연결된 프로세스가 있는지 — 다른 ros2_control이 점유 중일 수 있음."""
    section("4. PC 측 12345 포트 연결 중인 프로세스")
    try:
        r = subprocess.run(
            ["ss", "-tnp"],
            capture_output=True, text=True, timeout=5,
        )
        lines = [l for l in r.stdout.splitlines() if ":12345" in l]
        if not lines:
            ok("12345 포트 연결 중인 프로세스 없음 (다른 ros2_control이 잡고 있지 않음)")
        else:
            warn("12345 포트 사용 중인 프로세스 발견 — 다른 인스턴스가 DRCF 점유 중일 수 있음")
            for l in lines:
                print(f"    {l}")
    except Exception as e:
        warn(f"ss 명령 실패: {e}")


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    print(f"\033[1mDRCF 연결 진단\033[0m  target = {host}")

    ping_ok = check_ping(host)

    # DRCF 컨트롤 포트
    drcf_ok, drcf_sock = check_tcp_connect(host, DRCF_PORT, "DRCF 컨트롤")
    if drcf_ok and drcf_sock:
        check_drcf_traffic(drcf_sock, seconds=5.0)
        drcf_sock.close()

    # DRL 사용자 포트(그리퍼용)
    drl_ok, drl_sock = check_tcp_connect(host, DRL_USER_PORT, "DRL user (그리퍼)")
    if drl_ok and drl_sock:
        drl_sock.close()

    check_listening_processes()

    # 종합 진단
    section("종합")
    if not ping_ok:
        print("  → 네트워크/케이블 문제. 컨트롤러 IP·LAN 확인.")
    elif not drcf_ok:
        print("  → ping은 되는데 DRCF 포트 응답 없음. DRCF 프로세스가 죽었거나 부팅 미완료.")
        print("    펜던트 측에서 컨트롤러 재시작 또는 전원 사이클 필요.")
    else:
        # drcf_ok지만 traffic 0이었으면 위에서 fail 메시지 이미 출력됨.
        print("  → DRCF 포트는 응답. 위 3번 결과를 보라:")
        print("    (a) 바이트 수신됨 → DRCF는 살아있음. AUTHORITY 거부 가능성(펜던트 모드/E-STOP/알람).")
        print("    (b) 0 바이트 → DRCF hang. 펜던트 재시작 또는 전원 사이클 필요.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
