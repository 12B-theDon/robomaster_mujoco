# How to run
# ros2 run robomaster_ros discover --mode ap --sn 3JKCH9A00101RJ --modules

# Adapted SDK examples/01_robot/05_sta_conn_helper.py
import argparse
import time
import logging
import socket
from typing import Any, Dict, Optional

import sys
import os

import rclpy.utilities
import rclpy.logging

import robomaster
import robomaster.config
import robomaster.robot
import robomaster.conn
import robomaster.led

from robomaster_ros.ftp import FtpConnection



import robomaster.config
robomaster.config.LOCAL_IP_STR = "192.168.2.20"

# ===== 공통 설정: 무선 NIC 바인딩(필수) =====
# 환경변수 ROBOMASTER_LOCAL_IP 가 있으면 우선 사용
_LOCAL_IP = os.getenv("ROBOMASTER_LOCAL_IP", "192.168.2.20")
robomaster.config.LOCAL_IP_STR = _LOCAL_IP


# ===== STA 모드에서만 유효: 로봇 브로드캐스트 검색 =====
def discover_robots(timeout: float = 3.0) -> Dict[str, str]:
    """UDP 브로드캐스트 수신으로 (ip -> sn) 맵을 구성 (STA 전용)."""
    robots: Dict[str, str] = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 브로드캐스트/포트 재사용 옵션 강화
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # 모든 인터페이스에서 수신
    s.bind(('', robomaster.config.ROBOT_BROADCAST_PORT))
    if timeout > 0:
        s.settimeout(timeout)
    start = time.time()
    try:
        while time.time() - start < timeout:
            try:
                data, (ip, _port) = s.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[discover] recv error: {e}")
                continue
            if ip not in robots:
                # SN은 널 종료 바이트 제외 후 UTF-8 디코딩
                sn = data.rstrip(b'\x00').decode('utf-8', errors='ignore')
                robots[ip] = sn
    finally:
        s.close()
    return robots


def probe_robot(ip: str, sn: str, logger: logging.Logger, modules: bool = True) -> bool:
    """
    연결/진단: AP 또는 STA.
    - AP: conn_type='ap', sn 필요(이 SDK는 ip 키워드 미지원).
    - STA: conn_type='sta', sn 필요. discover_robots()로 얻은 ip/sn을 함께 로깅만.
    """
    import robomaster.led

    # 로컬 NIC 바인딩 재확인(다중 NIC 환경에서 안전)
    robomaster.config.LOCAL_IP_STR = os.getenv("ROBOMASTER_LOCAL_IP", _LOCAL_IP)

    # AP / STA 분기
    is_ap = ip.startswith("192.168.2.") or ip == "ap" or ip == "192.168.2.1"

    ep_robot = robomaster.robot.Robot()
    try:
        if is_ap:
            if not sn:
                logger.error("AP mode requires SN (SDK가 ip 파라미터를 지원하지 않습니다).")
                return False
            ep_robot.initialize(conn_type="ap", sn=sn)
        else:
            if not sn:
                logger.error("STA mode requires SN.")
                return False
            ep_robot.initialize(conn_type="sta", sn=sn)

        version = ep_robot.get_version()
        if not version:
            logger.error(f"Could not get version of robot (sn={sn}, ip={ip})")
            return False

        # 배터리 구독(타임아웃 포함)
        battery_level: Optional[float] = None

        def battery_cb(msg: float) -> None:
            nonlocal battery_level
            battery_level = msg

        try:
            ep_robot.battery.sub_battery_info(0, battery_cb)
        except Exception:
            pass

        t0 = time.time()
        while battery_level is None and time.time() - t0 < 3.0:
            time.sleep(0.1)

        battery_text = "unknown"
        if battery_level is not None:
            try:
                battery_text = f"{int(battery_level)}%"
            except Exception:
                battery_text = str(battery_level)

        logger.info(
            f'Connected to robot sn={sn} via {"AP" if is_ap else "STA"} '
            f'(host_local_ip={robomaster.config.LOCAL_IP_STR}): '
            f'version {version}, battery {battery_text}'
        )

        # LED/Sound & 모듈 버전
        play = None
        try:
            play = ep_robot.play_sound(7, times=1)
        except Exception:
            pass

        try:
            ep_robot.led.set_led(r=255, effect=robomaster.led.EFFECT_FLASH, freq=5)
        except Exception:
            pass

        if modules:
            mods = getattr(ep_robot, "_modules", {}) or {}
            for name, module in sorted(mods.items()):
                try:
                    ver = module.get_version()
                    logger.info(f'\t{name}: {ver if ver else "?"}')
                except Exception as e:
                    logger.info(f'\t{name}: get_version failed ({e})')
            else:
                time.sleep(1)

        if play is not None:
            try:
                play.wait_for_completed()
            except Exception:
                pass

        try:
            ep_robot.led.set_led(effect=robomaster.led.EFFECT_OFF)
        except Exception:
            pass

        try:
            ep_robot.play_sound(8, times=1).wait_for_completed()
        except Exception:
            pass

        return True

    except (AttributeError, TypeError) as e:
        logger.error(f"Initialize failed (sn={sn}, ip={ip}): {e}")
        return False
    finally:
        try:
            ep_robot.close()
        except Exception:
            pass


def main(args: Any = None) -> None:
    # ROS 인자 제거 후 argparse로 파싱
    raw = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    logger = rclpy.logging.get_logger("discover")

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--timeout', type=float, default=3.0, help='How much to wait for robots')
    parser.add_argument('--modules', action="store_true", help='Probe the robot modules')
    parser.add_argument('--mode', choices=["ap", "sta"], default="ap", help="Connection mode")
    parser.add_argument('--sn', type=str, default="", help="Robot serial number (required in AP mode)")
    args = parser.parse_args(raw)

    robomaster.logger.setLevel(logging.CRITICAL)
    robomaster.conn.FtpConnection = FtpConnection

    # NIC 바인딩(확실히 한 번 더)
    robomaster.config.LOCAL_IP_STR = os.getenv("ROBOMASTER_LOCAL_IP", _LOCAL_IP)

    if args.mode == "ap":
        if not args.sn:
            logger.error("AP mode requires --sn (또는 환경변수 ROBOMASTER_SN)")
            return
        logger.info("Connecting in AP mode (direct via SN)…")
        # AP는 discover가 설계상 불가 → 고정 'ap' 식별자와 함께 호출
        probe_robot(ip="ap", sn=args.sn, logger=logger, modules=args.modules)

    else:
        robots = discover_robots(timeout=args.timeout)
        logger.info(f'Discovered {len(robots)} robots')
        for ip, sn in robots.items():
            probe_robot(ip, sn, logger=logger, modules=args.modules)


if __name__ == "__main__":
    main()
