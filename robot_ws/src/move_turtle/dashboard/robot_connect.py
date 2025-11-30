"""
Utility helpers for connecting to a remote robot over SSH and executing ROS2 commands.
"""
from __future__ import annotations

import json
import socket
import time
import paramiko  # type: ignore
from typing import Tuple, Dict


def _check_port_open(host: str, port: int, timeout: int = 3) -> Tuple[bool, str]:
    """Check if a port is open on the remote host."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        if result == 0:
            return True, "포트가 열려있습니다."
        else:
            return False, f"포트 {port}에 연결할 수 없습니다. (에러 코드: {result})"
    except socket.gaierror as e:
        return False, f"호스트 이름을 확인할 수 없습니다: {e}"
    except socket.timeout:
        return False, f"포트 {port} 연결 타임아웃 (호스트가 응답하지 않음)"
    except Exception as e:
        return False, f"포트 확인 중 오류: {e}"


def _run_remote_command(robot_ip: str, username: str, password: str, command: str, port: int = 22) -> Tuple[bool, str, str]:
    """Execute a command on the remote robot via SSH."""
    ssh = None
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # SSH 연결 시도 (포트 체크는 건너뛰고 직접 연결 시도)
        try:
            ssh.connect(
                robot_ip,
                port=port,
                username=username,
                password=password,
                timeout=15,
                allow_agent=False,
                look_for_keys=False,
                banner_timeout=10
            )
        except paramiko.AuthenticationException:
            return False, "", f"인증 실패: 사용자명 '{username}' 또는 비밀번호가 올바르지 않습니다."
        except paramiko.SSHException as e:
            return False, "", f"SSH 연결 오류: {str(e)}\n\n가능한 원인:\n1. SSH 서비스가 실행 중인지 확인하세요\n2. 사용자명과 비밀번호가 올바른지 확인하세요"
        except socket.timeout:
            return False, "", f"연결 타임아웃: {robot_ip}:{port}에 연결할 수 없습니다.\n\n가능한 원인:\n1. 로봇이 같은 네트워크에 연결되어 있는지 확인하세요\n2. 로봇 IP 주소가 올바른지 확인하세요\n3. 방화벽이 연결을 차단하고 있는지 확인하세요"
        except socket.error as e:
            error_code = getattr(e, 'errno', None)
            error_msg = str(e)
            if error_code == 113 or "No route to host" in error_msg:
                return False, "", f"네트워크 연결 실패: {robot_ip}에 도달할 수 없습니다.\n\n오류 코드: {error_code}\n오류: {error_msg}\n\n가능한 원인:\n1. 로봇 IP 주소 확인: {robot_ip}\n2. 같은 네트워크에 연결되어 있는지 확인\n3. VMware 네트워크 설정 확인\n4. 로봇이 켜져 있고 네트워크에 연결되어 있는지 확인"
            elif "Unable to connect" in error_msg or "Errno" in error_msg:
                return False, "", f"연결 실패: {robot_ip}:{port}에 연결할 수 없습니다.\n\n오류 코드: {error_code}\n오류: {error_msg}\n\n가능한 원인:\n1. 로봇 IP 주소 확인: {robot_ip}\n2. SSH 포트 확인: {port}\n3. 같은 네트워크에 연결되어 있는지 확인\n4. 로봇에서 SSH 서비스 실행 확인: sudo systemctl status ssh\n5. 방화벽 확인: sudo ufw status"
            return False, "", f"SSH 연결 중 네트워크 오류: {error_msg} (코드: {error_code})"
        except Exception as e:
            error_msg = str(e)
            return False, "", f"SSH 연결 중 예상치 못한 오류: {error_msg}"

        # 명령 실행 (타임아웃 증가)
        stdin, stdout, stderr = ssh.exec_command(command, timeout=60)

        # 출력 읽기 - 간단하고 확실한 방법
        import time

        # exit_status가 준비될 때까지 대기 (최대 10초)
        max_wait = 10
        waited = 0
        while not stdout.channel.exit_status_ready() and waited < max_wait:
            time.sleep(0.2)
            waited += 0.2

        # 출력 읽기 - read()를 직접 사용
        out = ""
        err = ""

        try:
            # stdout 읽기
            out = stdout.read().decode('utf-8', errors='ignore')
        except Exception as e:
            out = f"stdout 읽기 오류: {str(e)}"

        try:
            # stderr 읽기
            err = stderr.read().decode('utf-8', errors='ignore')
        except Exception as e:
            err = f"stderr 읽기 오류: {str(e)}"

        # exit_status 확인
        exit_status = 0
        try:
            exit_status = stdout.channel.recv_exit_status()
        except Exception as e:
            # exit_status를 받을 수 없으면 에러로 간주
            exit_status = -1
            if not err.strip():
                err = f"Exit status를 받을 수 없음: {str(e)}"

        # stderr가 있으면 출력에 추가
        if err.strip():
            if out.strip():
                out += f"\n\n[Stderr]\n{err.strip()}"
            else:
                out = f"[Stderr]\n{err.strip()}"

        # 출력이 비어있으면 exit code 표시
        if not out.strip():
            out = f"[명령 실행됨] Exit code: {exit_status}\n"
            if err.strip():
                out += f"Stderr: {err.strip()}\n"
            out += "(stdout 출력 없음 - 명령이 실행되었지만 출력이 없습니다)"

        success = exit_status == 0 or "started" in out.lower() or "successfully" in out.lower()
        return success, out, err

    except Exception as e:
        return False, "", f"명령 실행 중 오류: {str(e)}"
    finally:
        if ssh:
            try:
                ssh.close()
            except:
                pass


def connect_and_bringup(robot_ip: str, username: str, password: str, domain_id: int = 3) -> Tuple[bool, str, str]:
    """
    Run the robot bringup launch file remotely.

    Args:
        robot_ip: Robot IP address
        username: SSH username
        password: SSH password
        domain_id: ROS_DOMAIN_ID (default: 3)
    """
    ssh_port = 22  # 항상 22 포트 사용
    # 1) 중복 실행 방지: 이미 실행 중이면 바로 리턴
    is_running, status_out, status_err = check_bringup_status(robot_ip, username, password, ssh_port)
    if is_running:
        return True, "bringup already running", ""

    # 5) ROS2 환경 자동 감지
    detect_ros2_cmd = """
        if [ -f /opt/ros/humble/setup.bash ]; then
            echo "humble"
        elif [ -f /opt/ros/foxy/setup.bash ]; then
            echo "foxy"
        else
            echo "none"
        fi
    """
    success_detect, ros2_distro, _ = _run_remote_command(robot_ip, username, password, detect_ros2_cmd, ssh_port)

    if not success_detect or ros2_distro.strip() == "none":
        return False, "", "ROS2 distribution not found (neither humble nor foxy)"

    ros2_distro = ros2_distro.strip()

    # 기존 프로세스 종료 및 PID 파일 삭제 (깔끔한 종료)
    cmd_kill = """
        pkill -9 -f "turtlebot3_bringup|robot.launch.py|turtlebot3_ros" 2>/dev/null || true;
        rm -f /tmp/turtlebot3_bringup.pid 2>/dev/null || true;
        sleep 2;
        echo 'Old processes killed'
    """
    success_kill, out_kill, err_kill = _run_remote_command(robot_ip, username, password, cmd_kill, ssh_port)

    # 3) 브링업 실행 및 PID 파일 기록
    cmd1 = f"""
        /bin/bash -ic '
        source /opt/ros/{ros2_distro}/setup.bash;
        source ~/robot_ws/install/setup.bash 2>/dev/null || true;
        export TURTLEBOT3_MODEL=burger;
        export ROS_DOMAIN_ID={domain_id};
        nohup ros2 launch turtlebot3_bringup robot.launch.py > /tmp/turtlebot3_bringup.log 2>&1 &
        PID=$!;
        echo $PID > /tmp/turtlebot3_bringup.pid;
        echo $PID
        '
    """

    # 명령 실행
    success1, out1, err1 = _run_remote_command(robot_ip, username, password, cmd1, ssh_port)

    # 프로세스 시작 대기
    time.sleep(5)

    # bringup 성공/실패 판별
    check_cmd = """
        if pgrep -f 'ros2 launch turtlebot3_bringup' > /dev/null 2>&1 || \
           pgrep -f 'robot.launch.py' > /dev/null 2>&1 || \
           pgrep -f 'turtlebot3_bringup' > /dev/null 2>&1; then
            echo "running"
        else
            echo "not_running"
        fi
    """
    success2, out2, err2 = _run_remote_command(robot_ip, username, password, check_cmd, ssh_port)

    # 결과 합치기
    out = "=== Step 1: Bringup Execution ===\n"
    if out1 and out1.strip():
        out += out1.strip()
    else:
        out += "(출력 없음 - 명령이 실행되지 않았거나 출력이 캡처되지 않았습니다)"
    if err1 and err1.strip():
        out += f"\n\n[Step 1 Stderr]\n{err1.strip()}"
    if out2:
        out += f"\n\n=== Step 2: Status Check ===\n{out2.strip()}"
    if err2 and err2.strip():
        out += f"\n\n[Step 2 Stderr]\n{err2.strip()}"
    err = ((err1 or "") + "\n" + (err2 or "")).strip()

    # success 판단: pgrep 결과로만 판단
    success = "running" in (out2 or "").strip()

    return success, out, err


def send_test_move(robot_ip: str, username: str, password: str, domain_id: int = 3) -> Tuple[bool, str, str]:
    """
    Execute the local test_move_command script on the robot to publish a short forward command.

    Args:
        robot_ip: Robot IP address
        username: SSH username
        password: SSH password
        domain_id: ROS_DOMAIN_ID (default: 3)
    """
    ssh_port = 22  # 항상 22 포트 사용
    # ROS2 환경 설정 후 test_move_command.py 직접 실행
    cmd = (
        f"export ROS_DOMAIN_ID={domain_id} && "
        "source /opt/ros/foxy/setup.bash && "
        "source ~/robot_ws/install/local_setup.bash && "
        "echo '=== Test Move Command Start ===' && "
        "echo 'ROS_DOMAIN_ID:' $ROS_DOMAIN_ID && "
        "python3 ~/robot_ws/src/test_move_command.py 2>&1 && "
        "echo '=== Test Move Command End ==='"
    )
    return _run_remote_command(robot_ip, username, password, cmd, ssh_port)


# def run_precomputed_move(robot_ip: str, username: str, password: str, domain_id: int = 3,
#                          precomputed_path: str = "move_path_9") -> Tuple[bool, str, str]:
#     """
#     Execute move_full_path.py with a precomputed path dataset.
#     로컬 PC에서 함수를 실행하여 commands를 생성하고, JSON으로 로봇 PC에 전달.
#     
#     ⚠️ 이 함수는 move_path_9.py를 사용하므로 해당 파일이 삭제되면 사용 불가
#     """
#     # 로컬 PC에서 move_path_9 함수 실행
#     try:
#         # move_path_9.py는 로컬 PC의 robot_ws/src/에 있음
#         import sys
#         from pathlib import Path
#         
#         # robot_connect.py: robot_ws/src/move_turtle/dashboard/robot_connect.py
#         # move_path_9.py: robot_ws/src/move_turtle/move_path_9.py
#         current_file = Path(__file__).resolve()
#         move_path_9_file = current_file.parent.parent / "move_path_9.py"
#         
#         if not move_path_9_file.exists():
#             return False, "", f"move_path_9.py를 찾을 수 없습니다. 경로: {move_path_9_file}"
#         
#         # move_path_9 모듈 import
#         import importlib.util
#         spec = importlib.util.spec_from_file_location("move_path_9", move_path_9_file)
#         move_path_9_module = importlib.util.module_from_spec(spec)
#         spec.loader.exec_module(move_path_9_module)
#         
#         # 함수 실행
#         commands = move_path_9_module.get_move_path_9_commands()
#         
#         # JSON 문자열 생성 (한 줄로, 따옴표 이스케이프 처리)
#         import json
#         json_str = json.dumps(commands, ensure_ascii=False)
#         # SSH 명령에서 사용할 수 있도록 이스케이프
#         json_str_escaped = json_str.replace("'", "'\"'\"'").replace("$", "\\$")
#         
#         # 로봇 PC에 JSON 파일 생성하고 실행
#         remote_json_path = "/tmp/robot_commands.json"
#         cmd = (
#             f"export ROS_DOMAIN_ID={domain_id} && "
#             "export ROS_LOCALHOST_ONLY=0 && "
#             "source /opt/ros/foxy/setup.bash && "
#             "cd ~/robot_ws && "
#             "source install/local_setup.bash && "
#             f"echo '{json_str_escaped}' > {remote_json_path} && "
#             "echo '=== Precomputed Path Start ===' && "
#             f"PYTHONPATH=~/robot_ws/src:$PYTHONPATH python3 src/move_full_path.py --commands-json {remote_json_path} 2>&1 && "
#             "echo '=== Precomputed Path End ==='"
#         )
#         return _run_remote_command(robot_ip, username, password, cmd)
#         
#     except Exception as e:
#         return False, "", f"로컬 PC에서 함수 실행 실패: {str(e)}"


def start_move_full_path(robot_ip: str, username: str, password: str, domain_id: str | None = None,
                         json_path: str | None = None, commands_json: str | None = None, ssh_port: int = 22) -> Tuple[bool, str, str]:
    """
    Run move_full_path.py on the remote robot to execute the full planned route.

    Args:
        robot_ip: Robot IP address
        username: SSH username
        password: SSH password
        domain_id: ROS_DOMAIN_ID (optional)
        json_path: Path to JSON file for walking route (optional, for walk mode)
        commands_json: Path to JSON file for car route commands (optional, for car mode)
        ssh_port: SSH port (default: 22)
    """
    domain_export = f"export ROS_DOMAIN_ID={domain_id} && " if domain_id not in (None, "", "default") else ""

    # 명령어 인자 구성
    args = []
    if json_path:
        args.append(f"--json-path {json_path}")
    if commands_json:
        args.append(f"--commands-json {commands_json}")

    args_str = " ".join(args)

    cmd = (
        "cd ~/robot_ws && "
        "source /opt/ros/foxy/setup.bash && "
        "source install/local_setup.bash && "
        f"{domain_export}"
        f"PYTHONPATH=~/robot_ws/src:$PYTHONPATH python3 src/move_full_path.py {args_str}"
    )
    return _run_remote_command(robot_ip, username, password, cmd, ssh_port)


def upload_json_file(robot_ip: str, username: str, password: str, json_data: Dict, remote_path: str) -> Tuple[bool, str, str]:
    """
    Upload JSON data to remote robot via SFTP.

    Args:
        robot_ip: Robot IP address
        username: SSH username
        password: SSH password
        json_data: JSON data to upload (dict)
        remote_path: Remote file path (e.g., "/tmp/walking_route.json")

    Returns:
        (success, stdout, stderr)
    """
    ssh_port = 22  # 항상 22 포트 사용
    ssh = None
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(
                robot_ip,
                port=ssh_port,
                username=username,
                password=password,
                timeout=15,
                allow_agent=False,
                look_for_keys=False,
                banner_timeout=10
            )
        except paramiko.AuthenticationException:
            return False, "", f"인증 실패: 사용자명 '{username}' 또는 비밀번호가 올바르지 않습니다."
        except socket.error as e:
            error_code = getattr(e, 'errno', None)
            error_msg = str(e)
            if error_code == 113 or "No route to host" in error_msg:
                return False, "", f"SFTP 네트워크 연결 실패: {robot_ip}에 도달할 수 없습니다.\n\n오류 코드: {error_code}\n오류: {error_msg}"
            return False, "", f"SFTP 연결 중 네트워크 오류: {error_msg} (코드: {error_code})"
        except Exception as e:
            error_msg = str(e)
            return False, "", f"SFTP 연결 중 오류: {error_msg}"

        sftp = ssh.open_sftp()

        # JSON 문자열 생성
        json_str = json.dumps(json_data, ensure_ascii=False, indent=2)

        # 원격 파일에 쓰기
        with sftp.file(remote_path, 'w') as remote_file:
            remote_file.write(json_str)

        sftp.close()
        return True, f"JSON file uploaded to {remote_path}", ""
    except Exception as e:
        return False, "", f"파일 업로드 중 오류: {str(e)}"
    finally:
        if ssh:
            try:
                ssh.close()
            except:
                pass


def stop_move_full_path(robot_ip: str, username: str, password: str, ssh_port: int = 22) -> Tuple[bool, str, str]:
    """
    Force stop move_full_path.py by terminating the running process.
    """
    cmd = "pkill -f move_full_path.py || true"
    return _run_remote_command(robot_ip, username, password, cmd, ssh_port)


def stop_bringup(robot_ip: str, username: str, password: str, ssh_port: int = 22) -> Tuple[bool, str, str]:
    """
    Force stop turtlebot3_bringup by terminating the running process.
    """
    # 깔끔한 종료를 위해 turtlebot3_ros 노드도 함께 종료
    cmd = """
        pkill -9 -f "turtlebot3_bringup|robot.launch.py|turtlebot3_ros" 2>/dev/null || true;
        rm -f /tmp/turtlebot3_bringup.pid 2>/dev/null || true;
        sleep 2;
        if pgrep -f 'turtlebot3_bringup|robot.launch.py|turtlebot3_ros' > /dev/null 2>&1; then
            echo "stop_failed"
        else
            echo "stop_success"
        fi
    """
    success, out, err = _run_remote_command(robot_ip, username, password, cmd, ssh_port)

    # 종료 확인 결과에 따라 success 결정
    if "stop_success" in (out or ""):
        return True, out, err
    else:
        return False, out, err


def check_bringup_status(robot_ip: str, username: str, password: str, ssh_port: int = 22) -> Tuple[bool, str, str]:
    """
    Check if turtlebot3_bringup is running.
    Returns (is_running, stdout, stderr)
    """
    # pgrep exit code 기반으로 판단
    cmd = "pgrep -f 'turtlebot3_bringup\\|robot.launch.py' > /dev/null 2>&1"
    ssh = None
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(
            robot_ip,
            port=ssh_port,
            username=username,
            password=password,
            timeout=15,
            allow_agent=False,
            look_for_keys=False,
            banner_timeout=10
        )

        stdin, stdout, stderr = ssh.exec_command(cmd, timeout=5)
        # exit_status가 준비될 때까지 대기
        import time
        max_wait = 5
        waited = 0
        while not stdout.channel.exit_status_ready() and waited < max_wait:
            time.sleep(0.1)
            waited += 0.1

        # exit code 확인
        exit_status = stdout.channel.recv_exit_status()
        out = stdout.read().decode().strip()
        err = stderr.read().decode().strip()

        # exit code가 0이면 실행 중, 아니면 미실행
        is_running = (exit_status == 0)
        print(f"[DEBUG] check_bringup_status - exit_status: {exit_status}, is_running: {is_running}, out: {out}, err: {err}")
        return is_running, out, err
    except Exception as e:
        print(f"[ERROR] check_bringup_status 예외: {str(e)}")
        return False, "", f"상태 확인 중 오류: {str(e)}"
    finally:
        if ssh:
            try:
                ssh.close()
            except:
                pass

