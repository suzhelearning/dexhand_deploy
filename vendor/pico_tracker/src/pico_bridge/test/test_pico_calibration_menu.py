import os
from pathlib import Path
import subprocess

import yaml


REPO_ROOT = Path(__file__).parents[3]
SCRIPT = REPO_ROOT / "scripts" / "calibrate_pico_arm.sh"


def run_script(*args, home=None, input_text=None):
    environment = os.environ.copy()
    environment["PIXI_PROJECT_ROOT"] = str(REPO_ROOT)
    environment["ROS_DOMAIN_ID"] = "120"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    if home is not None:
        environment["HOME"] = str(home)
    return subprocess.run(
        [str(SCRIPT), *args],
        input=input_text,
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )


def test_help_exposes_interactive_and_noninteractive_modes():
    result = run_script("--help")
    assert result.returncode == 0
    assert "交互式菜单" in result.stderr
    assert "<left|right> <tcp|wrist|geometry|all>" in result.stderr


def test_wrist_requires_same_side_tcp_before_starting_ros(tmp_path):
    result = run_script("left", "wrist", home=tmp_path)
    assert result.returncode == 2
    assert "缺少或无效的 left tcp artifact" in result.stderr
    assert "pico_palm_wrist_calibrator" not in result.stderr


def test_geometry_requires_same_side_tcp_and_wrist(tmp_path):
    result = run_script("right", "geometry", home=tmp_path)
    assert result.returncode == 2
    assert "缺少或无效的 right tcp artifact" in result.stderr
    assert "pico_arm_geometry_pixi.sh" not in result.stderr


def test_status_menu_is_read_only_and_reports_both_sides(tmp_path):
    result = run_script("status", home=tmp_path)
    assert result.returncode == 0
    assert "[left]" in result.stdout
    assert "[right]" in result.stdout
    assert not (tmp_path / ".config" / "pico_tracker").exists()


def test_all_dispatches_tcp_before_wrist_before_geometry():
    source = SCRIPT.read_text(encoding="utf-8")
    run_all = source.split("run_all()", 1)[1].split("interactive_menu()", 1)[0]
    assert run_all.index('run_operation "$side" tcp') < run_all.index(
        'run_operation "$side" wrist'
    ) < run_all.index('run_operation "$side" geometry')


def test_single_menu_has_independent_operation_choices():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "1|tcp" in source
    assert "2|wrist" in source
    assert "3|geometry|arm" in source


def test_wrist_calibration_has_exit_trap_for_tcp_publisher_cleanup():
    source = SCRIPT.read_text(encoding="utf-8")
    wrist_section = source.split("run_wrist()", 1)[1].split("run_geometry()", 1)[0]
    assert "trap cleanup_wrist EXIT INT TERM" in wrist_section


def test_wrist_publisher_cleanup_owns_and_bounds_the_whole_process_group():
    source = SCRIPT.read_text(encoding="utf-8")
    wrist_section = source.split("run_wrist()", 1)[1].split("run_geometry()", 1)[0]
    assert "setsid ros2 run pico_bridge pico_palm_tcp_publisher" in wrist_section
    assert "pico_stop_process_group" in wrist_section


def test_wrist_calibration_collects_a_multi_second_window():
    source = SCRIPT.read_text(encoding="utf-8")
    wrist_section = source.split("run_wrist()", 1)[1].split("run_geometry()", 1)[0]
    assert "--sample-count 450" in wrist_section


def test_wrist_calibration_fails_fast_when_temporary_publisher_dies():
    source = SCRIPT.read_text(encoding="utf-8")
    wrist_section = source.split("run_wrist()", 1)[1].split("run_geometry()", 1)[0]

    assert 'kill -0 -- "-$publisher_pid"' in wrist_section
    assert "临时 TCP publisher 启动失败" in wrist_section
    assert wrist_section.index("临时 TCP publisher 启动失败") < wrist_section.index(
        "pico_palm_wrist_calibrator"
    )


def test_tcp_and_wrist_write_candidates_before_atomic_activation():
    source = SCRIPT.read_text(encoding="utf-8")
    tcp_section = source.split("run_tcp()", 1)[1].split("run_wrist()", 1)[0]
    wrist_section = source.split("run_wrist()", 1)[1].split("run_geometry()", 1)[0]

    assert '--output "$output"' not in tcp_section
    assert '--output "$wrist"' not in wrist_section
    assert 'activate_calibration_candidate "$side" tcp "$candidate"' in tcp_section
    assert 'activate_calibration_candidate "$side" wrist "$candidate"' in wrist_section


def test_tcp_prompt_separates_position_samples_from_orientation_sample():
    source = SCRIPT.read_text(encoding="utf-8")
    tcp_section = source.split("run_tcp()", 1)[1].split("run_wrist()", 1)[0]

    assert "第 1～4 次空格只标定 TCP 位置" in tcp_section
    assert "第 5 次空格启动所选侧 TCP 姿态采集" in tcp_section
    assert "保持当前姿势约 1～2 秒" in tcp_section
    assert "双臂向正前方水平伸直、左右掌心相对" in tcp_section
    assert "第 4 次同时完成位置和姿态" not in tcp_section


def test_interactive_single_failure_returns_to_menu_instead_of_exiting():
    source = SCRIPT.read_text(encoding="utf-8")
    menu = source.split("interactive_menu()", 1)[1].split("if (($# == 0))", 1)[0]
    assert 'run_operation "$side" "$operation" || return' not in menu
    assert "标定失败，可修正后重新选择" in menu


def test_direct_geometry_success_explicitly_reports_command_exit():
    source = SCRIPT.read_text(encoding="utf-8")
    dispatch = source.split('case "$operation" in', 1)[1]

    assert "骨长标定命令已结束，已返回终端" in dispatch


def test_status_rejects_geometry_that_only_has_shallow_acceptance_fields(tmp_path):
    config = tmp_path / ".config" / "pico_tracker"
    config.mkdir(parents=True)
    (config / "pico_left_arm_geometry.yaml").write_text(
        yaml.safe_dump(
            {
                "artifact_type": "pico_left_arm_geometry_quick_v3",
                "schema_version": 3,
                "valid": True,
                "candidate_status": "accepted",
                "side": "left",
                "calibration_revision": 1,
                "upper_arm_length_m": 0.30,
                "forearm_length_m": 0.25,
            }
        ),
        encoding="utf-8",
    )

    result = run_script("status", home=tmp_path)

    assert result.returncode == 0
    left_block = result.stdout.split("[right]", 1)[0]
    assert "✗ geometry：尚未完成或文件无效" in left_block


def test_menu_delegates_all_validation_to_shared_python_contract():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "pico_calibration_artifact.py" in source
    assert "covariance_upper_triangle_8x8" not in source
    assert "quality_limits" not in source


def test_geometry_uses_space_start_and_interactive_result_menu():
    source = SCRIPT.read_text(encoding="utf-8")
    geometry = source.split("run_geometry()", 1)[1].split("show_one_status()", 1)[0]
    assert '--start-mode space' in geometry
    assert "1) 返回主菜单" in source
    assert "2) 重新进行当前骨长标定" in source
    assert "3) 退出" in source
    assert 'run_operation "$side" geometry' in source


def test_rejected_geometry_returns_failure_and_is_never_reported_as_completed():
    source = SCRIPT.read_text(encoding="utf-8")
    geometry = source.split("run_geometry()", 1)[1].split("show_one_status()", 1)[0]
    assert 'if ! "$repo_root/src/pico_bridge/scripts/pico_arm_geometry_pixi.sh"' in geometry
    assert 'activate_calibration_candidate "$side" geometry "$candidate"' in geometry
    assert "geometry artifact:" not in geometry
    assert 'if [[ ! -f "$candidate" ]]' in geometry
    assert "✗ $side 上臂/前臂骨长标定未通过" not in geometry


def test_default_output_is_concise_and_verbose_is_opt_in():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "--verbose" in source
    assert 'PICO_CALIBRATION_VERBOSE' in source
    assert '标定结果文件' in source


def test_invalid_status_explains_short_wrist_distance_in_plain_chinese():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "掌心到手腕距离过短" in source
    assert "请重新进行腕部标定" in source
