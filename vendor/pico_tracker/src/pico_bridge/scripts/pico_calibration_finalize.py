"""Finalize bilateral measured calibration into an opt-in symmetric runtime profile."""
import argparse
import json
import os
from pathlib import Path
import uuid

from pico_symmetric_geometry import FILES, POLICY_FILE, create_profile


def _check_link(link, versions):
    if not link.exists() and not link.is_symlink():
        return
    if (not link.is_symlink() or link.resolve().parent != versions
            or not (link.resolve() / POLICY_FILE).is_file()):
        raise ValueError(f'Refusing to replace an unmanaged runtime path: {link}')


def finalize(source, policy='symmetric_max'):
    source = Path(source).expanduser().resolve(strict=True)
    if (source / POLICY_FILE).exists():
        raise ValueError('Calibrate in the original directory, not a derived runtime profile')
    if policy not in ('original', 'symmetric_max'):
        raise ValueError(f'Unknown geometry policy: {policy}')
    if policy == 'original':
        return {'state': 'original', 'runtime_directory': str(source)}
    missing = [name for name in FILES if not (source / name).is_file()]
    if missing:
        return {'state': 'pending', 'missing': missing,
                'reason': 'Both sides require complete valid calibration; previous selection unchanged'}
    versions = source / 'symmetric_profiles'
    if versions.is_symlink() or (versions.exists() and not versions.is_dir()):
        raise ValueError(f'Refusing non-directory or symlink version store: {versions}')
    link = source / 'runtime_symmetric'
    _check_link(link, versions)
    identifier = uuid.uuid4().hex
    output = versions / identifier
    # The shared generator validates both sides, copies evidence unchanged and
    # only marks its profile complete after all fingerprints match.
    result = create_profile(source, output)
    temporary = source / f'.runtime_symmetric.{identifier}'
    temporary.symlink_to(output.relative_to(source), target_is_directory=True)
    try:
        _check_link(link, versions)
        os.replace(temporary, link)
    finally:
        if temporary.is_symlink():
            temporary.unlink()
    return {'state': 'ready', 'runtime_directory': str(link),
            'version_directory': str(output),
            'effective_lengths_m': result['effective_lengths_m']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--policy', choices=('original', 'symmetric_max'), default='symmetric_max')
    args = parser.parse_args()
    try:
        result = finalize(args.source, args.policy)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'Runtime geometry finalization failed: {exc}\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result['state'] == 'ready':
        print('对称运行配置已生成；遥操使用 --pico-calibration-dir "' + result['runtime_directory'] + '"')
    elif result['state'] == 'pending':
        print('对称化待完成：两侧标定尚未齐全；本侧测量结果已保存。旧派生配置未更新。')
    else:
        print('使用原始独立骨长；旧派生配置未更新，遥操请选择原始目录。')


if __name__ == '__main__':
    main()
