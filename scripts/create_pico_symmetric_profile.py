#!/usr/bin/env python3
"""Create a new calibration directory; never replace original measured artifacts."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vendor/pico_tracker/src/pico_bridge/scripts'))
from pico_symmetric_geometry import create_profile

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:policy=create_profile(args.source,args.output)
    except (OSError,ValueError,KeyError,TypeError) as exc:parser.exit(2,f'{exc}\n')
    print(json.dumps(dict(output=str(args.output.absolute()),**policy),indent=2))

if __name__=='__main__':main()
