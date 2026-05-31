#!/usr/bin/env python3

import argparse
import glob
import os

from tqdm import tqdm


def main():
    parser = argparse.ArgumentParser(description="Write ORB-SLAM timestamp file from Euroc images.")
    parser.add_argument("--base_dir", required=True, help="Input mav0 directory.")
    parser.add_argument("--output", required=True, help="Output timestamp text file.")
    args = parser.parse_args()

    cam0_folder = os.path.join(args.base_dir, "cam0", "data")
    files = sorted(glob.glob(os.path.join(cam0_folder, "*")))

    with open(args.output, "w") as orb_file:
        for file_path in tqdm(files, desc="timestamps"):
            stamp = os.path.splitext(os.path.basename(file_path))[0]
            orb_file.write(f"{stamp}\n")


if __name__ == "__main__":
    main()
