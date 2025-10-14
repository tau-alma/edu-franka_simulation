#!/usr/bin/env python3
import argparse, cv2, numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True, help="ArUco marker ID")
    ap.add_argument("--dict", type=str, default="DICT_4X4_50", help="OpenCV ArUco dictionary name")
    ap.add_argument("--pixels", type=int, default=1000, help="output image size (square)")
    ap.add_argument("--outfile", type=str, default="aruco.png", help="output PNG path")
    args = ap.parse_args()

    # Resolve dictionary
    if not hasattr(cv2.aruco, args.dict):
        raise ValueError(f"Unknown dictionary {args.dict}")
    dict_id = getattr(cv2.aruco, args.dict)
    adict = cv2.aruco.getPredefinedDictionary(dict_id)

    # Draw marker (black border is included automatically)
    img = cv2.aruco.generateImageMarker(adict, args.id, args.pixels)
    # ensure white background outside the border area (already white)
    cv2.imwrite(args.outfile, img)

    print(f"Saved {args.outfile} (id={args.id}, dict={args.dict}, {args.pixels}x{args.pixels})")

if __name__ == "__main__":
    main()
