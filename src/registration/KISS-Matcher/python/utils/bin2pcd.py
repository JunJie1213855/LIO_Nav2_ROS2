import struct
import sys

import numpy as np
from kiss_matcher.io_utils import write_pcd


def bin_to_pcd(bin_file_name):
    size_float = 4
    list_pcd = []
    with open(bin_file_name, "rb") as file:
        byte = file.read(size_float * 4)
        while byte:
            x, y, z, _ = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = file.read(size_float * 4)
    return np.asarray(list_pcd)


def main(binFileName, pcd_file_name):
    points = bin_to_pcd(binFileName)
    write_pcd(points, pcd_file_name)


if __name__ == "__main__":
    a = sys.argv[1]
    b = sys.argv[2]
    main(a, b)
