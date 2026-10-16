"""
Lightweight I/O utilities for point cloud files without heavy dependencies.
Replaces Open3D for basic file operations.
"""
import struct
import numpy as np


def read_pcd(pcd_path):
    """
    Read PCD file without Open3D dependency.
    Supports ASCII and binary PCD formats.
    """
    # First, read the header as text to determine format
    header_info = {}
    header_lines = []

    with open(pcd_path, 'rb') as f:
        while True:
            line = f.readline()
            if isinstance(line, bytes):
                try:
                    line_str = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    # If we can't decode, we've hit the binary data
                    break
            else:
                line_str = line.strip()

            header_lines.append(line_str)

            if line_str.startswith('VERSION'):
                header_info['version'] = line_str.split()[1]
            elif line_str.startswith('FIELDS'):
                header_info['fields'] = line_str.split()[1:]
            elif line_str.startswith('SIZE'):
                header_info['size'] = [int(x) for x in line_str.split()[1:]]
            elif line_str.startswith('TYPE'):
                header_info['type'] = line_str.split()[1:]
            elif line_str.startswith('COUNT'):
                header_info['count'] = [int(x) for x in line_str.split()[1:]]
            elif line_str.startswith('WIDTH'):
                header_info['width'] = int(line_str.split()[1])
            elif line_str.startswith('HEIGHT'):
                header_info['height'] = int(line_str.split()[1])
            elif line_str.startswith('VIEWPOINT'):
                header_info['viewpoint'] = [float(x) for x in line_str.split()[1:]]
            elif line_str.startswith('POINTS'):
                header_info['points'] = int(line_str.split()[1])
            elif line_str.startswith('DATA'):
                header_info['data'] = line_str.split()[1]
                break

    # Calculate header size in bytes
    header_size = sum(len(line.encode('utf-8')) + 1 for line in header_lines)

    # Read point data
    if header_info.get('data', '').upper() == 'ASCII':
        # ASCII format
        with open(pcd_path, 'r') as f:
            lines = f.readlines()
            # Find data start
            data_start_idx = 0
            for i, line in enumerate(lines):
                if line.strip().startswith('DATA'):
                    data_start_idx = i + 1
                    break

            points = []
            for line in lines[data_start_idx:]:
                line = line.strip()
                if line:
                    values = [float(x) for x in line.split()]
                    # Extract x, y, z (first 3 coordinates)
                    if len(values) >= 3:
                        points.append(values[:3])
            return np.array(points)

    elif header_info.get('data', '').upper() == 'BINARY':
        # Binary format
        with open(pcd_path, 'rb') as f:
            # Skip header
            f.seek(header_size)

            points = []
            num_points = header_info.get('points', 0)
            fields = header_info.get('fields', [])

            # Find x, y, z indices
            x_idx = fields.index('x') if 'x' in fields else 0
            y_idx = fields.index('y') if 'y' in fields else 1
            z_idx = fields.index('z') if 'z' in fields else 2

            # Read binary data
            for _ in range(num_points):
                field_values = []
                # Read all fields for this point
                for field in fields:
                    try:
                        value = struct.unpack('<f', f.read(4))[0]  # little endian float32
                        field_values.append(value)
                    except:
                        break

                if len(field_values) >= 3:
                    x = field_values[x_idx]
                    y = field_values[y_idx]
                    z = field_values[z_idx]
                    points.append([x, y, z])

            return np.array(points)

    else:
        raise ValueError(f"Unsupported PCD data format: {header_info.get('data', 'unknown')}")


def read_bin(bin_path):
    """
    Read KITTI-style binary point cloud file.
    """
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    return scan[:, :3]  # Return only x, y, z


def read_ply(ply_path):
    """
    Read PLY file without Open3D dependency.
    Supports ASCII and binary PLY formats.
    Only extracts x, y, z coordinates (ignores normals and other properties).
    """
    with open(ply_path, 'rb') as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline()
            if isinstance(line, bytes):
                line = line.decode('utf-8')
            line = line.strip()
            header_lines.append(line)

            if line == 'end_header':
                break

        # Parse header information
        vertex_count = 0
        format_type = 'ascii'
        properties = []

        for line in header_lines:
            if line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
            elif line.startswith('format'):
                format_type = line.split()[1]
            elif line.startswith('property float') or line.startswith('property double'):
                prop_name = line.split()[-1]
                properties.append(prop_name)

        # Find indices of x, y, z coordinates
        x_idx = properties.index('x') if 'x' in properties else None
        y_idx = properties.index('y') if 'y' in properties else None
        z_idx = properties.index('z') if 'z' in properties else None

        if x_idx is None or y_idx is None or z_idx is None:
            raise ValueError("PLY file must contain x, y, z coordinates")

        points = []

        if format_type == 'ascii':
            # ASCII format
            for _ in range(vertex_count):
                line = f.readline()
                if isinstance(line, bytes):
                    line = line.decode('utf-8')
                values = line.strip().split()
                if len(values) >= len(properties):
                    x = float(values[x_idx])
                    y = float(values[y_idx])
                    z = float(values[z_idx])
                    points.append([x, y, z])

        elif format_type == 'binary_little_endian':
            # Binary little endian format
            for _ in range(vertex_count):
                vertex_data = []
                for prop in properties:
                    # Assuming float32 for coordinates
                    value = struct.unpack('<f', f.read(4))[0]
                    vertex_data.append(value)

                x = vertex_data[x_idx]
                y = vertex_data[y_idx]
                z = vertex_data[z_idx]
                points.append([x, y, z])

        elif format_type == 'binary_big_endian':
            # Binary big endian format
            for _ in range(vertex_count):
                vertex_data = []
                for prop in properties:
                    # Assuming float32 for coordinates
                    value = struct.unpack('>f', f.read(4))[0]
                    vertex_data.append(value)

                x = vertex_data[x_idx]
                y = vertex_data[y_idx]
                z = vertex_data[z_idx]
                points.append([x, y, z])

        else:
            raise ValueError(f"Unsupported PLY format: {format_type}")

        return np.array(points)


def write_pcd(points, pcd_path):
    """
    Write points to PCD file in ASCII format.

    Args:
        points: numpy array of shape (N, 3) with x, y, z coordinates
        pcd_path: output file path
    """
    num_points = points.shape[0]

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""

    with open(pcd_path, 'w') as f:
        f.write(header)
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")