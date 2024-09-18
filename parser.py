import struct
import os


def read_vox_file(file_path):
    with open(file_path, 'rb') as f:
        # Read header
        header = f.read(4)
        if header != b'VOX ':
            raise ValueError("Not a valid .vox file")

        version = struct.unpack('<I', f.read(4))[0]

        if version != 200:
            raise ValueError("Not a valid .vox file")

        # Read chunks
        while f.tell() < os.fstat(f.fileno()).st_size:
            chunk_id = f.read(4)
            if len(chunk_id) < 4:
                break
            chunk_size = struct.unpack('<I', f.read(4))[0]
            children_size = struct.unpack('<I', f.read(4))[0]
            chunk_data = f.read(chunk_size)

            if chunk_id == b'SIZE':
                size_x, size_y, size_z = struct.unpack('<III', chunk_data[:12])
            elif chunk_id == b'XYZI':
                num_voxels = struct.unpack('<I', chunk_data[:4])[0]
                voxels = struct.unpack('<' + 'BBBB' * num_voxels, chunk_data[4:])
                voxels = [(voxels[i], voxels[i + 1], voxels[i + 2], voxels[i + 3]) for i in range(0, len(voxels), 4)]

        return {
            'size': (size_x, size_y, size_z),
            'voxels': voxels
        }
