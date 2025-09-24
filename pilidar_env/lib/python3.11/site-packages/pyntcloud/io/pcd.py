import re
import struct
import warnings

import numpy as np
import pandas as pd

numpy_pcd_type_mappings = [(np.dtype('float32'), ('F', 4)),
                           (np.dtype('float64'), ('F', 8)),
                           (np.dtype('uint8'), ('U', 1)),
                           (np.dtype('uint16'), ('U', 2)),
                           (np.dtype('uint32'), ('U', 4)),
                           (np.dtype('uint64'), ('U', 8)),
                           (np.dtype('int16'), ('I', 2)),
                           (np.dtype('int32'), ('I', 4)),
                           (np.dtype('int64'), ('I', 8))]
numpy_type_to_pcd_type = dict(numpy_pcd_type_mappings)
pcd_type_to_numpy_type = dict((q, p) for (p, q) in numpy_pcd_type_mappings)


def parse_header(lines):
    metadata = {}
    for ln in lines:
        if ln.startswith('#') or len(ln) < 2:
            continue
        match = re.match(r'(\w+)\s+([\w\s\.]+)', ln)
        if not match:
            warnings.warn("warning: can't understand line: %s" % ln)
            continue
        key, value = match.group(1).lower(), match.group(2)
        if key == 'version':
            metadata[key] = value
        elif key in ('fields', 'type'):
            metadata[key] = value.split()
        elif key in ('size', 'count'):
            metadata[key] = map(int, value.split())
        elif key in ('width', 'height', 'points'):
            metadata[key] = int(value)
        elif key == 'viewpoint':
            metadata[key] = map(float, value.split())
        elif key == 'data':
            metadata[key] = value.strip().lower()
        # TODO apparently count is not required?
    # add some reasonable defaults
    if 'count' not in metadata:
        metadata['count'] = [1] * len(metadata['fields'])
    if 'viewpoint' not in metadata:
        metadata['viewpoint'] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    if 'version' not in metadata:
        metadata['version'] = '.7'
    return metadata


def build_dtype(metadata):
    """ build numpy structured array dtype from pcl metadata.
    note that fields with count > 1 are 'flattened' by creating multiple
    single-count fields.
    TODO: allow 'proper' multi-count fields.
    """
    fieldnames = []
    typenames = []
    for f, c, t, s in zip(metadata['fields'],
                          metadata['count'],
                          metadata['type'],
                          metadata['size']):
        np_type = pcd_type_to_numpy_type[(t, s)]
        if c == 1:
            fieldnames.append(f)
            typenames.append(np_type)
        else:
            fieldnames.extend(['%s_%04d' % (f, i) for i in range(c)])
            typenames.extend([np_type] * c)

    dtype = np.dtype(list(zip(fieldnames, typenames)))
    return dtype


def read_pcd(filename):
    """ Reads and pcd file and return the elements as pandas Dataframes.

    Parameters
    ----------
    filename: str
        Path to the pcd file.

    Returns
    -------
    pandas Dataframe.

    """
    data = {}
    with open(filename, 'rb') as f:
        header = []
        while True:
            ln = f.readline().strip().decode()
            header.append(ln)
            if ln.startswith('DATA'):
                metadata = parse_header(header)
                dtype = build_dtype(metadata)
                break

        if metadata['data'] == 'ascii':
            pc_data = np.loadtxt(f, dtype=dtype, delimiter=' ')

        elif metadata['data'] == 'binary':
            rowstep = metadata['points'] * dtype.itemsize
            # for some reason pcl adds empty space at the end of files
            buf = f.read(rowstep)

            pc_data = np.fromstring(buf, dtype=dtype)

        elif metadata['data'] == 'binary_compressed':
            raise NotImplementedError("Go ask PCD why they use lzf compression.")
            # compressed size of data (uint32)
            # uncompressed size of data (uint32)
            # compressed data
            # junk
            fmt = 'II'
            compressed_size, uncompressed_size =\
                struct.unpack(fmt, f.read(struct.calcsize(fmt)))
            compressed_data = f.read(compressed_size)
            # TODO what to use as second argument? if buf is None
            # (compressed > uncompressed)
            # should we read buf as raw binary?
            #buf = lzf.decompress(compressed_data, uncompressed_size)
            if len(buf) != uncompressed_size:
                raise Exception('Error decompressing data')
            # the data is stored field-by-field
            pc_data = np.zeros(metadata['width'], dtype=dtype)
            ix = 0
            for dti in range(len(dtype)):
                dt = dtype[dti]
                bytes = dt.itemsize * metadata['width']
                column = np.fromstring(buf[ix:(ix + bytes)], dt)
                pc_data[dtype.names[dti]] = column
                ix += bytes

    df = pd.DataFrame(pc_data)

    # check if dataframe contains color info
    col = 'rgb'
    if col in df.columns:
        # get the 'rgb' column from dataframe
        packed_rgb = df.rgb.values
        # 'rgb' values are stored as float
        # treat them as int
        packed_rgb = packed_rgb.astype(np.float32).tostring()
        packed_rgb = np.frombuffer(packed_rgb, dtype=np.int32)
        # unpack 'rgb' into 'red', 'green' and 'blue' channel
        df['red'] = np.asarray((packed_rgb >> 16) & 255, dtype=np.uint8)
        df['green'] = np.asarray((packed_rgb >> 8) & 255, dtype=np.uint8)
        df['blue'] = np.asarray(packed_rgb & 255, dtype=np.uint8)
        # remove packed rgb since we don't need it anymore
        df.drop(col, axis=1, inplace=True)

    data['points'] = df
    return data
