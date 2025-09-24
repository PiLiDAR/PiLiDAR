from pye57 import libe57
from pye57.libe57 import NodeType

import numpy as np

def get_fields(node):
    return [node.get(id_).elementName() for id_ in range(node.childCount())]


def get_node(node, name):
    cast = {
        NodeType.E57_BLOB: libe57.BlobNode,
        NodeType.E57_COMPRESSED_VECTOR: libe57.CompressedVectorNode,
        NodeType.E57_FLOAT: libe57.FloatNode,
        NodeType.E57_INTEGER: libe57.IntegerNode,
        NodeType.E57_SCALED_INTEGER: libe57.ScaledIntegerNode,
        NodeType.E57_STRING: libe57.StringNode,
        NodeType.E57_STRUCTURE: libe57.StructureNode,
        NodeType.E57_VECTOR: libe57.VectorNode
    }
    n = node.get(name)
    return cast[n.type()](n)

def convert_spherical_to_cartesian(rae):
    """
    Converts spherical(rae) to cartesian(xyz), where rae = range, azimuth(theta), 
    elevation(phi). Where range is in meters and angles are in radians.
    
    Reference for formula: http://www.libe57.org/bestCoordinates.html (Note: the 
    formula is different from the one online, so please use formula at the above reference)
    """
    range_ = rae[:, :1]
    theta = rae[:, 1:2]
    phi =  rae[:, 2:3]
    range_cos_phi = range_ * np.cos(phi)
    return np.concatenate((
        range_cos_phi * np.cos(theta),
        range_cos_phi * np.sin(theta),
        range_ * np.sin(phi)
    ), axis=1)


def copy_node(node, dest_image):
    compressed_node_pairs = []
    blob_node_pairs = []

    out_node = None
    # 'Element' Types
    if (isinstance(node, libe57.FloatNode)):
        out_node = libe57.FloatNode(
            dest_image,
            value=node.value(),
            precision=node.precision(),
            minimum=node.minimum(),
            maximum=node.maximum())

    elif (isinstance(node, libe57.IntegerNode)):
        out_node = libe57.IntegerNode(
            dest_image,
            value=node.value(),
            minimum=node.minimum(),
            maximum=node.maximum())

    elif (isinstance(node, libe57.ScaledIntegerNode)):
        out_node = libe57.ScaledIntegerNode(
            dest_image,
            node.rawValue(),
            minimum=node.minimum(),
            maximum=node.maximum(),
            scale=node.scale(),
            offset=node.offset())

    elif (isinstance(node, libe57.StringNode)):
        out_node = libe57.StringNode(
            dest_image,
            node.value())

    elif (isinstance(node, libe57.BlobNode)):
        out_node = libe57.BlobNode(dest_image, node.byteCount())
        blob_node_pairs.append({ 'in': node, 'out': out_node })

    # 'Container' Types
    elif (isinstance(node, libe57.CompressedVectorNode)):
        in_prototype = libe57.StructureNode(node.prototype())
        out_prototype, _, _  = copy_node(in_prototype, dest_image)
        out_codecs, _, _ = copy_node(node.codecs(), dest_image)

        out_node = libe57.CompressedVectorNode(dest_image, out_prototype, out_codecs)

        compressed_node_pairs.append({
            'in': node,
            'out': out_node
        })

    elif isinstance(node, libe57.StructureNode):
        out_node = libe57.StructureNode(dest_image)
        for i in range(node.childCount()):
            in_child = get_node(node, i)
            in_child_name = in_child.elementName()
            out_child, out_child_compressed_node_pairs, out_child_blob_node_pairs = copy_node(in_child, dest_image)

            out_node.set(in_child_name, out_child)
            compressed_node_pairs.extend(out_child_compressed_node_pairs)
            blob_node_pairs.extend(out_child_blob_node_pairs)

    elif isinstance(node, libe57.VectorNode):
        out_node = libe57.VectorNode(dest_image, allowHeteroChildren=node.allowHeteroChildren())
        for i in range(node.childCount()):
            in_child = get_node(node, i)
            in_child_name = f'{i}'
            out_child, out_child_compressed_node_pairs, out_child_blob_node_pairs = copy_node(in_child, dest_image)

            out_node.append(out_child)
            compressed_node_pairs.extend(out_child_compressed_node_pairs)
            blob_node_pairs.extend(out_child_blob_node_pairs)

    return out_node, compressed_node_pairs, blob_node_pairs