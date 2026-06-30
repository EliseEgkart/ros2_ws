"""
Product and material definitions for RoboCup assembly challenge.

Material IDs:
  2x2: Red=1, Green=2, Blue=3, Yellow=4
  2x4: Red=5, Green=6, Blue=7, Yellow=8

Product IDs encode the material sequence used in the name / assembly diagram.
"""

from collections import Counter
from typing import List, Optional

# Size of each material block
MATERIAL_SIZE: dict = {
    1: '2x2', 2: '2x2', 3: '2x2', 4: '2x2',
    5: '2x4', 6: '2x4', 7: '2x4', 8: '2x4',
}

# Batch ID → raw material ID (IDs 10-80: known type, ≥5 blocks guaranteed)
BATCH_TO_MATERIAL: dict = {
    10: 1, 20: 2, 30: 3, 40: 4,
    50: 5, 60: 6, 70: 7, 80: 8,
}
BATCH_COUNT: int = 5    # treat each batch station as having exactly this many blocks
MIX_BATCH_ID: int = 90  # unknown type/count batch

# All 11 product definitions.
# Single-column products list 'blocks' in bottom-to-top build order.
# Multi-layer products list 'layers' in bottom-to-top order; each
# layer is a list of block IDs placed side-by-side at that level.
# All products are assembled by the AMR cargo arm (cargo 7/8).
# Multi-layer placement requires the arm to handle side-by-side positioning;
# the arm team is responsible for asymmetric placement given the product_id.
PRODUCTS: dict = {
    81: {
        'name': 'E-Stop',
        'blocks': [8, 1],
    },
    34: {
        'name': 'Battery',
        'blocks': [3, 4],
    },
    13: {
        'name': 'Magnet',
        'blocks': [1, 3],
    },
    442: {
        'name': 'Carrot',
        'blocks': [4, 4, 2],
    },
    241: {
        'name': 'Traffic Light',
        'blocks': [2, 4, 1],
    },
    462: {
        'name': 'Small Tree',
        'blocks': [4, 6, 2],
    },
    4482: {
        'name': 'Big Carrot',
        'blocks': [4, 4, 8, 2],
    },
    711: {
        'name': 'Hammer',
        'blocks': [1, 1, 7],
    },
    # --- Multi-layer products: contain side-by-side layers ---
    # Arm team handles side-by-side positioning from product_id alone.
    8518: {
        'name': 'Burger',
        # Bottom: [8], middle: [5, 1] side-by-side, top: [8]
        'layers': [[8], [5, 1], [8]],
    },
    46262: {
        'name': 'Big Tree',
        # Bottom: [4], then [6,2] side-by-side, then [6], top: [2]
        'layers': [[4], [6, 2], [6], [2]],
    },
    48132: {
        'name': 'Ice Cream',
        # Bottom: [4], then [8], then [1,3] side-by-side, top: [2]
        'layers': [[4], [8], [1, 3], [2]],
    },
}


def _is_multilayer(product_id: int) -> bool:
    return 'layers' in PRODUCTS[product_id]


def get_material_count(product_id: int) -> Counter:
    """Frequency map of materials required for one unit of product_id."""
    p = PRODUCTS[product_id]
    if _is_multilayer(product_id):
        materials = [m for layer in p['layers'] for m in layer]
    else:
        materials = p['blocks']
    return Counter(materials)


def is_intransit_eligible(product_id: int) -> bool:
    """All products are assembled in-transit by the AMR cargo arm (cargo 7/8).
    The workbench is used only for RECYCLE (disassembly) operations."""
    return True


def get_base_block(product_id: int) -> int:
    """The bottom-most block for a single-column product."""
    if _is_multilayer(product_id):
        return PRODUCTS[product_id]['layers'][0][0]
    return PRODUCTS[product_id]['blocks'][0]


def get_build_order(product_id: int) -> List[int]:
    """All block IDs in assembly order (bottom → top, left → right per layer).

    For single-column products: returns the blocks list directly.
    For multi-layer products: flattens layers in order.
    This flat list drives material tracking; the arm uses get_assembly_layers()
    for the actual side-by-side placement instructions.
    """
    p = PRODUCTS[product_id]
    if _is_multilayer(product_id):
        return [m for layer in p['layers'] for m in layer]
    return list(p['blocks'])


def get_assembly_layers(product_id: int) -> List[List[int]]:
    """Full layer structure for the arm ASSEMBLE command, bottom → top.

    Single-column products: each layer contains exactly one block.
    Multi-layer products: layers may contain multiple blocks placed side-by-side.
    The arm team uses this to determine positioning for each block.
    """
    p = PRODUCTS[product_id]
    if _is_multilayer(product_id):
        return [list(layer) for layer in p['layers']]
    return [[b] for b in p['blocks']]


def get_all_layers(product_id: int) -> List[List[int]]:
    """Alias for get_assembly_layers(); kept for backward compatibility."""
    return get_assembly_layers(product_id)


def product_name(product_id: int) -> str:
    return PRODUCTS.get(product_id, {}).get('name', f'Unknown({product_id})')


def validate_product_id(product_id: int) -> bool:
    return product_id in PRODUCTS
