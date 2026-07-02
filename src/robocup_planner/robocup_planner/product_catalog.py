"""
Product and material definitions for RoboCup assembly challenge.

Material IDs:
  2x2: Red=1, Green=2, Blue=3, Yellow=4
  2x4: Red=5, Green=6, Blue=7, Yellow=8

Product IDs encode the material sequence used in the name / assembly diagram.

All products are assembled by the single AMR cargo manipulator, in-transit,
on cargo slots 7/8 (Mod 3 — the workbench manipulator is recycle-only). Every
product therefore defines 'blocks': the flattened bottom-to-top build order
used for in-transit assembly. Products with side-by-side layers (asymmetric
placement) additionally define 'layers': the same blocks grouped by height
level, which the arm team uses to resolve left/right placement within a
layer. flatten(layers) == blocks always holds; this is asserted at import
time below.
"""

from collections import Counter
from typing import List

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
# 'blocks'  — bottom-to-top build order, single column (used by every
#             product for in-transit assembly and material counting).
# 'layers'  — optional; only present for products with side-by-side blocks
#             at one or more height levels. Bottom-to-top; each inner list
#             is the block IDs placed side-by-side at that level. Purely a
#             placement hint for the arm team — it does not gate eligibility.
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
    # --- Side-by-side (asymmetric) layers ---
    8518: {
        'name': 'Burger',
        # Bottom layer: [8], middle: [5, 1] side-by-side, top: [8]
        'layers': [[8], [5, 1], [8]],
        'blocks': [8, 5, 1, 8],
    },
    46262: {
        'name': 'Big Tree',
        # Bottom: [4], then [6,2] side-by-side, then [6], top: [2]
        'layers': [[4], [6, 2], [6], [2]],
        'blocks': [4, 6, 2, 6, 2],
    },
    48132: {
        'name': 'Ice Cream',
        # Bottom: [4], then [8], then [1,3] side-by-side, top: [2]
        'layers': [[4], [8], [1, 3], [2]],
        'blocks': [4, 8, 1, 3, 2],
    },
}


def _validate_catalog() -> None:
    for pid, p in PRODUCTS.items():
        layers = p.get('layers')
        if layers is not None:
            flattened = [b for layer in layers for b in layer]
            if flattened != p['blocks']:
                raise ValueError(
                    f"Product {pid} ({p['name']}): flatten(layers)={flattened} "
                    f"!= blocks={p['blocks']}"
                )


_validate_catalog()


def get_material_count(product_id: int) -> Counter:
    """Frequency map of materials required for one unit of product_id."""
    return Counter(PRODUCTS[product_id]['blocks'])


def is_intransit_eligible(product_id: int) -> bool:
    """True if product_id can be assembled in-transit by the AMR cargo arm.

    All catalog products are eligible (Mod 3: single AMR manipulator handles
    every product, including side-by-side layers). Returns False only for
    unknown product_ids.
    """
    return product_id in PRODUCTS


def get_base_block(product_id: int) -> int:
    """The bottom-most block for a product."""
    return PRODUCTS[product_id]['blocks'][0]


def get_build_order(product_id: int) -> List[int]:
    """Block IDs in assembly order (bottom → top) for in-transit assembly."""
    return list(PRODUCTS[product_id]['blocks'])


def get_all_layers(product_id: int) -> List[List[int]]:
    """All layers for product_id, bottom → top.

    Products without an explicit 'layers' entry are single-block-per-level,
    so each layer is just [block_id].
    """
    p = PRODUCTS[product_id]
    layers = p.get('layers')
    if layers is None:
        return [[b] for b in p['blocks']]
    return [list(layer) for layer in layers]


def has_side_by_side_layers(product_id: int) -> bool:
    """True if any layer of product_id contains more than one block
    (i.e. the arm must resolve left/right placement, not just stack height)."""
    return any(len(layer) > 1 for layer in get_all_layers(product_id))


def product_name(product_id: int) -> str:
    return PRODUCTS.get(product_id, {}).get('name', f'Unknown({product_id})')


def validate_product_id(product_id: int) -> bool:
    return product_id in PRODUCTS
