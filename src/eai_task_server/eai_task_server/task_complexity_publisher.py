#!/usr/bin/env python3
"""
Task Complexity compliant interactive task publisher for SML World Cup 2026 layout.

Actual event mode: Beginner / Advanced only.

This node extends the simple example publisher by adding:
  1) Tier selection
  2) Generation mode selection: manual / semi-auto / auto
  3) Stage-aware Task Complexity validation
  4) Automatic arena_layout generation that contains the required raw materials
     and returned products
  5) World Cup 2026 station numbering support

Expected message package is the same as the provided example:
    from sml_messages.msg import Order, Station, Task

World Cup 2026 station mapping used by this file:
    01 side_a_storage_1
    02 side_a_storage_2
    03 side_a_hybrid_1
    04 side_a_workbench_1
    05 side_a_workbench_2
    06 side_a_customer_1
    07 shared_storage_1
    08 side_b_customer_1
    09 side_b_workbench_1
    10 side_b_workbench_2
    11 side_b_hybrid_1
    12 side_b_storage_1
    13 side_b_storage_2
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import random
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sml_messages.msg import Order, Station, Task


# -----------------------------------------------------------------------------
# Object ID table
# -----------------------------------------------------------------------------

# Auto generation places only single raw-material IDs (1~8) in storage.
# Batch IDs (10~90) are intentionally not generated.
RAW_MATERIALS: Dict[int, str] = {
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
}

BEGINNER_PRODUCTS: Dict[int, str] = {
    34: "Battery",
    13: "Magnet",
    81: "E-Stop",
    442: "Carrot",
    241: "Traffic Light",
    462: "Small Tree",
    711: "Hammer",
}

ADVANCED_PRODUCTS: Dict[int, str] = {
    4482: "Big Carrot",
    8518: "Burger",
    48132: "Ice Cream",
    46262: "Big Tree",
}

ALL_PRODUCTS: Dict[int, str] = {**BEGINNER_PRODUCTS, **ADVANCED_PRODUCTS}


# -----------------------------------------------------------------------------
# Task Complexity table
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class ComplexityRule:
    tier: str
    stage: str
    time_min: int
    produce_count: int
    recycle_count: int
    raw_min: int
    raw_max: int
    product_min: int
    product_max: int
    arena: str
    fleet_min: int
    fleet_max: int

    @property
    def target_product_count(self) -> int:
        return self.produce_count + self.recycle_count


COMPLEXITY: Dict[Tuple[str, str], ComplexityRule] = {
    # Actual World Cup 2026 operation scope: Beginner and Advanced only.
    ("beginner", "production"): ComplexityRule("beginner", "production", 5, 2, 0, 4, 6, 2, 2, "A / B", 1, 3),
    ("beginner", "recycling"): ComplexityRule("beginner", "recycling", 5, 0, 2, 4, 6, 2, 2, "A / B", 1, 3),
    ("beginner", "lifecycle"): ComplexityRule("beginner", "lifecycle", 10, 3, 2, 7, 13, 4, 6, "A / B", 1, 3),

    ("advanced", "production"): ComplexityRule("advanced", "production", 10, 5, 0, 7, 13, 4, 6, "A / B", 1, 6),
    ("advanced", "recycling"): ComplexityRule("advanced", "recycling", 10, 0, 5, 7, 13, 4, 6, "A / B", 1, 6),
    ("advanced", "lifecycle"): ComplexityRule("advanced", "lifecycle", 15, 5, 5, 12, 28, 8, 12, "A / B", 1, 6),
}

TIERS = ["beginner", "advanced"]
MODES = ["manual", "semi", "auto"]
STAGES = ["production", "recycling", "lifecycle"]


# -----------------------------------------------------------------------------
# World Cup 2026 arena layout
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class StationSpec:
    side: str
    station_type: int
    name: str
    station_id: int


WORLDCUP_2026_STATIONS: List[StationSpec] = [
    # Side A
    StationSpec("side_a", Station.ST_STORAGE, "side_a_storage_1", 1),
    StationSpec("side_a", Station.ST_STORAGE, "side_a_storage_2", 2),
    StationSpec("side_a", Station.ST_HYBRID, "side_a_hybrid_1", 3),
    StationSpec("side_a", Station.ST_WORKBENCH, "side_a_workbench_1", 4),
    StationSpec("side_a", Station.ST_WORKBENCH, "side_a_workbench_2", 5),
    StationSpec("side_a", Station.ST_CUSTOMER, "side_a_customer_1", 6),

    # Center / shared warehouse shelf
    StationSpec("shared", Station.ST_STORAGE, "shared_storage_1", 7),

    # Side B
    StationSpec("side_b", Station.ST_CUSTOMER, "side_b_customer_1", 8),
    StationSpec("side_b", Station.ST_WORKBENCH, "side_b_workbench_1", 9),
    StationSpec("side_b", Station.ST_WORKBENCH, "side_b_workbench_2", 10),
    StationSpec("side_b", Station.ST_HYBRID, "side_b_hybrid_1", 11),
    StationSpec("side_b", Station.ST_STORAGE, "side_b_storage_1", 12),
    StationSpec("side_b", Station.ST_STORAGE, "side_b_storage_2", 13),
]


def selected_station_specs(selected_sides: Sequence[str]) -> List[StationSpec]:
    selected = set(selected_sides)
    return [spec for spec in WORLDCUP_2026_STATIONS if spec.side in selected or spec.side == "shared"]


def storage_station_names(selected_sides: Sequence[str]) -> List[str]:
    selected = set(selected_sides)
    if selected == {"side_a"}:
        return ["side_a_storage_1", "side_a_storage_2", "shared_storage_1"]
    if selected == {"side_b"}:
        return ["side_b_storage_1", "side_b_storage_2", "shared_storage_1"]
    return [
        "side_a_storage_1",
        "side_a_storage_2",
        "shared_storage_1",
        "side_b_storage_1",
        "side_b_storage_2",
    ]


def customer_station_names(selected_sides: Sequence[str]) -> List[str]:
    selected = set(selected_sides)
    if selected == {"side_a"}:
        return ["side_a_customer_1"]
    if selected == {"side_b"}:
        return ["side_b_customer_1"]
    return ["side_a_customer_1", "side_b_customer_1"]


MIRRORED_STORAGE_PAIRS: Tuple[Tuple[str, str], ...] = (
    ("side_a_storage_1", "side_b_storage_1"),
    ("side_a_storage_2", "side_b_storage_2"),
)


# -----------------------------------------------------------------------------
# Message helpers
# -----------------------------------------------------------------------------

def make_order(order_type: int, name: str, product_id: int) -> Order:
    order = Order()
    order.order_type = order_type
    order.name = name
    order.product_id = product_id
    return order


def make_station(station_type: int, name: str, station_id: int, material_ids: List[int]) -> Station:
    station = Station()
    station.station_type = station_type
    station.name = name
    station.station_id = station_id
    station.material_ids = material_ids
    return station


def split_evenly(items: Sequence[int], bucket_count: int) -> List[List[int]]:
    buckets: List[List[int]] = [[] for _ in range(bucket_count)]
    for index, item in enumerate(items):
        buckets[index % bucket_count].append(item)
    return buckets


def distribute_to_stations(items: Sequence[int], station_names: Sequence[str]) -> Dict[str, List[int]]:
    buckets = split_evenly(items, len(station_names))
    return {name: list(bucket) for name, bucket in zip(station_names, buckets)}


def build_fair_material_distribution(
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    rng: random.Random,
) -> Dict[str, List[int]]:
    """Create identical material conditions for side A and side B."""
    storage_items = needed_raw_materials(produce_ids)
    customer_items = list(recycle_ids)
    rng.shuffle(storage_items)
    rng.shuffle(customer_items)

    # The shared storage is visible to both sides. Distribute once on side A,
    # then mirror only the side-specific storage shelves to side B. Hybrid
    # stations never receive raw materials.
    materials_by_station = distribute_to_stations(
        storage_items,
        storage_station_names(["side_a"]),
    )
    for side_a_name, side_b_name in MIRRORED_STORAGE_PAIRS:
        materials_by_station[side_b_name] = list(materials_by_station.get(side_a_name, []))

    materials_by_station["side_a_customer_1"] = list(customer_items)
    materials_by_station["side_b_customer_1"] = list(customer_items)
    return materials_by_station


def build_arena_layout(
    selected_sides: Sequence[str],
    materials_by_station: Dict[str, List[int]],
) -> List[Station]:
    layout: List[Station] = []
    for spec in selected_station_specs(selected_sides):
        material_ids = list(materials_by_station.get(spec.name, []))
        # Competition rule: workbenches and hybrid stations start without
        # materials. Raw materials may only be placed in storage stations.
        if spec.station_type in (Station.ST_WORKBENCH, Station.ST_HYBRID):
            material_ids = []
        layout.append(make_station(spec.station_type, spec.name, spec.station_id, material_ids))
    return layout


def fill_task(
    orders: List[Order],
    selected_sides: Sequence[str],
    materials_by_station: Dict[str, List[int]],
) -> Task:
    task = Task()
    task.order_list = orders
    task.arena_layout = build_arena_layout(selected_sides, materials_by_station)
    return task


def build_side_only_task(task: Task, side: str) -> Task:
    """Build side-specific task.

    If the requested side does not exist in the full task, publish an empty side
    task instead of sending orders with only the shared station. This prevents the
    inactive side planner from receiving impossible orders.
    """
    side_task = Task()
    side_prefix = f"{side}_"
    side_stations = [s for s in task.arena_layout if s.name.startswith(side_prefix)]
    if not side_stations:
        side_task.order_list = []
        side_task.arena_layout = []
        return side_task

    side_task.order_list = list(task.order_list)
    side_task.arena_layout = [
        s for s in task.arena_layout
        if s.name.startswith(side_prefix) or s.name.startswith("shared_")
    ]
    return side_task


# -----------------------------------------------------------------------------
# Task generation helpers
# -----------------------------------------------------------------------------

def decompose_product(product_id: int) -> List[int]:
    return [int(digit) for digit in str(product_id)]


def product_name(product_id: int) -> str:
    return ALL_PRODUCTS.get(product_id, f"product_{product_id}")


def product_slug(product_id: int) -> str:
    return product_name(product_id).lower().replace(" ", "_").replace("-", "_")


def allowed_products_for_tier(tier: str) -> Dict[int, str]:
    if tier == "beginner":
        return dict(BEGINNER_PRODUCTS)
    if tier == "advanced":
        # Advanced Task Complexity has low raw-material targets compared with
        # order count. Therefore the generator allows both Beginner and
        # Advanced Object IDs, then validates the final raw-material count.
        return dict(ALL_PRODUCTS)
    raise ValueError(f"Unsupported tier: {tier}. Valid: {TIERS}")


def total_raw_usage(product_ids: Iterable[int]) -> int:
    return sum(len(decompose_product(pid)) for pid in product_ids)


def needed_raw_materials(produce_ids: Sequence[int]) -> List[int]:
    materials: List[int] = []
    for product_id in produce_ids:
        materials.extend(decompose_product(product_id))
    return materials


def validate_product_ids(tier: str, product_ids: Sequence[int]) -> List[str]:
    errors: List[str] = []
    allowed = allowed_products_for_tier(tier)
    for product_id in product_ids:
        if product_id not in allowed:
            errors.append(f"지원하지 않는 product_id: {product_id}")
            continue
        digits = decompose_product(product_id)
        invalid_digits = [d for d in digits if d not in RAW_MATERIALS]
        if invalid_digits:
            errors.append(f"product_id={product_id}에 잘못된 원재료 ID가 포함됨: {invalid_digits}")
    return errors


def validate_complexity(rule: ComplexityRule, produce_ids: Sequence[int], recycle_ids: Sequence[int]) -> List[str]:
    errors: List[str] = []
    produce_count = len(produce_ids)
    if produce_count < rule.produce_count:
        errors.append(
            f"생산 주문 수 부족: {produce_count}개 "
            f"(필요 {rule.produce_count}개보다 {rule.produce_count - produce_count}개 부족)"
        )
    elif produce_count > rule.produce_count:
        errors.append(
            f"생산 주문 수 초과: {produce_count}개 "
            f"(필요 {rule.produce_count}개보다 {produce_count - rule.produce_count}개 초과)"
        )

    recycle_count = len(recycle_ids)
    if recycle_count < rule.recycle_count:
        errors.append(
            f"반환/재활용 주문 수 부족: {recycle_count}개 "
            f"(필요 {rule.recycle_count}개보다 {rule.recycle_count - recycle_count}개 부족)"
        )
    elif recycle_count > rule.recycle_count:
        errors.append(
            f"반환/재활용 주문 수 초과: {recycle_count}개 "
            f"(필요 {rule.recycle_count}개보다 {recycle_count - rule.recycle_count}개 초과)"
        )

    product_count = produce_count + recycle_count
    if product_count < rule.product_min:
        errors.append(
            f"Products 수 부족: {product_count}개 "
            f"(최소 {rule.product_min}개보다 {rule.product_min - product_count}개 부족)"
        )
    elif product_count > rule.product_max:
        errors.append(
            f"Products 수 초과: {product_count}개 "
            f"(최대 {rule.product_max}개보다 {product_count - rule.product_max}개 초과)"
        )

    raw_count = total_raw_usage(list(produce_ids) + list(recycle_ids))
    if raw_count < rule.raw_min:
        errors.append(
            f"Raw Mat. 수 부족: {raw_count}개 "
            f"(최소 {rule.raw_min}개보다 {rule.raw_min - raw_count}개 부족)"
        )
    elif raw_count > rule.raw_max:
        errors.append(
            f"Raw Mat. 수 초과: {raw_count}개 "
            f"(최대 {rule.raw_max}개보다 {raw_count - rule.raw_max}개 초과)"
        )
    return errors


def validate_storage_has_required_materials(storage_items: Sequence[int], produce_ids: Sequence[int]) -> List[str]:
    errors: List[str] = []
    required = Counter(needed_raw_materials(produce_ids))
    available = Counter([item for item in storage_items if item in RAW_MATERIALS])
    missing: Dict[int, int] = {}
    for material_id, count in required.items():
        if available[material_id] < count:
            missing[material_id] = count - available[material_id]
    if missing:
        errors.append(f"생산에 필요한 원재료 부족: {missing}")
    return errors


def generate_product_lists(rule: ComplexityRule, rng: random.Random) -> Tuple[List[int], List[int]]:
    """Randomly select product IDs until the selected set satisfies the rule.

    Product lengths are intentionally biased toward short products first because
    the official complexity table often asks for small raw-material counts even
    when the order count is large.
    """
    catalog = list(allowed_products_for_tier(rule.tier).keys())
    short_first = sorted(catalog, key=lambda pid: (len(str(pid)), pid))

    # Weighted pool: short products appear more often to make the raw range reachable.
    weighted_pool: List[int] = []
    for pid in short_first:
        length = len(str(pid))
        weight = max(1, 6 - length)
        weighted_pool.extend([pid] * weight)

    for _ in range(5000):
        produce_ids = [rng.choice(weighted_pool) for _ in range(rule.produce_count)]
        recycle_ids = [rng.choice(weighted_pool) for _ in range(rule.recycle_count)]
        if not validate_complexity(rule, produce_ids, recycle_ids):
            return produce_ids, recycle_ids

    # Fallback: greedily choose the shortest products. This should still satisfy
    # the current table for all tiers/stages.
    produce_ids = [short_first[i % len(short_first)] for i in range(rule.produce_count)]
    recycle_ids = [short_first[(i + rule.produce_count) % len(short_first)] for i in range(rule.recycle_count)]
    errors = validate_complexity(rule, produce_ids, recycle_ids)
    if errors:
        raise RuntimeError("자동 생성 실패: " + "; ".join(errors))
    return produce_ids, recycle_ids


def generate_product_lists_with_selected(
    rule: ComplexityRule,
    selected_produce_ids: Sequence[int],
    selected_recycle_ids: Sequence[int],
    rng: random.Random,
) -> Tuple[List[int], List[int]]:
    """Keep selected IDs and automatically fill the remaining orders.

    Automatically generated orders use IDs other than the ones explicitly
    selected by the user. Repetition among automatically generated IDs is
    allowed because some complexity ranges cannot be reached with unique
    products only.
    """
    selected_produce = list(selected_produce_ids)
    selected_recycle = list(selected_recycle_ids)

    if len(selected_produce) > rule.produce_count:
        raise ValueError(
            f"선택한 생산 주문이 너무 많습니다: {len(selected_produce)}개, "
            f"최대 {rule.produce_count}개"
        )
    if len(selected_recycle) > rule.recycle_count:
        raise ValueError(
            f"선택한 재활용 주문이 너무 많습니다: {len(selected_recycle)}개, "
            f"최대 {rule.recycle_count}개"
        )

    selected_ids = selected_produce + selected_recycle
    errors = validate_product_ids(rule.tier, selected_ids)
    if errors:
        raise ValueError("선택한 product_id 검증 실패:\n  - " + "\n  - ".join(errors))

    selected_set = set(selected_ids)
    auto_catalog = [
        product_id
        for product_id in allowed_products_for_tier(rule.tier)
        if product_id not in selected_set
    ]
    remaining_produce = rule.produce_count - len(selected_produce)
    remaining_recycle = rule.recycle_count - len(selected_recycle)

    if (remaining_produce or remaining_recycle) and not auto_catalog:
        raise ValueError("선택하지 않은 product_id가 없어 나머지 주문을 자동 생성할 수 없습니다.")

    weighted_pool: List[int] = []
    for product_id in sorted(auto_catalog, key=lambda pid: (len(str(pid)), pid)):
        weight = max(1, 6 - len(str(product_id)))
        weighted_pool.extend([product_id] * weight)

    for _ in range(5000):
        produce_ids = selected_produce + [
            rng.choice(weighted_pool) for _ in range(remaining_produce)
        ]
        recycle_ids = selected_recycle + [
            rng.choice(weighted_pool) for _ in range(remaining_recycle)
        ]
        if not validate_complexity(rule, produce_ids, recycle_ids):
            rng.shuffle(produce_ids)
            rng.shuffle(recycle_ids)
            return produce_ids, recycle_ids

    raise ValueError(
        "선택한 product_id를 포함하면서 Task Complexity를 만족하는 "
        "나머지 주문을 생성할 수 없습니다."
    )


def build_task_from_products(
    rule: ComplexityRule,
    produce_ids: Sequence[int],
    recycle_ids: Sequence[int],
    selected_sides: Sequence[str],
    rng: random.Random,
) -> Task:
    errors = validate_product_ids(rule.tier, list(produce_ids) + list(recycle_ids))
    errors.extend(validate_complexity(rule, produce_ids, recycle_ids))
    if errors:
        raise ValueError("작업 복잡도 검증 실패:\n  - " + "\n  - ".join(errors))

    orders: List[Order] = []
    for index, product_id in enumerate(produce_ids, start=1):
        orders.append(make_order(Order.OT_PRODUCE, f"produce_{index}_{product_slug(product_id)}", product_id))
    for index, product_id in enumerate(recycle_ids, start=1):
        orders.append(make_order(Order.OT_RECYCLE, f"recycle_{index}_{product_slug(product_id)}", product_id))

    if set(selected_sides) == {"side_a", "side_b"}:
        materials_by_station = build_fair_material_distribution(produce_ids, recycle_ids, rng)
    else:
        storage_items = needed_raw_materials(produce_ids)
        customer_items = list(recycle_ids)
        rng.shuffle(storage_items)
        rng.shuffle(customer_items)
        materials_by_station = {}
        materials_by_station.update(
            distribute_to_stations(storage_items, storage_station_names(selected_sides))
        )
        materials_by_station.update(
            distribute_to_stations(customer_items, customer_station_names(selected_sides))
        )

    # Safety check: generated storage must contain all materials required for production.
    all_storage_materials: List[int] = []
    for station_name in storage_station_names(selected_sides):
        all_storage_materials.extend(materials_by_station.get(station_name, []))
    storage_errors = validate_storage_has_required_materials(all_storage_materials, produce_ids)
    if storage_errors:
        raise RuntimeError("자동 배치 검증 실패:\n  - " + "\n  - ".join(storage_errors))

    return fill_task(orders, selected_sides, materials_by_station)


# -----------------------------------------------------------------------------
# Interactive prompt helpers
# -----------------------------------------------------------------------------

def prompt_choice(title: str, choices: Sequence[str], default: Optional[str] = None) -> str:
    print(f"\n{title}")
    for index, choice in enumerate(choices, start=1):
        suffix = " (default)" if default == choice else ""
        print(f"  {index}. {choice}{suffix}")
    while True:
        value = input("> ").strip().lower()
        if not value and default:
            return default
        if value.isdigit():
            index = int(value)
            if 1 <= index <= len(choices):
                return choices[index - 1]
        if value in choices:
            return value
        print(f"잘못된 입력입니다. 가능한 값: {', '.join(choices)}")


def prompt_int_list(
    title: str,
    expected_count: Optional[int],
    allowed: Optional[Sequence[int]] = None,
    max_count: Optional[int] = None,
) -> List[int]:
    allowed_set = set(allowed) if allowed is not None else None
    while True:
        print(f"\n{title}")
        if expected_count is None and max_count is not None:
            print(
                f"  최대 {max_count}개를 쉼표로 입력하세요. "
                "비워두면 모두 자동 생성합니다. 예: 34,13"
            )
        elif expected_count is None:
            print("  쉼표로 입력하세요. 비워둘 수 있습니다. 예: 34,13,241")
        elif expected_count == 0:
            print("  이 단계에서는 입력할 항목이 없습니다. Enter를 누르세요.")
        else:
            print(f"  {expected_count}개를 쉼표로 입력하세요. 예: 34,13")
        if allowed_set:
            print("  사용 가능 ID:", ", ".join(str(x) for x in sorted(allowed_set)))
        raw = input("> ").strip()
        if not raw:
            if expected_count in (None, 0):
                return []
            print(f"개수 위반: 0개 입력됨, 필요 {expected_count}개")
            continue
        try:
            values = [int(x.strip()) for x in raw.split(",") if x.strip()]
        except ValueError:
            print("숫자와 쉼표만 입력하세요.")
            continue
        if expected_count is not None and len(values) != expected_count:
            print(f"개수 위반: {len(values)}개 입력됨, 필요 {expected_count}개")
            continue
        if max_count is not None and len(values) > max_count:
            print(f"개수 위반: {len(values)}개 입력됨, 최대 {max_count}개")
            continue
        if allowed_set is not None:
            invalid = [x for x in values if x not in allowed_set]
            if invalid:
                print(f"지원하지 않는 ID: {invalid}")
                continue
        return values


def choose_rule_interactively(default_tier: str = "beginner") -> ComplexityRule:
    tier = prompt_choice("티어 선택", TIERS, default=default_tier)
    supported_stages = [stage for stage in STAGES if (tier, stage) in COMPLEXITY]
    stage = prompt_choice("스테이지 선택", supported_stages, default=supported_stages[0])
    return COMPLEXITY[(tier, stage)]


def choose_sides(rule: ComplexityRule) -> List[str]:
    del rule
    print("\n경기 공정성을 위해 Side A와 Side B에 동일한 조건을 생성합니다.")
    return ["side_a", "side_b"]


def build_interactive_task(rng: random.Random) -> Tuple[Task, ComplexityRule, str, List[str]]:
    rule = choose_rule_interactively()
    mode = prompt_choice(
        "생성 방식 선택",
        MODES,
        default="auto",
    )
    selected_sides = choose_sides(rule)
    allowed_ids = sorted(allowed_products_for_tier(rule.tier).keys())

    if mode == "manual":
        print("\n[수동생성] 생산/재활용 product_id를 직접 입력합니다. 배치는 자동 생성합니다.")
        while True:
            produce_ids = prompt_int_list("생산 주문 product_id 입력", rule.produce_count, allowed_ids)
            recycle_ids = prompt_int_list("재활용/반환 product_id 입력", rule.recycle_count, allowed_ids)
            complexity_errors = validate_complexity(rule, produce_ids, recycle_ids)
            if not complexity_errors:
                break
            print("\n[수동생성 실패] Task Complexity를 만족하지 못했습니다.")
            for error in complexity_errors:
                print(f"  - {error}")
            print("product_id를 다시 입력하세요.")
    elif mode == "semi":
        print(
            "\n[반자동생성] 선택한 product_id는 반드시 포함하고, "
            "나머지는 다른 ID로 자동 생성합니다."
        )
        selected_produce_ids = (
            prompt_int_list(
                "반드시 포함할 생산 product_id 입력",
                None,
                allowed_ids,
                max_count=rule.produce_count,
            )
            if rule.produce_count
            else []
        )
        selected_recycle_ids = (
            prompt_int_list(
                "반드시 포함할 재활용/반환 product_id 입력",
                None,
                allowed_ids,
                max_count=rule.recycle_count,
            )
            if rule.recycle_count
            else []
        )
        produce_ids, recycle_ids = generate_product_lists_with_selected(
            rule,
            selected_produce_ids,
            selected_recycle_ids,
            rng,
        )
    else:
        print("\n[자동생성] 주문과 arena_layout을 모두 자동 생성합니다.")
        produce_ids, recycle_ids = generate_product_lists(rule, rng)

    task = build_task_from_products(rule, produce_ids, recycle_ids, selected_sides, rng)
    return task, rule, mode, selected_sides


def parse_int_list_param(raw: str) -> List[int]:
    raw = raw.strip()
    if not raw:
        return []
    return [int(x.strip()) for x in raw.split(",") if x.strip()]


# -----------------------------------------------------------------------------
# ROS2 publisher node
# -----------------------------------------------------------------------------

class TaskComplexityPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("task_complexity_publisher")

        self.declare_parameter("interactive", True)
        self.declare_parameter("topic_name", "/eai/task")
        self.declare_parameter("side_a_topic_name", "/eai/task/side_a")
        self.declare_parameter("side_b_topic_name", "/eai/task/side_b")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("publish_once", True)
        self.declare_parameter("seed", 0)

        # Non-interactive fallback parameters.
        self.declare_parameter("tier", "beginner")
        self.declare_parameter("stage", "production")
        self.declare_parameter("generation_mode", "auto")
        self.declare_parameter("produce_product_ids", "")
        self.declare_parameter("recycle_product_ids", "")

        interactive = self.get_parameter("interactive").get_parameter_value().bool_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        side_a_topic_name = self.get_parameter("side_a_topic_name").get_parameter_value().string_value
        side_b_topic_name = self.get_parameter("side_b_topic_name").get_parameter_value().string_value
        period = self.get_parameter("publish_period_sec").get_parameter_value().double_value
        self._publish_once = self.get_parameter("publish_once").get_parameter_value().bool_value
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self._rng = random.Random(seed if seed != 0 else None)

        if interactive:
            self._task, self._rule, self._mode, self._selected_sides = build_interactive_task(self._rng)
        else:
            self._task, self._rule, self._mode, self._selected_sides = self._build_from_parameters()

        self._publisher = self.create_publisher(Task, topic_name, 10)
        self._side_a_publisher = self.create_publisher(Task, side_a_topic_name, 10)
        self._side_b_publisher = self.create_publisher(Task, side_b_topic_name, 10)
        self._timer = self.create_timer(period, self._publish_task)

        self.get_logger().info(
            f"Task Complexity publisher ready: tier={self._rule.tier}, stage={self._rule.stage}, "
            f"mode={self._mode}, sides={self._selected_sides}, publish_once={self._publish_once}"
        )
        self._log_task_summary(self._task)

    def _build_from_parameters(self) -> Tuple[Task, ComplexityRule, str, List[str]]:
        tier = self.get_parameter("tier").get_parameter_value().string_value.strip().lower()
        stage = self.get_parameter("stage").get_parameter_value().string_value.strip().lower()
        mode = self.get_parameter("generation_mode").get_parameter_value().string_value.strip().lower()

        if (tier, stage) not in COMPLEXITY:
            valid = ", ".join(f"{t}/{s}" for t, s in sorted(COMPLEXITY.keys()))
            raise ValueError(f"Unsupported tier/stage: {tier}/{stage}. Valid: {valid}")
        if mode not in MODES:
            raise ValueError(f"Unsupported generation_mode: {mode}. Valid: {MODES}")

        rule = COMPLEXITY[(tier, stage)]
        selected_sides = ["side_a", "side_b"]

        if mode == "manual":
            produce_raw = self.get_parameter("produce_product_ids").get_parameter_value().string_value
            recycle_raw = self.get_parameter("recycle_product_ids").get_parameter_value().string_value
            produce_ids = parse_int_list_param(produce_raw)
            recycle_ids = parse_int_list_param(recycle_raw)
        elif mode == "semi":
            produce_raw = self.get_parameter("produce_product_ids").get_parameter_value().string_value
            recycle_raw = self.get_parameter("recycle_product_ids").get_parameter_value().string_value
            selected_produce_ids = parse_int_list_param(produce_raw)
            selected_recycle_ids = parse_int_list_param(recycle_raw)
            produce_ids, recycle_ids = generate_product_lists_with_selected(
                rule,
                selected_produce_ids,
                selected_recycle_ids,
                self._rng,
            )
        else:
            produce_ids, recycle_ids = generate_product_lists(rule, self._rng)

        task = build_task_from_products(rule, produce_ids, recycle_ids, selected_sides, self._rng)
        return task, rule, mode, selected_sides

    def _publish_task(self) -> None:
        side_a_task = build_side_only_task(self._task, "side_a")
        side_b_task = build_side_only_task(self._task, "side_b")

        self._publisher.publish(self._task)
        self._side_a_publisher.publish(side_a_task)
        self._side_b_publisher.publish(side_b_task)

        self.get_logger().info(
            f"Published task: orders={len(self._task.order_list)}, stations={len(self._task.arena_layout)}"
        )
        self.get_logger().info(
            f"Published side_a task: orders={len(side_a_task.order_list)}, stations={len(side_a_task.arena_layout)}"
        )
        self.get_logger().info(
            f"Published side_b task: orders={len(side_b_task.order_list)}, stations={len(side_b_task.arena_layout)}"
        )

        if self._publish_once:
            self.get_logger().info("Published one task. Shutting down.")
            self._timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()

    def _log_task_summary(self, task: Task) -> None:
        self.get_logger().info(
            f"Complexity rule: time={self._rule.time_min}min, "
            f"orders={self._rule.produce_count}, returns={self._rule.recycle_count}, "
            f"raw={self._rule.raw_min}~{self._rule.raw_max}, "
            f"products={self._rule.product_min}~{self._rule.product_max}, "
            f"arena={self._rule.arena}, fleet={self._rule.fleet_min}~{self._rule.fleet_max}"
        )
        for order in task.order_list:
            order_type = "PRODUCE" if order.order_type == Order.OT_PRODUCE else "RECYCLE"
            self.get_logger().info(
                f"  Order: type={order_type}, product_id={order.product_id}, "
                f"materials={decompose_product(order.product_id)}, name={order.name}"
            )
        for station in task.arena_layout:
            self.get_logger().info(
                f"  Station: name={station.name}, id={station.station_id}, "
                f"type={station.station_type}, materials={list(station.material_ids)}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskComplexityPublisherNode()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
