from collections import deque
import threading
from types import SimpleNamespace

from sml_system_pkg.sml_manager_node import SmlManagerNode


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warn(self, message):
        self.messages.append(('warn', message))

    def error(self, message):
        self.messages.append(('error', message))


class _ArmFuture:
    def __init__(self):
        self._callback = None
        self._response = None

    def add_done_callback(self, callback):
        self._callback = callback

    def result(self):
        return self._response

    def complete(self, response):
        self._response = response
        self._callback(self)


class _ArmClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def call_async(self, request):
        future = _ArmFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future


def _manager_for_slot_test():
    manager = object.__new__(SmlManagerNode)
    manager._arm_raw_slot_indices = [2, 3, 4, 5, 6]
    manager._arm_raw_slots = {
        slot: {'units': [None, None, None], 'items': {}}
        for slot in manager._arm_raw_slot_indices
    }
    manager._arm_item_keys = {}
    manager._arm_next_uid = 1
    manager._arm_cmd_slide_cache = {}
    manager._arm_pending_removals = {}
    manager._test_logger = _Logger()
    manager.get_logger = lambda: manager._test_logger
    return manager


def test_assembled_product_inherits_base_physical_position():
    manager = _manager_for_slot_test()

    # raw 8(size 2)은 logical slide 6에서 physical arm slide 51을 차지한다.
    assert manager._assign_raw_arm_position(6, 8) == 51

    assemble_step = SimpleNamespace(
        step_id=3,
        slide_ids=[6, 1],
        object_ids=[81],
    )
    assert manager._convert_step_slide_ids_for_arm(
        assemble_step, 'ASSEMBLE'
    ) == [5100, 1001]

    manager._commit_amr_assemble_slot_changes(assemble_step)

    # 완성품 81이 raw 8의 정확한 물리 위치 51을 이어받아야 한다.
    unload_step = SimpleNamespace(
        step_id=4,
        slide_ids=[6],
        object_ids=[81],
    )
    assert manager._convert_step_slide_ids_for_arm(
        unload_step, 'UNLOAD'
    ) == [51]
    assert not any(
        level == 'warn' and 'fallback' in message
        for level, message in manager._test_logger.messages
    )

    manager._commit_arm_slot_removals(unload_step)
    assert manager._arm_raw_slots[6]['units'] == [None, None, None]
    assert (6, 81) not in manager._arm_item_keys


def test_batched_assembly_ids_keep_target_order_and_sequence():
    manager = _manager_for_slot_test()

    # 실제 실행 로그와 같은 두 주문:
    # order 0 -> assembly 7 + raw slide 1/pos 0
    # order 1 -> assembly 8 + raw slide 1/pos 2 + raw slide 2/pos 0
    manager._assign_raw_arm_position(2, 1)
    manager._assign_raw_arm_position(12, 4)
    manager._assign_raw_arm_position(13, 2)
    step = SimpleNamespace(
        step_id=3,
        slide_ids=[7, 2, 18, 12, 13],
        object_ids=[81, 442],
    )

    assert manager._convert_step_slide_ids_for_arm(step, 'ASSEMBLE') == [
        7000, 1001, 8010, 1211, 2012,
    ]


def test_batched_assembly_is_sent_as_slot_7_then_slot_8_requests():
    manager = _manager_for_slot_test()
    manager._lock = threading.Lock()
    manager.arm_client = _ArmClient()
    manager.committed = []
    manager.arm_done = []
    manager._commit_amr_assemble_slot_changes = (
        lambda step: manager.committed.append(step.step_id)
    )
    manager._mark_amr_assemble_part_done = (
        lambda step, part: manager.arm_done.append((step.step_id, part))
    )

    manager._assign_raw_arm_position(2, 1)
    manager._assign_raw_arm_position(12, 4)
    manager._assign_raw_arm_position(13, 2)
    step = SimpleNamespace(
        step_id=3,
        station_id=8,
        slide_ids=[7, 2, 18, 12, 13],
        object_ids=[81, 442],
    )
    jobs = manager._build_amr_assemble_jobs(step)
    manager._amr_assemble_states = {
        3: {'jobs': jobs, 'arm_slots': [], 'nav_target': 8}
    }

    manager._send_amr_assemble_arm(step, nav_target=8)
    first = manager.arm_client.requests[0]
    assert list(first.object_ids) == [81]
    assert first.station_id == 7
    assert list(first.slide_ids) == [7000, 1001]

    manager.arm_client.futures[0].complete(SimpleNamespace(
        success=True, slots=[7], message='assemble success'
    ))
    second = manager.arm_client.requests[1]
    assert list(second.object_ids) == [442]
    assert second.station_id == 8
    assert list(second.slide_ids) == [8010, 1211, 2012]

    manager.arm_client.futures[1].complete(SimpleNamespace(
        success=True, slots=[8], message='assemble success'
    ))
    assert manager.committed == [3]
    assert manager.arm_done == [(3, 'arm')]
