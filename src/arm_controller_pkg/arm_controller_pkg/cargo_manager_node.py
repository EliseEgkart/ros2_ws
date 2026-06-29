import rclpy
from rclpy.node import Node
from arm_interfaces.srv import Cargo


MATERIAL_NAMES = {
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
}

# 블록별 높이 (칸 단위)
BLOCK_HEIGHT = {
    1: 2, 2: 2, 3: 2, 4: 2,  # 2×2 블록: 2칸
    5: 4, 6: 4, 7: 4, 8: 4,  # 4×2 블록: 4칸
}

# 블록별 픽업 오프셋 (바닥에서 몇 칸 위를 집는가)
PICK_OFFSET = {
    1: 0, 2: 0, 3: 0, 4: 0,  # 2×2: 바닥에서 0칸
    5: 1, 6: 1, 7: 1, 8: 1,  # 4×2: 바닥에서 1칸
}

MAX_STACK_HEIGHT = 6  # 슬롯당 최대 적재 높이 (칸)

PRODUCT_SLOT = 1
MATERIAL_SLOTS = [2, 3, 4, 5, 6]
ASSEMBLY_SLOTS = [7, 8]  # 조립 슬롯 (FIND_EMPTY 검색 대상 아님)

# 경기 당일 워크벤치 스테이션 아이디를 여기에 입력
# 워크벤치 스테이션은 delivery 기록을 하지 않음
WORKBENCH_STATION_IDS = {1}


class CargoManagerNode(Node):
    def __init__(self):
        super().__init__('cargo_manager_node')
        self.srv = self.create_service(Cargo, '/cargo', self.cargo_cb)

        # {slot: [bottom, ..., top] 순서의 object_id 리스트}
        self.slot_state = {
            slot: []
            for slot in [PRODUCT_SLOT] + MATERIAL_SLOTS + ASSEMBLY_SLOTS
        }

        # 커스터머 센터 delivery 상태: {station_id: {delivery_idx: object_id or None}}
        self.delivery_state = {}

        self.get_logger().info('[CARGO] cargo_manager_node started')
        self.get_logger().info(f'[CARGO] slots: {list(self.slot_state.keys())}')

    # ── 내부 헬퍼 ──────────────────────────────────────────────────

    def _stack_height(self, slot):
        return sum(BLOCK_HEIGHT.get(obj, 2) for obj in self.slot_state[slot])

    def _layer_index(self, slot, object_id):
        """슬롯 스택에서 object_id의 픽업 layer_index를 계산한다."""
        h = 0
        for obj in self.slot_state[slot]:
            if obj == object_id:
                return h + PICK_OFFSET.get(obj, 0)
            h += BLOCK_HEIGHT.get(obj, 2)
        return None

    # ── 서비스 콜백 ────────────────────────────────────────────────

    def cargo_cb(self, request, response):
        action = request.action.upper()
        response.layer_index = 0

        if action == 'FIND_EMPTY':
            if request.object_id > 8:
                search_slots = [PRODUCT_SLOT]
            else:
                search_slots = MATERIAL_SLOTS

            needed = BLOCK_HEIGHT.get(request.object_id, 2)
            for slot in search_slots:
                if self._stack_height(slot) + needed <= MAX_STACK_HEIGHT:
                    future_layer = self._stack_height(slot) + PICK_OFFSET.get(request.object_id, 0)
                    response.success = True
                    response.slot = slot
                    response.layer_index = future_layer
                    response.message = f'empty slot found: slot={slot}, future_layer={future_layer}'
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = 'no empty slot'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_OBJECT':
            for slot, stack in self.slot_state.items():
                if request.object_id in stack:
                    layer = self._layer_index(slot, request.object_id)
                    response.success = True
                    response.slot = slot
                    response.layer_index = layer
                    response.message = (
                        f'object found: object_id={request.object_id}, '
                        f'slot={slot}, layer_index={layer}'
                    )
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = f'object_id={request.object_id} not found'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'SET':
            slot = request.slot
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            else:
                obj = request.object_id
                self.slot_state[slot].append(obj)
                layer = self._layer_index(slot, obj)
                name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                response.success = True
                response.slot = slot
                response.layer_index = layer
                response.message = (
                    f'slot={slot} set: object_id={obj} ({name}), '
                    f'layer_index={layer}, stack={self.slot_state[slot]}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'CLEAR':
            slot = request.slot
            obj = request.object_id
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            elif obj not in self.slot_state[slot]:
                response.success = False
                response.message = f'object_id={obj} not in slot={slot}'
            else:
                self.slot_state[slot].remove(obj)
                response.success = True
                response.slot = slot
                response.message = (
                    f'slot={slot} cleared: object_id={obj}, '
                    f'remaining={self.slot_state[slot]}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'STATUS':
            lines = []
            for slot, stack in self.slot_state.items():
                if not stack:
                    lines.append(f'slot={slot}: empty')
                else:
                    items = []
                    for obj in stack:
                        name = MATERIAL_NAMES.get(obj, f'product_id={obj}')
                        layer = self._layer_index(slot, obj)
                        items.append(f'{name}(layer={layer})')
                    lines.append(f'slot={slot}: [{", ".join(items)}]')
            response.success = True
            response.message = ' | '.join(lines)
            self.get_logger().info(f'[CARGO] STATUS: {response.message}')

        elif action == 'FIND_DELIVERY_EMPTY':
            station_id = request.station_id
            if station_id in WORKBENCH_STATION_IDS:
                response.success = True
                response.slot = 0
                response.message = f'workbench station={station_id}: delivery tracking skipped'
                self.get_logger().info(f'[CARGO] {response.message}')
                return response
            station = self.delivery_state.setdefault(station_id, {})
            for idx in range(6):
                if station.get(idx) is None:
                    response.success = True
                    response.slot = idx
                    response.message = f'empty delivery slot found: station={station_id}, idx={idx}'
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response
            response.success = False
            response.slot = -1
            response.message = f'no empty delivery slot at station={station_id}'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'SET_DELIVERY':
            station_id = request.station_id
            if station_id in WORKBENCH_STATION_IDS:
                response.success = True
                response.slot = request.slot
                response.message = f'workbench station={station_id}: delivery tracking skipped'
                self.get_logger().info(f'[CARGO] {response.message}')
                return response
            idx = request.slot
            station = self.delivery_state.setdefault(station_id, {})
            prev = station.get(idx)
            station[idx] = request.object_id
            name = MATERIAL_NAMES.get(request.object_id, f'product_id={request.object_id}')
            response.success = True
            response.slot = idx
            response.message = (
                f'delivery updated: station={station_id}, idx={idx}: '
                f'{prev} -> object_id={request.object_id} ({name})'
            )
            self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'CLEAR_DELIVERY':
            station_id = request.station_id
            if station_id in WORKBENCH_STATION_IDS:
                response.success = True
                response.slot = request.slot
                response.message = f'workbench station={station_id}: delivery tracking skipped'
                self.get_logger().info(f'[CARGO] {response.message}')
                return response
            idx = request.slot
            station = self.delivery_state.setdefault(station_id, {})
            station[idx] = None
            response.success = True
            response.slot = idx
            response.message = f'delivery cleared: station={station_id}, idx={idx}'
            self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'STATUS_DELIVERY':
            station_id = request.station_id
            if station_id in WORKBENCH_STATION_IDS:
                response.success = True
                response.message = f'workbench station={station_id}: no delivery tracking'
                self.get_logger().info(f'[CARGO] {response.message}')
                return response
            station = self.delivery_state.get(station_id, {})
            lines = []
            for idx in range(6):
                obj = station.get(idx)
                name = MATERIAL_NAMES.get(obj, f'product_id={obj}') if obj is not None else 'empty'
                lines.append(f'idx={idx}: {name}')
            response.success = True
            response.message = f'station={station_id} | ' + ' | '.join(lines)
            self.get_logger().info(f'[CARGO] {response.message}')

        else:
            response.success = False
            response.message = f'unknown action: {action}'
            self.get_logger().error(f'[CARGO] {response.message}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CargoManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()