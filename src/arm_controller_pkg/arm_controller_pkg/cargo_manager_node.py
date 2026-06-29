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

PRODUCT_SLOT = 1
MATERIAL_SLOTS = [2, 3, 4, 5, 6]
ASSEMBLY_SLOTS = [7, 8]  # 이동 중 조립 슬롯 (FIND_EMPTY 검색 대상 아님)


class CargoManagerNode(Node):
    def __init__(self):
        super().__init__('cargo_manager_node')
        self.srv = self.create_service(Cargo, '/cargo', self.cargo_cb)

        # Each slot holds a list of object_ids (ramp supports multiple blocks per slot).
        self.slot_state = {slot: [] for slot in [PRODUCT_SLOT] + MATERIAL_SLOTS + ASSEMBLY_SLOTS}

        # 커스터머 센터 delivery 상태: {station_id: {delivery_idx: object_id or None}}
        self.delivery_state = {}

        self.get_logger().info('[CARGO] cargo_manager_node started')
        self.get_logger().info(f'[CARGO] slots: {list(self.slot_state.keys())}')

    def cargo_cb(self, request, response):
        action = request.action.upper()

        if action == 'FIND_EMPTY':
            # A slot is empty when its list has no occupants.
            if request.object_id > 8:
                search_slots = [PRODUCT_SLOT]
            else:
                search_slots = MATERIAL_SLOTS

            for slot in search_slots:
                if len(self.slot_state[slot]) == 0:
                    response.success = True
                    response.slot = slot
                    response.message = f'empty slot found: slot={slot}'
                    self.get_logger().info(f'[CARGO] {response.message}')
                    return response

            response.success = False
            response.slot = -1
            response.message = 'no empty slot'
            self.get_logger().warn(f'[CARGO] {response.message}')

        elif action == 'FIND_OBJECT':
            for slot, obj_list in self.slot_state.items():
                if request.object_id in obj_list:
                    response.success = True
                    response.slot = slot
                    response.message = f'object found: object_id={request.object_id}, slot={slot}'
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
                self.slot_state[slot].append(request.object_id)
                name = MATERIAL_NAMES.get(request.object_id, f'product_id={request.object_id}')
                response.success = True
                response.slot = slot
                response.message = (
                    f'slot={slot} set: object_id={request.object_id} ({name}), '
                    f'contents={self.slot_state[slot]}'
                )
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'CLEAR':
            slot = request.slot
            if slot not in self.slot_state:
                response.success = False
                response.message = f'invalid slot={slot}'
            else:
                obj_list = self.slot_state[slot]
                obj_id = request.object_id
                if obj_id != 0 and obj_id in obj_list:
                    obj_list.remove(obj_id)  # removes first occurrence
                    response.success = True
                    response.slot = slot
                    response.message = (
                        f'slot={slot} cleared object_id={obj_id}, '
                        f'remaining={obj_list}'
                    )
                elif obj_list:
                    # Fallback: remove front item (FIFO for ramp, no object_id specified)
                    removed = obj_list.pop(0)
                    response.success = True
                    response.slot = slot
                    response.message = (
                        f'slot={slot} cleared first item={removed} '
                        f'(no object_id specified), remaining={obj_list}'
                    )
                else:
                    response.success = True
                    response.slot = slot
                    response.message = f'slot={slot} already empty'
                self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'STATUS':
            lines = []
            for slot, obj_list in self.slot_state.items():
                if not obj_list:
                    names = 'empty'
                else:
                    names = ', '.join(
                        MATERIAL_NAMES.get(o, f'product_id={o}') for o in obj_list
                    )
                lines.append(f'slot={slot}: [{names}]')
            response.success = True
            response.message = ' | '.join(lines)
            self.get_logger().info(f'[CARGO] STATUS: {response.message}')

        elif action == 'FIND_DELIVERY_EMPTY':
            station_id = request.station_id
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
            idx = request.slot
            station = self.delivery_state.setdefault(station_id, {})
            station[idx] = None
            response.success = True
            response.slot = idx
            response.message = f'delivery cleared: station={station_id}, idx={idx}'
            self.get_logger().info(f'[CARGO] {response.message}')

        elif action == 'STATUS_DELIVERY':
            station_id = request.station_id
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
