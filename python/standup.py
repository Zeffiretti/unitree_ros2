from unitree_ros2_sim import UnitreeRos2Real
import torch
import time
import rclpy


class StandupPolicy(torch.nn.Module):
    def __init__(self, durations=[1.0, 1.0, 2.0, 1.8], device="cpu"):
        super(StandupPolicy, self).__init__()
        self.target_positions = torch.tensor(  # note: this should be in isaacgym order
            [
                [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65],
                [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3],
                [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3],
                [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65],
            ],
            dtype=torch.float32,
            device=device,
        )
        self.durations = torch.tensor(durations, dtype=torch.float32, device=device)
        # accumulate the durations for each phase
        self.phase_durations = torch.cumsum(self.durations, dim=0)

    def forward(self, t):
        # calc the phase
        phase, start_pos, end_pos = self._calc_phase_and_start_end_pos(t)
        joint_pos = start_pos + phase * (end_pos - start_pos)
        return joint_pos

    def _calc_phase_and_start_end_pos(self, t):
        assert self.target_positions.shape[0] == 5, f"Start position is not set {self.target_positions.shape}"
        if t < 0.0:
            return 0, self.target_positions[0], self.target_positions[0]
        if t >= self.phase_durations[-1]:
            return 3, self.target_positions[-1], self.target_positions[-1]
        phase_index = torch.nonzero(t < self.phase_durations, as_tuple=False)[0].item()
        phase_durations = self.durations[phase_index]
        phase = 1.0 - (self.phase_durations[phase_index] - t) / phase_durations
        print(f"t: {t}, phase: {phase}, phase_index: {phase_index}")
        # if phase_index == 2:
        #     return phase, self.target_positions[phase_index], self.target_positions[phase_index]
        return phase, self.target_positions[phase_index], self.target_positions[phase_index + 1]

    def set_start_pos(self, start_pos):
        start_pos = start_pos.to(self.target_positions.device)
        self.target_positions = torch.cat([start_pos, self.target_positions], dim=0)


class StandupNode(UnitreeRos2Real):
    def __init__(self):
        self.policy = StandupPolicy()
        self.policy.eval()
        self.start_time = time.monotonic()
        super(StandupNode, self).__init__(robot_class_name="Go2", dryrun=False, safety_check=False)
        self.start_ros_handlers()

    def start_main_loop_timer(self):
        # super(StandupNode, self).start_main_loop_timer()
        self.get_logger().info("Starting main loop timer...")
        self.policy.set_start_pos(self.dof_pos_)
        self.start_time = time.monotonic()

        self.main_loop_timer = self.create_timer(0.02, self.main_loop)

    def main_loop(self):
        # super(Standup, self).main_loop()
        current_time = time.monotonic()
        t = current_time - self.start_time
        target_pos = self.policy(t)
        self.send_action(target_pos)
        if t > 8.0:
            self.get_logger().info("Standup finished.")
            exit(0)


if __name__ == "__main__":
    rclpy.init()
    node = StandupNode()
    node.start_main_loop_timer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
