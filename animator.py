import math
import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame
import imageio

class Animator:
    def __init__(self, motor_positions = [
        np.array([-1.5, 1.]),  # motor 1
        np.array([-1.5, -1.]),  # motor 2
        np.array([1.5, 1.]),  # motor 3
        np.array([1.5, -1.]),  # motor 4
    ], motor_directions = [
        np.array([1., 1.]),  # motor 1
        np.array([1., -1.]),  # motor 2
        np.array([1., -1.]),  # motor 3
        np.array([1., 1.]),  # motor 4
    ], width=608, height=608, fps=30, frames=100, output_dir="videos", dimensions=[(-50, 50), (-50, 50)],):
        self.width = width
        self.height = height
        self.scale_factor = min(width / (dimensions[0][1] - dimensions[0][0]),
                                height / (dimensions[1][1] - dimensions[1][0]))
        self.origin = np.array([width // 2, height // 2])
        self.fps = fps
        self.frames = frames
        self.output_dir = output_dir
        self.frame_count = 0
        self.frame_data = []
        self.motor_positions = motor_positions
        self.motor_directions = motor_directions

        os.makedirs(output_dir, exist_ok=True)
        self.screen = pygame.Surface((width, height))

        self.video_path = os.path.join(self.output_dir, "animation.mp4")

        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)

    def to_screen(self, unit_position):
        return (unit_position * self.scale_factor + self.origin).astype(int)


    def append(self, position, rotation, velocity, motor_accelerations, timestamp,
               debug_text="", waypoint_locations=None, obstacles=None, visible_obstacles=None, obstacle_pixels=None, goal_pixels=None, goal_location=None):
        """Store frame data without rendering immediately"""
        self.frame_data.append({
            'position': position.copy(),
            'rotation': rotation,
            'velocity': velocity.copy(),
            'motor_accelerations': motor_accelerations, # list of scalars
            'timestamp': timestamp,
            'debug_text': debug_text,
            'waypoint_locations': waypoint_locations.copy() if waypoint_locations is not None else None,
            'obstacles': [obs for obs in obstacles] if obstacles else [],
            'visible_obstacles': [obs for obs in visible_obstacles] if visible_obstacles else [],
            'obstacle_pixels': [pix for pix in obstacle_pixels] if obstacle_pixels else [],
            'goal_pixels': [pix for pix in goal_pixels] if goal_pixels else [],
            'goal_location': goal_location.copy() if goal_location is not None else None
        })

    def _render_frame(self, frame_info):
        """Render a single frame from stored data"""
        self.screen.fill((71, 158, 245))

        position = frame_info['position']
        rotation = frame_info['rotation']
        velocity = frame_info['velocity']
        motor_positions = (np.array(self.motor_positions) + position) @ np.array([[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]])
        motor_accelerations = [acc * direction for acc, direction in zip(frame_info['motor_accelerations'], self.motor_directions)]
        angle = rotation
        cos_a, sin_a = np.cos(angle), np.sin(angle)

        square_points = np.array([[-1.5, -1], [1.5, -1],
                                  [1.5, 1], [-1.5, 1]])
        rotated_points = [(cos_a * x - sin_a * y, sin_a * x + cos_a * y) for x, y in square_points]
        screen_points = [self.to_screen(np.array(p) + position) for p in rotated_points]
        for pix in frame_info['obstacle_pixels']:
            self.screen.set_at(self.to_screen(pix), (255, 0, 255))
        for pix in frame_info['goal_pixels']:
            self.screen.set_at(self.to_screen(pix), (255, 255, 0))

        # print(screen_points)
        pygame.draw.polygon(self.screen, (0,0,0), screen_points)

        front_endpoint = position + np.array([cos_a, sin_a]) * 1.5
        pygame.draw.line(self.screen, (0, 0, 0), self.to_screen(position), self.to_screen(front_endpoint), 2)

        velocity_endpoint = position + velocity * 0.5
        pygame.draw.line(self.screen, (255, 0, 0), self.to_screen(position), self.to_screen(velocity_endpoint), 3)

        for pos, acc in zip(motor_positions, motor_accelerations):
            pygame.draw.circle(self.screen, (200, 150, 0), self.to_screen(pos), int(0.5 * self.scale_factor))
            acc_endpoint = pos + acc
            pygame.draw.line(self.screen, (200, 150, 0), self.to_screen(pos), self.to_screen(acc_endpoint), 2)

        text_surface = self.font.render(frame_info['debug_text'], False, (255, 255, 255))
        self.screen.blit(text_surface, (0, 0))

        if frame_info['waypoint_locations'] is not None:
            for waypoint in frame_info['waypoint_locations']:
                pygame.draw.circle(self.screen, (0, 255, 0), self.to_screen(waypoint), int(0.8 * self.scale_factor), 2)

        for obs in frame_info['visible_obstacles']:
            # make visible obstacles have a white outline
            color = (255, 255, 255)
            if hasattr(obs, 'position'):
                pygame.draw.circle(self.screen, color, self.to_screen(np.array([obs.position[0], obs.position[1]])), 2*int(obs.radius * self.scale_factor), 2)

        for obs in frame_info['obstacles']:
            color = (100, 100, 100)
            if hasattr(obs, 'color'):
                if obs.color.value == "red":
                    color = (255, 0, 0)
                elif obs.color.value == "green":
                    color = (0, 255, 0)
                elif obs.color.value == "yellow":
                    color = (255, 255, 0)
                elif obs.color.value == "black":
                    color = (0, 0, 0)
            if hasattr(obs, 'beacon') and obs.beacon:
                pygame.draw.circle(self.screen, color, self.to_screen(np.array([obs.position[0], obs.position[1]])), int(obs.radius * self.scale_factor), 3)
            elif hasattr(obs, 'position'):
                pygame.draw.circle(self.screen, color, self.to_screen(np.array([obs.position[0], obs.position[1]])), int(obs.radius * self.scale_factor))

        if frame_info['goal_location'] is not None:
            pygame.draw.circle(self.screen, (255, 215, 0), self.to_screen(frame_info['goal_location']), int(1.0 * self.scale_factor), 3)

        raw_frame = pygame.surfarray.array3d(self.screen)
        raw_frame = np.rot90(raw_frame, k=3)
        raw_frame = np.fliplr(raw_frame)
        return raw_frame

    def render(self):
        """Render all stored frames to video at the end"""
        print("Rendering video...")
        writer = imageio.get_writer(self.video_path, fps=self.fps, codec='libx264', quality=8)

        repeats = self._frame_repeats()
        for frame_info, count in zip(self.frame_data, repeats):
            raw_frame = self._render_frame(frame_info)
            for _ in range(count):
                writer.append_data(raw_frame)

        writer.close()
        pygame.quit()
        print(f"Video saved at: {self.video_path}")

    def _frame_repeats(self):
        """Compute how many video frames to emit per stored frame based on timestamps."""
        if not self.frame_data:
            return []

        timestamps = [frame.get('timestamp') for frame in self.frame_data]
        if any(t is None for t in timestamps):
            return [1] * len(self.frame_data)

        repeats = [1] * len(self.frame_data)
        carry = 0.0
        for i in range(len(self.frame_data) - 1):
            dt = timestamps[i + 1] - timestamps[i]
            if dt < 0:
                # Non-monotonic timestamps; fall back to 1 frame per sample.
                return [1] * len(self.frame_data)
            if dt == 0:
                repeats[i] = 1
                continue
            frames_float = dt * self.fps + carry
            frames = int(round(frames_float))
            if frames < 1:
                frames = 1
            carry = frames_float - frames
            repeats[i] = frames

        # The last frame has no subsequent timestamp; emit once.
        repeats[-1] = 1
        return repeats
