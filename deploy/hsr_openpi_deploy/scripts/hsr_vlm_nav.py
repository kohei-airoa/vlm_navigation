#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import math
from pathlib import Path
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from hsr_data_msgs.srv import StringTrigger
    from hsr_data_msgs.srv import StringTriggerResponse
except Exception:  # pragma: no cover - optional dependency
    StringTrigger = None
    StringTriggerResponse = None


DEFAULT_SYSTEM_PROMPT = """
You are a household robot navigation planner. You receive:
- A head-camera image of the robot (current view)
- A user command (task)

Your job:
1) Understand the command.
2) Analyze the scene from the image.
3) Decide a short navigation plan for the robot base.

Navigation output:
- x: forward/backward displacement in meters (positive = forward)
- y: left/right displacement in meters (positive = left)
- theta: rotation in radians (positive = counterclockwise)

Controller note:
- The controller sends velocity at 10 Hz.
- Your x, y, theta will be divided by 10 to form velocities.
- Keep motions small and safe; prefer improving viewpoint or approaching the target.

Rules:
- Think privately; do NOT output step-by-step reasoning.
- You must call the function "set_nav_plan" exactly once.
""".strip()

DEFAULT_USER_FORMAT = """
The image shows the head camera view of the robot.

User command:
"{TASK}"

Provide a short navigation plan.

Call set_nav_plan with:
- x, y, theta
- confidence in [0,1]
- note: one short sentence justification
""".strip()


def request_json(url: str, method: str = "GET", payload: Optional[dict[str, Any]] = None, timeout: float = 60):
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8")
            return json.loads(body)
    except HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {e.code} from {url}: {body}")
    except URLError as e:
        raise RuntimeError(f"Failed to reach {url}: {e}")


def pick_model(base_url: str, requested: Optional[str] = None) -> str:
    if requested:
        return requested
    data = request_json(f"{base_url}/models")
    models = [m.get("id") for m in data.get("data", []) if m.get("id")]
    if not models:
        raise RuntimeError("No models returned from /v1/models")
    return models[0]


def coerce_content(content: Any) -> str:
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(item.get("text", ""))
        return "".join(parts).strip()
    if content is None:
        return ""
    return str(content)


def read_text(path: Path) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def image_to_data_url(bgr: np.ndarray) -> str:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    ok, buf = cv2.imencode(".png", rgb)
    if not ok:
        raise RuntimeError("Failed to encode image")
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return f"data:image/png;base64,{b64}"


def safe_json_load(payload: Any) -> Optional[dict[str, Any]]:
    if isinstance(payload, dict):
        return payload
    if payload is None:
        return None
    if not isinstance(payload, str):
        return None
    text = payload.strip()
    if not text:
        return None
    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            return obj
    except json.JSONDecodeError:
        pass
    start = text.find("{")
    end = text.rfind("}")
    if start != -1 and end != -1 and end > start:
        try:
            obj = json.loads(text[start : end + 1])
            if isinstance(obj, dict):
                return obj
        except json.JSONDecodeError:
            return None
    return None


class VlmNavNode:
    def __init__(self) -> None:
        self.control_hz = float(rospy.get_param("~control_hz", 10.0))
        self.plan_steps = int(rospy.get_param("~plan_steps", 10))
        self.plan_interval_s = float(
            rospy.get_param(
                "~plan_interval_s",
                self.plan_steps / self.control_hz if self.control_hz > 0 else 1.0,
            )
        )
        self.image_timeout_s = float(rospy.get_param("~image_timeout_s", 1.0))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.0))

        default_scale = 1.0 / self.control_hz if self.control_hz > 0 else 0.1
        self.linear_scale = float(rospy.get_param("~linear_scale", default_scale))
        self.angular_scale = float(rospy.get_param("~angular_scale", default_scale))

        self.max_linear = float(rospy.get_param("~max_linear", 0.0))
        self.max_angular = float(rospy.get_param("~max_angular", 0.0))
        self.stop_when_idle = bool(rospy.get_param("~stop_when_idle", True))

        self.image_topic = rospy.get_param(
            "~image_topic", "/hsrb/head_rgbd_sensor/rgb/image_rect_color/compressed"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/hsrb/command_velocity")
        self.publish_plan_topic = rospy.get_param("~publish_plan_topic", "/vlm/nav_plan")

        self.base_url = rospy.get_param("~base_url", "http://localhost:18000/v1").rstrip("/")
        self.model = rospy.get_param("~model", "")
        self.max_tokens = int(rospy.get_param("~max_tokens", 128))
        self.temperature = float(rospy.get_param("~temperature", 0.2))
        self.timeout = float(rospy.get_param("~timeout", 30.0))
        self.use_function_calling = bool(rospy.get_param("~use_function_calling", True))

        self.instruction = rospy.get_param("~instruction", "Navigate.")
        self.instruction_topic = rospy.get_param("~instruction_topic", "")
        self.instruction_service = rospy.get_param("~instruction_service", "/hsr_vlm_nav/update_instruction")

        prompt_root = Path(__file__).resolve().parents[3] / "text_format"
        default_system = prompt_root / "systemprompt.txt"
        default_user = prompt_root / "userformat.txt"

        system_path_param = rospy.get_param("~system_prompt_file", "")
        user_path_param = rospy.get_param("~user_format_file", "")

        self.system_prompt = self.load_prompt(system_path_param, default_system, DEFAULT_SYSTEM_PROMPT)
        self.user_format = self.load_prompt(user_path_param, default_user, DEFAULT_USER_FORMAT)

        self.latest_bgr: Optional[np.ndarray] = None
        self.latest_image_time: Optional[rospy.Time] = None

        self.remaining_steps = 0
        self.current_twist = Twist()
        self.last_request_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.plan_pub = None
        if self.publish_plan_topic:
            self.plan_pub = rospy.Publisher(self.publish_plan_topic, Pose2D, queue_size=1)

        self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback, queue_size=1)
        if self.instruction_topic:
            self.instruction_sub = rospy.Subscriber(
                self.instruction_topic, String, self.instruction_callback, queue_size=1
            )
        else:
            self.instruction_sub = None

        if StringTrigger and StringTriggerResponse:
            rospy.Service(self.instruction_service, StringTrigger, self.update_instruction_srv)
        elif self.instruction_service:
            rospy.logwarn("hsr_data_msgs not available; instruction service disabled")

        self.rate = rospy.Rate(self.control_hz)

        self.nav_tool = {
            "type": "function",
            "function": {
                "name": "set_nav_plan",
                "description": "Provide a short base motion plan in robot frame.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number", "description": "Forward/backward displacement in meters."},
                        "y": {"type": "number", "description": "Left/right displacement in meters."},
                        "theta": {"type": "number", "description": "Rotation in radians (CCW positive)."},
                        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                        "note": {"type": "string"},
                    },
                    "required": ["x", "y", "theta", "confidence", "note"],
                },
            },
        }

        rospy.loginfo(
            "VLM nav node ready: image_topic=%s cmd_vel_topic=%s base_url=%s",
            self.image_topic,
            self.cmd_vel_topic,
            self.base_url,
        )

    def load_prompt(self, param_value: str, default_path: Path, fallback: str) -> str:
        if param_value:
            path = Path(param_value)
            if path.exists():
                return read_text(path)
            rospy.logwarn("Prompt file not found: %s", path)
        if default_path.exists():
            return read_text(default_path)
        return fallback

    def instruction_callback(self, msg: String) -> None:
        self.instruction = msg.data

    def update_instruction_srv(self, req: StringTrigger):
        self.instruction = req.message
        rospy.loginfo("Instruction updated: %s", self.instruction)
        return StringTriggerResponse(success=True)

    def image_callback(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_bgr is None:
            return
        self.latest_bgr = image_bgr
        stamp = msg.header.stamp if msg.header and msg.header.stamp else rospy.Time.now()
        self.latest_image_time = stamp

    def ensure_model(self) -> bool:
        if self.model:
            return True
        try:
            self.model = pick_model(self.base_url)
            return True
        except Exception as exc:
            rospy.logwarn("Failed to pick model: %s", exc)
            return False

    def build_messages(self, image_url: str) -> list[dict[str, Any]]:
        user_prompt = self.user_format.replace("{TASK}", self.instruction)
        parts = [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {"url": image_url}},
        ]
        messages = []
        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})
        messages.append({"role": "user", "content": parts})
        return messages

    def query_vlm(self, image_url: str) -> dict[str, Any]:
        messages = self.build_messages(image_url)
        payload: dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
        }
        if self.use_function_calling:
            payload["tools"] = [self.nav_tool]
            payload["tool_choice"] = {"type": "function", "function": {"name": "set_nav_plan"}}
        return request_json(
            f"{self.base_url}/chat/completions", method="POST", payload=payload, timeout=self.timeout
        )

    def parse_nav_plan(self, response: dict[str, Any]) -> Optional[tuple[float, float, float, float, str]]:
        choices = response.get("choices", [])
        if not choices:
            return None
        message = choices[0].get("message", {})
        args = None

        tool_calls = message.get("tool_calls")
        if isinstance(tool_calls, list) and tool_calls:
            args = tool_calls[0].get("function", {}).get("arguments")
        elif "function_call" in message:
            args = message.get("function_call", {}).get("arguments")

        if args is None:
            args = coerce_content(message.get("content"))

        data = safe_json_load(args)
        if not isinstance(data, dict):
            return None

        try:
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            theta = float(data.get("theta", 0.0))
            confidence = float(data.get("confidence", 0.0))
            note = str(data.get("note", ""))
        except Exception:
            return None

        return x, y, theta, confidence, note

    def plan_to_twist(self, plan: Pose2D) -> Twist:
        twist = Twist()
        vx = plan.x * self.linear_scale
        vy = plan.y * self.linear_scale
        wz = plan.theta * self.angular_scale

        if self.max_linear > 0:
            vx = max(min(vx, self.max_linear), -self.max_linear)
            vy = max(min(vy, self.max_linear), -self.max_linear)
        if self.max_angular > 0:
            wz = max(min(wz, self.max_angular), -self.max_angular)

        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0

        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        return twist

    def accept_plan(self, x: float, y: float, theta: float, confidence: float, note: str) -> None:
        plan = Pose2D(x=x, y=y, theta=theta)
        self.current_twist = self.plan_to_twist(plan)
        self.remaining_steps = max(self.plan_steps, 1)
        if self.plan_pub:
            self.plan_pub.publish(plan)
        rospy.loginfo("VLM plan: x=%.3f y=%.3f th=%.3f conf=%.2f note=%s", x, y, theta, confidence, note)

    def maybe_request_plan(self) -> None:
        now = rospy.Time.now()
        if (now - self.last_request_time).to_sec() < self.plan_interval_s:
            return
        if self.latest_bgr is None or self.latest_image_time is None:
            return
        if (now - self.latest_image_time).to_sec() > self.image_timeout_s:
            return
        if not self.ensure_model():
            return

        try:
            image_url = image_to_data_url(self.latest_bgr)
            response = self.query_vlm(image_url)
        except Exception as exc:
            rospy.logwarn("VLM request failed: %s", exc)
            self.last_request_time = now
            return

        self.last_request_time = now
        plan = self.parse_nav_plan(response)
        if plan is None:
            rospy.logwarn("VLM response missing plan")
            return
        x, y, theta, confidence, note = plan
        if confidence < self.min_confidence:
            rospy.logwarn("VLM plan below confidence threshold: %.2f", confidence)
            return
        self.accept_plan(x, y, theta, confidence, note)

    def step(self) -> None:
        if self.remaining_steps > 0:
            self.cmd_pub.publish(self.current_twist)
            self.remaining_steps -= 1
            return

        self.maybe_request_plan()

        if self.remaining_steps > 0:
            self.cmd_pub.publish(self.current_twist)
            self.remaining_steps -= 1
            return

        if self.stop_when_idle:
            self.cmd_pub.publish(Twist())

    def spin(self) -> None:
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


def main() -> None:
    rospy.init_node("hsr_vlm_nav")
    node = VlmNavNode()
    node.spin()


if __name__ == "__main__":
    main()
