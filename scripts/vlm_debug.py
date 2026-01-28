#!/usr/bin/env python3
import argparse
import base64
import json
import mimetypes
import os
import sys
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError


def request_json(url, method="GET", payload=None, timeout=3600):
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


def encode_image(path):
    if not os.path.isfile(path):
        raise RuntimeError(f"Image file not found: {path}")
    mime, _ = mimetypes.guess_type(path)
    if not mime or not mime.startswith("image/"):
        mime = "image/png"
    with open(path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return f"data:{mime};base64,{b64}"


def list_images(directory):
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".webp", ".tif", ".tiff"}
    root = Path(directory)
    if not root.exists():
        return []
    files = [p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in exts]
    return sorted(files)


def resolve_image_path(image_path, image_dir, image_index):
    if image_path:
        return Path(image_path)
    if not image_dir:
        return None
    images = list_images(image_dir)
    if not images:
        raise RuntimeError(f"No images found in directory: {image_dir}")
    idx = max(0, min(image_index, len(images) - 1))
    return images[idx]


def pick_model(base_url, requested=None):
    if requested:
        return requested
    models_url = f"{base_url}/models"
    data = request_json(models_url)
    models = [m.get("id") for m in data.get("data", []) if m.get("id")]
    if not models:
        raise RuntimeError(f"No models returned from {models_url}")
    return models[0]


def coerce_content(content):
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(item.get("text", ""))
        return "".join(parts).strip()
    if content is None:
        return ""
    return str(content)


def read_text(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def print_input_summary(base_url, model, image_path, system_prompt, user_prompt):
    print("# input")
    print(f"base_url: {base_url}")
    print(f"model: {model}")
    print(f"image: {image_path if image_path else 'none'}")
    if system_prompt:
        print("system_prompt:")
        print(system_prompt)
    if user_prompt:
        print("user_prompt:")
        print(user_prompt)
    print("# end input")


def main():
    parser = argparse.ArgumentParser(
        description="Debug an OpenAI-compatible VLM endpoint via /v1/chat/completions"
    )
    parser.add_argument(
        "--base-url",
        default="http://localhost:18000/v1",
        help="OpenAI-compatible base URL (default: http://localhost:18000/v1)",
    )
    parser.add_argument("--model", help="Model id (defaults to first /v1/models entry)")
    parser.add_argument("--prompt", default="Navigate.")
    repo_root = Path(__file__).resolve().parents[1]
    default_image_dir = repo_root / "debug_frames"
    default_system_prompt = repo_root / "text_format" / "systemprompt.txt"
    default_user_format = repo_root / "text_format" / "userformat.txt"

    parser.add_argument("--image", help="Path to local image file (optional)")
    parser.add_argument(
        "--image-dir",
        default=str(default_image_dir) if default_image_dir.exists() else "",
        help="Directory of images (defaults to ./debug_frames if present)",
    )
    parser.add_argument(
        "--image-index",
        type=int,
        default=0,
        help="Index into the image directory (default: 0)",
    )
    parser.add_argument("--system", help="System prompt override (optional)")
    parser.add_argument("--task", help="Task command (fills {TASK} in user format)")
    parser.add_argument(
        "--system-prompt-file",
        default=str(default_system_prompt),
        help="Path to system prompt file (defaults to text_format/systemprompt.txt)",
    )
    parser.add_argument(
        "--user-format-file",
        default=str(default_user_format),
        help="Path to user prompt format file (defaults to text_format/userformat.txt)",
    )
    parser.add_argument(
        "--use-function-calling",
        action="store_true",
        default=True,
        help="Enable function calling with set_nav_plan (default: true)",
    )
    parser.add_argument(
        "--no-function-calling",
        dest="use_function_calling",
        action="store_false",
        help="Disable function calling",
    )
    parser.add_argument("--max-tokens", type=int, default=256)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--timeout", type=int, default=3600)
    args = parser.parse_args()

    base_url = args.base_url.rstrip("/")
    model = pick_model(base_url, args.model)

    system_prompt = args.system
    if not system_prompt:
        system_path = Path(args.system_prompt_file)
        if system_path.exists():
            system_prompt = read_text(system_path)

    user_format = None
    user_format_path = Path(args.user_format_file)
    if user_format_path.exists():
        user_format = read_text(user_format_path)

    task_text = args.task or args.prompt
    if user_format:
        user_prompt = user_format.replace("{TASK}", task_text)
    else:
        user_prompt = args.prompt

    messages = []
    if system_prompt:
        messages.append({"role": "system", "content": system_prompt})

    image_path = resolve_image_path(args.image, args.image_dir, args.image_index)
    if image_path:
        image_url = encode_image(str(image_path))
        user_content = [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {"url": image_url}},
        ]
    else:
        user_content = user_prompt

    messages.append({"role": "user", "content": user_content})

    print_input_summary(base_url, model, image_path, system_prompt, user_prompt)

    payload = {
        "model": model,
        "messages": messages,
        "max_tokens": args.max_tokens,
        "temperature": args.temperature,
    }
    if args.use_function_calling:
        payload["tools"] = [
            {
                "type": "function",
                "function": {
                    "name": "set_nav_plan",
                    "description": "Provide a short base motion plan in robot frame.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "x": {
                                "type": "number",
                                "description": "Forward/backward displacement in meters.",
                            },
                            "y": {"type": "number", "description": "Left/right displacement in meters."},
                            "theta": {
                                "type": "number",
                                "description": "Rotation in radians (CCW positive).",
                            },
                            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
                            "note": {"type": "string"},
                        },
                        "required": ["x", "y", "theta", "confidence", "note"],
                    },
                },
            }
        ]
        payload["tool_choice"] = {"type": "function", "function": {"name": "set_nav_plan"}}

    chat_url = f"{base_url}/chat/completions"
    response = request_json(chat_url, method="POST", payload=payload, timeout=args.timeout)

    choices = response.get("choices", [])
    if not choices:
        print(json.dumps(response, indent=2))
        return 0

    message = choices[0].get("message", {})
    tool_calls = message.get("tool_calls") or []
    if isinstance(tool_calls, list) and tool_calls:
        args_text = tool_calls[0].get("function", {}).get("arguments")
        if args_text:
            print(args_text)
            return 0
    content = coerce_content(message.get("content"))
    print(content or json.dumps(response, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
