#!/usr/bin/env python3
"""Load images from a directory and query an OpenAI-compatible VLM endpoint."""

import argparse
import base64
import json
import mimetypes
import sys
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def request_json(url, method="GET", payload=None, timeout=60):
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


def pick_model(base_url, requested=None):
    if requested:
        return requested
    data = request_json(f"{base_url}/models")
    models = [m.get("id") for m in data.get("data", []) if m.get("id")]
    if not models:
        raise RuntimeError("No models returned from /v1/models")
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


def encode_image(path):
    if not path.is_file():
        raise RuntimeError(f"Image file not found: {path}")
    mime, _ = mimetypes.guess_type(str(path))
    if not mime or not mime.startswith("image/"):
        mime = "image/png"
    with open(path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return f"data:{mime};base64,{b64}"


def list_images(directory, glob_pattern):
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".webp", ".tif", ".tiff"}
    root = Path(directory)
    if not root.exists():
        return []
    if glob_pattern:
        candidates = list(root.rglob(glob_pattern))
    else:
        candidates = list(root.rglob("*"))
    images = [p for p in candidates if p.is_file() and p.suffix.lower() in exts]
    return sorted(images)


def main():
    parser = argparse.ArgumentParser(
        description="Load images from a directory and query an OpenAI-compatible VLM endpoint"
    )
    repo_root = Path(__file__).resolve().parent
    default_image_dir = repo_root / "debug_frames"

    parser.add_argument(
        "--image-dir",
        default=str(default_image_dir) if default_image_dir.exists() else "",
        help="Directory containing images (defaults to ./debug_frames if present)",
    )
    parser.add_argument(
        "--image-glob",
        default="",
        help="Optional glob pattern under image-dir (e.g. '**/*.png')",
    )
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)

    parser.add_argument("--prompt", default="Describe the image.")
    parser.add_argument("--task", help="Override task command (fills {TASK} in user format)")
    parser.add_argument("--system", help="System prompt override (optional)")
    parser.add_argument(
        "--system-prompt-file",
        default=str(repo_root / "text_format" / "systemprompt.txt"),
        help="Path to system prompt file",
    )
    parser.add_argument(
        "--user-format-file",
        default=str(repo_root / "text_format" / "userformat.txt"),
        help="Path to user prompt format file (expects {TASK})",
    )
    parser.add_argument(
        "--base-url",
        default="http://localhost:18000/v1",
        help="OpenAI-compatible base URL (default: http://localhost:18000/v1)",
    )
    parser.add_argument("--model", help="Model id (defaults to first /v1/models entry)")
    parser.add_argument("--max-tokens", type=int, default=256)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--timeout", type=int, default=60)
    args = parser.parse_args()

    if not args.image_dir:
        raise RuntimeError("--image-dir is required (no default image directory found)")

    base_url = args.base_url.rstrip("/")
    model = pick_model(base_url, args.model)

    system_prompt = args.system
    if not system_prompt:
        system_path = Path(args.system_prompt_file)
        if system_path.exists():
            system_prompt = read_text(system_path)

    user_format_path = Path(args.user_format_file)
    if not user_format_path.exists():
        raise RuntimeError(f"User format file not found: {user_format_path}")
    user_format = read_text(user_format_path)

    images = list_images(args.image_dir, args.image_glob)
    if not images:
        raise RuntimeError(f"No images found in {args.image_dir}")

    for offset in range(args.count):
        idx = args.index + offset
        if idx < 0 or idx >= len(images):
            raise RuntimeError(f"Image index {idx} out of range (0..{len(images)-1})")
        image_path = images[idx]

        task_text = args.task or args.prompt
        user_prompt = user_format.replace("{TASK}", task_text)

        parts = [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {"url": encode_image(image_path)}},
        ]

        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": parts})

        payload = {
            "model": model,
            "messages": messages,
            "max_tokens": args.max_tokens,
            "temperature": args.temperature,
        }

        response = request_json(
            f"{base_url}/chat/completions", method="POST", payload=payload, timeout=args.timeout
        )

        choices = response.get("choices", [])
        if not choices:
            print(json.dumps({"index": idx, "image": str(image_path), "response": response}, indent=2))
            continue

        content = coerce_content(choices[0].get("message", {}).get("content"))
        print(f"# index={idx} image={image_path}")
        print(content or json.dumps(response, indent=2))

    return 0


if __name__ == "__main__":
    sys.exit(main())
