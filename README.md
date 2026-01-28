# vlmnavigation

## VLM debug
Use `debug_vlm.py` (or `scripts/vlm_debug.py`) to send a single image + prompt to an OpenAI-compatible VLM endpoint.

Defaults (aligned with deploy):
- system prompt: `text_format/systemprompt.txt`
- user format: `text_format/userformat.txt`
- image directory: `debug_frames/` (first image in sorted order)
- base URL: `http://localhost:18000/v1`
- function calling enabled (`set_nav_plan`)

Example:
```
python3 debug_vlm.py \
  --task "Go to the shelf and approach the bottle." \
  --image-dir debug_frames \
  --image-index 0
```

To use a specific image file, pass `--image /path/to/your.png`.
To disable tool calling, pass `--no-function-calling`.

## Directory batch VLM (no LeRobot dataset)
`datset_load.py` now loads images from a directory instead of LeRobot datasets.

Example:
```
python3 datset_load.py \
  --image-dir debug_frames \
  --image-glob "**/*.png" \
  --count 3 \
  --prompt "Generate a short navigation plan (x,y,theta)."
```
