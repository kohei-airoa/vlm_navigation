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

## Deploy VLM navigation
The deploy node `hsr_vlm_nav.py` calls the VLM internally and publishes base velocity.

Start:
```
roslaunch hsr_openpi hsr_vlm_nav.launch
```

### Update instruction (task name)
Topic (default):
```
rostopic pub /hsr_vlm_nav/instruction std_msgs/String "data: 'Go to the shelf and approach the bottle.'" -1
```

Service (if `hsr_data_msgs` is available):
```
rosservice call /hsr_vlm_nav/update_instruction "message: 'Go to the shelf.'"
```

### Check VLM input/output
Enable logging with:
```
roslaunch hsr_openpi hsr_vlm_nav.launch print_vlm_io:=true
```
This prints the instruction, system/user prompts, image info, and the VLM output/tool args.

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
