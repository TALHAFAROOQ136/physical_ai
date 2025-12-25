---
sidebar_position: 3
title: LLM Planning
description: Task planning with large language models
---

# LLM Planning

Large Language Models can decompose high-level commands into executable robot actions.

## Learning Objectives

- Connect LLMs to robot systems
- Implement task decomposition
- Create action primitives
- Handle errors and replanning

## Task Decomposition

```python
from openai import OpenAI

client = OpenAI()

SYSTEM_PROMPT = """You are a robot task planner. Given a high-level command,
break it down into a sequence of primitive actions.

Available primitives:
- move_to(location)
- pick_up(object)
- place(object, location)
- open(container)
- close(container)

Respond with a JSON list of actions."""

def plan_task(command: str) -> list[dict]:
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": command}
        ],
        response_format={"type": "json_object"}
    )

    import json
    return json.loads(response.choices[0].message.content)
```

## Example Planning

**Input:** "Put the red cup in the dishwasher"

**Output:**
```json
{
  "actions": [
    {"action": "move_to", "args": {"location": "kitchen_counter"}},
    {"action": "pick_up", "args": {"object": "red_cup"}},
    {"action": "move_to", "args": {"location": "dishwasher"}},
    {"action": "open", "args": {"container": "dishwasher"}},
    {"action": "place", "args": {"object": "red_cup", "location": "dishwasher_rack"}},
    {"action": "close", "args": {"container": "dishwasher"}}
  ]
}
```

## Action Execution

```python
class ActionExecutor:
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.primitives = {
            "move_to": self.robot.navigate_to,
            "pick_up": self.robot.grasp,
            "place": self.robot.release,
            "open": self.robot.open_gripper,
            "close": self.robot.close_gripper,
        }

    def execute_plan(self, plan: list[dict]):
        for action in plan["actions"]:
            func = self.primitives[action["action"]]
            result = func(**action["args"])

            if not result.success:
                # Replan on failure
                new_plan = self.replan(action, result.error)
                return self.execute_plan(new_plan)
```

## Grounding with Vision

```python
def ground_objects(image, command: str) -> dict:
    """Use vision-language model to ground objects in image."""
    from transformers import CLIPProcessor, CLIPModel

    model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
    processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

    # Extract object references from command
    objects = extract_objects(command)

    # Find objects in image
    grounded = {}
    for obj in objects:
        inputs = processor(
            text=[f"a photo of a {obj}"],
            images=image,
            return_tensors="pt"
        )
        outputs = model(**inputs)
        grounded[obj] = outputs.logits_per_image

    return grounded
```

## Exercise

Build an LLM planner that:
1. Takes voice commands
2. Generates action sequences
3. Verifies actions are safe
4. Executes on a simulated robot

---

Congratulations! You've completed Module 4. Now it's time for the [Capstone Project](/docs/capstone)!
