---
sidebar_position: 5
title: "Milestone 4: AI Integration"
description: Add AI capabilities to your robot
---

# Milestone 4: AI Integration

Integrate AI capabilities for perception and planning.

## Objectives

- Add object detection
- Implement LLM planning
- Create voice interface
- Build end-to-end demo

## Deliverables

### 1. Perception Pipeline

```python
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/robot/head_camera/image_raw',
            self.image_callback, 10
        )

        # Publish detections
        self.detection_pub = self.create_publisher(
            DetectionArray, '/detections', 10
        )

        # Load detection model
        self.model = load_detection_model()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        detections = self.model.predict(image)
        self.detection_pub.publish(detections)
```

### 2. LLM Planner

```python
class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        self.llm = OpenAI()

        self.command_sub = self.create_subscription(
            String, '/voice_command',
            self.plan_callback, 10
        )

    def plan_callback(self, msg):
        # Get scene context
        detections = self.get_current_detections()

        # Create prompt with context
        prompt = self.create_prompt(msg.data, detections)

        # Get plan from LLM
        plan = self.llm.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        # Execute plan
        self.execute_plan(plan)
```

### 3. Voice Interface

```python
class VoiceInterfaceNode(Node):
    def __init__(self):
        super().__init__('voice_interface')

        self.whisper = whisper.load_model("base")

        self.command_pub = self.create_publisher(
            String, '/voice_command', 10
        )

        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()

    def listen_loop(self):
        while rclpy.ok():
            audio = self.record_audio(duration=5.0)
            text = self.whisper.transcribe(audio)

            if self.is_robot_command(text):
                msg = String(data=text)
                self.command_pub.publish(msg)
```

## Integration Testing

Test the full pipeline:

1. Say "Pick up the red cup"
2. Verify voice recognition
3. Check LLM generates valid plan
4. Watch robot execute in simulation

---

Continue to [Milestone 5: Presentation](/docs/capstone/milestone-5).
