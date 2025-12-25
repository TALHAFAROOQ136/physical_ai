"""Capstone troubleshooting embeddings script.

This script indexes capstone-specific troubleshooting content for enhanced
chatbot support during capstone project work.

Run with: python -m scripts.index_capstone [--dry-run]
"""

import argparse
import asyncio
import sys
from pathlib import Path
from uuid import uuid4

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lib.config import get_settings
from src.lib.qdrant import (
    upsert_chunks,
    delete_by_chapter,
)
from src.services.embedding_service import (
    get_embeddings_batch,
    generate_chunk_id,
)

settings = get_settings()

# Capstone troubleshooting content organized by topic
CAPSTONE_TROUBLESHOOTING = {
    "capstone-troubleshoot-ros2": {
        "module_id": "capstone",
        "title": "ROS 2 Troubleshooting for Capstone",
        "chunks": [
            {
                "title": "Node Communication Issues",
                "text": """Common ROS 2 node communication problems and solutions:

1. **Nodes not discovering each other**
   - Check if ROS_DOMAIN_ID is the same across all terminals
   - Verify network connectivity with `ros2 doctor --report`
   - Ensure you've sourced the workspace: `source install/setup.bash`

2. **Topics not connecting**
   - Use `ros2 topic list` to see available topics
   - Check QoS compatibility between publishers and subscribers
   - Verify topic names match exactly (case-sensitive)

3. **Service calls timing out**
   - Ensure the service server is running: `ros2 service list`
   - Check if the service name matches the client's target
   - Verify network firewall settings allow DDS communication

4. **Action server not responding**
   - Verify action server is running: `ros2 action list`
   - Check action interface definitions match
   - Ensure goal callbacks are implemented correctly""",
            },
            {
                "title": "Launch File Issues",
                "text": """ROS 2 launch file troubleshooting:

1. **Package not found error**
   - Ensure package is built: `colcon build --packages-select <pkg>`
   - Source the workspace after building
   - Check package name in setup.py matches launch reference

2. **Node executable not found**
   - Verify entry point in setup.py console_scripts
   - Rebuild after changes: `colcon build`
   - Check executable permissions

3. **Parameter loading fails**
   - Verify YAML syntax is correct
   - Use absolute paths or package share paths
   - Check parameter names match node declarations

4. **Launch arguments not working**
   - Use DeclareLaunchArgument for command-line args
   - Access with LaunchConfiguration
   - Check default values are set correctly""",
            },
        ],
    },
    "capstone-troubleshoot-gazebo": {
        "module_id": "capstone",
        "title": "Gazebo Troubleshooting for Capstone",
        "chunks": [
            {
                "title": "Gazebo Plugin Issues",
                "text": """Common Gazebo plugin problems:

1. **Plugin not loading**
   - Check plugin filename matches installed library
   - Verify GAZEBO_PLUGIN_PATH includes your plugin path
   - Look for errors in terminal output when Gazebo starts

2. **Camera not publishing images**
   - Verify sensor type is "camera" in URDF
   - Check topic namespace matches expected
   - Ensure update_rate is set appropriately

3. **Robot falling through floor**
   - Add collision geometry to all links
   - Set appropriate inertia values
   - Check ground plane is at z=0

4. **Physics instabilities**
   - Reduce simulation step size
   - Add damping to joints
   - Simplify collision meshes""",
            },
            {
                "title": "World File Issues",
                "text": """Gazebo world file troubleshooting:

1. **Model not loading**
   - Verify model path is correct
   - Check GAZEBO_MODEL_PATH environment variable
   - Download missing models: `gazebo --verbose` shows errors

2. **Lighting problems**
   - Ensure sun model is included
   - Check shadow settings
   - Verify material shaders are installed

3. **Performance issues**
   - Reduce model complexity
   - Lower texture resolution
   - Disable unnecessary physics calculations
   - Use GPU acceleration if available""",
            },
        ],
    },
    "capstone-troubleshoot-isaac": {
        "module_id": "capstone",
        "title": "Isaac Sim Troubleshooting for Capstone",
        "chunks": [
            {
                "title": "Isaac Sim Installation Issues",
                "text": """NVIDIA Isaac Sim setup troubleshooting:

1. **Driver compatibility**
   - Require NVIDIA driver 525.60.11 or newer
   - Check with: `nvidia-smi`
   - Update drivers from NVIDIA website

2. **Insufficient GPU memory**
   - Isaac Sim requires RTX 2070+ with 8GB+ VRAM
   - Close other GPU-intensive applications
   - Reduce viewport resolution

3. **ROS 2 bridge not working**
   - Enable ROS 2 extension in Isaac Sim
   - Source ROS 2 before launching Isaac Sim
   - Check ROS_DISTRO matches (Humble recommended)

4. **Import errors**
   - Verify Omniverse Nucleus is running
   - Check asset paths are correct
   - Clear cache: ~/.local/share/ov/data""",
            },
            {
                "title": "Perception Pipeline Issues",
                "text": """Isaac Sim perception troubleshooting:

1. **Camera data not publishing**
   - Enable camera sensor in Isaac Sim
   - Configure ROS 2 camera publisher
   - Check frame rate settings

2. **Point cloud empty**
   - Verify depth camera is enabled
   - Check sensor range settings
   - Ensure scene has visible geometry

3. **Object detection failing**
   - Verify model weights are loaded
   - Check input image format matches model expectation
   - Adjust confidence threshold

4. **Semantic segmentation issues**
   - Enable semantic labels in Isaac Sim
   - Configure label mapping
   - Verify synthetic data generation settings""",
            },
        ],
    },
    "capstone-troubleshoot-ai": {
        "module_id": "capstone",
        "title": "AI Integration Troubleshooting for Capstone",
        "chunks": [
            {
                "title": "LLM Integration Issues",
                "text": """LLM integration troubleshooting for robotics:

1. **API rate limiting**
   - Implement exponential backoff
   - Cache repeated queries
   - Use batch requests where possible

2. **Response latency too high**
   - Use streaming responses
   - Reduce context window size
   - Consider using faster models for simple queries

3. **Invalid action plans**
   - Improve prompt engineering
   - Add output validation
   - Include examples in system prompt
   - Use function calling for structured output

4. **Context window exceeded**
   - Summarize conversation history
   - Limit retrieved context chunks
   - Use sliding window for long conversations""",
            },
            {
                "title": "Voice Interface Issues",
                "text": """Voice recognition troubleshooting:

1. **Whisper not recognizing speech**
   - Check microphone input levels
   - Try different model sizes (base, small, medium)
   - Ensure audio sample rate is 16kHz

2. **High latency**
   - Use local Whisper for faster inference
   - Consider streaming recognition
   - Optimize audio buffer size

3. **Background noise issues**
   - Apply noise reduction preprocessing
   - Use noise-robust model variant
   - Add voice activity detection (VAD)

4. **Wake word detection**
   - Implement separate wake word model
   - Use always-on lightweight detector
   - Configure sensitivity threshold""",
            },
        ],
    },
    "capstone-troubleshoot-integration": {
        "module_id": "capstone",
        "title": "System Integration Troubleshooting for Capstone",
        "chunks": [
            {
                "title": "End-to-End Pipeline Issues",
                "text": """End-to-end system integration troubleshooting:

1. **Components not synchronized**
   - Use ROS 2 message filters for synchronization
   - Implement proper timestamp handling
   - Add synchronization barriers where needed

2. **System bottlenecks**
   - Profile with ROS 2 tracing
   - Identify slow nodes with `ros2 node list -a`
   - Monitor CPU/GPU usage

3. **State machine issues**
   - Use behavior trees or state machines
   - Implement proper state transitions
   - Add timeout handling for all states

4. **Error recovery**
   - Implement watchdog nodes
   - Add graceful degradation
   - Log errors comprehensively
   - Create automatic recovery procedures""",
            },
            {
                "title": "Testing and Validation",
                "text": """Capstone testing best practices:

1. **Unit testing**
   - Write tests for individual nodes
   - Mock external dependencies
   - Use pytest with colcon test

2. **Integration testing**
   - Test node communication
   - Validate message types
   - Check error handling

3. **Simulation validation**
   - Compare sim vs expected behavior
   - Test edge cases safely
   - Record and replay scenarios

4. **Demo preparation**
   - Practice full pipeline
   - Prepare failure recovery scripts
   - Have backup demonstration plan
   - Test with fresh environment setup""",
            },
        ],
    },
}


async def index_troubleshooting(dry_run: bool = False) -> int:
    """
    Index capstone troubleshooting content.

    Returns:
        Number of chunks indexed
    """
    total_chunks = 0

    for chapter_id, content in CAPSTONE_TROUBLESHOOTING.items():
        print(f"Processing: {content['title']}")

        # Delete existing chunks for this chapter
        if not dry_run:
            await delete_by_chapter(chapter_id)

        # Prepare chunks for embedding
        chunk_texts = [chunk["text"] for chunk in content["chunks"]]

        if dry_run:
            print(f"  Would index {len(chunk_texts)} chunks")
            total_chunks += len(chunk_texts)
            continue

        # Generate embeddings
        embeddings = await get_embeddings_batch(chunk_texts)

        # Prepare points for Qdrant
        points = []
        for i, (chunk, embedding) in enumerate(zip(content["chunks"], embeddings)):
            point_id = generate_chunk_id(chapter_id, i)
            points.append({
                "id": point_id,
                "vector": embedding,
                "payload": {
                    "module_id": content["module_id"],
                    "chapter_id": chapter_id,
                    "title": f"{content['title']} - {chunk['title']}",
                    "slug": chapter_id,
                    "text": chunk["text"],
                    "chunk_index": i,
                    "difficulty": "advanced",
                    "type": "troubleshooting",
                },
            })

        # Upsert to Qdrant
        await upsert_chunks(points)
        print(f"  Indexed {len(points)} chunks")
        total_chunks += len(points)

    return total_chunks


async def main(dry_run: bool = False):
    """Main function."""
    print("=" * 60)
    print("Capstone Troubleshooting Indexing Script")
    print("=" * 60)
    print()
    print(f"Dry run: {dry_run}")
    print()

    total_chunks = await index_troubleshooting(dry_run)

    print()
    print("=" * 60)
    print("Indexing complete!")
    print(f"  Total chunks: {total_chunks}")
    if dry_run:
        print("  (Dry run - no data written)")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Index capstone troubleshooting content"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without writing to Qdrant",
    )
    args = parser.parse_args()

    asyncio.run(main(dry_run=args.dry_run))
