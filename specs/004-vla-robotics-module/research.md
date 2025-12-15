# Research: LLM for Cognitive Planning in VLA Robotics

**Date**: 2025-12-15
**Feature**: `004-vla-robotics-module`

## Decision: Use OpenAI API (GPT-4 or GPT-3.5-turbo) for Cognitive Planning

After researching the available options for Large Language Models (LLMs) in robotics task planning, the decision is to use the OpenAI API (specifically GPT-4 or GPT-3.5-turbo) as the primary tool for the cognitive planning component of this educational module.

### Rationale

The primary goals for this educational module are to teach the concepts of Vision-Language-Action (VLA) robotics and to ensure the material is accessible to students. The choice of the OpenAI API supports these goals for the following reasons:

1.  **Accessibility and Ease of Use**: The OpenAI API is well-documented and widely used, making it one of the easiest ways to integrate powerful LLM capabilities into a project. Students can get started quickly by obtaining an API key and using standard HTTP libraries, allowing them to focus on the VLA pipeline logic rather than on complex model deployment and setup.
2.  **High Performance**: GPT-4 and its variants have demonstrated strong performance in natural language understanding, reasoning, and code generation. This is crucial for translating high-level commands (e.g., "pick up the box and move it to the table") into accurate and logical sequences of ROS 2 actions.
3.  **Cost-Effectiveness for Education**: The pay-as-you-go model of the OpenAI API is suitable for the scale of an educational project. The number of API calls required for completing the module's labs and capstone project will be relatively low, keeping costs minimal for students.
4.  **Focus on Core Concepts**: By abstracting away the complexities of hosting and running a large model, students can better focus on the core VLA concepts being taught, such as prompt engineering for robotics, parsing LLM outputs, and integrating the planner with ROS 2.

### Alternatives Considered

-   **Local, Open-Source LLMs (e.g., Llama3, Mistral)**:
    -   **Pros**: No external API dependency, no per-request cost, and can run offline.
    -   **Cons**: These models require significant local computational resources (GPU with substantial VRAM), and the setup can be complex (e.g., managing model weights, quantization, and setting up an inference server). This could create a significant barrier to entry for students with less powerful hardware and distract from the primary learning objectives of the module.
-   **Specialized Robotics LLM Frameworks (e.g., `ROS-LLM`)**:
    -   **Pros**: These frameworks provide pre-built integrations between ROS 2 and LLMs.
    -   **Cons**: They can add another layer of abstraction that might obscure the fundamental principles of how the LLM interacts with the robotic system. For a foundational module, it is more instructive for students to build the integration logic themselves to fully understand the voice-to-action pipeline.

### Conclusion

The OpenAI API provides the best balance of performance, accessibility, and cost for this educational context. The implementation will involve creating a ROS 2 node that takes transcribed text as input, formats it into a prompt for the OpenAI API, sends the request, and then parses the response to generate a sequence of actions for the robot. This approach directly supports the learning objectives of the module while minimizing unnecessary technical overhead for the students.
