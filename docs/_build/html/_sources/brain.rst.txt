==============================
Subsystem Brain
==============================

What the “Brain” does
---------------------
The Brain is the **central decision-making layer** of each TIAGo robot.  It takes in  
– high-level customer intent (orders parsed from speech),  
– real-time context (table occupancy, robot locations, task queue state), and  
– execution feedback (navigation and manipulation results)  

and synthesises a **coherent, ordered plan of actions** for the robot to follow.  
Its responsibilities include:

  1. **Task arbitration**  
     Polling the central `/robot_state_decision` service to claim and sequence tasks  
     in a multi-robot fleet, ensuring no two robots attempt the same table at once.

  2. **Motion planning handoff**  
     Emitting a minimal global path (via `/planned_path`) and table commands  
     (`PLACE`/`CLEAR`) so that downstream navigation and manipulation layers can take over.

  3. **Manipulation choreography**  
     Orchestrating a tight sequence of ActionLib goals (base move → arm motion  
     → gripper open/close), recovering from failures or pre-emptions gracefully.

  4. **Customer interaction timing**  
     Buffering and throttling spoken notifications (e.g. “Your sushi is here”)  
     so that announcements occur at natural breakpoints without overlapping.

Design Patterns
~~~~~~~~~~~~~~~~
Below are the key design patterns applied in the Brain subsystem, with explanations of their roles and benefits.

- **Strategy**  

  - **What it is**: Defines a family of interchangeable algorithms behind a common interface.  

  - **Where used**:  

    - *Reasoning Action* can swap different Table-Placement strategies (e.g. “FindPlacement” vs. “Clearing”) without changing its core logic.  
 
  - **Why**:  

    - Promotes **extensibility**—new reasoning heuristics can be added by implementing the same interface.

    - Keeps high-level orchestration code **decoupled** from specific placement algorithms.

- **Command**  

  - **What it is**: Encapsulates a request as an object, decoupling the sender of a command from its executor.  
  
  - **Where used**:  

    - Each ActionLib goal (`MovementControlAction`, `ArmControlAction`, `GripperControlAction`) wraps a motion or manipulation request.  
 
  - **Why**:  

    - Enables **queuing**, **logging**, **undo** and **retry** of discrete robot actions.  
    
    - Standardizes how tasks are issued and tracked through feedback/result callbacks.

- **Observer**  

  - **What it is**: Objects (observers) register interest in state changes or events from another object (subject).  

  - **Where used**:  

    - *Task Manager* subscribes to `/task_feedback` and reacts whenever controllers publish status updates.  
 
  - **Why**:  

    - Provides a **push-based** mechanism for keeping the orchestrator in sync with low-level controllers without tight polling loops.  
    
    - Simplifies **event-driven** reac­tion to success, failure or pre-emption.

- **Adapter**  

  - **What it is**: Converts the interface of one class into another the client expects.  

  - **Where used**:  

    - *Reasoning Table Placement* takes a high-level decision string (e.g. `"Decision: PLACE"`) and adapts it into the exact low-level command (`"PLACE_DISH"`) required by the manipulation stack.  
  
  - **Why**:  

    - Isolates **format translation** between cognitive outputs and motor command inputs.  

    - Allows the front-end reasoning logic to evolve independently of downstream command formats.

- **Template Method**  

  - **What it is**: Defines the skeleton of an algorithm in a base class, deferring some steps to subclasses.  
 
  - **Where used**:  
 
    - *Reasoning Speech Generation* implements a fixed loop—cache latest text, then republish at 1 Hz—while allowing future variants to override caching or timing behavior.  
 
  - **Why**:  
 
    - Ensures **consistent control flow** for speech throttling, while making it easy to customize individual steps (e.g. different throttling rates, latching policies) without rewriting the loop.

.. image:: images/brain_subsystem.png
   :alt: Component diagram of the Brain subsystem
   :align: center
   :width: 90%

Component roles
~~~~~~~~~~~~~~~
- **Reasoning Action**  
  - Receives a `nav_msgs/Path` (first waypoint) and a table-command string.  
  - Sends a sequence of three ActionLib goals (Command pattern):  
    1. **MovementControlAction** (drive to the table)  
    2. **ArmControlAction** (lower/lift arm)  
    3. **GripperControlAction** (open/close)  
  - Monitors feedback at each stage and retries or aborts on error.

- **Reasoning Table Placement**  
  - Subscribes to `/placement_decision` and applies a substring match.  
  - Maps `"PLACE"` → `"PLACE_DISH"`, `"CLEAR"` → `"CLEAR_TABLE"`, others → `"NO_ACTION"`.  
  - Adapts the high-level decision into a ROS command (Adapter pattern) and publishes to `/table_reasoning_commands`.

- **Task Manager**  
  - Observes `/task_feedback` messages (`Observer` pattern) from controllers (e.g. `"BASE_DONE"`, `"ARM_DONE"`).  
  - Caches the latest state and calls the `/robot_state_decision` service with it.  
  - Receives the next high-level task (order ID + dish list) and hands it to Reasoning Action.

- **Reasoning Speech Generation**  
  - Subscribes to `/task_speech_command` (text) and caches the latest message.  
  - Runs a 1 Hz republish loop (Template Method) to `/speaker_channel`, ensuring at most one announcement per second.  
  - Uses ROS latching so late subscribers still receive the current sentence.

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - **`/planned_path` → Reasoning Action**  
     - `nav_msgs/Path`  
     - First waypoint within **0.5 m** of true goal; end-to-end latency ≤ **500 ms**  
   * - **`/table_reasoning_commands` ← Table Placement**  
     - `std_msgs/String`  
     - Commands must be issued within **200 ms** of decision change  
   * - **`/task_feedback` (out)**  
     - `std_msgs/String`  
     - Propagate controller state changes within **200 ms**  
   * - **`/robot_state_decision` (srv)**  
     - `tiago1/robotstatedecision`  
     - Service response time ≤ **100 ms**, ≥ 99.9 % success rate  
   * - **`/speaker_channel` (out)**  
     - `std_msgs/String`  
     - Announcements delivered ≤ **1 s** after cognitive request  

Implementation modules
----------------------
Detailed API docs for each Brain component can be found below.

.. toctree::
   :maxdepth: 1
   :caption: Brain Components

   brain_modules/reasoning_action
   brain_modules/reasoning_table_placement
   brain_modules/task_manager
   brain_modules/reasoning_speech_generation
