========================
Subsystem Interaction
========================

Purpose
-------
The Interaction layer is the robot’s **voice and ears**:

* **Microphone** – captures or fakes raw audio events.  
* **Voice Recognition** – converts those events into plain-text commands.  
* **Reasoning Order Verification** – validates customer requests before they
  enter the orchestration queue.  
* **Speaker** – delivers synthesised sentences to nearby diners.

.. image:: images/interaction_subsystem.png
   :alt: Component diagram of the Interaction subsystem
   :align: center
   :width: 90%

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 33 22 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - ``/<robot>/mic_channel`` ← *microphone.py*
     - ``std_msgs/Int32``
     - 1 Hz, latency ≤ 100 ms
   * - ``/voice_recogn`` ← *voice_recognition.py*
     - ``std_msgs/String``
     - Word error rate ≤ 5 % on test set
   * - ``/error_from_interaction`` ← *reasoning_order_verification.py*
     - ``std_msgs/Int32``
     - Code 1 (no dish) < 2 % of valid orders
   * - ``/verif_T_manager`` ← *reasoning_order_verification.py*
     - ``tiago1/Voice_rec``
     - Published ≤ 200 ms after utterance
   * - ``/speaker_channel`` ← *reasoning_speech_generation.py*
     - ``std_msgs/String``
     - One buffer slot, no drops
   * - ``/speaker_output`` ← *speaker.py*
     - ``std_msgs/String``
     - Latched; late TTS subscribers still get last sentence

Implementation modules
----------------------
Click any component for full API documentation.

.. toctree::
   :maxdepth: 1
   :caption: Interaction Components

   interaction_modules/voice_recognition
   interaction_modules/microphone
   interaction_modules/reasoning_order_verification
   interaction_modules/speaker
