=============================
Subsystem Server
=============================

What the Server subsystem does
------------------------------
The central Server component, **orchestration_and_coordination.py**, manages the global workflow across all TIAGo robots:

1. **Order ingestion**  
   Exposes `/robot_state_decision_add` (tiago1/send_order) so that verified orders arrive and are appended to a **persistent FIFO** (`tiago_data.yaml`).

2. **Order dispatch**  
   Implements `/robot_state_decision` (tiago1/robotstatedecision) which robots call with their current state (`Free`, `Wait`, etc.).  
   - If the robot is `Free` and the queue is nonempty, the server **pops** the next order and returns it with `Busy`.  
   - If the queue is empty, returns `Wait`.  
   - If the robot is already `Busy`, acknowledges without popping.

3. **Persistence & resilience**  
   All changes (adds/removes) are flushed immediately to YAML.  On restart the server recovers the queue in <1 s, and a corrupted file triggers automatic reconstruction from a backup.

.. image:: images/server_subsystem.png
   :alt: Deployment diagram – server, robots and services
   :align: center
   :width: 85%

Design Patterns
---------------

Singleton
~~~~~~~~~
In our server, the `OrderQueue` class wraps all YAML I/O and in-memory queue state.  By making it a singleton, every service callback, recovery routine and backup handler uses the same instance, preventing concurrent writes and ensuring **consistency** across the node.

Strategy
~~~~~~~~
Currently the server uses a FIFO strategy (`FirstInFirstOutStrategy`), but by abstracting dispatch logic into a pluggable `DispatchStrategy` interface, we could swap in a **priority-based** strategy (e.g., urgent orders first) or **round-robin** across tables without touching the service handlers themselves.

Template Method
~~~~~~~~~~~~~~~
The `ServiceHandler` base class in the server defines:
  1. **authenticate & validate** request  
  2. **execute** (add or dispatch)  
  3. **format** and **return** response  
By overriding only the `execute()` hook for `AddOrderHandler` and `DispatchOrderHandler`, we enforce a consistent flow—validation, execution, persistence, response—for both services.

Command
~~~~~~~
Every incoming `send_orderRequest` or `robotstatedecisionRequest` is wrapped in a `Command` subclass (`AddOrderCommand`, `DispatchOrderCommand`) with an `execute()` method.  This allows uniform logging, retry logic, or queuing of failed commands without duplicating service code.

Observer
~~~~~~~~
The `OrderQueue` singleton notifies registered observers (e.g., a backup manager, a metrics logger) whenever an order is added or removed.  Observers then perform side‐effects—writing backups, updating monitoring dashboards—**automatically**, keeping the core queue logic free of ancillary concerns.


ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 35 25 40

   * - Service / Topic
     - Type
     - KPI / Note
   * - ``/robot_state_decision`` (srv)
     - ``tiago1/robotstatedecision``
     - ≥ 99.9 % success, response < 100 ms; returning `Busy` only when queue ≥ 1
   * - ``/robot_state_decision_add`` (srv)
     - ``tiago1/send_order``
     - YAML write < 50 ms, supports ≥ 20 req/s
   * - ``tiago_data.yaml`` (file I/O)
     - YAML
     - Survive restart; corruption detection & recovery < 1 s

Data life-cycle
---------------
#. **Order added** – `send_order` service appends to in-memory queue and saves YAML.  
#. **Robot query** – `robotstatedecision` service reads queue head and robot state:
   - `Free` → pop & return order + set `Busy`.  
   - `Wait` or empty queue → return `Wait`.  
#. **Persistence** – every mutation triggers an immediate YAML flush.  
#. **Recovery** – on startup, existing YAML is loaded; if malformed, a backup is restored or recreated.



Implementation module
---------------------
Full API documentation:

.. toctree::
   :maxdepth: 1
   :caption: Server Component

   server_modules/orchestration_and_coordination
