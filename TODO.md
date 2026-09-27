Yes. I’d use essentially that order. The architecture has a nice separation: **episodic world knowledge ↔ semantic RoboKGNet ↔ procedural/planning knowledge**, with the Knowledge Interface sitting across them.

Here’s a clean project TODO markdown:

# CMOC / RoboKGNet TODO

## 1. Complete `look_for`

Implement and test the search behaviour:

```text
look_for(theme)
    ↓
query semantic memory for likely locations
    ↓
rank locations
    ↓
go_to(location)
    ↓
observe_with_vlm(theme)
    ↓
found?
 ├─ yes → update episodic memory / scene graph
 └─ no  → try next location
```

### Done when
- Robot searches multiple candidate locations automatically.
- Perception determines whether the requested object is present.
- Successful observations update the scene graph.
- `look_for()` returns the grounded object and its Source.

---

## 2. Support Two World-Knowledge Modes

### Mode A: Known Environment

Robot starts with an existing scene graph.

```text
initial scene graph
→ query known objects / locations
→ search only when knowledge is missing
```

Useful for deterministic demonstrations and testing.

### Mode B: Learning Environment

Robot starts with no episodic scene graph.

```text
empty episodic memory
→ explore environment
→ perceive objects / locations
→ construct scene graph incrementally
→ reuse learned knowledge later
```

This demonstrates knowledge acquisition rather than merely knowledge retrieval.

---

# 3. Build RoboKGNet

RoboKGNet should provide the robot's reusable semantic knowledge.

## WordNet

Provides the **concept/class hierarchy**.

Example:

```text
mug
  → cup
    → container
      → physical object
```

Tasks:
- Import required WordNet concepts.
- Represent class hierarchy.
- Align perceived object classes to WordNet concepts.

---

## VerbNet

Provides **actions, semantic frames and frame semantics**.

Example:

```text
bring-11.3

Agent
Theme
Source
Destination

cause(Agent, E)
motion(during(E), Theme)
location(start(E), Theme, Source)
location(end(E), Theme, Destination)
```

Tasks:
- Import relevant VerbNet classes.
- Represent thematic roles.
- Represent semantic predicates.
- Connect verbs/actions to frames.

---

## FrameNet

Provides complementary **frame semantics and Frame Element descriptions**.

Example:

```text
Bringing

Agent
Theme
Source
Destination
```

Tasks:
- Represent FrameNet frames.
- Represent Frame Elements and their descriptions.
- Store VerbNet ↔ FrameNet alignments.
- Use descriptions as semantic knowledge about role meaning.

---

## ConceptNet-inspired Commonsense Knowledge

Represent weighted commonsense relations such as:

```text
coffee_mug --LocatedAt(0.95)--> kitchen
coffee_mug --LocatedAt(0.60)--> living_room

knife --UsedFor(0.90)--> cutting
```

We do **not** need to reproduce ConceptNet itself.

Tasks:
- Implement `LocatedAt`.
- Implement `UsedFor`.
- Associate weights/confidence values with edges.
- Integrate existing commonsense-generation components where possible.

This knowledge supports semantic search when episodic knowledge is missing.

---

# 4. Planning / Procedural Knowledge

Integrate the existing Planning Ontology / Unified Planning work.

Represent:

```text
PlanningDomain
PlanningAction
PlanningPredicate
PlanningProblem
InitialState
GoalState
Plan
```

Tasks:
- Reuse the existing planning ontology.
- Store generated planning problems.
- Store generated plans.
- Connect semantic frames to planning actions/problems.
- Update planning knowledge after execution where appropriate.
- Maintain conversion to/from Unified Planning.

---

# 5. Extend the Knowledge Interface

The Knowledge Interface becomes the main API over the knowledge system.

It should provide access to:

### Episodic knowledge

```python
query_theme("muffin")
query_theme_location("muffin")
query_locations()
```

### Semantic knowledge

```python
get_superclasses("muffin")
get_frame("bring")
get_frame_elements("Bringing")
get_likely_locations("muffin")
get_affordances("knife")
```

### Procedural knowledge

```python
get_action("pick")
get_planning_domain(...)
get_plan(...)
store_plan(...)
```

The rest of the architecture should preferably access the graphs through this interface rather than directly querying RDF everywhere.

---

# 6. Integrate the Complete Pipeline

Target pipeline:

```text
Natural-language instruction
        ↓
Frame identification / filling
        ↓
RoboKGNet semantic knowledge
        ↓
Knowledge Interface
        ↓
Episodic memory lookup
        ↓
missing knowledge?
        ↓ yes
look_for / exploration / perception
        ↓
update episodic scene graph
        ↓
instantiate planning problem
        ↓
Unified Planning
        ↓
execute plan
        ↓
update knowledge
```

---

# 7. End-to-End Demonstrations

## Demo A — Known World

```text
"Bring me the muffin"
→ scene graph already knows muffin location
→ generate plan
→ robot retrieves muffin
```

## Demo B — Unknown Object Location

```text
"Bring me the muffin"
→ Source unknown
→ RoboKGNet predicts likely locations
→ look_for(muffin)
→ object discovered
→ scene graph updated
→ plan generated
→ robot retrieves muffin
```

## Demo C — Initially Unknown Environment

```text
empty scene graph
→ robot explores / maps
→ perceives environment
→ constructs episodic knowledge
→ receives task
→ combines learned episodic knowledge + RoboKGNet
→ plans and acts
```

---

# Immediate Order

- [ ] Stabilize autonomous SLAM/exploration
- [ ] Implement and test `look_for`
- [ ] Implement known-world mode
- [ ] Implement empty/learning-world mode
- [ ] Assemble RoboKGNet
  - [ ] WordNet hierarchy
  - [ ] VerbNet frames
  - [ ] FrameNet semantics/alignment
  - [ ] Weighted `LocatedAt`
  - [ ] Weighted `UsedFor`
  - [ ] Planning ontology
- [ ] Extend Knowledge Interface
- [ ] Connect frame → knowledge → search → planning
- [ ] Run known-world end-to-end demo
- [ ] Run unknown-object demo
- [ ] Run learning-environment demo

The important thing is that most of these are **integration tasks rather than greenfield implementation**. You already have pieces of the scene graph, commonsense knowledge, planning ontology, Unified Planning, frame work, perception, and navigation.