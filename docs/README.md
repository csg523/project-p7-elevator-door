# Project Documentation (docs/)

This folder contains all documentation for your capstone project.  
It is used to generate a **single-page project website** using GitHub Pages.

You will **not** modify the website structure, layout, or scripts.  
Your responsibility is to **add and maintain content** in the designated Markdown files.

---

## 1. How this documentation system works

- The project website is generated automatically from the `docs/` folder.
- GitHub Pages builds the site whenever you push changes to the repository.
- The site is **single-page**, with a floating table of contents (TOC).
- Each major section of the project (Requirements, Architecture, UML, etc.)
  is stored in a **separate Markdown file**.
- These files are assembled into the final page by the template.

You should think of this as a **structured technical report**, not a blog
or a UI-heavy website.

---

## 2. Folder structure (what you should and should not edit)

    docs/
    ├── README.md              ← this file (read it!)
    ├── index.html             ← DO NOT edit
    ├── _config.yml            ← minimal edits allowed (see below)
    ├── sections/              ← YOU EDIT THESE FILES
    │   ├── intro.md
    │   ├── requirements.md
    │   ├── sdlc-traceability.md
    │   ├── uml-statecharts.md
    │   ├── architecture.md
    │   ├── rtos-design.md
    │   ├── multicore-ipc.md
    │   ├── testing-verification.md
    │   ├── dependability-safety.md
    │   ├── security.md
    │   └── lessons-learned.md
    ├── assets/
    │   ├── css/               ← DO NOT edit
    │   └── js/                ← DO NOT edit
    └── _includes/, _layouts/  ← DO NOT edit

### You are expected to edit
- `docs/sections/*.md`
- selected fields in `docs/_config.yml`

### You must NOT edit
- `index.html`
- `_layouts/`, `_includes/`
- any JavaScript or CSS files

---

## 3. Editing section files (`sections/*.md`)

Each file in `sections/` corresponds to **one major project artifact**.

Examples:
- `requirements.md` → Requirements specification  
- `uml-statecharts.md` → Behavioral modeling  
- `architecture.md` → Software architecture  

### Writing rules

- Use **Markdown only**
- Use headings (`#`, `##`, `###`) to structure your content
- Do **not** worry about heading levels relative to the page  
  (the site handles this automatically)
- Do **not** insert navigation, links to other sections, or HTML layout code

Example:

    ## Functional Requirements

    ### FR-1 Event Handling
    The system shall respond to external events within 10 ms.

---

## 4. Diagrams (Mermaid – IMPORTANT)

All diagrams must be written using **fenced Mermaid blocks**.

Correct format:

    ```mermaid
    stateDiagram-v2
        [*] --> Idle
        Idle --> Active
        Active --> Fault
    ```

### Why this format is required

- Diagrams render **directly on GitHub**
- Diagrams also render on the **project website**
- You write the diagram **once**, not twice

### Rules

- Do NOT use `<div class="mermaid">`
- Do NOT embed images of diagrams unless explicitly required
- Mermaid blocks must contain **only diagram code**

---

## 5. Images and figures

You may include images using standard Markdown syntax:

    ![Layered software architecture](IMAGE_URL_HERE)

### Captions (required style)

Always place the caption **immediately after the image**, in italics:

    ![Layered software architecture](IMAGE_URL_HERE)

    *Figure 2: Layered architecture separating hardware abstraction,
    core services, and application logic.*

- Images are centered automatically
- Captions are styled consistently
- Numbering figures is recommended but not mandatory

Avoid screenshots of text or code unless explicitly justified.

---

## 6. Tables and lists

Use Markdown tables and lists normally.

Example:

    | Requirement | Description | Verification |
    |------------|-------------|--------------|
    | FR-1 | Event response | Unit test |

Do **not** embed tables as images.

---

## 7. Customizing the project banner (allowed)

You may customize **metadata only** in `_config.yml`.

Allowed fields:

    title: "Project Title"
    description: "One-line technical description of the system"

    project_meta: >
      CS G523 – Embedded Software Design<br>
      Platform: Dual-core MCU<br>
      Team: Group X

    banner_image: /assets/images/banner.jpg   # optional

Notes:
- `banner_image` is optional
- If omitted, a default gradient is used
- Do not change layout or theme settings

---

## 8. Version control expectations

- Commit documentation updates **regularly**
- Commit messages should be meaningful:
  - “Add initial requirements”
  - “Update UML state machine after design review”
- Documentation is treated as a **first-class artifact**, not an afterthought

---

## 9. What this documentation is evaluated on

Your documentation will be assessed for:

- Technical clarity and correctness
- Traceability between requirements, design, and testing
- Appropriate use of modeling (UML, state machines, architecture)
- Consistency and organization
- Alignment with lectures and labs

Visual polish matters **only insofar as it improves clarity**.

---

## 10. Final advice

Think of this website as:

> **A structured technical design document that happens to be rendered as a website.**

If something would not be acceptable in a professional design review,
it is probably not acceptable here either.

When in doubt:
- Prefer clarity over verbosity
- Prefer diagrams over prose where appropriate
- Prefer explicit assumptions over implicit ones

---

If you follow these rules, your documentation will:
- render correctly
- be easy to review
- align cleanly with grading rubrics
- scale as your project evolves
