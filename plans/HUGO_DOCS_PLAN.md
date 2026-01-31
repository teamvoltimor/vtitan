klevor-v2/docs-planning/hugo-docs-plan.md
# Hugo Documentation Site Plan

## Overview

This document outlines the plan for building a documentation site using [Hugo](https://gohugo.io/) and the [hugo-book](https://themes.gohugo.io/themes/hugo-book/) theme. The site will serve as a powerful, minimalist, and dark-themed documentation portal, featuring sidebar navigation, versioning, code highlighting, custom shortcodes, and search functionality.

---

## Goals

- **Minimalist, dark-themed design** for readability and focus.
- **Sidebar navigation** for easy access to all documentation pages.
- **Landing page** that explains the documentation structure and purpose.
- **Each file in the docs directory becomes a separate page**.
- **Versioning** to support multiple documentation versions (e.g., v1, v2).
- **Code highlighting** for technical documentation.
- **Custom shortcodes** for notes, warnings, tips, etc.
- **Search functionality** for quick information retrieval.

---

## Structure

```
hugo-docs/
├── config.toml
├── content/
│   ├── _index.md         # Landing page
│   ├── v1/               # Versioned docs (example)
│   │   ├── _index.md
│   │   ├── getting-started.md
│   │   └── usage.md
│   └── v2/               # Another version (optional)
│       ├── _index.md
│       └── ...
├── layouts/shortcodes/   # Custom shortcodes
├── static/               # Static assets (images, etc.)
└── themes/
```

- **content/**: All documentation lives here, organized by version.
- **layouts/shortcodes/**: Custom Hugo shortcodes for reusable content blocks.
- **static/**: Images, logos, and other static assets.
- **themes/**: Hugo themes, including hugo-book.

---

## Features & Implementation

### 1. Theme: hugo-book

- Supports dark mode (default).
- Sidebar navigation auto-generated from content structure.
- Built-in search (Lunr.js).
- Code highlighting with customizable styles.

### 2. Versioning

- Each version is a subdirectory under `content/` (e.g., `v1`, `v2`).
- Users can switch between versions via sidebar or links.

### 3. Sidebar Navigation

- Automatically reflects the folder and file structure in `content/`.
- Each Markdown file becomes a page in the sidebar.

### 4. Landing Page

- `content/_index.md` serves as the landing page.
- Explains the documentation structure, navigation, and versioning.

### 5. Code Highlighting

- Enabled via Hugo’s built-in Chroma highlighter.
- Theme set to a dark style (e.g., Monokai).

### 6. Custom Shortcodes

- Shortcodes for notes, warnings, tips, etc., placed in `layouts/shortcodes/`.
- Example usage: `{{< note >}}This is a note.{{< /note >}}`

### 7. Search Functionality

- Enabled by default in hugo-book.
- Indexes all content for fast, client-side search.

---

## Example Configuration (`config.toml`)

```toml
baseURL = "http://example.org/"
languageCode = "en-us"
title = "My Docs"
theme = "hugo-book"
defaultContentLanguage = "en"
enableGitInfo = true

[params]
  BookTheme = "dark"
  BookToC = true
  BookSearch = true
  BookRepo = "https://github.com/your/repo"
  BookLogo = "logo.png"

[markup]
  [markup.highlight]
    noClasses = false
    style = "monokai"

[outputs]
  home = ["HTML", "RSS", "JSON"]
```

---

## Implementation Steps

1. **Initialize Hugo site**  
   - `hugo new site hugo-docs`
   - `cd hugo-docs`
   - `git init`
   - `git submodule add https://github.com/alex-shpak/hugo-book themes/hugo-book`

2. **Configure `config.toml`**  
   - Set theme, dark mode, code highlighting, and search.

3. **Organize Documentation Content**  
   - Place docs in `content/v1/`, `content/v2/`, etc.
   - Each Markdown file = one page.

4. **Create Landing Page**  
   - Write `content/_index.md` to introduce the docs.

5. **Add Custom Shortcodes**  
   - Create files in `layouts/shortcodes/` (e.g., `note.html`, `warning.html`).

6. **Add Static Assets**  
   - Place images, logos, etc., in `static/`.

7. **Test Locally**  
   - Run `hugo server` and verify navigation, search, code highlighting, and shortcodes.

8. **Iterate and Polish**  
   - Adjust sidebar, landing page, and styles as needed.

---

## Next Steps

- Scaffold the Hugo site structure.
- Migrate documentation files into the appropriate versioned folders.
- Implement custom shortcodes.
- Finalize configuration and test all features.

---