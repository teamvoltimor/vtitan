---
title: "Getting Started"
description: "Set up your environment and start using vTitan v2 - prerequisites, installation, and first steps."
---

# Getting Started

Welcome to the documentation for **vtitan**!

This guide will help you get up and running quickly. Follow the steps below to set up your environment and start using the project.

---

## Example Project Diagram

Below is an example image included from the `static/` directory:

![Project Diagram](/images/project-diagram.png)


---

## Prerequisites

- [Go](https://golang.org/) 1.18+ installed
- [Git](https://git-scm.com/) installed
- A code editor (e.g., VS Code, Vim)

---

## Installation

Clone the repository:

```sh
git clone https://github.com/teamvoltimor/vtitan.git
cd vtitan
```

Install dependencies:

```sh
go mod download
```

---

## Running the Project

To start the application, run:

```sh
go run main.go
```

You should see output indicating the server is running.

---

## Next Steps

- Explore the [Usage](usage.md) guide for more details.
- Check the sidebar for additional documentation topics.
- Use the search bar to quickly find what you need.

---

{{< note >}}
If you encounter any issues, please check the FAQ or open an issue on GitHub.
{{< /note >}}

{{< info >}}
For more detailed explanations about each feature, visit the relevant section in the sidebar.
{{< /info >}}

{{< danger >}}
Running untrusted code or scripts may harm your system. Always review code before executing!
{{< /danger >}}

{{< tip >}}
You can use the search bar at the top to quickly find any topic or keyword in the documentation.
{{< /tip >}}